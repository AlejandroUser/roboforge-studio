from __future__ import annotations

import pytest

from app.physics.dfm_rules import evaluate_global_alerts
from app.physics.mechanics import (
    evaluate_variant_metrics,
    load_materials,
    load_presets,
)
from app.schemas import EndInterfaceParams, LinkDesignRequest


EXPECTED_MATERIAL_IDS = {
    "pla_plus",
    "petg",
    "abs_asa",
    "petg_cf",
    "pa12_cf",
    "al6061_t6",
    "carbon_plate",
}

REQUIRED_MATERIAL_FIELDS = {
    "id",
    "name",
    "category",
    "density_g_cm3",
    "youngs_xy_mpa",
    "youngs_z_mpa",
    "yield_xy_mpa",
    "yield_z_mpa",
    "max_temp_c",
    "cost_per_kg_usd",
    "color_hex",
    "notes",
}


def test_load_materials_complete_and_valid():
    materials = load_materials()
    assert set(materials.keys()) == EXPECTED_MATERIAL_IDS
    for mat_id, mat in materials.items():
        assert mat["id"] == mat_id
        assert REQUIRED_MATERIAL_FIELDS.issubset(mat.keys())
        assert mat["category"] in ("fdm", "cnc")
        assert mat["density_g_cm3"] > 0
        assert mat["youngs_xy_mpa"] >= mat["youngs_z_mpa"] > 0
        assert mat["yield_xy_mpa"] >= mat["yield_z_mpa"] > 0
        assert mat["max_temp_c"] > 0
        assert mat["cost_per_kg_usd"] > 0
        assert mat["color_hex"].startswith("#")


def test_load_presets_validate_against_schemas():
    presets = load_presets()
    assert "architectures" in presets
    assert "end_interfaces" in presets

    expected_archs = {"articulated_6dof", "scara", "delta_rod", "robot_leg"}
    assert set(presets["architectures"].keys()) == expected_archs
    for arch_id, entry in presets["architectures"].items():
        req = LinkDesignRequest(**entry["defaults"])
        assert req.architecture == arch_id

    expected_ends = {"nema17", "nema23", "bearing_608", "bearing_6805", "custom"}
    assert set(presets["end_interfaces"].keys()) == expected_ends
    for end_id, entry in presets["end_interfaces"].items():
        end = EndInterfaceParams(**entry["params"])
        assert end.preset == end_id


def test_anisotropy_z_vertical_lowers_safety_factor_vs_xy_flat():
    materials = load_materials()
    petg_cf = materials["petg_cf"]

    req_xy = LinkDesignRequest(material_id="petg_cf", print_orientation="xy_flat")
    req_z = LinkDesignRequest(material_id="petg_cf", print_orientation="z_vertical")

    volume_mm3 = 55_000.0
    m_xy, _ = evaluate_variant_metrics(req_xy, "organic_ibeam", volume_mm3, None, petg_cf)
    m_z, _ = evaluate_variant_metrics(req_z, "organic_ibeam", volume_mm3, None, petg_cf)

    assert m_z.effective_yield_mpa < m_xy.effective_yield_mpa
    assert m_z.safety_factor < m_xy.safety_factor
    assert m_z.tip_deflection_mm > m_xy.tip_deflection_mm

    # Global DFM alert should flag Z-orientation weakness for FDM
    alerts_z = evaluate_global_alerts(req_z, petg_cf)
    assert any(a.code == "Z_ORIENTATION_WEAKNESS" for a in alerts_z)


def test_thermal_alert_when_pla_plus_near_60c():
    materials = load_materials()
    pla_plus = materials["pla_plus"]

    req_hot = LinkDesignRequest(material_id="pla_plus", operating_temp_c=58.0)
    alerts_hot = evaluate_global_alerts(req_hot, pla_plus)
    thermal_alerts = [
        a for a in alerts_hot if a.code in ("THERMAL_LIMIT_EXCEEDED", "THERMAL_MARGIN_LOW")
    ]
    assert len(thermal_alerts) >= 1
    assert any(a.level == "critical" for a in thermal_alerts)

    # Near 50C (within 10C of 55C HDT) should emit a warning
    req_warm = LinkDesignRequest(material_id="pla_plus", operating_temp_c=50.0)
    alerts_warm = evaluate_global_alerts(req_warm, pla_plus)
    assert any(a.code == "THERMAL_MARGIN_LOW" and a.level == "warning" for a in alerts_warm)

    # PA12-CF at 58C should have no thermal alerts
    pa12_cf = materials["pa12_cf"]
    alerts_pa12 = evaluate_global_alerts(req_hot, pa12_cf)
    assert not any("THERMAL" in a.code for a in alerts_pa12)


def test_deflection_scales_linearly_with_tip_load():
    materials = load_materials()
    petg_cf = materials["petg_cf"]

    # High tip load relative to self-weight so self-weight contribution is negligible
    req_low = LinkDesignRequest(
        material_id="petg_cf",
        tip_load_n=50.0,
        root_width_mm=36.0,
        tip_width_mm=24.0,
        length_mm=180.0,
    )
    req_high = req_low.model_copy(update={"tip_load_n": 100.0})

    volume_mm3 = 50_000.0
    m_low, _ = evaluate_variant_metrics(req_low, "tapered_solid", volume_mm3, None, petg_cf)
    m_high, _ = evaluate_variant_metrics(req_high, "tapered_solid", volume_mm3, None, petg_cf)

    ratio = m_high.tip_deflection_mm / m_low.tip_deflection_mm
    assert ratio == pytest.approx(2.0, rel=0.03)
    assert m_high.max_stress_mpa / m_low.max_stress_mpa == pytest.approx(2.0, rel=0.03)


def test_uniform_beam_matches_analytical_cantilever_formula():
    """Verify Simpson numerical integration matches delta = F*L^3 / (3*E*I) for uniform beam."""
    materials = load_materials()
    al = materials["al6061_t6"]

    L = 200.0
    b = 20.0
    h = 30.0
    F = 100.0
    req = LinkDesignRequest(
        material_id="al6061_t6",
        length_mm=L,
        root_width_mm=h,
        tip_width_mm=h,
        thickness_mm=b,
        organic_taper=0.0,
        cable_channel=False,
        tip_load_n=F,
    )
    # Near-zero volume so self-weight is negligible
    metrics, _ = evaluate_variant_metrics(req, "tapered_solid", 10.0, None, al)
    I_exact = (b * (h**3)) / 12.0
    expected_deflection = (F * (L**3)) / (3.0 * al["youngs_xy_mpa"] * I_exact)
    assert metrics.tip_deflection_mm == pytest.approx(expected_deflection, rel=0.02)


def test_stiffness_to_weight_score_and_all_variants():
    materials = load_materials()
    petg_cf = materials["petg_cf"]
    req = LinkDesignRequest(material_id="petg_cf")

    solid_vol = 90_000.0
    m_solid, _ = evaluate_variant_metrics(req, "tapered_solid", solid_vol, None, petg_cf)

    # Lighter I-beam with 35% less volume should show positive mass savings and higher stiffness/weight
    ibeam_vol = solid_vol * 0.65
    m_ibeam, _ = evaluate_variant_metrics(
        req, "organic_ibeam", ibeam_vol, m_solid.mass_g, petg_cf
    )
    assert m_ibeam.mass_savings_pct == pytest.approx(35.0, abs=0.2)
    assert m_ibeam.stiffness_to_weight_score > m_solid.stiffness_to_weight_score

    # Ensure all 4 variants evaluate cleanly with positive engineering metrics
    for variant_id in ("tapered_solid", "organic_ibeam", "rounded_truss", "hollow_cobot"):
        m, _ = evaluate_variant_metrics(req, variant_id, 60_000.0, m_solid.mass_g, petg_cf)
        assert m.mass_g > 0
        assert m.rotational_inertia_kg_cm2 > 0
        assert m.dynamic_torque_nm > 0
        assert m.estimated_cost_usd > 0
        assert m.stiffness_to_weight_score > 0


def test_dfm_geometry_and_nozzle_alerts():
    materials = load_materials()
    petg_cf = materials["petg_cf"]

    req_bad = LinkDesignRequest(
        material_id="petg_cf",
        nozzle_diameter_mm=0.25,
        root_width_mm=24.0,
        tip_width_mm=34.0,
        thickness_mm=10.0,
        cable_channel=True,
        root_end=EndInterfaceParams(
            preset="custom",
            outer_diameter_mm=24.0,
            bore_diameter_mm=22.0,
            bolt_count=4,
            bolt_pcd_mm=23.0,
            bolt_size="M3",
        ),
    )
    alerts = evaluate_global_alerts(req_bad, petg_cf)
    codes = {a.code for a in alerts}
    assert "CF_NOZZLE_CLOG_RISK" in codes
    assert "INVERSE_TAPER" in codes
    assert "CABLE_CHANNEL_THIN_LINK" in codes
    assert "END_WALL_TOO_THIN" in codes
