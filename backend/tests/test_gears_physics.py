from __future__ import annotations

import math
from pathlib import Path

import pytest
from build123d import Align, Cylinder, export_step

from app.physics.fea_solver import run_gear_fea
from app.physics.gear_mechanics import (
    compute_gear_diameters,
    evaluate_gear_global_alerts,
    evaluate_gear_metrics,
    load_gear_presets,
)
from app.physics.mechanics import load_materials
from app.schemas import GearDesignRequest


def test_load_gear_presets_validate_against_schema():
    data = load_gear_presets()
    assert "presets" in data
    expected = {"spur_nema17", "spur_heavy", "gt2_pulley_20t", "gt2_pulley_40t"}
    assert set(data["presets"].keys()) == expected
    for preset_id, entry in data["presets"].items():
        assert entry["id"] == preset_id
        assert entry["name"]
        assert entry["subtitle"]
        req = GearDesignRequest(**entry["defaults"])
        assert req.preset == preset_id


def test_compute_gear_diameters_spur_and_gt2():
    req_spur = GearDesignRequest(gear_kind="spur_involute", module_mm=2.0, teeth=24)
    dp_s, do_s = compute_gear_diameters(req_spur)
    assert dp_s == pytest.approx(48.0, abs=1e-3)
    assert do_s == pytest.approx(52.0, abs=1e-3)

    req_gt2 = GearDesignRequest(gear_kind="gt2_pulley", teeth=20)
    dp_g, do_g = compute_gear_diameters(req_gt2)
    expected_dp = (2.0 * 20.0) / math.pi
    assert dp_g == pytest.approx(expected_dp, abs=1e-2)
    assert do_g == pytest.approx(expected_dp - 0.508 + 2.4, abs=1e-2)


def test_gear_lewis_stress_inertia_and_gt2_metrics():
    petg_cf = load_materials()["petg_cf"]
    req_spur = GearDesignRequest(
        gear_kind="spur_involute",
        teeth=24,
        module_mm=2.0,
        face_width_mm=10.0,
        torque_nm=1.5,
        rpm=120.0,
    )

    m_solid, _ = evaluate_gear_metrics(req_spur, "gear_solid", 18_000.0, None, petg_cf)
    m_webbed, _ = evaluate_gear_metrics(req_spur, "gear_webbed", 13_500.0, m_solid.mass_g, petg_cf)
    m_spoked, _ = evaluate_gear_metrics(req_spur, "gear_spoked", 11_000.0, m_solid.mass_g, petg_cf)

    # Rotational inertia drops significantly for webbed & spoked variants
    assert m_webbed.rotational_inertia_kg_cm2 < m_solid.rotational_inertia_kg_cm2 * 0.75
    assert m_spoked.rotational_inertia_kg_cm2 < m_webbed.rotational_inertia_kg_cm2
    assert m_spoked.mass_savings_pct > m_webbed.mass_savings_pct > 0
    assert m_solid.print_without_supports is True
    assert m_spoked.print_without_supports is True

    # Doubling torque doubles Lewis tooth stress
    req_2x = req_spur.model_copy(update={"torque_nm": 3.0})
    m_2x, _ = evaluate_gear_metrics(req_2x, "gear_solid", 18_000.0, None, petg_cf)
    assert m_2x.max_stress_mpa / m_solid.max_stress_mpa == pytest.approx(2.0, rel=0.02)

    # GT2 pulley evaluates cleanly with 12-point stress profile
    req_gt2 = GearDesignRequest(gear_kind="gt2_pulley", teeth=40, face_width_mm=8.0, torque_nm=1.2)
    m_gt2, _ = evaluate_gear_metrics(req_gt2, "gear_spoked", 7_000.0, None, petg_cf)
    assert m_gt2.max_stress_mpa > 0
    assert m_gt2.safety_factor > 0
    assert len(m_gt2.stress_profile) == 12
    assert m_gt2.stress_profile[0] == pytest.approx(1.0, abs=1e-3)
    assert m_gt2.stress_profile[-1] == pytest.approx(0.0, abs=1e-6)


def test_gear_global_dfm_alerts():
    materials = load_materials()
    petg_cf = materials["petg_cf"]
    pla_plus = materials["pla_plus"]

    # Undercutting (Z < 17), fine module vs nozzle, tight backlash
    req_bad_spur = GearDesignRequest(
        gear_kind="spur_involute",
        teeth=14,
        module_mm=0.9,
        nozzle_diameter_mm=0.4,
        backlash_mm=0.04,
        operating_temp_c=60.0,
        material_id="pla_plus",
    )
    alerts = evaluate_gear_global_alerts(req_bad_spur, pla_plus)
    codes = {a.code for a in alerts}
    assert "GEAR_UNDERCUTTING_RISK" in codes
    assert "GEAR_NOZZLE_RESOLUTION" in codes
    assert "FDM_BACKLASH_RECOMMENDATION" in codes
    assert "THERMAL_LIMIT_EXCEEDED" in codes

    # GT2 pulley with nozzle > 0.4 mm
    req_gt2_big_nozzle = GearDesignRequest(
        gear_kind="gt2_pulley", teeth=20, nozzle_diameter_mm=0.6, material_id="petg_cf"
    )
    alerts_gt2 = evaluate_gear_global_alerts(req_gt2_big_nozzle, petg_cf)
    assert any(a.code == "GT2_NOZZLE_TOO_LARGE" for a in alerts_gt2)


def test_run_gear_fea_on_gear_disc_step(tmp_path: Path):
    req = GearDesignRequest(
        gear_kind="spur_involute",
        teeth=24,
        module_mm=2.0,
        face_width_mm=10.0,
        bore_diameter_mm=6.0,
        torque_nm=1.5,
        material_id="petg_cf",
    )
    material = load_materials()[req.material_id]

    # Gear blank centered at (0, 0), Z in [0, 10], OD=52, bore=6
    blank = Cylinder(
        radius=26.0, height=10.0, align=(Align.CENTER, Align.CENTER, Align.MIN)
    ) - Cylinder(
        radius=3.0, height=10.0, align=(Align.CENTER, Align.CENTER, Align.MIN)
    )

    step_path = tmp_path / "gear_solid.step"
    export_step(blank, str(step_path))
    assert step_path.exists() and step_path.stat().st_size > 0

    glb_path = tmp_path / "gear_solid_fea_y_inplane.glb"
    res = run_gear_fea(
        step_path=step_path,
        output_glb_path=glb_path,
        req=req,
        design_hash="gear_hash_01",
        variant_id="gear_solid",
        material=material,
        mesh_size_mm=6.0,
        load_direction="y_inplane",
        deform_scale=10.0,
    )

    assert res.design_hash == "gear_hash_01"
    assert res.variant_id == "gear_solid"
    assert res.node_count > 50
    assert res.element_count > 100
    assert res.max_von_mises_mpa >= res.p95_von_mises_mpa > 0.0
    assert res.max_displacement_mm > 0.0
    assert res.fea_safety_factor > 0.0
    assert res.fea_glb_url == "/api/files/gear_hash_01/gear_solid_fea_y_inplane.glb"
    assert glb_path.exists()
    glb_bytes = glb_path.read_bytes()
    assert len(glb_bytes) > 1000
    assert b"COLOR_0" in glb_bytes
