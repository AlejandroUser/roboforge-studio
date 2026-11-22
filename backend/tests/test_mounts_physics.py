from __future__ import annotations

from pathlib import Path

import pytest
from build123d import Align, Box, Pos, export_step

from app.physics.fea_solver import run_mount_fea
from app.physics.mechanics import load_materials
from app.physics.mount_mechanics import (
    evaluate_mount_global_alerts,
    evaluate_mount_metrics,
    load_mount_presets,
)
from app.schemas import MountDesignRequest


def test_load_mount_presets_validate_against_schema():
    presets = load_mount_presets()
    assert "actuators" in presets
    expected = {"nema17", "nema23", "servo_mg996r", "profile_2020_gusset"}
    assert set(presets["actuators"].keys()) == expected
    for act_id, entry in presets["actuators"].items():
        assert entry["id"] == act_id
        assert entry["name"]
        assert entry["subtitle"]
        req = MountDesignRequest(**entry["defaults"])
        assert req.actuator_type == act_id


def test_gusseted_and_lightweight_stiffness_gain_vs_standard():
    petg_cf = load_materials()["petg_cf"]
    req = MountDesignRequest(
        actuator_type="nema17",
        wall_thickness_mm=5.0,
        base_depth_mm=42.0,
        shaft_height_mm=32.0,
        gusset_thickness_mm=4.0,
        radial_load_n=40.0,
        motor_torque_nm=0.6,
    )

    m_std, _ = evaluate_mount_metrics(req, "mount_standard", 18_000.0, None, petg_cf)
    m_gus, _ = evaluate_mount_metrics(req, "mount_gusseted", 23_000.0, m_std.mass_g, petg_cf)
    m_lw, _ = evaluate_mount_metrics(req, "mount_lightweight", 19_500.0, m_std.mass_g, petg_cf)

    # Gussets should provide 3x-5x stiffness gain (3x-5x lower deflection)
    stiffness_ratio = m_std.tip_deflection_mm / m_gus.tip_deflection_mm
    assert 3.0 <= stiffness_ratio <= 5.5
    assert m_gus.safety_factor > m_std.safety_factor * 2.0
    assert m_lw.stiffness_to_weight_score > m_std.stiffness_to_weight_score

    for m in (m_std, m_gus, m_lw):
        assert len(m.stress_profile) == 12
        assert all(0.0 <= v <= 1.0 for v in m.stress_profile)
        assert m.stress_profile[0] == pytest.approx(1.0, abs=1e-3)
        assert m.stress_profile[-1] == pytest.approx(0.0, abs=1e-6)


def test_mount_anisotropy_and_motor_thermal_alerts():
    materials = load_materials()
    pla_plus = materials["pla_plus"]
    petg_cf = materials["petg_cf"]

    req_xy = MountDesignRequest(material_id="petg_cf", print_orientation="xy_flat")
    req_z = req_xy.model_copy(update={"print_orientation": "z_vertical"})
    m_xy, _ = evaluate_mount_metrics(req_xy, "mount_gusseted", 22_000.0, None, petg_cf)
    m_z, _ = evaluate_mount_metrics(req_z, "mount_gusseted", 22_000.0, None, petg_cf)
    assert m_z.safety_factor < m_xy.safety_factor
    assert m_z.tip_deflection_mm > m_xy.tip_deflection_mm

    alerts_z = evaluate_mount_global_alerts(req_z, petg_cf)
    assert any(a.code == "MOUNT_Z_ORIENTATION_DELAMINATION" for a in alerts_z)

    # PLA+ (max_temp_c = 55C) with NEMA 17 at 52C (>= 55 - 5 = 50C) -> critical thermal alert
    req_pla_hot = MountDesignRequest(
        actuator_type="nema17", material_id="pla_plus", motor_temp_c=52.0
    )
    alerts_pla = evaluate_mount_global_alerts(req_pla_hot, pla_plus)
    assert any(a.code == "MOTOR_THERMAL_CRITICAL" and a.level == "critical" for a in alerts_pla)

    # Thin wall vs nozzle alert
    req_thin = MountDesignRequest(
        material_id="petg_cf", wall_thickness_mm=2.5, nozzle_diameter_mm=0.6
    )
    alerts_thin = evaluate_mount_global_alerts(req_thin, petg_cf)
    assert any(a.code == "MOUNT_WALL_TOO_THIN" for a in alerts_thin)


def test_run_mount_fea_on_l_bracket_step(tmp_path: Path):
    req = MountDesignRequest(
        actuator_type="nema17",
        wall_thickness_mm=5.0,
        base_depth_mm=42.0,
        shaft_height_mm=32.0,
        radial_load_n=35.0,
        material_id="petg_cf",
    )
    material = load_materials()[req.material_id]

    # Build a clean L-bracket matching the Hito 3B coordinate convention:
    # base: x in [-22, 22], y in [0, 42], z in [0, 5]
    # vertical plate: x in [-22, 22], y in [0, 5], z in [0, 54]
    base = Pos(0, 21.0, 2.5) * Box(44.0, 42.0, 5.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    wall = Pos(0, 2.5, 27.0) * Box(44.0, 5.0, 54.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    l_bracket = base + wall

    step_path = tmp_path / "mount_standard.step"
    export_step(l_bracket, str(step_path))
    assert step_path.exists() and step_path.stat().st_size > 0

    glb_path = tmp_path / "mount_standard_fea_y_inplane.glb"
    res = run_mount_fea(
        step_path=step_path,
        output_glb_path=glb_path,
        req=req,
        design_hash="mount_hash_01",
        variant_id="mount_standard",
        material=material,
        mesh_size_mm=6.5,
        load_direction="y_inplane",
        deform_scale=10.0,
    )

    assert res.design_hash == "mount_hash_01"
    assert res.variant_id == "mount_standard"
    assert res.node_count > 50
    assert res.element_count > 100
    assert res.max_von_mises_mpa >= res.p95_von_mises_mpa > 0.0
    assert res.max_displacement_mm > 0.0
    assert res.fea_safety_factor > 0.0
    assert res.fea_glb_url == "/api/files/mount_hash_01/mount_standard_fea_y_inplane.glb"
    assert glb_path.exists()
    glb_bytes = glb_path.read_bytes()
    assert len(glb_bytes) > 1000
    assert b"COLOR_0" in glb_bytes
