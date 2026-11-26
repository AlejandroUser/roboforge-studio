from __future__ import annotations

import pytest

from app.physics.arm_kinematics import evaluate_arm_assembly, load_arm_presets
from app.physics.copilot import run_closed_loop_copilot
from app.physics.mechanics import load_materials
from app.schemas import ArmAssemblyRequest, CopilotRequest


def test_load_arm_presets_validate_against_schema():
    data = load_arm_presets()
    assert "presets" in data
    expected = {
        "parol6_desktop",
        "ar4_industrial",
        "so_arm_lerobot",
        "cobot_hollow_ur",
        "fabri_arm_5",
        "pybot_scara",
        "openarm_7dof",
        "arctos_6dof",
        "bcn3d_moveo",
        "thor_6dof",
        "dobot_4bar",
        "lumen_pnp",
        "delta_flexpicker",
        "barrett_wam",
        "surgical_continuum",
        "rov_subsea",
        "bimanual_aloha",
        "stewart_hexapod",
        "canadarm_space",
        "festo_bellows",
        "kuka_kr210",
        "mars_mini",
        "franka_panda",
    }
    assert set(data["presets"].keys()) == expected
    for preset_id, entry in data["presets"].items():
        assert entry["id"] == preset_id
        assert entry["name"]
        assert entry["subtitle"]
        req = ArmAssemblyRequest(**entry["defaults"])
        assert req.preset == preset_id


def test_arm_fk_cascaded_torques_bom_and_printer_alerts():
    materials = load_materials()
    petg_cf = materials["petg_cf"]

    # Horizontal stretched arm (0, 0, 0 deg) -> TCP X == L1 + L2 + L3, Z == base_height
    req_flat = ArmAssemblyRequest(
        preset="parol6_desktop",
        base_height_mm=100.0,
        upper_arm_mm=200.0,
        forearm_mm=170.0,
        wrist_mm=70.0,
        shoulder_joint_deg=0.0,
        elbow_joint_deg=0.0,
        wrist_joint_deg=0.0,
        payload_kg=1.2,
        material_id="petg_cf",
        printer_id="bambu_x1c",
    )
    vols = {
        "base": 140_000.0,
        "upper_arm": 95_000.0,
        "forearm": 70_000.0,
        "wrist": 32_000.0,
    }
    res = evaluate_arm_assembly(req_flat, vols, petg_cf)

    assert res["max_reach_mm"] == pytest.approx(440.0)
    assert res["tcp_position_mm"][0] == pytest.approx(440.0, abs=0.2)
    assert res["tcp_position_mm"][1] == pytest.approx(0.0, abs=0.2)
    assert res["tcp_position_mm"][2] == pytest.approx(100.0, abs=0.2)

    joints = {j.joint_id: j for j in res["joints"]}
    assert set(joints.keys()) == {"J1", "J2", "J3"}
    # Cascaded torque ordering: Shoulder > Elbow > Wrist
    assert joints["J1"].static_torque_nm > joints["J2"].static_torque_nm > joints["J3"].static_torque_nm > 0
    assert joints["J1"].dynamic_torque_nm > joints["J2"].dynamic_torque_nm > joints["J3"].dynamic_torque_nm > 0
    for j in joints.values():
        assert j.required_motor_torque_nm > 0
        assert j.recommended_actuator
        assert j.safety_factor > 0

    assert res["printed_mass_g"] > 0
    assert res["total_arm_mass_kg"] > res["printed_mass_g"] / 1000.0
    assert res["tcp_deflection_mm"] > 0
    assert len(res["bom"]) >= 6
    assert res["estimated_total_cost_usd"] == pytest.approx(
        sum(item.total_cost_usd for item in res["bom"]), abs=0.05
    )

    # Long upper arm in PA12-CF on open Ender-3 V3 triggers bed & enclosure alerts
    pa12_cf = materials["pa12_cf"]
    req_big = ArmAssemblyRequest(
        preset="ar4_industrial",
        upper_arm_mm=290.0,
        material_id="pa12_cf",
        printer_id="ender3_v3",
    )
    res_big = evaluate_arm_assembly(req_big, vols, pa12_cf)
    codes = {a.code for a in res_big["global_alerts"]}
    assert "PRINTER_BED_EXCEEDED" in codes
    assert "PRINTER_ENCLOSURE_REQUIRED" in codes


def test_closed_loop_copilot_parsing_and_convergence():
    resp = run_closed_loop_copilot(
        CopilotRequest(
            prompt="Brazo SCARA de 240mm para 40N en PA12-CF con FS >= 2.5 y deflexión <= 0.6 mm"
        )
    )
    assert resp.converged is True
    assert 1 <= len(resp.iterations) <= 12
    assert resp.optimized_request.architecture == "scara"
    assert resp.optimized_request.length_mm == pytest.approx(240.0)
    assert resp.optimized_request.tip_load_n == pytest.approx(40.0)
    assert resp.optimized_request.material_id == "pa12_cf"
    assert resp.optimized_request.target_safety_factor == pytest.approx(2.5)
    assert resp.optimized_request.max_deflection_mm == pytest.approx(0.6)
    assert any(it.passes for it in resp.iterations)
    assert len(resp.parsed_constraints) >= 5
    assert "PA12-CF" in resp.summary

    # English + kg load + cm length prompt
    resp_en = run_closed_loop_copilot(
        CopilotRequest(prompt="Robot leg 20 cm for 3 kg load in aluminum with SF >= 3.0")
    )
    assert resp_en.converged is True
    assert resp_en.optimized_request.architecture == "robot_leg"
    assert resp_en.optimized_request.length_mm == pytest.approx(200.0)
    assert resp_en.optimized_request.tip_load_n == pytest.approx(29.4, abs=0.2)
    assert resp_en.optimized_request.material_id == "al6061_t6"
    assert resp_en.optimized_request.target_safety_factor == pytest.approx(3.0)
