from __future__ import annotations

import pytest

from app.physics.arm_compatibility import evaluate_modular_arm
from app.physics.arm_kinematics import load_arm_presets
from app.physics.mechanics import load_materials
from app.physics.mechatronics import evaluate_mechatronics
from app.schemas import (
    ArmAssemblyRequest,
    ModularArmRequest,
    PrintPlateInfo,
    PrintReadinessSummary,
)


def test_v7_materials_and_21_presets():
    mats = load_materials()
    for mid in ("al_7075_t6", "ti6al4v_titanium", "steel_4140", "tpu_95a"):
        assert mid in mats
    assert mats["ti6al4v_titanium"]["yield_xy_mpa"] == 880.0
    assert mats["steel_4140"]["youngs_xy_mpa"] == 205000.0
    assert mats["tpu_95a"]["category"] == "fdm"

    presets = load_arm_presets()["presets"]
    assert len(presets) >= 21
    for key in (
        "rov_subsea",
        "bimanual_aloha",
        "stewart_hexapod",
        "canadarm_space",
        "festo_bellows",
        "kuka_kr210",
    ):
        assert key in presets
        req = ArmAssemblyRequest(**presets[key]["defaults"])
        assert req.preset == key


def test_canadarm_zero_g_and_rov_subsea_and_kuka_counterbalance():
    # 1. Canadarm 0-G: zero gravity eliminates static torque & prevents false failure
    req_earth = ModularArmRequest(
        dof=7,
        environment_mode="earth",
        payload_kg=25.0,
        max_accel_rad_s2=1.0,
        material_id="al_7075_t6",
    )
    req_zero_g = req_earth.model_copy(update={"environment_mode": "space_zero_g"})
    res_earth = evaluate_modular_arm(req_earth, {})
    res_0g = evaluate_modular_arm(req_zero_g, {})
    assert res_0g["joints"][0].static_torque_nm == pytest.approx(0.0, abs=1e-4)
    assert res_0g["shoulder_torque_nm"] < res_earth["shoulder_torque_nm"] * 0.25
    assert res_0g["compatible"] is True

    # 2. Underwater ROV: buoyancy reduces static torque + adds Viton seal kit
    req_rov = req_earth.model_copy(
        update={"environment_mode": "underwater_rov", "material_id": "ti6al4v_titanium"}
    )
    res_rov = evaluate_modular_arm(req_rov, {})
    assert res_rov["joints"][0].static_torque_nm < res_earth["joints"][0].static_torque_nm
    codes_rov = {i.part_code for i in evaluate_mechatronics(req_rov, res_rov).master_bom}
    assert "ROV-SEAL-VITON" in codes_rov

    # 3. KUKA KR 210 (210 kg + Steel 4140 + Counterbalance J2)
    req_kuka_no_cb = ModularArmRequest(
        dof=6,
        payload_kg=210.0,
        max_accel_rad_s2=2.0,
        material_id="steel_4140",
        counterbalance_j2=False,
    )
    req_kuka_cb = req_kuka_no_cb.model_copy(update={"counterbalance_j2": True})
    res_no_cb = evaluate_modular_arm(req_kuka_no_cb, {})
    res_cb = evaluate_modular_arm(req_kuka_cb, {})
    assert res_cb["joints"][0].static_torque_nm == pytest.approx(
        res_no_cb["joints"][0].static_torque_nm * 0.35, rel=0.02
    )
    codes_k = {i.part_code for i in evaluate_mechatronics(req_kuka_cb, res_cb).master_bom}
    assert "KUK-CB-CYL-J2" in codes_k


def test_stewart_bimanual_soft_bellows_and_print_readiness_schema():
    # 1. Stewart Hexapod 6-UPS
    req_stw = ModularArmRequest(
        dof=6,
        kinematics_type="stewart_hexapod",
        payload_kg=25.0,
        material_id="al_7075_t6",
    )
    res_stw = evaluate_modular_arm(req_stw, {})
    assert res_stw["compatible"] is True
    codes_stw = {i.part_code for i in evaluate_mechatronics(req_stw, res_stw).master_bom}
    assert {"STW-LIN-ACT-6X", "STW-UJOINT-12X"}.issubset(codes_stw)

    # 2. Bimanual Dual-Arm 14-DOF (doubles electrical power budget)
    req_single = ModularArmRequest(dof=7, kinematics_type="articulated", payload_kg=1.0)
    req_bim = ModularArmRequest(dof=14, kinematics_type="bimanual_dual_arm", payload_kg=1.0)
    mech_s = evaluate_mechatronics(req_single, evaluate_modular_arm(req_single, {}))
    mech_b = evaluate_mechatronics(req_bim, evaluate_modular_arm(req_bim, {}))
    assert mech_b.power_budget.continuous_power_w > mech_s.power_budget.continuous_power_w * 1.5
    assert "BIM-TORSO-FRAME" in {i.part_code for i in mech_b.master_bom}

    # 3. Soft Pneumatic Bellows (TPU 95A)
    req_pnu = ModularArmRequest(
        dof=6,
        kinematics_type="soft_pneumatic_bellows",
        payload_kg=0.4,
        material_id="tpu_95a",
    )
    res_pnu = evaluate_modular_arm(req_pnu, {})
    assert res_pnu["compatible"] is True
    codes_pnu = {i.part_code for i in evaluate_mechatronics(req_pnu, res_pnu).master_bom}
    assert {"PNU-PIEZO-VALVE", "PNU-COMPRESSOR"}.issubset(codes_pnu)

    # 4. PrintReadinessSummary schema contract
    pr = PrintReadinessSummary(
        fits_bed=True,
        printer_name="Bambu Lab X1-Carbon",
        bed_mm=[256.0, 256.0, 256.0],
        total_plates=2,
        total_filament_g=420.5,
        estimated_print_hours=11.4,
        layer_height_mm=0.2,
        wall_loops=4,
        infill_pct=25,
        infill_pattern="Gyroid",
        nozzle_temp_c=255,
        bed_temp_c=75,
        plates=[
            PrintPlateInfo(
                plate_index=1,
                filename="build_plate_1.3mf",
                url="/api/files/abc12345/build_plate_1.3mf",
                parts=["base_pedestal", "upper_arm_link"],
                estimated_mass_g=215.0,
                estimated_hours=5.8,
            )
        ],
        warnings=[],
    )
    assert pr.fits_bed is True
    assert pr.total_plates == 2
