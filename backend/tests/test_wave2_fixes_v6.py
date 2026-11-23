from __future__ import annotations

import pytest

from app.physics.arm_compatibility import evaluate_modular_arm
from app.physics.arm_kinematics import load_arm_presets
from app.physics.mechatronics import evaluate_mechatronics
from app.schemas import (
    ArmAssemblyRequest,
    LinkDesignRequest,
    ModularArmRequest,
    ModularArmSlots,
)


def test_micro_arm_below_100mm_fix_and_all_6_new_presets():
    # Bug fix verification: 40-99 mm micro-links must validate cleanly without ValueError/500
    micro = ArmAssemblyRequest(
        preset="surgical_continuum",
        dof=4,
        base_height_mm=65.0,
        upper_arm_mm=90.0,
        forearm_mm=60.0,
        wrist_mm=25.0,
    )
    assert micro.upper_arm_mm == 90.0

    presets = load_arm_presets()["presets"]
    assert len(presets) == 15
    for key in (
        "thor_6dof",
        "dobot_4bar",
        "lumen_pnp",
        "delta_flexpicker",
        "barrett_wam",
        "surgical_continuum",
    ):
        assert key in presets
        req = ArmAssemblyRequest(**presets[key]["defaults"])
        assert req.preset == key


def test_barrett_wam_capstan_remote_base_motors_improves_torque_and_sf():
    base_kwargs = dict(
        dof=7,
        kinematics_type="articulated",
        payload_kg=1.5,
        material_id="pa12_cf",
        printer_id="voron_24",
        max_accel_rad_s2=15.0,
    )
    req_distal = ModularArmRequest(**base_kwargs, remote_base_motors=False)
    req_capstan = ModularArmRequest(**base_kwargs, remote_base_motors=True)

    res_distal = evaluate_modular_arm(req_distal, {})
    res_capstan = evaluate_modular_arm(req_capstan, {})

    assert res_capstan["shoulder_torque_nm"] < res_distal["shoulder_torque_nm"] * 0.85
    assert res_capstan["min_safety_factor"] >= 2.0
    assert res_capstan["compatible"] is True

    mech = evaluate_mechatronics(req_capstan, res_capstan)
    codes = {i.part_code for i in mech.master_bom}
    assert "CAP-DRUM-SET" in codes
    assert "CAP-TENDON-DYN" in codes


def test_delta_parallel_3arm_load_sharing_avoids_serial_collapse():
    req = ModularArmRequest(
        dof=4,
        kinematics_type="delta_parallel",
        vacuum_eoat=True,
        payload_kg=1.0,
        max_accel_rad_s2=95.0,
        material_id="pa12_cf",
        printer_id="bambu_x1c",
    )
    res = evaluate_modular_arm(req, {})
    assert res["compatible"] is True
    assert res["min_safety_factor"] >= 2.0
    assert res["shoulder_torque_nm"] < 15.0  # vs 178 N*m serial collapse!

    mech = evaluate_mechatronics(req, res)
    codes = {i.part_code for i in mech.master_bom}
    assert "DLT-CF-RODS" in codes
    assert "DLT-BALL-JOINTS" in codes
    assert "VAC-NOZZLE-PUMP" in codes


def test_cartesian_gantry_and_continuum_pcc_and_4bar_bevel():
    # 1. Cartesian Gantry (LumenPnP)
    req_gantry = ModularArmRequest(
        dof=4,
        kinematics_type="cartesian_gantry",
        vacuum_eoat=True,
        payload_kg=0.3,
        material_id="petg_cf",
    )
    res_g = evaluate_modular_arm(req_gantry, {})
    assert res_g["compatible"] is True
    assert res_g["min_safety_factor"] >= 2.0
    codes_g = {i.part_code for i in evaluate_mechatronics(req_gantry, res_g).master_bom}
    assert {"GNT-2020-PROF", "GNT-MGN12-XYZ", "GNT-GT2-BELT", "VAC-NOZZLE-PUMP"}.issubset(codes_g)

    # 2. Surgical Continuum PCC (90 mm micro-link)
    req_pcc = ModularArmRequest(
        dof=4,
        kinematics_type="continuum_pcc",
        payload_kg=0.1,
        material_id="pa12_cf",
        slots=ModularArmSlots(
            upper_arm=LinkDesignRequest(length_mm=90.0, root_width_mm=20.0, tip_width_mm=16.0, thickness_mm=10.0),
            forearm=LinkDesignRequest(length_mm=60.0, root_width_mm=18.0, tip_width_mm=14.0, thickness_mm=10.0),
        ),
    )
    res_pcc = evaluate_modular_arm(req_pcc, {})
    assert res_pcc["compatible"] is True
    assert res_pcc["min_safety_factor"] >= 2.0
    codes_p = {i.part_code for i in evaluate_mechatronics(req_pcc, res_pcc).master_bom}
    assert {"CNT-NITINOL-2MM", "CNT-SPACER-PEEK", "CNT-TENDON-3X"}.issubset(codes_p)

    # 3. 4-Bar Parallelogram + Bevel Differential Wrist
    req_mech = ModularArmRequest(
        dof=6,
        kinematics_type="articulated",
        four_bar_parallelogram=True,
        bevel_differential_wrist=True,
        payload_kg=0.6,
        material_id="petg_cf",
    )
    res_m = evaluate_modular_arm(req_mech, {})
    assert res_m["compatible"] is True
    assert res_m["min_safety_factor"] >= 1.5
    codes_m = {i.part_code for i in evaluate_mechatronics(req_mech, res_m).master_bom}
    assert "FBR-PARA-ROD" in codes_m
    assert "BVL-DIFF-SET" in codes_m
