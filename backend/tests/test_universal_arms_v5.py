from __future__ import annotations

import json
import xml.etree.ElementTree as ET

import pytest

from app.physics.arm_compatibility import evaluate_modular_arm
from app.physics.arm_kinematics import load_arm_presets
from app.physics.mechatronics import (
    ACTUATOR_CATALOG,
    evaluate_mechatronics,
    generate_circuit_schematic_svg,
    generate_lerobot_config_json,
    solve_belt_center_distance,
)
from app.schemas import (
    ArmAssemblyRequest,
    JointActuatorSpec,
    MechatronicsConfig,
    ModularArmRequest,
)


def test_v5_arm_presets_and_7dof_schema():
    presets = load_arm_presets()["presets"]
    for key in ("pybot_scara", "openarm_7dof", "arctos_6dof", "bcn3d_moveo"):
        assert key in presets
        req = ArmAssemblyRequest(**presets[key]["defaults"])
        assert req.preset == key

    assert presets["openarm_7dof"]["defaults"]["dof"] == 7
    assert presets["pybot_scara"]["defaults"]["dof"] == 4


def test_openarm_7dof_damiao_qdd_and_cnc_plates():
    for aid in ("servo_sts3215", "damiao_dm_j4310", "damiao_dm_j4340", "nema14_34mm"):
        assert aid in ACTUATOR_CATALOG

    req = ModularArmRequest(
        dof=7,
        kinematics_type="articulated",
        wrist_3dof=True,
        cnc_plate_mode=True,
        payload_kg=4.0,
        material_id="pa12_cf",
        mechatronics=MechatronicsConfig(
            bus_architecture="can_fd_bus",
            system_voltage_v=24.0,
            joint_actuators={
                "J1": JointActuatorSpec(motor_id="damiao_dm_j4340", reducer_id="direct", target_rpm=30.0),
                "J2": JointActuatorSpec(motor_id="damiao_dm_j4310", reducer_id="direct", target_rpm=35.0),
            },
        ),
    )
    eval_res = evaluate_modular_arm(req, {})
    mech = evaluate_mechatronics(req, eval_res)

    assert len(mech.joint_reports) == 7
    assert mech.joint_reports[0].motor_id == "damiao_dm_j4340"
    assert mech.joint_reports[6].joint_id == "J7"

    codes = {i.part_code for i in mech.master_bom}
    assert "CNC-AL6061-PLT" in codes
    assert "BRG-CRB-RU42" in codes
    assert "BRG-TAP-30205" in codes


def test_scara_and_teensy_dm542_and_uart_lerobot():
    # 1. SCARA + Teensy DM542T
    req_scara = ModularArmRequest(
        dof=4,
        kinematics_type="scara",
        z_stroke_mm=150.0,
        payload_kg=0.8,
        material_id="petg_cf",
        mechatronics=MechatronicsConfig(bus_architecture="teensy_dm542", system_voltage_v=24.0),
    )
    mech_scara = evaluate_mechatronics(req_scara, evaluate_modular_arm(req_scara, {}))
    codes_s = {i.part_code for i in mech_scara.master_bom}
    assert "LIN-MGN12H" in codes_s
    assert "LIN-T8-LEAD" in codes_s
    assert "EL-TEENSY41" in codes_s
    assert "EL-DRV-DM542T" in codes_s
    assert "EL-SENS-HOMING" in codes_s
    svg_t = generate_circuit_schematic_svg(mech_scara, 4)
    assert "TEENSY 4.1" in svg_t
    assert ET.fromstring(svg_t).tag.endswith("svg")

    # 2. UART Serial Bus (STS3215 / LeRobot)
    req_uart = ModularArmRequest(
        dof=6,
        payload_kg=0.4,
        material_id="pla_plus",
        mechatronics=MechatronicsConfig(bus_architecture="uart_serial_bus", system_voltage_v=7.4),
    )
    mech_uart = evaluate_mechatronics(req_uart, evaluate_modular_arm(req_uart, {}))
    assert mech_uart.power_budget.system_voltage_v == pytest.approx(7.4)
    assert mech_uart.joint_reports[0].motor_id == "servo_sts3215"
    assert mech_uart.joint_reports[0].motor_type == "serial_bus_servo"
    codes_u = {i.part_code for i in mech_uart.master_bom}
    assert "EL-WS-TTL" in codes_u
    svg_u = generate_circuit_schematic_svg(mech_uart, 6)
    assert "WAVESHARE USB-TTL" in svg_u
    assert ET.fromstring(svg_u).tag.endswith("svg")

    # LeRobot calibrated JSON
    lerobot_raw = generate_lerobot_config_json(req_uart)
    lerobot_cfg = json.loads(lerobot_raw)
    assert lerobot_cfg["baudrate"] == 1000000
    assert lerobot_cfg["motors"]["shoulder_pan"]["model"] == "sts3215"
    assert len(lerobot_cfg["motors"]) == 6


def test_solve_belt_center_distance_gt2():
    res = solve_belt_center_distance(z1=20, z2=40, pitch_mm=2.0, belt_teeth=200)
    assert res["valid"] is True
    assert res["belt_length_mm"] == pytest.approx(400.0)
    assert res["gear_ratio"] == pytest.approx(2.0)
    assert 168.0 <= res["center_distance_mm"] <= 171.0
