from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest

from app.physics.arm_compatibility import evaluate_modular_arm
from app.physics.arm_kinematics import load_arm_presets
from app.physics.mechatronics import (
    ACTUATOR_CATALOG,
    evaluate_mechatronics,
    generate_circuit_schematic_svg,
    generate_wireviz_yaml,
)
from app.schemas import (
    ArmAssemblyRequest,
    JointActuatorSpec,
    MechatronicsConfig,
    ModularArmRequest,
)


def test_fabri_arm_5_preset_exists_and_validates():
    presets = load_arm_presets()["presets"]
    assert "fabri_arm_5" in presets
    entry = presets["fabri_arm_5"]
    req = ArmAssemblyRequest(**entry["defaults"])
    assert req.preset == "fabri_arm_5"
    assert req.upper_arm_mm + req.forearm_mm + req.wrist_mm == pytest.approx(400.0)


def test_dual_parallel_servo_doubles_torque_and_current():
    assert "servo_td8120mg" in ACTUATOR_CATALOG
    assert "servo_ds3218" in ACTUATOR_CATALOG

    req_1x = ModularArmRequest(
        dof=6,
        payload_kg=0.35,
        material_id="pla_plus",
        mechatronics=MechatronicsConfig(
            bus_architecture="arduino_pca9685_i2c",
            system_voltage_v=6.5,
            joint_actuators={
                "J1": JointActuatorSpec(
                    motor_id="servo_td8120mg",
                    reducer_id="direct",
                    target_rpm=25.0,
                    parallel_count=1,
                ),
            },
        ),
    )
    req_2x = req_1x.model_copy(
        update={
            "mechatronics": MechatronicsConfig(
                bus_architecture="arduino_pca9685_i2c",
                system_voltage_v=6.5,
                joint_actuators={
                    "J1": JointActuatorSpec(
                        motor_id="servo_td8120mg",
                        reducer_id="direct",
                        target_rpm=25.0,
                        parallel_count=2,
                    ),
                },
            )
        }
    )

    eval_res = evaluate_modular_arm(req_1x, {})
    m1 = evaluate_mechatronics(req_1x, eval_res)
    m2 = evaluate_mechatronics(req_2x, eval_res)

    j1_1x = m1.joint_reports[0]
    j1_2x = m2.joint_reports[0]
    assert j1_1x.parallel_count == 1
    assert j1_2x.parallel_count == 2
    assert j1_2x.available_peak_torque_nm == pytest.approx(
        j1_1x.available_peak_torque_nm * 2.0, rel=0.02
    )
    assert j1_2x.available_continuous_torque_nm == pytest.approx(
        j1_1x.available_continuous_torque_nm * 2.0, rel=0.02
    )
    assert j1_2x.torque_speed_curve[0].peak_torque_nm == pytest.approx(
        2.15 * 2 * 0.98, abs=0.05
    )


def test_arduino_pca9685_i2c_power_budget_bom_and_schematics():
    req = ModularArmRequest(
        dof=6,
        payload_kg=0.35,
        material_id="pla_plus",
        mechatronics=MechatronicsConfig(
            bus_architecture="arduino_pca9685_i2c",
            system_voltage_v=6.5,
            joint_actuators={
                "J1": JointActuatorSpec(
                    motor_id="servo_td8120mg", reducer_id="direct", target_rpm=30.0, parallel_count=2
                ),
                "J2": JointActuatorSpec(
                    motor_id="servo_td8120mg", reducer_id="direct", target_rpm=30.0, parallel_count=2
                ),
                "J3": JointActuatorSpec(
                    motor_id="servo_td8120mg", reducer_id="direct", target_rpm=35.0, parallel_count=1
                ),
            },
        ),
    )
    eval_res = evaluate_modular_arm(req, {})
    mech = evaluate_mechatronics(req, eval_res)

    assert mech.bus_architecture == "arduino_pca9685_i2c"
    assert mech.power_budget.system_voltage_v == pytest.approx(6.5)
    assert "3–12V 10A" in mech.power_budget.recommended_psu
    assert "22 AWG" in mech.power_budget.signal_wire_awg

    bom_by_code = {item.part_code: item for item in mech.master_bom}
    assert bom_by_code["SRV-SERVO_TD81"].qty == 8
    assert bom_by_code["HRN-ALU-25T"].qty == 8
    assert bom_by_code["EL-ARDUINO-UNO"].qty == 1
    assert bom_by_code["EL-PCA9685-16CH"].qty == 1
    assert bom_by_code["EL-PSU-10A"].qty == 1
    assert bom_by_code["EL-SERVO-EXT"].qty == 5
    assert bom_by_code["ISO4762-M3x8"].qty == 35
    assert bom_by_code["DIN7981-M3x6"].qty == 5
    assert bom_by_code["DIN7981-M4x8"].qty == 10
    assert bom_by_code["DIN7981-M4x12"].qty == 70
    # Direct servo architecture has no external metallic bearings
    assert "BRG-6808-2RS" not in bom_by_code
    assert "BRG-608-2RS" not in bom_by_code

    svg = generate_circuit_schematic_svg(mech, req.dof)
    assert "PCA9685" in svg and "ARDUINO UNO" in svg and "CH7" in svg
    root = ET.fromstring(svg)
    assert root.tag.endswith("svg")

    yml = generate_wireviz_yaml(mech, req.dof)
    assert "X3_PCA9685:" in yml and "X2_ARDUINO_UNO:" in yml and "S_CH7_Gripper:" in yml
