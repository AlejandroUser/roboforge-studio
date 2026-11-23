from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest

from app.physics.arm_compatibility import evaluate_modular_arm
from app.physics.mechatronics import (
    ACTUATOR_CATALOG,
    REDUCER_CATALOG,
    _motor_torque_at_rpm,
    evaluate_mechatronics,
    generate_circuit_schematic_svg,
    generate_wireviz_yaml,
    get_mechatronics_catalog,
    master_bom_to_csv,
)
from app.schemas import JointActuatorSpec, MechatronicsConfig, ModularArmRequest


def test_mechatronics_catalog_complete():
    cat = get_mechatronics_catalog()
    assert set(ACTUATOR_CATALOG.keys()) == {
        "nema17_48mm",
        "nema17_mks_servo42d",
        "nema23_76mm",
        "nema23_mks_servo57d",
        "bldc_qdd_5010",
        "bldc_qdd_8308",
        "servo_td8120mg",
        "servo_ds3218",
        "servo_sts3215",
        "damiao_dm_j4310",
        "damiao_dm_j4340",
        "nema14_34mm",
    }
    assert set(REDUCER_CATALOG.keys()) == {
        "direct",
        "gt2_belt_3",
        "planetary_10",
        "planetary_20",
        "cycloidal_25",
        "cycloidal_40",
        "harmonic_50",
    }
    assert len(cat["actuators"]) == 12
    assert len(cat["reducers"]) == 7
    assert len(cat["architectures"]) == 5


def test_torque_speed_curve_bldc_flat_vs_stepper_dropoff():
    bldc = ACTUATOR_CATALOG["bldc_qdd_8308"]
    stepper = ACTUATOR_CATALOG["nema23_76mm"]

    # BLDC stays flat from 0 up to corner_rpm
    t0_b, p0_b = _motor_torque_at_rpm(bldc, 0.0)
    tc_b, pc_b = _motor_torque_at_rpm(bldc, bldc["corner_rpm"] * 0.9)
    assert tc_b == pytest.approx(t0_b, rel=1e-6)
    assert pc_b == pytest.approx(p0_b, rel=1e-6)

    # Stepper drops significantly by 60% of no_load_rpm
    t0_s, _ = _motor_torque_at_rpm(stepper, 0.0)
    tmid_s, _ = _motor_torque_at_rpm(stepper, stepper["no_load_rpm"] * 0.6)
    assert tmid_s < t0_s * 0.45


def test_evaluate_mechatronics_power_budget_bom_svg_yaml_csv():
    req_can = ModularArmRequest(
        dof=6,
        payload_kg=1.5,
        material_id="petg_cf",
        mechatronics=MechatronicsConfig(
            bus_architecture="can_fd_bus",
            system_voltage_v=24.0,
            joint_actuators={
                "J1": JointActuatorSpec(
                    motor_id="bldc_qdd_8308",
                    reducer_id="cycloidal_25",
                    target_rpm=25.0,
                ),
                "J2": JointActuatorSpec(
                    motor_id="nema17_mks_servo42d",
                    reducer_id="planetary_20",
                    target_rpm=30.0,
                ),
            },
        ),
    )
    eval_res = evaluate_modular_arm(req_can, {})
    mech = evaluate_mechatronics(req_can, eval_res)

    assert len(mech.joint_reports) == 6
    assert mech.joint_reports[0].motor_id == "bldc_qdd_8308"
    assert len(mech.joint_reports[0].torque_speed_curve) == 13
    # bldc_qdd_8308 is 48V -> system voltage escalates to 48V
    assert mech.power_budget.system_voltage_v == 48.0
    assert mech.power_budget.peak_power_w > mech.power_budget.continuous_power_w > 0
    assert "Mean Well" in mech.power_budget.recommended_psu
    assert "AWG" in mech.power_budget.power_wire_awg
    assert mech.power_budget.total_harness_length_m > 0

    # All 4 Master BOM categories present + ISO 4762 screws + bearings
    categories = {item.category for item in mech.master_bom}
    assert categories == {
        "printed_cnc",
        "actuators_reducers",
        "electronics_wiring",
        "fasteners_hardware",
    }
    codes = {item.part_code for item in mech.master_bom}
    assert "ISO4762-M3x12" in codes
    assert "BRG-6808-2RS" in codes
    assert "BRG-608-2RS" in codes
    assert mech.total_bom_cost_usd == pytest.approx(
        sum(i.total_cost_usd for i in mech.master_bom), abs=0.05
    )

    # Compare harness length: Star Step/Dir uses more cable than CAN-FD Daisy-Chain
    req_star = req_can.model_copy(
        update={
            "mechatronics": MechatronicsConfig(
                bus_architecture="stm32_step_dir",
                system_voltage_v=24.0,
            )
        }
    )
    mech_star = evaluate_mechatronics(req_star, eval_res)
    assert mech_star.power_budget.total_harness_length_m > mech.power_budget.total_harness_length_m

    # SVG schematic is well-formed XML
    svg_str = generate_circuit_schematic_svg(mech, req_can.dof)
    assert svg_str.startswith("<svg") and svg_str.endswith("</svg>")
    root = ET.fromstring(svg_str)
    assert root.tag.endswith("svg")

    # WireViz YAML & Master BOM CSV
    yml_str = generate_wireviz_yaml(mech, req_can.dof)
    assert "connectors:" in yml_str and "cables:" in yml_str and "connections:" in yml_str
    csv_str = master_bom_to_csv(mech.master_bom)
    assert "ISO4762-M3x12" in csv_str
    assert len(csv_str.strip().splitlines()) == len(mech.master_bom) + 1
