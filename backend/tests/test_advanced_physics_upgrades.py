"""Tests for v10.1 advanced physics upgrades: Coupled RNEA + Stribeck/LuGre, Timoshenko shear, and Electro-Thermal RC + Bus I*R."""

from __future__ import annotations

import math

import pytest

from app.physics.arm_compatibility import evaluate_modular_arm
from app.physics.arm_kinematics import (
    compute_rnea_lugre_torques,
    evaluate_arm_assembly,
    load_arm_presets,
)
from app.physics.mechanics import load_materials
from app.physics.mechatronics import evaluate_electro_thermal_bus, evaluate_mechatronics
from app.schemas import ArmAssemblyRequest, MechatronicsConfig, ModularArmRequest


def test_rnea_lugre_coupled_dynamics_mars_mini_so100_and_ur5e() -> None:
    # Mars-Mini (compact 4-bar servo)
    r_mars = compute_rnea_lugre_torques(
        l_m=(0.105, 0.100, 0.045),
        m_links_kg=(0.06, 0.05, 0.03),
        m_motors_kg=(0.055, 0.055, 0.014),
        payload_kg=0.15,
        q_rad=(0.4, -0.8, 0.2),
        qd_rad_s=(1.8, 2.2, 2.5),
        qdd_rad_s2=(10.0, 8.0, 6.0),
        gear_type="servo",
    )
    # SO-100 (LeRobot 5-DOF serial bus servo)
    r_so100 = compute_rnea_lugre_torques(
        l_m=(0.135, 0.120, 0.060),
        m_links_kg=(0.11, 0.09, 0.05),
        m_motors_kg=(0.08, 0.08, 0.06),
        payload_kg=0.40,
        q_rad=(0.5, -0.9, 0.3),
        qd_rad_s=(1.8, 2.2, 2.5),
        qdd_rad_s2=(12.0, 9.0, 6.0),
        gear_type="servo",
    )
    # UR5e-class cobot (harmonic drive)
    r_ur5e = compute_rnea_lugre_torques(
        l_m=(0.425, 0.392, 0.100),
        m_links_kg=(2.20, 1.60, 0.65),
        m_motors_kg=(1.10, 0.90, 0.50),
        payload_kg=5.0,
        q_rad=(0.4, -0.85, 0.3),
        qd_rad_s=(1.6, 2.0, 2.2),
        qdd_rad_s2=(10.0, 8.0, 6.0),
        gear_type="harmonic",
    )

    for res in (r_mars, r_so100, r_ur5e):
        M = res["mass_matrix"]
        # Mass matrix must be symmetric with non-zero off-diagonal coupling
        assert M[0][1] == pytest.approx(M[1][0], abs=1e-4)
        assert M[0][2] == pytest.approx(M[2][0], abs=1e-4)
        assert abs(M[0][1]) > 0.0
        assert res["tau_coriolis_nm"]["J1"] > 0.0
        assert res["tau_friction_nm"]["J1"] > 0.0
        assert res["tau_total_nm"]["J1"] > res["tau_static_nm"]["J1"]
        assert res["overhead_vs_quasistatic_pct"] > 20.0

    # At identical kinematics/load, harmonic friction > planetary > servo
    f_servo = compute_rnea_lugre_torques(
        (0.25, 0.22, 0.08), (0.3, 0.25, 0.1), (0.3, 0.25, 0.15), 1.0,
        (0.3, -0.7, 0.2), (1.2, 1.5, 1.8), (8.0, 6.0, 4.0), gear_type="servo"
    )["tau_friction_nm"]["J1"]
    f_plan = compute_rnea_lugre_torques(
        (0.25, 0.22, 0.08), (0.3, 0.25, 0.1), (0.3, 0.25, 0.15), 1.0,
        (0.3, -0.7, 0.2), (1.2, 1.5, 1.8), (8.0, 6.0, 4.0), gear_type="planetary"
    )["tau_friction_nm"]["J1"]
    f_harm = compute_rnea_lugre_torques(
        (0.25, 0.22, 0.08), (0.3, 0.25, 0.1), (0.3, 0.25, 0.15), 1.0,
        (0.3, -0.7, 0.2), (1.2, 1.5, 1.8), (8.0, 6.0, 4.0), gear_type="harmonic"
    )["tau_friction_nm"]["J1"]
    assert f_harm > f_plan > f_servo


def test_evaluate_arm_assembly_rnea_alert_and_timoshenko_short_link() -> None:
    presets = load_arm_presets()["presets"]
    materials = load_materials()

    # Mars-Mini has short links (L1=105, L2=100, L3=45 mm -> L/h < 5) so Timoshenko shear must activate
    mars_req = ArmAssemblyRequest(**presets["mars_mini"]["defaults"])
    mars_res = evaluate_arm_assembly(
        mars_req,
        {"base": 40_000.0, "upper_arm": 32_000.0, "forearm": 26_000.0, "wrist": 14_000.0},
        materials["resin_abs_like"],
    )
    assert mars_res["timoshenko_shear_applied"] is True
    assert "rnea_dynamics" in mars_res
    assert mars_res["rnea_dynamics"]["overhead_vs_quasistatic_pct"] > 20.0
    alert_codes = {a.code for a in mars_res["global_alerts"]}
    assert "ARM_RNEA_LUGRE_OVERHEAD" in alert_codes
    assert mars_res["joints"][0].dynamic_torque_nm > mars_res["joints"][0].static_torque_nm


def test_electro_thermal_bus_voltage_droop_and_rc_stall_model() -> None:
    # Severe droop scenario: 5V bus, 12A peak, thin 22 AWG over 1.5 m
    bad_bus = evaluate_electro_thermal_bus(
        bus_voltage_v=5.0,
        peak_current_a=12.0,
        continuous_current_a=5.0,
        awg="22 AWG",
        cable_length_m=1.5,
        actuator_class="rc_servo",
    )
    assert bad_bus["bus_voltage_droop_pct"] > 5.0
    assert bad_bus["estimated_stall_temp_30s_c"] > 85.0
    assert bad_bus["safe_stall_time_s"] < 30.0

    # Healthy high-voltage CAN-FD bus: 48V, 14A peak, 14 AWG over 1.2 m
    good_bus = evaluate_electro_thermal_bus(
        bus_voltage_v=48.0,
        peak_current_a=14.0,
        continuous_current_a=8.0,
        awg="14 AWG",
        cable_length_m=1.2,
        actuator_class="bldc_qdd",
    )
    assert good_bus["bus_voltage_droop_pct"] < 1.0
    assert good_bus["safe_stall_time_s"] > bad_bus["safe_stall_time_s"]


def test_evaluate_mechatronics_populates_electro_thermal_fields_and_warnings() -> None:
    # 1. Arduino PCA9685 (6.5V high-current RC servos -> RC stall warning expected)
    req_ard = ModularArmRequest(
        dof=6,
        payload_kg=0.5,
        mechatronics=MechatronicsConfig(bus_architecture="arduino_pca9685_i2c", system_voltage_v=6.5),
    )
    m_ard = evaluate_mechatronics(req_ard, evaluate_modular_arm(req_ard, {}))
    assert m_ard.power_budget.bus_voltage_drop_v > 0.0
    assert m_ard.power_budget.bus_voltage_droop_pct > 0.0
    assert m_ard.power_budget.estimated_stall_temp_30s_c > 85.0
    assert m_ard.power_budget.safe_stall_time_s < 30.0
    assert any("Riesgo térmico en bloqueo" in n for n in m_ard.notes)

    # 2. ESP32 Mars-Mini (5V 5A)
    req_esp = ModularArmRequest(
        dof=4,
        payload_kg=0.15,
        material_id="resin_abs_like",
        mechatronics=MechatronicsConfig(bus_architecture="esp32_pca9685_5v", system_voltage_v=5.0),
    )
    m_esp = evaluate_mechatronics(req_esp, evaluate_modular_arm(req_esp, {}))
    assert m_esp.power_budget.bus_voltage_drop_v > 0.0
    assert m_esp.power_budget.bus_voltage_droop_pct < 5.0
    assert m_esp.power_budget.estimated_stall_temp_30s_c > 25.0

    # 3. CAN-FD 48V (very low droop %)
    req_can = ModularArmRequest(
        dof=6,
        payload_kg=3.5,
        mechatronics=MechatronicsConfig(bus_architecture="can_fd_bus", system_voltage_v=48.0),
    )
    m_can = evaluate_mechatronics(req_can, evaluate_modular_arm(req_can, {}))
    assert m_can.power_budget.bus_voltage_droop_pct < 2.0
    assert m_can.power_budget.estimated_stall_temp_30s_c > 25.0
