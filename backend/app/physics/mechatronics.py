from __future__ import annotations

import csv
import io
import json
import math
from html import escape

from ..schemas import (
    JointActuatorSpec,
    JointMechatronicsReport,
    MasterBomItem,
    MechatronicsConfig,
    MechatronicsResponse,
    ModularArmRequest,
    PowerBudget,
    TorqueSpeedPoint,
)
from .arm_compatibility import resolve_default_slots
from .mechanics import load_materials

ACTUATOR_CATALOG: dict[str, dict] = {
    "nema17_48mm": {
        "id": "nema17_48mm",
        "name": "NEMA 17 48 mm (17HS19-2004S)",
        "motor_type": "stepper",
        "voltage_v": 24.0,
        "rated_torque_nm": 0.48,
        "peak_torque_nm": 0.59,
        "no_load_rpm": 650.0,
        "corner_rpm": 140.0,
        "rated_current_a": 1.7,
        "peak_current_a": 2.0,
        "mass_g": 360.0,
        "shaft_d_mm": 5.0,
        "bolt_pcd_mm": 31.0,
        "bolt_size": "M3",
        "unit_cost_usd": 14.5,
        "bus_supported": ["stm32_step_dir"],
    },
    "nema17_mks_servo42d": {
        "id": "nema17_mks_servo42d",
        "name": "NEMA 17 MKS SERVO42D (Closed-Loop CAN)",
        "motor_type": "closed_loop_stepper",
        "voltage_v": 24.0,
        "rated_torque_nm": 0.52,
        "peak_torque_nm": 0.68,
        "no_load_rpm": 900.0,
        "corner_rpm": 220.0,
        "rated_current_a": 1.8,
        "peak_current_a": 2.4,
        "mass_g": 410.0,
        "shaft_d_mm": 5.0,
        "bolt_pcd_mm": 31.0,
        "bolt_size": "M3",
        "unit_cost_usd": 36.0,
        "bus_supported": ["can_fd_bus", "stm32_step_dir"],
    },
    "nema23_76mm": {
        "id": "nema23_76mm",
        "name": "NEMA 23 76 mm (23HS30-2804S)",
        "motor_type": "stepper",
        "voltage_v": 24.0,
        "rated_torque_nm": 1.55,
        "peak_torque_nm": 1.90,
        "no_load_rpm": 550.0,
        "corner_rpm": 120.0,
        "rated_current_a": 2.4,
        "peak_current_a": 2.8,
        "mass_g": 1050.0,
        "shaft_d_mm": 6.35,
        "bolt_pcd_mm": 66.7,
        "bolt_size": "M5",
        "unit_cost_usd": 32.0,
        "bus_supported": ["stm32_step_dir"],
    },
    "nema23_mks_servo57d": {
        "id": "nema23_mks_servo57d",
        "name": "NEMA 23 MKS SERVO57D (Closed-Loop CAN)",
        "motor_type": "closed_loop_stepper",
        "voltage_v": 24.0,
        "rated_torque_nm": 1.75,
        "peak_torque_nm": 2.30,
        "no_load_rpm": 850.0,
        "corner_rpm": 210.0,
        "rated_current_a": 2.8,
        "peak_current_a": 3.8,
        "mass_g": 1150.0,
        "shaft_d_mm": 6.35,
        "bolt_pcd_mm": 66.7,
        "bolt_size": "M5",
        "unit_cost_usd": 62.0,
        "bus_supported": ["can_fd_bus", "stm32_step_dir"],
    },
    "bldc_qdd_5010": {
        "id": "bldc_qdd_5010",
        "name": "BLDC QDD 5010 Pancake (FOC + AS5048A)",
        "motor_type": "bldc_qdd",
        "voltage_v": 24.0,
        "rated_torque_nm": 0.65,
        "peak_torque_nm": 1.45,
        "no_load_rpm": 1800.0,
        "corner_rpm": 1150.0,
        "rated_current_a": 3.2,
        "peak_current_a": 7.5,
        "mass_g": 195.0,
        "shaft_d_mm": 8.0,
        "bolt_pcd_mm": 38.0,
        "bolt_size": "M3",
        "unit_cost_usd": 78.0,
        "bus_supported": ["can_fd_bus"],
    },
    "bldc_qdd_8308": {
        "id": "bldc_qdd_8308",
        "name": "BLDC QDD 8308 High-Torque (FOC CAN-FD)",
        "motor_type": "bldc_qdd",
        "voltage_v": 48.0,
        "rated_torque_nm": 1.90,
        "peak_torque_nm": 4.20,
        "no_load_rpm": 1600.0,
        "corner_rpm": 1050.0,
        "rated_current_a": 5.0,
        "peak_current_a": 11.5,
        "mass_g": 460.0,
        "shaft_d_mm": 12.0,
        "bolt_pcd_mm": 66.7,
        "bolt_size": "M4",
        "unit_cost_usd": 148.0,
        "bus_supported": ["can_fd_bus"],
    },
    "servo_td8120mg": {
        "id": "servo_td8120mg",
        "name": "TD-8120MG 20kg·cm (180° Metal Gear)",
        "motor_type": "rc_servo",
        "voltage_v": 6.8,
        "rated_torque_nm": 1.65,
        "peak_torque_nm": 2.15,
        "no_load_rpm": 62.0,
        "corner_rpm": 38.0,
        "rated_current_a": 1.1,
        "peak_current_a": 2.5,
        "mass_g": 60.0,
        "shaft_d_mm": 6.0,
        "bolt_pcd_mm": 16.0,
        "bolt_size": "M3",
        "unit_cost_usd": 9.5,
        "bus_supported": ["arduino_pca9685_i2c"],
    },
    "servo_ds3218": {
        "id": "servo_ds3218",
        "name": "DS3218MG 20kg·cm (Digital Metal Gear)",
        "motor_type": "rc_servo",
        "voltage_v": 6.8,
        "rated_torque_nm": 1.65,
        "peak_torque_nm": 2.15,
        "no_load_rpm": 62.0,
        "corner_rpm": 38.0,
        "rated_current_a": 1.1,
        "peak_current_a": 2.5,
        "mass_g": 60.0,
        "shaft_d_mm": 6.0,
        "bolt_pcd_mm": 16.0,
        "bolt_size": "M3",
        "unit_cost_usd": 12.0,
        "bus_supported": ["arduino_pca9685_i2c"],
    },
    "servo_sts3215": {
        "id": "servo_sts3215",
        "name": "Feetech STS3215 30kg·cm (Bus Serie UART)",
        "motor_type": "serial_bus_servo",
        "voltage_v": 7.4,
        "rated_torque_nm": 1.45,
        "peak_torque_nm": 2.94,
        "no_load_rpm": 50.0,
        "corner_rpm": 32.0,
        "rated_current_a": 0.9,
        "peak_current_a": 2.7,
        "mass_g": 55.0,
        "shaft_d_mm": 6.0,
        "bolt_pcd_mm": 16.0,
        "bolt_size": "M3",
        "unit_cost_usd": 18.0,
        "bus_supported": ["uart_serial_bus"],
    },
    "damiao_dm_j4310": {
        "id": "damiao_dm_j4310",
        "name": "Damiao DM-J4310 QDD (FOC CAN-FD)",
        "motor_type": "bldc_qdd",
        "voltage_v": 24.0,
        "rated_torque_nm": 3.0,
        "peak_torque_nm": 7.0,
        "no_load_rpm": 200.0,
        "corner_rpm": 130.0,
        "rated_current_a": 2.5,
        "peak_current_a": 7.5,
        "mass_g": 305.0,
        "shaft_d_mm": 10.0,
        "bolt_pcd_mm": 44.0,
        "bolt_size": "M3",
        "unit_cost_usd": 85.0,
        "bus_supported": ["can_fd_bus"],
    },
    "damiao_dm_j4340": {
        "id": "damiao_dm_j4340",
        "name": "Damiao DM-J4340 High-Torque QDD (CAN-FD)",
        "motor_type": "bldc_qdd",
        "voltage_v": 24.0,
        "rated_torque_nm": 9.0,
        "peak_torque_nm": 27.0,
        "no_load_rpm": 95.0,
        "corner_rpm": 60.0,
        "rated_current_a": 4.0,
        "peak_current_a": 12.0,
        "mass_g": 465.0,
        "shaft_d_mm": 14.0,
        "bolt_pcd_mm": 56.0,
        "bolt_size": "M4",
        "unit_cost_usd": 135.0,
        "bus_supported": ["can_fd_bus"],
    },
    "nema14_34mm": {
        "id": "nema14_34mm",
        "name": "NEMA 14 34 mm (Muñeca Compacta)",
        "motor_type": "stepper",
        "voltage_v": 24.0,
        "rated_torque_nm": 0.26,
        "peak_torque_nm": 0.32,
        "no_load_rpm": 720.0,
        "corner_rpm": 160.0,
        "rated_current_a": 1.0,
        "peak_current_a": 1.3,
        "mass_g": 180.0,
        "shaft_d_mm": 5.0,
        "bolt_pcd_mm": 26.0,
        "bolt_size": "M3",
        "unit_cost_usd": 14.0,
        "bus_supported": ["stm32_step_dir", "teensy_dm542"],
    },
    "servo_mg996r": {
        "id": "servo_mg996r",
        "name": "MG996R 10kg·cm (Metal Gear Económico)",
        "motor_type": "rc_servo",
        "voltage_v": 5.0,
        "rated_torque_nm": 0.68,
        "peak_torque_nm": 0.98,
        "no_load_rpm": 58.0,
        "corner_rpm": 35.0,
        "rated_current_a": 0.75,
        "peak_current_a": 1.4,
        "mass_g": 55.0,
        "shaft_d_mm": 6.0,
        "bolt_pcd_mm": 16.0,
        "bolt_size": "M3",
        "unit_cost_usd": 3.5,
        "bus_supported": ["esp32_pca9685_5v", "arduino_pca9685_i2c"],
    },
    "servo_mg90s": {
        "id": "servo_mg90s",
        "name": "MG90S Micro-Servo 2.2kg·cm (Pinza / Muñeca)",
        "motor_type": "rc_servo",
        "voltage_v": 5.0,
        "rated_torque_nm": 0.15,
        "peak_torque_nm": 0.22,
        "no_load_rpm": 95.0,
        "corner_rpm": 55.0,
        "rated_current_a": 0.25,
        "peak_current_a": 0.55,
        "mass_g": 13.4,
        "shaft_d_mm": 4.8,
        "bolt_pcd_mm": 12.0,
        "bolt_size": "M2",
        "unit_cost_usd": 2.0,
        "bus_supported": ["esp32_pca9685_5v", "arduino_pca9685_i2c"],
    },
}

REDUCER_CATALOG: dict[str, dict] = {
    "direct": {
        "id": "direct",
        "name": "Directo (1:1)",
        "ratio": 1.0,
        "efficiency": 0.98,
        "backlash_arcmin": 1.0,
        "mass_g": 0.0,
        "unit_cost_usd": 0.0,
    },
    "gt2_belt_3": {
        "id": "gt2_belt_3",
        "name": "Correa GT2 (3:1)",
        "ratio": 3.0,
        "efficiency": 0.94,
        "backlash_arcmin": 8.0,
        "mass_g": 45.0,
        "unit_cost_usd": 12.0,
    },
    "planetary_10": {
        "id": "planetary_10",
        "name": "Planetario (10:1)",
        "ratio": 10.0,
        "efficiency": 0.88,
        "backlash_arcmin": 12.0,
        "mass_g": 180.0,
        "unit_cost_usd": 34.0,
    },
    "planetary_20": {
        "id": "planetary_20",
        "name": "Planetario (20:1)",
        "ratio": 20.0,
        "efficiency": 0.85,
        "backlash_arcmin": 15.0,
        "mass_g": 260.0,
        "unit_cost_usd": 46.0,
    },
    "cycloidal_25": {
        "id": "cycloidal_25",
        "name": "Cicloidal (25:1)",
        "ratio": 25.0,
        "efficiency": 0.78,
        "backlash_arcmin": 4.0,
        "mass_g": 240.0,
        "unit_cost_usd": 52.0,
    },
    "cycloidal_40": {
        "id": "cycloidal_40",
        "name": "Cicloidal (40:1)",
        "ratio": 40.0,
        "efficiency": 0.74,
        "backlash_arcmin": 3.5,
        "mass_g": 310.0,
        "unit_cost_usd": 68.0,
    },
    "harmonic_50": {
        "id": "harmonic_50",
        "name": "Armónico Strain-Wave (50:1)",
        "ratio": 50.0,
        "efficiency": 0.80,
        "backlash_arcmin": 1.0,
        "mass_g": 380.0,
        "unit_cost_usd": 165.0,
    },
}

ARCHITECTURES_CATALOG: list[dict] = [
    {
        "id": "can_fd_bus",
        "name": "Bus CAN-FD 4 Hilos (Daisy-Chain)",
        "description": "STM32H7 / ESP32-CAN + 2 hilos potencia + par trenzado 120Ω CAN-H/L hacia motores inteligentes (MKS SERVO / BLDC QDD).",
    },
    {
        "id": "stm32_step_dir",
        "name": "Estrella STM32 Step/Dir (TMC2209)",
        "description": "Controlador STM32F4 + drivers TMC2209 en caja base con cableado punto a punto de 4 hilos por motor stepper.",
    },
    {
        "id": "arduino_pca9685_i2c",
        "name": "Arduino UNO + PCA9685 I2C (6.5V / 10A)",
        "description": "Controlador Arduino UNO + Servo Shield PCA9685 I2C de 16 canales para 8× servos TD-8120MG (doble servo paralelo en hombro y codo) + garra.",
    },
    {
        "id": "uart_serial_bus",
        "name": "Bus Serie UART 1 Mbps (STS3215 / LeRobot)",
        "description": "Bus TTL half-duplex 3 hilos en cadena (Waveshare USB-TTL) para servos inteligentes Feetech STS3215 con retroalimentación de posición y carga.",
    },
    {
        "id": "teensy_dm542",
        "name": "Teensy 4.1 + Drivers DM542T (Industrial AR4)",
        "description": "Controlador Teensy 4.1 + drivers externos DM542T 4.2A, encoders ópticos 1000 PPR y sensores inductivos de homing (arquitectura AR4-MK3).",
    },
    {
        "id": "esp32_pca9685_5v",
        "name": "ESP32 Wi-Fi + PCA9685 (5V 5A · Mars-Mini)",
        "description": "Controlador ESP32 con Web Server Wi-Fi embebido + PCA9685 I2C + Fuente 5V 5A (25W) para 3× MG996R + 1× MG90S (< $45 USD).",
    },
]

CATEGORY_LABELS: dict[str, str] = {
    "printed_cnc": "Piezas Impresas / CNC",
    "actuators_reducers": "Motores y Reductores",
    "electronics_wiring": "Electrónica y Cableado",
    "fasteners_hardware": "Tornillería ISO y Rodamientos",
}


def get_mechatronics_catalog() -> dict:
    """Return full actuator, reducer, and electrical architecture catalog."""
    return {
        "actuators": list(ACTUATOR_CATALOG.values()),
        "reducers": list(REDUCER_CATALOG.values()),
        "architectures": ARCHITECTURES_CATALOG,
    }


def _motor_torque_at_rpm(motor: dict, motor_rpm: float) -> tuple[float, float]:
    """
    Compute (continuous_torque_nm, peak_torque_nm) at the motor shaft for a given
    motor_rpm:
    - BLDC QDD: flat constant-torque plateau up to corner_rpm, then linear roll-off
      to 0 at no_load_rpm.
    - Stepper / Closed-Loop Stepper: inductive/back-EMF roll-off above corner_rpm
      (steepest on open-loop steppers).
    """
    rpm = max(0.0, float(motor_rpm))
    no_load = float(motor["no_load_rpm"])
    corner = float(motor["corner_rpm"])
    t_rated = float(motor["rated_torque_nm"])
    t_peak = float(motor["peak_torque_nm"])
    m_type = motor["motor_type"]

    if rpm >= no_load:
        return 0.0, 0.0

    if m_type in ("bldc_qdd", "rc_servo", "serial_bus_servo"):
        if rpm <= corner:
            return t_rated, t_peak
        frac = max(0.0, (no_load - rpm) / max(no_load - corner, 1.0))
        return t_rated * frac, t_peak * frac

    # Stepper / closed-loop stepper inductive roll-off
    if rpm <= corner * 0.6:
        return t_rated, t_peak

    span = max(no_load - corner * 0.6, 1.0)
    u = (rpm - corner * 0.6) / span
    exponent = 1.45 if m_type == "closed_loop_stepper" else 1.85
    attenuation = max(0.0, (1.0 - u) / (1.0 + (2.2 if m_type == "stepper" else 1.4) * (u**exponent)))
    return t_rated * attenuation, t_peak * attenuation


def _default_spec_for_joint(
    joint_idx: int,
    required_peak_nm: float,
    bus_arch: str,
) -> JointActuatorSpec:
    """Choose sensible default motor + reducer for joint J1..J6."""
    if bus_arch == "esp32_pca9685_5v":
        if joint_idx <= 3:
            return JointActuatorSpec(
                motor_id="servo_mg996r",
                reducer_id="direct",
                target_rpm=32.0,
                parallel_count=1,
            )
        return JointActuatorSpec(
            motor_id="servo_mg90s",
            reducer_id="direct",
            target_rpm=45.0,
            parallel_count=1,
        )
    if bus_arch == "uart_serial_bus":
        return JointActuatorSpec(
            motor_id="servo_sts3215",
            reducer_id="direct",
            target_rpm=32.0,
            parallel_count=1,
        )
    if bus_arch == "teensy_dm542":
        if joint_idx == 1:
            return JointActuatorSpec(
                motor_id="nema23_76mm", reducer_id="planetary_20", target_rpm=25.0
            )
        if joint_idx == 2:
            return JointActuatorSpec(
                motor_id="nema17_48mm", reducer_id="planetary_10", target_rpm=30.0
            )
        return JointActuatorSpec(
            motor_id="nema14_34mm", reducer_id="gt2_belt_3", target_rpm=40.0
        )
    if bus_arch == "arduino_pca9685_i2c":
        if joint_idx in (1, 2):
            return JointActuatorSpec(
                motor_id="servo_td8120mg",
                reducer_id="direct",
                target_rpm=30.0,
                parallel_count=2,
            )
        return JointActuatorSpec(
            motor_id="servo_td8120mg",
            reducer_id="direct",
            target_rpm=38.0,
            parallel_count=1,
        )
    if joint_idx == 1:
        if required_peak_nm > 18.0:
            return JointActuatorSpec(
                motor_id="bldc_qdd_8308" if bus_arch == "can_fd_bus" else "nema23_mks_servo57d",
                reducer_id="cycloidal_25",
                target_rpm=25.0,
            )
        return JointActuatorSpec(
            motor_id="nema23_mks_servo57d" if bus_arch == "can_fd_bus" else "nema23_76mm",
            reducer_id="planetary_20",
            target_rpm=28.0,
        )
    if joint_idx == 2:
        return JointActuatorSpec(
            motor_id="nema17_mks_servo42d" if bus_arch == "can_fd_bus" else "nema17_48mm",
            reducer_id="planetary_20",
            target_rpm=32.0,
        )
    return JointActuatorSpec(
        motor_id="nema17_mks_servo42d" if bus_arch == "can_fd_bus" else "nema17_48mm",
        reducer_id="planetary_10",
        target_rpm=40.0,
    )


def _select_psu(voltage_v: float, peak_power_w: float) -> tuple[str, float]:
    if voltage_v >= 40.0:
        if peak_power_w <= 360.0:
            return "Mean Well LRS-450-48 (48V 9.4A)", 44.0
        if peak_power_w <= 550.0:
            return "Mean Well LRS-600-48 (48V 12.5A)", 62.0
        return "Mean Well SE-1000-48 (48V 20.8A)", 118.0
    if peak_power_w <= 135.0:
        return "Mean Well LRS-150-24 (24V 6.5A)", 22.0
    if peak_power_w <= 315.0:
        return "Mean Well LRS-350-24 (24V 14.6A)", 34.0
    if peak_power_w <= 540.0:
        return "Mean Well LRS-600-24 (24V 25A)", 58.0
    return "Mean Well SE-1000-24 (24V 41.7A)", 115.0


def _select_power_awg(peak_current_a: float) -> str:
    if peak_current_a <= 8.0:
        return "18 AWG (Silicona)"
    if peak_current_a <= 14.0:
        return "16 AWG (Silicona)"
    if peak_current_a <= 22.0:
        return "14 AWG (Silicona)"
    if peak_current_a <= 35.0:
        return "12 AWG (Silicona)"
    return "10 AWG (Silicona)"


def evaluate_mechatronics(
    req: ModularArmRequest,
    eval_result: dict,
) -> MechatronicsResponse:
    """
    Evaluate per-joint Torque-Speed curves (J1..J_dof), thermal & torque margins,
    full electrical Power Budget (voltage, current, power, PSU, AWG, harness length),
    and unit-level Master BOM across all 4 manufacturing categories.
    """
    cfg = req.mechatronics or MechatronicsConfig()
    bus_arch = cfg.bus_architecture
    slots = resolve_default_slots(req)
    ua = slots.upper_arm
    fa = slots.forearm
    sm = slots.shoulder_mount
    assert ua and fa and sm

    # Map cascaded joint torques from eval_result (J1..J3) and extrapolate wrist joints J4..J6
    joint_map = {j.joint_id: j for j in eval_result.get("joints", [])}
    j1_s = joint_map["J1"].static_torque_nm if "J1" in joint_map else 8.0
    j1_d = joint_map["J1"].dynamic_torque_nm if "J1" in joint_map else 10.0
    j2_s = joint_map["J2"].static_torque_nm if "J2" in joint_map else 4.0
    j2_d = joint_map["J2"].dynamic_torque_nm if "J2" in joint_map else 5.2
    j3_s = joint_map["J3"].static_torque_nm if "J3" in joint_map else 1.2
    j3_d = joint_map["J3"].dynamic_torque_nm if "J3" in joint_map else 1.6

    joint_defs: list[tuple[str, str, float, float]] = [
        ("J1", "J1 Hombro (Pitch)", j1_s, j1_d),
        ("J2", "J2 Codo (Pitch)", j2_s, j2_d),
        ("J3", "J3 Muñeca (Pitch)", j3_s, j3_d),
    ]
    if req.dof >= 4:
        joint_defs.append(("J4", "J4 Base (Yaw)", j1_s * 0.25, j1_d * 0.45))
    if req.dof >= 5:
        joint_defs.append(("J5", "J5 Muñeca (Roll)", j3_s * 0.65, j3_d * 0.75))
    if req.dof >= 6:
        joint_defs.append(("J6", "J6 Brida TCP (Roll)", j3_s * 0.45, j3_d * 0.55))
    if req.dof >= 7:
        joint_defs.append(("J7", "J7 Brazo Redundante (Roll)", j2_s * 0.40, j2_d * 0.55))

    joint_reports: list[JointMechatronicsReport] = []
    motor_counts: dict[str, int] = {}
    reducer_counts: dict[str, int] = {}
    sum_cont_i = 0.0
    sum_peak_i = 0.0
    max_motor_voltage = float(cfg.system_voltage_v)
    notes: list[str] = []

    for idx, (jid, jname, tau_s, tau_d) in enumerate(joint_defs, start=1):
        spec = cfg.joint_actuators.get(jid) or _default_spec_for_joint(idx, tau_d, bus_arch)
        motor = ACTUATOR_CATALOG.get(spec.motor_id, ACTUATOR_CATALOG["nema17_mks_servo42d"])
        reducer = REDUCER_CATALOG.get(spec.reducer_id, REDUCER_CATALOG["planetary_10"])

        ratio = float(reducer["ratio"])
        eff = float(reducer["efficiency"])
        p_count = max(1, min(2, int(spec.parallel_count)))
        max_motor_voltage = max(max_motor_voltage, float(motor["voltage_v"]))

        motor_counts[motor["id"]] = motor_counts.get(motor["id"], 0) + p_count
        if reducer["id"] != "direct":
            reducer_counts[reducer["id"]] = reducer_counts.get(reducer["id"], 0) + p_count

        max_joint_rpm = max(1.0, float(motor["no_load_rpm"]) / ratio)
        op_rpm = min(float(spec.target_rpm), max_joint_rpm * 0.92)

        # Build 13-point Torque-Speed curve at joint output (scaled by parallel_count)
        curve: list[TorqueSpeedPoint] = []
        for k in range(13):
            j_rpm = max_joint_rpm * (k / 12.0)
            m_cont, m_peak = _motor_torque_at_rpm(motor, j_rpm * ratio)
            curve.append(
                TorqueSpeedPoint(
                    rpm=round(j_rpm, 1),
                    continuous_torque_nm=round(m_cont * p_count * ratio * eff, 2),
                    peak_torque_nm=round(m_peak * p_count * ratio * eff, 2),
                )
            )

        m_cont_op, m_peak_op = _motor_torque_at_rpm(motor, op_rpm * ratio)
        avail_cont = max(m_cont_op * p_count * ratio * eff, 1e-4)
        avail_peak = max(m_peak_op * p_count * ratio * eff, 1e-4)

        req_rms = math.sqrt(0.65 * (tau_s**2) + 0.35 * (tau_d**2))
        req_peak = max(tau_d, 0.05)

        torque_margin_pct = ((avail_peak - req_peak) / avail_peak) * 100.0
        # Joule thermal margin ~ 1 - (I_rms / I_rated)^2
        load_ratio = req_rms / avail_cont
        thermal_margin_pct = (1.0 - (load_ratio**2)) * 100.0

        # Operating electrical currents (scaled by parallel_count)
        cont_i = float(motor["rated_current_a"]) * p_count * min(1.35, max(0.35, load_ratio))
        peak_i = float(motor["peak_current_a"]) * p_count * min(1.25, max(0.45, req_peak / avail_peak))
        sum_cont_i += cont_i
        sum_peak_i += peak_i

        if req_peak > avail_peak or req_rms > avail_cont:
            status = "critical"
            notes.append(
                f"{jid}: torque requerido ({req_peak:.1f} N·m pico / {req_rms:.1f} N·m RMS) excede {p_count}× {motor['name']} + {reducer['name']}."
            )
        elif torque_margin_pct < 20.0 or thermal_margin_pct < 25.0:
            status = "warning"
        else:
            status = "ok"

        joint_reports.append(
            JointMechatronicsReport(
                joint_id=jid,
                joint_name=jname,
                motor_id=motor["id"],
                motor_name=motor["name"],
                motor_type=motor["motor_type"],
                parallel_count=p_count,
                reducer_id=reducer["id"],
                reducer_name=reducer["name"],
                gear_ratio=ratio,
                efficiency=eff,
                operating_rpm=round(op_rpm, 1),
                required_rms_torque_nm=round(req_rms, 2),
                required_peak_torque_nm=round(req_peak, 2),
                available_continuous_torque_nm=round(avail_cont, 2),
                available_peak_torque_nm=round(avail_peak, 2),
                torque_margin_pct=round(torque_margin_pct, 1),
                thermal_margin_pct=round(thermal_margin_pct, 1),
                continuous_current_a=round(cont_i, 2),
                peak_current_a=round(peak_i, 2),
                status=status,
                torque_speed_curve=curve,
            )
        )

    # Power Budget (double current/power for bimanual_dual_arm 2-arm systems!)
    arm_mult = 2.0 if req.kinematics_type == "bimanual_dual_arm" else 1.0
    is_pca_servo = bus_arch == "arduino_pca9685_i2c"
    is_esp32_5v = bus_arch == "esp32_pca9685_5v"
    is_uart_bus = bus_arch == "uart_serial_bus"
    is_teensy_dm = bus_arch == "teensy_dm542"

    if is_esp32_5v:
        system_v = 5.0
        cont_i_total = round(min(4.2, (sum_cont_i * 0.65 + 0.45) * arm_mult), 2)
        peak_i_total = round(min(5.0, (sum_peak_i * 0.70 + 0.65) * arm_mult), 2)
        cont_w = round(system_v * cont_i_total, 1)
        peak_w = round(system_v * peak_i_total, 1)
        psu_name = "Fuente Conmutada 5V 5A (25W · Jack DC 5.5×2.1mm)"
        psu_cost = 8.0 * arm_mult
        power_awg = "18 AWG (Jack DC → PCA9685 V+)"
        signal_awg = "22 AWG Cables Servo 3-Pin + Dupont I2C"
    elif is_pca_servo:
        system_v = (
            float(cfg.system_voltage_v)
            if 5.5 <= float(cfg.system_voltage_v) <= 7.4
            else 6.5
        )
        cont_i_total = round((sum_cont_i * 0.65 + 0.9) * arm_mult, 2)
        peak_i_total = round(min(20.0, (sum_peak_i * 0.65 + 1.5) * arm_mult), 2)
        cont_w = round(system_v * cont_i_total, 1)
        peak_w = round(system_v * peak_i_total, 1)
        psu_name = "Fuente Ajustable 3–12V 10A (6.5V · Jack DC 5.5×2.1mm)"
        psu_cost = 16.5 * arm_mult
        power_awg = "16 AWG (Jack DC → PCA9685 V+)"
        signal_awg = "22 AWG Extensiones Servo (10/30/50 cm)"
    elif is_uart_bus:
        system_v = 12.0 if float(cfg.system_voltage_v) >= 10.0 else 7.4
        cont_i_total = round((sum_cont_i * 0.65 + 0.4) * arm_mult, 2)
        peak_i_total = round((sum_peak_i * 0.72 + 0.6) * arm_mult, 2)
        cont_w = round(system_v * cont_i_total, 1)
        peak_w = round(system_v * peak_i_total, 1)
        psu_name = f"Fuente Conmutada {system_v:.1f}V {int(5 * arm_mult)}A (Jack DC / XT30)"
        psu_cost = 15.0 * arm_mult
        power_awg = "18 AWG (Silicona)"
        signal_awg = "22 AWG Bus Serie TTL 3-Pin Half-Duplex (1 Mbps)"
    else:
        system_v = 48.0 if max_motor_voltage >= 40.0 else 24.0
        cont_i_total = round((sum_cont_i * 0.72 + 0.8) * arm_mult, 2)
        peak_i_total = round((sum_peak_i * 0.85 + 1.0) * arm_mult, 2)
        cont_w = round(system_v * cont_i_total, 1)
        peak_w = round(system_v * peak_i_total, 1)
        psu_name, psu_cost = _select_psu(system_v, peak_w)
        power_awg = _select_power_awg(peak_i_total)
        if is_teensy_dm:
            signal_awg = "24 AWG Apantallado (Step/Dir + Encoders + Homing)"
        elif bus_arch == "can_fd_bus":
            signal_awg = "24 AWG Par Trenzado 120Ω (CAN-FD)"
        else:
            signal_awg = "26 AWG Apantallado (Step/Dir)"

    # Harness length (m): sum real arm segment runs * 1.20 dynamic service loop
    h_base_m = float(req.base_height_mm) / 1000.0
    l1_m = float(ua.length_mm) / 1000.0
    l2_m = float(fa.length_mm) / 1000.0
    l3_m = float(req.wrist_mm) / 1000.0
    if bus_arch == "can_fd_bus":
        # Single daisy-chain trunk through base -> shoulder -> elbow -> wrist + PSU lead
        raw_harness_m = 0.45 + h_base_m + l1_m + l2_m + l3_m + 0.12 * req.dof
    else:
        # Star wiring: individual cable from base controller to every joint
        raw_harness_m = (
            0.45
            + h_base_m * 2.0
            + (h_base_m + l1_m)
            + (h_base_m + l1_m + l2_m)
            + max(0, req.dof - 3) * (h_base_m + l1_m + l2_m + l3_m)
        )
    total_harness_m = round(raw_harness_m * 1.20, 2)
    fuse_a = float(math.ceil((peak_i_total * 1.25) / 5.0) * 5)

    power_budget = PowerBudget(
        bus_architecture=bus_arch,
        system_voltage_v=system_v,
        continuous_current_a=cont_i_total,
        peak_current_a=peak_i_total,
        continuous_power_w=cont_w,
        peak_power_w=peak_w,
        recommended_psu=psu_name,
        psu_cost_usd=psu_cost,
        power_wire_awg=power_awg,
        signal_wire_awg=signal_awg,
        total_harness_length_m=total_harness_m,
        fuse_rating_a=fuse_a,
    )

    # Master Unit-Level BOM (4 categories)
    materials = load_materials()
    material = materials.get(req.material_id) or materials["petg_cf"]
    mat_cost_kg = float(material["cost_per_kg_usd"])
    master_bom: list[MasterBomItem] = []

    def _add_bom(
        cat: str, code: str, name: str, spec_txt: str, qty: int, unit_usd: float
    ) -> None:
        master_bom.append(
            MasterBomItem(
                category=cat,  # type: ignore[arg-type]
                category_label=CATEGORY_LABELS[cat],
                part_code=code,
                part_name=name,
                spec=spec_txt,
                qty=qty,
                unit_cost_usd=round(unit_usd, 2),
                total_cost_usd=round(unit_usd * qty, 2),
            )
        )

    # Category 1: printed_cnc
    if is_esp32_5v:
        printed_g = max(120.0, min(220.0, float(eval_result.get("printed_mass_g", 180.0))))
        resin_cost = round((printed_g / 1000.0) * mat_cost_kg, 2)
        _add_bom(
            "printed_cnc",
            "RF-MARSMINI-RES",
            "Estructura Completa Mars-Mini (Base + 4-Barras + Pinza)",
            f"{material['name']} · ~{printed_g:.0f} g resina en 2 platos Elegoo Mars 5 Ultra",
            1,
            max(3.5, resin_cost),
        )
    else:
        for pm in eval_result.get("per_part_metrics", []):
            unit_c = max(0.85, (pm.mass_g / 1000.0) * mat_cost_kg * 1.2 + 0.6)
            _add_bom(
                "printed_cnc",
                f"RF-{pm.slot.upper()[:8]}",
                pm.part_name,
                f"{material['name']} · {pm.variant} · {pm.mass_g:.0f} g",
                1,
                unit_c,
            )
        _add_bom(
            "printed_cnc",
            "RF-BASE-WRST",
            "Pedestal Base + Tubo Muñeca + Brida TCP",
            f"{material['name']} · ~195 g estimados",
            1,
            max(2.5, 0.195 * mat_cost_kg * 1.2 + 1.5),
        )
        if req.cnc_plate_mode:
            _add_bom(
                "printed_cnc",
                "CNC-AL6061-PLT",
                "Placas Laterales CNC Aluminio 6061-T6 (DXF)",
                "Corte 2.5D espesor 6 mm · rigidez torsional industrial",
                2,
                24.0,
            )

    # Category 2, 3, 4: actuators_reducers, electronics_wiring, fasteners_hardware
    if is_esp32_5v:
        # Ultra-budget Mars-Mini MSLA BOM (< $45 USD total!)
        _add_bom(
            "actuators_reducers",
            "SRV-MG996R",
            "Servo MG996R 10kg·cm (Metal Gear)",
            "5V · Base J0 + Hombro J1 + Codo J2 en base fija",
            3,
            3.5,
        )
        _add_bom(
            "actuators_reducers",
            "SRV-MG90S",
            "Micro-Servo MG90S 2.2kg·cm (Metal Gear)",
            "5V · Accionamiento Pinza 2-Dedos",
            1,
            2.0,
        )
        _add_bom(
            "electronics_wiring",
            "EL-ESP32-DEV",
            "ESP32 DevKit V1 (Wi-Fi + Web Server)",
            "Control inalámbrico desde celular/PC · I2C GPIO21/22",
            1,
            5.5,
        )
        _add_bom(
            "electronics_wiring",
            "EL-PCA9685-16CH",
            "Módulo PCA9685 16ch I2C (PWM 12-bit)",
            "Bornera 5V 5A · canales CH0–CH3 activos",
            1,
            3.0,
        )
        _add_bom(
            "electronics_wiring",
            "EL-PSU-5V5A",
            psu_name,
            "Salida 5.0V DC · 5A (25W) · Jack DC + bornera",
            1,
            8.0,
        )
        _add_bom(
            "electronics_wiring",
            "EL-DUPONT-KIT",
            "Cables Dupont I2C + Adaptador Jack DC a Bornera",
            f"Arnés compacto {total_harness_m:.2f} m",
            1,
            1.8,
        )
        _add_bom(
            "fasteners_hardware",
            "ISO4762-M3x12",
            "Tornillo Allen ISO 4762 M3×12",
            "Acero 12.9 · uniones con tuerca cautiva (cero autorroscantes en resina)",
            24,
            0.05,
        )
        _add_bom(
            "fasteners_hardware",
            "ISO4032-M3-NUT",
            "Tuercas Cautivas Hexagonales ISO 4032 M3",
            "Bolsillos hexagonales 5.65 mm en Resina ABS-Like",
            24,
            0.04,
        )
        _add_bom(
            "fasteners_hardware",
            "BRG-608-2RS",
            "Rodamiento de Bolas 608-2RS",
            "8×22×7 mm · soporte axial de base giratoria J0",
            1,
            1.1,
        )
    elif is_pca_servo:
        # Exact FABRI Brazo Robótico 5.0 8-servo kit Master BOM (no external metallic bearings)
        primary_servo_id = next(iter(motor_counts.keys()), "servo_td8120mg")
        primary_servo = ACTUATOR_CATALOG.get(primary_servo_id, ACTUATOR_CATALOG["servo_td8120mg"])
        servo_qty = max(8, sum(motor_counts.values()))
        _add_bom(
            "actuators_reducers",
            f"SRV-{primary_servo_id.upper()[:10]}",
            primary_servo["name"],
            f"6.8V · 20 kg·cm · doble servo paralelo en hombro/codo + pinza",
            servo_qty,
            float(primary_servo["unit_cost_usd"]),
        )
        _add_bom(
            "actuators_reducers",
            "HRN-ALU-25T",
            "Horns Circulares de Aluminio 25T",
            "Ø20 × 4 mm anodizado rojo · 4 barrenos M3 (PCD 14 mm)",
            servo_qty,
            1.8,
        )

        _add_bom(
            "electronics_wiring",
            "EL-ARDUINO-UNO",
            "Arduino UNO R3 (ATmega328P)",
            "Controlador principal I2C (SDA/SCL) + USB programación",
            1,
            12.0,
        )
        _add_bom(
            "electronics_wiring",
            "EL-PCA9685-16CH",
            "Servo Shield PCA9685 (I2C 16 Canales)",
            "Driver PWM 12-bit · bornera V+ 10A · canales CH0–CH7",
            1,
            6.5,
        )
        _add_bom(
            "electronics_wiring",
            "EL-PSU-10A",
            psu_name,
            f"Salida {system_v:.1f}V DC · 10A continuos · Jack DC 5.5×2.1 mm",
            1,
            psu_cost,
        )
        _add_bom(
            "electronics_wiring",
            "EL-SERVO-EXT",
            "Cables de Extensión Servo 22 AWG (10/30/50 cm)",
            f"3 pines JR/Futaba · arnés total {total_harness_m:.2f} m",
            5,
            1.4,
        )

        # Exact 120-screw hardware kit for Brazo 5.0 (no external bearings)
        _add_bom(
            "fasteners_hardware",
            "ISO4762-M3x8",
            "Tornillo Allen ISO 4762 M3×8",
            "Acero 12.9 · fijación horns aluminio 25T",
            35,
            0.07,
        )
        _add_bom(
            "fasteners_hardware",
            "DIN7981-M3x6",
            "Tornillo Autorroscante M3×6",
            "Acero zincado · tapas y electrónica",
            5,
            0.05,
        )
        _add_bom(
            "fasteners_hardware",
            "DIN7981-M4x8",
            "Tornillo Autorroscante M4×8",
            "Acero zincado · soporte pinza y muñeca",
            10,
            0.06,
        )
        _add_bom(
            "fasteners_hardware",
            "DIN7981-M4x12",
            "Tornillo Autorroscante M4×12",
            "Acero zincado · orejas de servos y estructura 3D",
            70,
            0.07,
        )
    else:
        # Category 2: actuators_reducers
        for mid, qty in motor_counts.items():
            m = ACTUATOR_CATALOG[mid]
            _add_bom(
                "actuators_reducers",
                f"MOT-{mid.upper()[:10]}",
                m["name"],
                f"{m['voltage_v']:.0f}V · {m['rated_torque_nm']:.2f} N·m nom / {m['peak_torque_nm']:.2f} pico",
                qty,
                float(m["unit_cost_usd"]),
            )
        for rid, qty in reducer_counts.items():
            r = REDUCER_CATALOG[rid]
            _add_bom(
                "actuators_reducers",
                f"RED-{rid.upper()[:10]}",
                f"Reductor {r['name']}",
                f"Relación {r['ratio']:.0f}:1 · η={int(r['efficiency'] * 100)}%",
                qty,
                float(r["unit_cost_usd"]),
            )

        # Category 3: electronics_wiring
        if is_uart_bus:
            _add_bom(
                "electronics_wiring",
                "EL-WS-TTL",
                "Waveshare Bus Servo Adapter (USB-TTL 1 Mbps)",
                "Bus serie half-duplex 3 hilos · compatible LeRobot / Feetech",
                1,
                11.0,
            )
            _add_bom(
                "electronics_wiring",
                "EL-CONN-3PIN",
                "Cables Bus Serie 3-Pin Molex/JST (Daisy-Chain)",
                "22 AWG · conexión en cadena ID1..ID6",
                req.dof,
                1.2,
            )
        elif is_teensy_dm:
            _add_bom(
                "electronics_wiring",
                "EL-TEENSY41",
                "Controlador Teensy 4.1 + Breakout AR4",
                "ARM Cortex-M7 600 MHz · Step/Dir + Encoders + Homing",
                1,
                36.0,
            )
            _add_bom(
                "electronics_wiring",
                "EL-DRV-DM542T",
                "Driver Industrial DM542T (4.2 A · 24–50V)",
                "Microstepping configurable · entradas optoaisladas",
                req.dof,
                22.0,
            )
            _add_bom(
                "electronics_wiring",
                "EL-SENS-HOMING",
                "Sensores Inductivos de Homing (LJ12A3 / Final Carrera)",
                "Detección de cero mecánico en cada eje",
                req.dof,
                4.2,
            )
        elif bus_arch == "can_fd_bus":
            _add_bom(
                "electronics_wiring",
                "EL-MCU-CANFD",
                "Controlador Principal STM32H7 + CAN-FD",
                "STM32H743 + transceptor MCP2562FD 5 Mbps",
                1,
                38.0,
            )
            _add_bom(
                "electronics_wiring",
                "EL-CONN-JSTGH",
                "Conectores JST-GH 4-Pin (Bus CAN-FD)",
                "Paso 1.25 mm con traba + terminadores 120Ω",
                req.dof + 2,
                1.4,
            )
        else:
            _add_bom(
                "electronics_wiring",
                "EL-MCU-STM32",
                "Controlador STM32F407 + Shield Step/Dir",
                "Soporte hasta 6 ejes + finales de carrera",
                1,
                26.0,
            )
            _add_bom(
                "electronics_wiring",
                "EL-DRV-TMC2209",
                "Driver Silencioso TMC2209 (UART)",
                "2.0A RMS · 256 microsteps",
                req.dof,
                5.5,
            )
        _add_bom(
            "electronics_wiring",
            "EL-PSU-MAIN",
            psu_name,
            f"Salida {system_v:.0f}V DC · Pico {peak_w:.0f} W",
            1,
            psu_cost,
        )
        _add_bom(
            "electronics_wiring",
            "EL-ESTOP-FUSE",
            "Paro de Emergencia E-Stop + Portafusible",
            f"Seta NC + fusible automotriz {fuse_a:.0f} A",
            1,
            12.0,
        )
        _add_bom(
            "electronics_wiring",
            "EL-CONN-XT60",
            "Conectores de Potencia XT60 (Par Macho/Hembra)",
            "60A continuos chapados en oro",
            2,
            1.8,
        )
        _add_bom(
            "electronics_wiring",
            "EL-HARNESS-M",
            "Arnés Eléctrico Completo (Potencia + Señal)",
            f"{power_awg} + {signal_awg} · {total_harness_m:.2f} m",
            max(1, int(math.ceil(total_harness_m))),
            2.8,
        )

        # Category 4: fasteners_hardware (unit-level ISO 4762 / ISO 4032 / bearings)
        m3_qty = 16 + 4 * req.dof
        m4_qty = 12
        m5_qty = 8 if sm.actuator_type == "nema23" else 4
        _add_bom(
            "fasteners_hardware",
            "ISO4762-M3x12",
            "Tornillo Allen ISO 4762 M3×12",
            "Acero 12.9 pavonado · bridas NEMA17 / muñeca / TCP",
            m3_qty,
            0.08,
        )
        _add_bom(
            "fasteners_hardware",
            "ISO4762-M4x16",
            "Tornillo Allen ISO 4762 M4×16",
            "Acero 12.9 pavonado · tapas de juntas y caja MCU",
            m4_qty,
            0.12,
        )
        _add_bom(
            "fasteners_hardware",
            "ISO4762-M5x20",
            "Tornillo Allen ISO 4762 M5×20",
            "Acero 12.9 pavonado · anclaje base y montura hombro",
            m5_qty,
            0.18,
        )
        _add_bom(
            "fasteners_hardware",
            "ISO4032-NUTS",
            "Tuercas Hexagonales ISO 4032 (M3/M4/M5)",
            "Acero inox A2-70",
            m3_qty // 2 + m4_qty + m5_qty,
            0.06,
        )
        _add_bom(
            "fasteners_hardware",
            "INS-BRASS-M3M4",
            "Insertos Térmicos de Latón CNC (M3/M4)",
            "Moleteados para termoplásticos PETG-CF / PA12-CF",
            m3_qty + m4_qty,
            0.10,
        )
        _add_bom(
            "fasteners_hardware",
            "BRG-6808-2RS",
            "Rodamiento Sección Delgada 6808-2RS",
            "40×52×7 mm · juntas principales Hombro y Codo",
            4,
            4.8,
        )
        _add_bom(
            "fasteners_hardware",
            "BRG-608-2RS",
            "Rodamiento de Bolas 608-2RS",
            "8×22×7 mm · muñeca y ejes secundarios",
            4,
            1.1,
        )
        if req.kinematics_type == "scara":
            _add_bom(
                "fasteners_hardware",
                "LIN-MGN12H",
                "Guía Lineal MGN12H + Carro de Precisión",
                f"Acero endurecido · carrera Z {req.z_stroke_mm:.0f} mm",
                2,
                16.5,
            )
            _add_bom(
                "fasteners_hardware",
                "LIN-T8-LEAD",
                "Husillo Trapezoidal T8 + Tuerca Anti-Backlash",
                f"Paso 8 mm · eje prismático Z SCARA ({req.z_stroke_mm + 60:.0f} mm)",
                1,
                9.5,
            )
        if is_teensy_dm or req.payload_kg >= 2.0:
            _add_bom(
                "fasteners_hardware",
                "BRG-CRB-RU42",
                "Rodamiento Rodillos Cruzados CRB RU42",
                "Alta rigidez a momento de vuelco (Base / Hombro)",
                2,
                32.0,
            )
            _add_bom(
                "fasteners_hardware",
                "BRG-TAP-30205",
                "Rodamiento de Rodillos Cónicos 30205",
                "25×52×16.25 mm · carga axial + radial combinada",
                2,
                6.5,
            )

    # v6.0 Specialized Mechanism & Topology Hardware BOM Items (applies to all bus architectures)
    if req.remote_base_motors:
        _add_bom(
            "fasteners_hardware",
            "CAP-DRUM-SET",
            "Tambores Capstan CNC Aluminio 6061",
            "Transmisión remota de baja inercia con motores en base",
            4,
            14.0,
        )
        _add_bom(
            "fasteners_hardware",
            "CAP-TENDON-DYN",
            "Cables Tendón Dyneema / Acero Ø1.0 mm",
            "Lazo pretensado cero-juego para ejes J1/J2/J3",
            2,
            8.5,
        )
    if req.kinematics_type == "delta_parallel":
        _add_bom(
            "fasteners_hardware",
            "DLT-CF-RODS",
            "Varillas Paralelas Fibra de Carbono Ø6 mm",
            "3 pares de varillas ultraligeras de alta rigidez axial",
            6,
            4.5,
        )
        _add_bom(
            "fasteners_hardware",
            "DLT-BALL-JOINTS",
            "Rótulas Esféricas Magnéticas / Igus M4",
            "12× articulaciones esféricas de bajo rozamiento",
            12,
            2.2,
        )
    if req.kinematics_type == "cartesian_gantry":
        _add_bom(
            "fasteners_hardware",
            "GNT-2020-PROF",
            "Perfiles Extrusión Aluminio 2020 Anodizado",
            "Marco rígido pórtico X-Y-Z",
            6,
            6.5,
        )
        _add_bom(
            "fasteners_hardware",
            "GNT-MGN12-XYZ",
            "Guías Lineales MGN12H X/Y/Z + Carros",
            "Ejes prismáticos de precisión para Pick & Place",
            4,
            16.5,
        )
        _add_bom(
            "fasteners_hardware",
            "GNT-GT2-BELT",
            "Correa GT2 6 mm + Poleas 20T",
            "Transmisión lineal X/Y de alta repetibilidad",
            3,
            5.5,
        )
    if req.kinematics_type == "continuum_pcc":
        _add_bom(
            "fasteners_hardware",
            "CNT-NITINOL-2MM",
            "Espina Flexible Tubo Nitinol Superelástico Ø2 mm",
            "Aleación NiTi superelástica (E=75 GPa) para arco PCC",
            1,
            28.0,
        )
        _add_bom(
            "fasteners_hardware",
            "CNT-SPACER-PEEK",
            "Discos Espaciadores PEEK / Resina Médica",
            "8× discos guía de tendones a 120°",
            8,
            2.5,
        )
        _add_bom(
            "fasteners_hardware",
            "CNT-TENDON-3X",
            "Micro-Tendones Acero Inox Ø0.4 mm (120°)",
            "Actuación remota desde base actuadora",
            3,
            4.0,
        )
    if req.four_bar_parallelogram and not is_esp32_5v:
        _add_bom(
            "fasteners_hardware",
            "FBR-PARA-ROD",
            "Varilla Paralelogramo 4-Barras + Pasadores",
            "Mantiene efector nivelado pasivamente (Dobot/uArm)",
            1,
            9.0,
        )
    if req.bevel_differential_wrist:
        _add_bom(
            "fasteners_hardware",
            "BVL-DIFF-SET",
            "Juego 3× Engranajes Cónicos Bevel Diferencial",
            "Muñeca diferencial Pitch+Roll acoplada (Thor/Niryo)",
            1,
            18.5,
        )
    if req.vacuum_eoat:
        _add_bom(
            "electronics_wiring",
            "VAC-NOZZLE-PUMP",
            "Boquilla de Vacío + Ventosa Silicona + Bomba 12V",
            "EOAT Pick & Place neumático con electroválvula",
            1,
            19.0,
        )
    # v7.0 Wave 3 Topologies & Environments BOM Items
    if req.kinematics_type == "stewart_hexapod":
        _add_bom(
            "actuators_reducers",
            "STW-LIN-ACT-6X",
            "Actuadores Lineales Telescópicos 6-UPS (Husillo + Encoder)",
            "6× patas paralelas de alta rigidez axial",
            6,
            48.0,
        )
        _add_bom(
            "fasteners_hardware",
            "STW-UJOINT-12X",
            "Juntas Universales / Rótulas Esféricas M8 (12×)",
            "Acoplamiento base y plataforma móvil sin juego",
            12,
            6.5,
        )
    if req.kinematics_type == "bimanual_dual_arm":
        _add_bom(
            "printed_cnc",
            "BIM-TORSO-FRAME",
            "Torso Central Estructural Bimanual (Izq + Der)",
            "Base compartida para 2 brazos espejados 14-DOF",
            1,
            45.0,
        )
    if req.kinematics_type == "soft_pneumatic_bellows":
        _add_bom(
            "electronics_wiring",
            "PNU-PIEZO-VALVE",
            "Electroválvulas Proporcionales Piezo (3 Cámaras 120°)",
            "Control de presión 0.6–1.5 bar por cámara de fuelle TPU",
            3,
            24.0,
        )
        _add_bom(
            "electronics_wiring",
            "PNU-COMPRESSOR",
            "Micro-Compresor Neumático 12V + Manifold + Tubing PU 4 mm",
            "Suministro de aire comprimido silencioso",
            1,
            38.0,
        )
    if req.counterbalance_j2:
        _add_bom(
            "fasteners_hardware",
            "KUK-CB-CYL-J2",
            "Cilindro Contrapeso Hidroneumático J2",
            "Compensa -65% del torque estático gravitacional del hombro",
            1,
            145.0,
        )
    if req.environment_mode == "underwater_rov":
        _add_bom(
            "fasteners_hardware",
            "ROV-SEAL-VITON",
            "Kit Retenes Rotativos Viton + Compensador de Presión Aceite",
            "Sellado submarino IP68 para manipulador ROV",
            1,
            65.0,
        )

    total_bom_cost_usd = round(sum(item.total_cost_usd for item in master_bom), 2)
    if not notes:
        arch_labels = {
            "arduino_pca9685_i2c": "Arduino UNO + PCA9685 I2C (8 Servos)",
            "esp32_pca9685_5v": "ESP32 Wi-Fi + PCA9685 I2C (5V 5A · Mars-Mini)",
            "uart_serial_bus": "Bus Serie UART 1 Mbps (STS3215 / LeRobot)",
            "teensy_dm542": "Teensy 4.1 + Drivers DM542T Industrial",
            "can_fd_bus": "Bus CAN-FD 4 hilos",
            "stm32_step_dir": "STM32 Step/Dir",
        }
        arch_label = arch_labels.get(bus_arch, bus_arch)
        notes.append(
            f"Arquitectura {arch_label} validada a {system_v:.1f}V con margen térmico y de torque positivos."
        )

    return MechatronicsResponse(
        bus_architecture=bus_arch,
        joint_reports=joint_reports,
        power_budget=power_budget,
        master_bom=master_bom,
        total_bom_cost_usd=total_bom_cost_usd,
        notes=notes,
    )


def _generate_pca9685_schematic_svg(mechatronics: MechatronicsResponse) -> str:
    """Dedicated dark-mode SVG schematic for Arduino UNO + PCA9685 16ch I2C (8 Servos + Gripper)."""
    pb = mechatronics.power_budget
    channels = [
        ("CH0", "J1 Hombro Servo A (Paralelo ×2)", "TD-8120MG · 22 AWG"),
        ("CH1", "J1 Hombro Servo B (Paralelo ×2)", "TD-8120MG · Invertido"),
        ("CH2", "J2 Codo Servo A (Paralelo ×2)", "TD-8120MG · 22 AWG"),
        ("CH3", "J2 Codo Servo B (Paralelo ×2)", "TD-8120MG · Invertido"),
        ("CH4", "J3 Muñeca Pitch", "TD-8120MG · Ext 30cm"),
        ("CH5", "J4 Base Yaw", "TD-8120MG · Ext 10cm"),
        ("CH6", "J5 Muñeca Roll", "TD-8120MG · Ext 50cm"),
        ("CH7", "Pinza / Gripper 2-Dedos", "TD-8120MG · Ext 50cm"),
    ]
    svg_w, svg_h = 980, 560
    lines: list[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {svg_w} {svg_h}" width="{svg_w}" height="{svg_h}">',
        "<defs>",
        '  <pattern id="rf-grid-pca" width="24" height="24" patternUnits="userSpaceOnUse">',
        '    <path d="M 24 0 L 0 0 0 24" fill="none" stroke="#1e293b" stroke-width="0.6"/>',
        "  </pattern>",
        "</defs>",
        f'<rect width="{svg_w}" height="{svg_h}" fill="#0b1120"/>',
        f'<rect width="{svg_w}" height="{svg_h}" fill="url(#rf-grid-pca)"/>',
        '<text x="24" y="32" fill="#f8fafc" font-family="ui-monospace,Consolas,monospace" font-size="16" font-weight="700">'
        'ROBOFORGE v4.1 — ESQUEMÁTICO BRAZO 5.0 (ARDUINO UNO + PCA9685 I2C · 8 SERVOS)</text>',
        '<text x="24" y="52" fill="#94a3b8" font-family="ui-monospace,Consolas,monospace" font-size="11">'
        f'{escape(pb.recommended_psu)} · {pb.system_voltage_v:.1f}V · Pico {pb.peak_current_a:.1f}A ({pb.peak_power_w:.0f}W) · Arnés {pb.total_harness_length_m:.2f} m</text>',
        # PSU block
        '<rect x="24" y="120" width="190" height="104" rx="8" fill="#111827" stroke="#f59e0b" stroke-width="1.8"/>',
        '<text x="36" y="144" fill="#fbbf24" font-family="ui-monospace,Consolas,monospace" font-size="12" font-weight="700">FUENTE 6.5V 10A</text>',
        '<text x="36" y="164" fill="#e2e8f0" font-family="ui-monospace,Consolas,monospace" font-size="10">Ajustable 3–12V · 10A</text>',
        '<text x="36" y="182" fill="#94a3b8" font-family="ui-monospace,Consolas,monospace" font-size="10">Jack DC 5.5×2.1 mm</text>',
        f'<text x="36" y="200" fill="#94a3b8" font-family="ui-monospace,Consolas,monospace" font-size="10">Vout: {pb.system_voltage_v:.1f} V DC (V+)</text>',
        # Arduino UNO block
        '<rect x="24" y="290" width="190" height="114" rx="8" fill="#111827" stroke="#38bdf8" stroke-width="1.8"/>',
        '<text x="36" y="314" fill="#38bdf8" font-family="ui-monospace,Consolas,monospace" font-size="12" font-weight="700">ARDUINO UNO R3</text>',
        '<text x="36" y="334" fill="#e2e8f0" font-family="ui-monospace,Consolas,monospace" font-size="10">ATmega328P · 5V Logic</text>',
        '<text x="36" y="352" fill="#22c55e" font-family="ui-monospace,Consolas,monospace" font-size="10">A4 (SDA) / A5 (SCL)</text>',
        '<text x="36" y="370" fill="#94a3b8" font-family="ui-monospace,Consolas,monospace" font-size="10">GND común + VCC 5V</text>',
        '<text x="36" y="388" fill="#94a3b8" font-family="ui-monospace,Consolas,monospace" font-size="10">USB Secuenciador Poses</text>',
        # Wires PSU -> PCA9685 and Arduino -> PCA9685
        '<line x1="214" y1="162" x2="310" y2="162" stroke="#ef4444" stroke-width="2.8"/>',
        '<line x1="214" y1="184" x2="310" y2="184" stroke="#64748b" stroke-width="2.4" stroke-dasharray="4 2"/>',
        '<text x="224" y="154" fill="#fca5a5" font-family="ui-monospace,Consolas,monospace" font-size="9">Jack DC V+ 16AWG</text>',
        '<line x1="214" y1="336" x2="310" y2="274" stroke="#22c55e" stroke-width="2.0"/>',
        '<line x1="214" y1="354" x2="310" y2="292" stroke="#38bdf8" stroke-width="2.0"/>',
        '<line x1="214" y1="372" x2="310" y2="310" stroke="#64748b" stroke-width="1.8" stroke-dasharray="4 2"/>',
        '<text x="220" y="294" fill="#86efac" font-family="ui-monospace,Consolas,monospace" font-size="9">I2C SDA/SCL/GND</text>',
        # PCA9685 16ch Shield block
        '<rect x="310" y="130" width="230" height="215" rx="8" fill="#111827" stroke="#a855f7" stroke-width="2.0"/>',
        '<text x="324" y="158" fill="#c084fc" font-family="ui-monospace,Consolas,monospace" font-size="12.5" font-weight="700">PCA9685 16ch I2C</text>',
        '<text x="324" y="178" fill="#e2e8f0" font-family="ui-monospace,Consolas,monospace" font-size="10">Driver PWM 12-bit (50 Hz)</text>',
        '<text x="324" y="198" fill="#fca5a5" font-family="ui-monospace,Consolas,monospace" font-size="10">Bornera V+: 6.5V / 10A</text>',
        '<text x="324" y="218" fill="#86efac" font-family="ui-monospace,Consolas,monospace" font-size="10">Dir I2C: 0x40 (SDA/SCL)</text>',
        '<text x="324" y="244" fill="#94a3b8" font-family="ui-monospace,Consolas,monospace" font-size="10">Canales Activos: CH0..CH7</text>',
        '<text x="324" y="264" fill="#94a3b8" font-family="ui-monospace,Consolas,monospace" font-size="10">8× Servos TD-8120MG</text>',
        '<text x="324" y="284" fill="#fbbf24" font-family="ui-monospace,Consolas,monospace" font-size="10">CH0–CH1: Hombro ×2</text>',
        '<text x="324" y="304" fill="#fbbf24" font-family="ui-monospace,Consolas,monospace" font-size="10">CH2–CH3: Codo ×2</text>',
        '<text x="324" y="324" fill="#38bdf8" font-family="ui-monospace,Consolas,monospace" font-size="10">CH7: Pinza / Gripper</text>',
        # Bus trunk to CH0..CH7
        '<line x1="540" y1="238" x2="612" y2="238" stroke="#ef4444" stroke-width="2.4"/>',
        '<line x1="612" y1="96" x2="612" y2="490" stroke="#ef4444" stroke-width="2.0"/>',
        '<line x1="622" y1="104" x2="622" y2="498" stroke="#f59e0b" stroke-width="1.8"/>',
    ]
    for idx, (ch, label, sub) in enumerate(channels):
        cy = 96 + idx * 56
        stroke = "#dc2626" if idx < 4 else ("#38bdf8" if idx == 7 else "#22c55e")
        lines.extend(
            [
                f'<line x1="612" y1="{cy}" x2="674" y2="{cy}" stroke="#ef4444" stroke-width="1.6"/>',
                f'<line x1="622" y1="{cy + 8}" x2="674" y2="{cy + 8}" stroke="#f59e0b" stroke-width="1.5"/>',
                f'<rect x="674" y="{cy - 18}" width="282" height="44" rx="6" fill="#0f172a" stroke="{stroke}" stroke-width="1.6"/>',
                f'<text x="684" y="{cy - 2}" fill="#f8fafc" font-family="ui-monospace,Consolas,monospace" font-size="10.5" font-weight="700">{ch} · {escape(label)}</text>',
                f'<text x="684" y="{cy + 15}" fill="#94a3b8" font-family="ui-monospace,Consolas,monospace" font-size="9.5">{escape(sub)}</text>',
            ]
        )
    lines.extend(
        [
            '<text x="24" y="542" fill="#cbd5e1" font-family="ui-monospace,Consolas,monospace" font-size="10">'
            'Rojo: V+ 6.5V (16 AWG) · Ámbar: PWM 50Hz (22 AWG) · Verde/Cian: Bus I2C SDA/SCL</text>',
            "</svg>",
        ]
    )
    return "\n".join(lines)


def generate_circuit_schematic_svg(mechatronics: MechatronicsResponse, dof: int) -> str:
    """
    Generate a clean dark-mode WireViz-style electrical schematic SVG (100% offline).
    Shows PSU -> E-Stop/Fuse -> MCU -> CAN-FD Bus, Step/Dir, or Arduino + PCA9685 I2C.
    """
    pb = mechatronics.power_budget
    if pb.bus_architecture == "arduino_pca9685_i2c":
        return _generate_pca9685_schematic_svg(mechatronics)
    is_can = pb.bus_architecture == "can_fd_bus"
    is_uart = pb.bus_architecture == "uart_serial_bus"
    is_teensy = pb.bus_architecture == "teensy_dm542"
    is_esp32 = pb.bus_architecture == "esp32_pca9685_5v"
    n_joints = max(1, min(dof, len(mechatronics.joint_reports)))
    svg_h = max(420, 120 + n_joints * 58)
    svg_w = 980

    header_arch = (
        "ESP32 WI-FI + PCA9685 I2C (5V 5A · MARS-MINI)"
        if is_esp32
        else (
            "BUS CAN-FD 4 HILOS"
            if is_can
            else (
                "BUS SERIE UART 1 Mbps (STS3215 / LEROBOT)"
                if is_uart
                else ("TEENSY 4.1 + DM542T INDUSTRIAL" if is_teensy else "STM32 STEP/DIR")
            )
        )
    )
    lines: list[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {svg_w} {svg_h}" width="{svg_w}" height="{svg_h}">',
        "<defs>",
        '  <pattern id="rf-grid" width="24" height="24" patternUnits="userSpaceOnUse">',
        '    <path d="M 24 0 L 0 0 0 24" fill="none" stroke="#1e293b" stroke-width="0.6"/>',
        "  </pattern>",
        "</defs>",
        f'<rect width="{svg_w}" height="{svg_h}" fill="#0b1120"/>',
        f'<rect width="{svg_w}" height="{svg_h}" fill="url(#rf-grid)"/>',
        '<text x="24" y="32" fill="#f8fafc" font-family="ui-monospace,Consolas,monospace" font-size="16" font-weight="700">'
        f'ROBOFORGE STUDIO v5.0 — ESQUEMÁTICO ELÉCTRICO ({escape(header_arch)})</text>',
        '<text x="24" y="52" fill="#94a3b8" font-family="ui-monospace,Consolas,monospace" font-size="11">'
        f'{escape(pb.recommended_psu)} · Pico {pb.peak_power_w:.0f} W ({pb.peak_current_a:.1f} A) · Potencia {escape(pb.power_wire_awg)} · Arnés {pb.total_harness_length_m:.2f} m</text>',
    ]

    # Block 1: PSU
    lines.extend(
        [
            '<rect x="24" y="130" width="165" height="96" rx="8" fill="#111827" stroke="#f59e0b" stroke-width="1.8"/>',
            '<text x="36" y="154" fill="#fbbf24" font-family="ui-monospace,Consolas,monospace" font-size="12" font-weight="700">FUENTE PSU</text>',
            f'<text x="36" y="174" fill="#e2e8f0" font-family="ui-monospace,Consolas,monospace" font-size="10">{escape(pb.recommended_psu[:24])}</text>',
            f'<text x="36" y="192" fill="#94a3b8" font-family="ui-monospace,Consolas,monospace" font-size="10">Vout: {pb.system_voltage_v:.0f} V DC</text>',
            f'<text x="36" y="210" fill="#94a3b8" font-family="ui-monospace,Consolas,monospace" font-size="10">Cont: {pb.continuous_current_a:.1f}A / Pico: {pb.peak_current_a:.1f}A</text>',
        ]
    )

    # Wire PSU -> E-Stop
    lines.extend(
        [
            '<line x1="189" y1="168" x2="240" y2="168" stroke="#ef4444" stroke-width="2.6"/>',
            '<line x1="189" y1="190" x2="240" y2="190" stroke="#64748b" stroke-width="2.2" stroke-dasharray="4 2"/>',
            '<text x="195" y="160" fill="#fca5a5" font-family="ui-monospace,Consolas,monospace" font-size="9">XT60</text>',
        ]
    )

    # Block 2: E-Stop & Fuse
    lines.extend(
        [
            '<rect x="240" y="136" width="136" height="84" rx="8" fill="#111827" stroke="#ef4444" stroke-width="1.8"/>',
            '<text x="252" y="160" fill="#f87171" font-family="ui-monospace,Consolas,monospace" font-size="12" font-weight="700">E-STOP + FUSIBLE</text>',
            f'<text x="252" y="180" fill="#e2e8f0" font-family="ui-monospace,Consolas,monospace" font-size="10">Fusible: {pb.fuse_rating_a:.0f} A</text>',
            f'<text x="252" y="198" fill="#94a3b8" font-family="ui-monospace,Consolas,monospace" font-size="10">{escape(pb.power_wire_awg[:18])}</text>',
        ]
    )

    # Wire E-Stop -> MCU
    lines.extend(
        [
            '<line x1="376" y1="168" x2="428" y2="168" stroke="#ef4444" stroke-width="2.6"/>',
            '<line x1="376" y1="190" x2="428" y2="190" stroke="#64748b" stroke-width="2.2" stroke-dasharray="4 2"/>',
        ]
    )

    # Block 3: MCU / Distribution
    if is_esp32:
        mcu_title = "ESP32 WI-FI + PCA9685"
        mcu_sub = "AP Web Server · I2C 0x40 (50Hz)"
    elif is_uart:
        mcu_title = "WAVESHARE USB-TTL BUS"
        mcu_sub = "Half-Duplex 1 Mbps · 3-Pin"
    elif is_teensy:
        mcu_title = "TEENSY 4.1 + DM542T"
        mcu_sub = f"{n_joints}× Drivers 4.2A + Homing"
    elif is_can:
        mcu_title = "MCU STM32H7 + CAN-FD"
        mcu_sub = "Transceptor 5 Mbps · 120Ω"
    else:
        mcu_title = "MCU STM32F4 + TMC2209"
        mcu_sub = f"{n_joints}× Drivers Step/Dir UART"
    lines.extend(
        [
            '<rect x="428" y="122" width="184" height="112" rx="8" fill="#111827" stroke="#38bdf8" stroke-width="1.8"/>',
            f'<text x="440" y="148" fill="#38bdf8" font-family="ui-monospace,Consolas,monospace" font-size="11.5" font-weight="700">{escape(mcu_title)}</text>',
            f'<text x="440" y="168" fill="#e2e8f0" font-family="ui-monospace,Consolas,monospace" font-size="10">{escape(mcu_sub)}</text>',
            '<text x="440" y="188" fill="#94a3b8" font-family="ui-monospace,Consolas,monospace" font-size="10">Regulador Buck 5V / 3.3V</text>',
            '<text x="440" y="206" fill="#22c55e" font-family="ui-monospace,Consolas,monospace" font-size="10">USB-C / ROS 2 micro-ROS</text>',
        ]
    )

    # Bus trunk / star wires to J1..Jn
    trunk_x = 656
    y_first = 104
    y_last = y_first + (n_joints - 1) * 56
    lines.extend(
        [
            f'<line x1="612" y1="168" x2="{trunk_x}" y2="168" stroke="#ef4444" stroke-width="2.4"/>',
            f'<line x1="612" y1="184" x2="{trunk_x + 10}" y2="184" stroke="#22c55e" stroke-width="2.0"/>',
            f'<line x1="{trunk_x}" y1="{y_first}" x2="{trunk_x}" y2="{y_last}" stroke="#ef4444" stroke-width="2.2"/>',
            f'<line x1="{trunk_x + 10}" y1="{y_first + 12}" x2="{trunk_x + 10}" y2="{y_last + 12}" stroke="#22c55e" stroke-width="1.8"/>',
        ]
    )

    status_colors = {"ok": "#22c55e", "warning": "#f59e0b", "critical": "#ef4444"}
    for idx, jr in enumerate(mechatronics.joint_reports[:n_joints]):
        cy = y_first + idx * 56
        stroke_c = status_colors.get(jr.status, "#38bdf8")
        conn_tag = "JST-GH 4P" if is_can else "JST-XH 4P"
        lines.extend(
            [
                f'<line x1="{trunk_x}" y1="{cy}" x2="716" y2="{cy}" stroke="#ef4444" stroke-width="1.8"/>',
                f'<line x1="{trunk_x + 10}" y1="{cy + 12}" x2="716" y2="{cy + 12}" stroke="#22c55e" stroke-width="1.6"/>',
                f'<rect x="716" y="{cy - 18}" width="240" height="44" rx="6" fill="#0f172a" stroke="{stroke_c}" stroke-width="1.6"/>',
                f'<text x="726" y="{cy - 2}" fill="#f8fafc" font-family="ui-monospace,Consolas,monospace" font-size="10.5" font-weight="700">'
                f'{escape(jr.joint_id)} · {escape(jr.motor_name[:22])}</text>',
                f'<text x="726" y="{cy + 15}" fill="#94a3b8" font-family="ui-monospace,Consolas,monospace" font-size="9.5">'
                f'{escape(conn_tag)} · {jr.reducer_name} · {jr.peak_current_a:.1f}A · Margen {jr.torque_margin_pct:.0f}%</text>',
            ]
        )

    # Legend footer
    fy = svg_h - 22
    lines.extend(
        [
            f'<line x1="24" y1="{fy - 4}" x2="54" y2="{fy - 4}" stroke="#ef4444" stroke-width="2.5"/>',
            f'<text x="60" y="{fy}" fill="#cbd5e1" font-family="ui-monospace,Consolas,monospace" font-size="10">Potencia +{pb.system_voltage_v:.0f}V ({escape(pb.power_wire_awg)})</text>',
            f'<line x1="270" y1="{fy - 4}" x2="300" y2="{fy - 4}" stroke="#22c55e" stroke-width="2.0"/>',
            f'<text x="306" y="{fy}" fill="#cbd5e1" font-family="ui-monospace,Consolas,monospace" font-size="10">Señal ({escape(pb.signal_wire_awg)})</text>',
            "</svg>",
        ]
    )
    return "\n".join(lines)


def generate_wireviz_yaml(mechatronics: MechatronicsResponse, dof: int) -> str:
    """Export wiring harness definition in standard WireViz YAML format."""
    pb = mechatronics.power_budget
    if pb.bus_architecture == "arduino_pca9685_i2c":
        out_pca = [
            "# RoboForge Studio Pro v4.1 — Brazo Arduino 5.0 (PCA9685 I2C · 8 Servos)",
            "connectors:",
            "  X1_PSU_6V5:",
            f"    type: {pb.recommended_psu}",
            "    subtype: DC-Jack-5.5x2.1mm",
            f"    pins: ['+{pb.system_voltage_v:.1f}V', 'GND']",
            "  X2_ARDUINO_UNO:",
            "    type: Arduino UNO R3 (ATmega328P)",
            "    pins: ['5V', 'GND', 'A4_SDA', 'A5_SCL']",
            "  X3_PCA9685:",
            "    type: PCA9685 16ch I2C Servo Shield",
            "    pins: ['V+', 'GND', 'SDA', 'SCL', 'PWM']",
        ]
        ch_names = [
            "CH0_J1_Shoulder_A",
            "CH1_J1_Shoulder_B",
            "CH2_J2_Elbow_A",
            "CH3_J2_Elbow_B",
            "CH4_J3_Wrist_Pitch",
            "CH5_J4_Base_Yaw",
            "CH6_J5_Wrist_Roll",
            "CH7_Gripper",
        ]
        for ch in ch_names:
            out_pca.extend(
                [
                    f"  S_{ch}:",
                    "    type: Servo TD-8120MG 20kg-cm",
                    "    subtype: JR-3Pin",
                    "    pins: ['PWM', 'V+', 'GND']",
                ]
            )
        out_pca.extend(
            [
                "",
                "cables:",
                "  W1_PSU_DC:",
                "    gauge: 16 AWG",
                "    length: 0.40 m",
                "    colors: [RD, BK]",
                "  W2_I2C_BUS:",
                "    gauge: 24 AWG",
                "    length: 0.15 m",
                "    colors: [RD, BK, GN, BU]",
                "  W3_SERVO_EXT:",
                "    gauge: 22 AWG",
                f"    length: {pb.total_harness_length_m:.2f} m",
                "    colors: [OG, RD, BN]",
                "",
                "connections:",
                "  - [X1_PSU_6V5: [1, 2], W1_PSU_DC: [1, 2], X3_PCA9685: [1, 2]]",
                "  - [X2_ARDUINO_UNO: [1, 2, 3, 4], W2_I2C_BUS: [1, 2, 3, 4], X3_PCA9685: [1, 2, 3, 4]]",
            ]
        )
        for ch in ch_names:
            out_pca.append(f"  - [X3_PCA9685: [5, 1, 2], W3_SERVO_EXT: [1, 2, 3], S_{ch}: [1, 2, 3]]")
        return "\n".join(out_pca) + "\n"

    is_can = pb.bus_architecture == "can_fd_bus"
    n_joints = max(1, min(dof, len(mechatronics.joint_reports)))

    out: list[str] = [
        "# RoboForge Studio Pro v4.0 — WireViz Harness Specification",
        "connectors:",
        "  X1_PSU:",
        f"    type: {pb.recommended_psu}",
        "    subtype: XT60-M",
        f"    pins: ['+{pb.system_voltage_v:.0f}V', 'GND']",
        "  X2_ESTOP:",
        f"    type: E-Stop + Fuse {pb.fuse_rating_a:.0f}A",
        "    pins: ['IN+', 'OUT+', 'GND']",
        "  X3_MCU:",
        f"    type: {'STM32H743 CAN-FD Controller' if is_can else 'STM32F407 + TMC2209 Shield'}",
        "    pins: ['VBUS', 'GND', 'CAN_H', 'CAN_L']" if is_can else "    pins: ['VBUS', 'GND', 'STEP', 'DIR']",
    ]
    for jr in mechatronics.joint_reports[:n_joints]:
        out.extend(
            [
                f"  M_{jr.joint_id}:",
                f"    type: {jr.motor_name}",
                f"    subtype: {'JST-GH 4P' if is_can else 'JST-XH 4P'}",
                "    pins: ['VBUS', 'GND', 'CAN_H', 'CAN_L']" if is_can else "    pins: ['A+', 'A-', 'B+', 'B-']",
            ]
        )

    out.extend(
        [
            "",
            "cables:",
            "  W1_MAIN_PWR:",
            f"    gauge: {pb.power_wire_awg}",
            "    length: 0.45 m",
            "    colors: [RD, BK]",
            "  W2_TRUNK_BUS:",
            f"    gauge: {pb.signal_wire_awg}",
            f"    length: {pb.total_harness_length_m:.2f} m",
            "    colors: [RD, BK, GN, YE]",
            "",
            "connections:",
            "  - [X1_PSU: [1, 2], W1_MAIN_PWR: [1, 2], X2_ESTOP: [1, 3]]",
            "  - [X2_ESTOP: [2, 3], W1_MAIN_PWR: [1, 2], X3_MCU: [1, 2]]",
        ]
    )
    for jr in mechatronics.joint_reports[:n_joints]:
        out.append(
            f"  - [X3_MCU: [1, 2, 3, 4], W2_TRUNK_BUS: [1, 2, 3, 4], M_{jr.joint_id}: [1, 2, 3, 4]]"
        )
    return "\n".join(out) + "\n"


def master_bom_to_csv(bom_items: list[MasterBomItem]) -> str:
    """Export unit-level Master BOM to UTF-8 CSV."""
    buf = io.StringIO()
    writer = csv.writer(buf)
    writer.writerow(
        [
            "category",
            "category_label",
            "part_code",
            "part_name",
            "spec",
            "qty",
            "unit_cost_usd",
            "total_cost_usd",
        ]
    )
    for item in bom_items:
        writer.writerow(
            [
                item.category,
                item.category_label,
                item.part_code,
                item.part_name,
                item.spec,
                item.qty,
                f"{item.unit_cost_usd:.2f}",
                f"{item.total_cost_usd:.2f}",
            ]
        )
    return buf.getvalue()


def solve_belt_center_distance(
    z1: int,
    z2: int,
    pitch_mm: float = 2.0,
    belt_teeth: int = 200,
) -> dict:
    """
    Exact closed timing-belt center distance solver (GT2 / HTD):
        L = belt_teeth * pitch_mm
        b = 2*L - pitch_mm * (z1 + z2)
        C = (b + sqrt(b^2 - 8 * pitch_mm^2 * (z2 - z1)^2 / pi^2)) / 8
    """
    z1_i = max(8, int(z1))
    z2_i = max(8, int(z2))
    p = max(0.5, float(pitch_mm))
    zb = max(z1_i + z2_i + 4, int(belt_teeth))

    belt_len_mm = zb * p
    b = 2.0 * belt_len_mm - p * (z1_i + z2_i)
    disc = (b**2) - (8.0 * (p**2) * ((z2_i - z1_i) ** 2)) / (math.pi**2)
    valid = disc > 0.0 and b > 0.0
    center_dist = (b + math.sqrt(max(disc, 0.0))) / 8.0 if valid else 0.0

    r1_mm = (z1_i * p) / (2.0 * math.pi)
    r2_mm = (z2_i * p) / (2.0 * math.pi)
    if center_dist <= (r1_mm + r2_mm + 2.0):
        valid = False

    return {
        "center_distance_mm": round(center_dist, 3),
        "belt_length_mm": round(belt_len_mm, 2),
        "pitch_mm": round(p, 2),
        "z1": z1_i,
        "z2": z2_i,
        "belt_teeth": zb,
        "gear_ratio": round(z2_i / float(z1_i), 3),
        "r1_mm": round(r1_mm, 3),
        "r2_mm": round(r2_mm, 3),
        "valid": valid,
    }


def generate_lerobot_config_json(req: ModularArmRequest) -> str:
    """
    Generate a calibrated HuggingFace LeRobot follower-arm JSON configuration
    for Feetech STS3215 UART half-duplex bus arms (SO-ARM100 / SO-ARM101 style).
    """
    slots = resolve_default_slots(req)
    l1 = float(slots.upper_arm.length_mm) if slots.upper_arm else 135.0
    l2 = float(slots.forearm.length_mm) if slots.forearm else 120.0
    cfg = {
        "schema": "roboforge_lerobot_v1",
        "robot_type": "so100_follower_roboforge",
        "dof": req.dof,
        "bus_protocol": "feetech_sts3215_uart_half_duplex",
        "baudrate": 1000000,
        "port": "/dev/ttyACM0",
        "geometry_mm": {
            "base_height_mm": req.base_height_mm,
            "upper_arm_mm": l1,
            "forearm_mm": l2,
            "wrist_mm": req.wrist_mm,
        },
        "motors": {
            "shoulder_pan": {"id": 1, "model": "sts3215", "homing_offset": 0, "range_deg": [-150, 150]},
            "shoulder_lift": {"id": 2, "model": "sts3215", "homing_offset": 0, "range_deg": [-60, 90]},
            "elbow_flex": {"id": 3, "model": "sts3215", "homing_offset": 0, "range_deg": [-130, 130]},
            "wrist_flex": {"id": 4, "model": "sts3215", "homing_offset": 0, "range_deg": [-90, 90]},
            "wrist_roll": {"id": 5, "model": "sts3215", "homing_offset": 0, "range_deg": [-180, 180]},
            "gripper": {"id": 6, "model": "sts3215", "homing_offset": 0, "range_deg": [0, 100]},
        },
        "calibration_mode": "zero_center_4096_ticks",
    }
    return json.dumps(cfg, indent=2, ensure_ascii=False) + "\n"
