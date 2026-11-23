from __future__ import annotations

import csv
import io
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

    if m_type == "bldc_qdd":
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
        max_motor_voltage = max(max_motor_voltage, float(motor["voltage_v"]))

        motor_counts[motor["id"]] = motor_counts.get(motor["id"], 0) + 1
        if reducer["id"] != "direct":
            reducer_counts[reducer["id"]] = reducer_counts.get(reducer["id"], 0) + 1

        max_joint_rpm = max(1.0, float(motor["no_load_rpm"]) / ratio)
        op_rpm = min(float(spec.target_rpm), max_joint_rpm * 0.92)

        # Build 13-point Torque-Speed curve at joint output
        curve: list[TorqueSpeedPoint] = []
        for k in range(13):
            j_rpm = max_joint_rpm * (k / 12.0)
            m_cont, m_peak = _motor_torque_at_rpm(motor, j_rpm * ratio)
            curve.append(
                TorqueSpeedPoint(
                    rpm=round(j_rpm, 1),
                    continuous_torque_nm=round(m_cont * ratio * eff, 2),
                    peak_torque_nm=round(m_peak * ratio * eff, 2),
                )
            )

        m_cont_op, m_peak_op = _motor_torque_at_rpm(motor, op_rpm * ratio)
        avail_cont = max(m_cont_op * ratio * eff, 1e-4)
        avail_peak = max(m_peak_op * ratio * eff, 1e-4)

        req_rms = math.sqrt(0.65 * (tau_s**2) + 0.35 * (tau_d**2))
        req_peak = max(tau_d, 0.05)

        torque_margin_pct = ((avail_peak - req_peak) / avail_peak) * 100.0
        # Joule thermal margin ~ 1 - (I_rms / I_rated)^2
        load_ratio = req_rms / avail_cont
        thermal_margin_pct = (1.0 - (load_ratio**2)) * 100.0

        # Operating electrical currents
        cont_i = float(motor["rated_current_a"]) * min(1.35, max(0.35, load_ratio))
        peak_i = float(motor["peak_current_a"]) * min(1.25, max(0.45, req_peak / avail_peak))
        sum_cont_i += cont_i
        sum_peak_i += peak_i

        if req_peak > avail_peak or req_rms > avail_cont:
            status = "critical"
            notes.append(
                f"{jid}: torque requerido ({req_peak:.1f} N·m pico / {req_rms:.1f} N·m RMS) excede {motor['name']} + {reducer['name']}."
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

    # Power Budget
    system_v = 48.0 if max_motor_voltage >= 40.0 else 24.0
    cont_i_total = round(sum_cont_i * 0.72 + 0.8, 2)
    peak_i_total = round(sum_peak_i * 0.85 + 1.0, 2)
    cont_w = round(system_v * cont_i_total, 1)
    peak_w = round(system_v * peak_i_total, 1)
    psu_name, psu_cost = _select_psu(system_v, peak_w)
    power_awg = _select_power_awg(peak_i_total)
    signal_awg = (
        "24 AWG Par Trenzado 120Ω (CAN-FD)"
        if bus_arch == "can_fd_bus"
        else "26 AWG Apantallado (Step/Dir)"
    )

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
    if bus_arch == "can_fd_bus":
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

    total_bom_cost_usd = round(sum(item.total_cost_usd for item in master_bom), 2)
    if not notes:
        notes.append(
            f"Arquitectura {'Bus CAN-FD 4 hilos' if bus_arch == 'can_fd_bus' else 'STM32 Step/Dir'} validada a {system_v:.0f}V con margen térmico y de torque positivos."
        )

    return MechatronicsResponse(
        bus_architecture=bus_arch,
        joint_reports=joint_reports,
        power_budget=power_budget,
        master_bom=master_bom,
        total_bom_cost_usd=total_bom_cost_usd,
        notes=notes,
    )


def generate_circuit_schematic_svg(mechatronics: MechatronicsResponse, dof: int) -> str:
    """
    Generate a clean dark-mode WireViz-style electrical schematic SVG (100% offline).
    Shows PSU -> E-Stop/Fuse -> MCU -> CAN-FD Bus or Step/Dir Drivers -> Motors J1..Jn.
    """
    pb = mechatronics.power_budget
    is_can = pb.bus_architecture == "can_fd_bus"
    n_joints = max(1, min(dof, len(mechatronics.joint_reports)))
    svg_h = max(420, 120 + n_joints * 58)
    svg_w = 980

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
        f'ROBOFORGE STUDIO v4.0 — ESQUEMÁTICO ELÉCTRICO ({escape("BUS CAN-FD 4 HILOS" if is_can else "STM32 STEP/DIR")})</text>',
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
    mcu_title = "MCU STM32H7 + CAN-FD" if is_can else "MCU STM32F4 + TMC2209"
    mcu_sub = "Transceptor 5 Mbps · 120Ω" if is_can else f"{n_joints}× Drivers Step/Dir UART"
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
