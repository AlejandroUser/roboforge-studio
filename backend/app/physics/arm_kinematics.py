from __future__ import annotations

import copy
import json
import math
from functools import lru_cache
from pathlib import Path

from ..schemas import (
    AlertItem,
    ArmAssemblyRequest,
    BOMItem,
    JointTorqueSummary,
)
from .profiles import evaluate_printer_dfm_alerts

DATA_DIR = Path(__file__).resolve().parent.parent / "data"
GRAVITY_M_S2 = 9.81
GEARBOX_EFFICIENCY = 0.85


@lru_cache(maxsize=1)
def _read_arm_presets_raw() -> dict:
    path = DATA_DIR / "arm_presets.json"
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def load_arm_presets() -> dict:
    """Load full robotic arm assembly presets keyed under 'presets'."""
    return copy.deepcopy(_read_arm_presets_raw())


def _recommend_actuator(
    required_motor_torque_nm: float, dynamic_torque_nm: float
) -> tuple[str, float, float]:
    """
    Recommend a realistic actuator based on required motor torque & joint torque.
    Returns (label, unit_cost_usd, nominal_motor_torque_nm).
    """
    if dynamic_torque_nm <= 2.2 and required_motor_torque_nm <= 0.25:
        return "Servo Bus 25kg", 24.0, 0.35
    if required_motor_torque_nm <= 0.45:
        return "NEMA 17 + 10:1", 38.0, 0.55
    if required_motor_torque_nm <= 1.35:
        return "NEMA 23 + 20:1", 72.0, 1.80
    return "BLDC QDD Cicloidal", 135.0, 3.20


def _link_section_inertia_mm4(
    length_mm: float, link_style: str, scale: float
) -> tuple[float, float]:
    """Estimate representative link section inertia I (mm^4) and half-height c (mm)."""
    h_mm = max(24.0, min(68.0, (24.0 + 0.11 * length_mm) * scale))
    b_mm = max(18.0, min(46.0, (16.0 + 0.07 * length_mm) * scale))
    i_solid = (b_mm * (h_mm**3)) / 12.0
    style_factor = {
        "tapered_solid": 1.00,
        "hollow_cobot": 0.78,
        "organic_ibeam": 0.74,
        "rounded_truss": 0.64,
    }.get(link_style, 0.75)
    return max(i_solid * style_factor, 1e3), h_mm * 0.5


def evaluate_arm_assembly(
    req: ArmAssemblyRequest,
    segment_volumes_mm3: dict[str, float],
    material: dict,
) -> dict:
    """
    Evaluate forward kinematics (TCP position), cascaded joint static & dynamic
    torques (J3 Wrist -> J2 Elbow -> J1 Shoulder), actuator sizing, cumulative
    TCP deflection, complete BOM, and printer/thermal DFM alerts.
    """
    presets = load_arm_presets().get("presets", {})
    preset_name = presets.get(req.preset, {}).get("name", req.preset)

    density_g_cm3 = float(material["density_g_cm3"])
    if req.print_orientation == "z_vertical":
        youngs_eff_mpa = float(material["youngs_z_mpa"])
        yield_eff_mpa = float(material["yield_z_mpa"])
    else:
        youngs_eff_mpa = float(material["youngs_xy_mpa"])
        yield_eff_mpa = float(material["yield_xy_mpa"])

    # Segment printed masses (g)
    m_base_g = max(float(segment_volumes_mm3.get("base", 120_000.0)) / 1000.0, 10.0) * density_g_cm3
    m_upper_g = max(float(segment_volumes_mm3.get("upper_arm", 95_000.0)) / 1000.0, 10.0) * density_g_cm3
    m_fore_g = max(float(segment_volumes_mm3.get("forearm", 70_000.0)) / 1000.0, 10.0) * density_g_cm3
    m_wrist_g = max(float(segment_volumes_mm3.get("wrist", 35_000.0)) / 1000.0, 5.0) * density_g_cm3
    printed_mass_g = m_base_g + m_upper_g + m_fore_g + m_wrist_g

    # Forward Kinematics in XZ pitch plane (0 deg = horizontal +X)
    l1 = float(req.upper_arm_mm)
    l2 = float(req.forearm_mm)
    l3 = float(req.wrist_mm)
    h_base = float(req.base_height_mm)

    a1 = math.radians(float(req.shoulder_joint_deg))
    a2 = math.radians(float(req.shoulder_joint_deg + req.elbow_joint_deg))
    a3 = math.radians(float(req.shoulder_joint_deg + req.elbow_joint_deg + req.wrist_joint_deg))

    tcp_x = l1 * math.cos(a1) + l2 * math.cos(a2) + l3 * math.cos(a3)
    tcp_y = 0.0
    tcp_z = h_base + l1 * math.sin(a1) + l2 * math.sin(a2) + l3 * math.sin(a3)
    tcp_position_mm = [round(tcp_x, 1), round(tcp_y, 1), round(tcp_z, 1)]
    max_reach_mm = round(l1 + l2 + l3, 1)

    # Actuator masses (g) scaled realistically with payload class
    payload_kg = float(req.payload_kg)
    if payload_kg <= 0.6:
        m_mot_wrist_g, m_mot_elbow_g, m_mot_shoulder_g = 75.0, 140.0, 220.0
    elif payload_kg <= 1.8:
        m_mot_wrist_g, m_mot_elbow_g, m_mot_shoulder_g = 140.0, 320.0, 520.0
    else:
        m_mot_wrist_g, m_mot_elbow_g, m_mot_shoulder_g = 240.0, 560.0, 920.0

    # Convert to kg and meters for exact SI torque calculation (worst-case horizontal arm)
    l1_m, l2_m, l3_m = l1 / 1000.0, l2 / 1000.0, l3 / 1000.0
    m_u = m_upper_g / 1000.0
    m_f = m_fore_g / 1000.0
    m_w = m_wrist_g / 1000.0
    mm_w = m_mot_wrist_g / 1000.0
    mm_e = m_mot_elbow_g / 1000.0
    mm_s = m_mot_shoulder_g / 1000.0
    alpha = float(req.max_accel_rad_s2)

    # J3 (Wrist)
    gr_wrist = max(4.0, float(req.gear_ratio_elbow) * 0.5)
    tau_s3 = GRAVITY_M_S2 * (m_w * (l3_m * 0.5) + payload_kg * l3_m)
    i_rot3 = m_w * (l3_m**2) / 3.0 + payload_kg * (l3_m**2)
    tau_d3 = tau_s3 + i_rot3 * alpha
    req_mot3 = tau_d3 / (gr_wrist * GEARBOX_EFFICIENCY)
    act3, cost_act3, _ = _recommend_actuator(req_mot3, tau_d3)
    i3_mm4, c3_mm = _link_section_inertia_mm4(l3, req.link_style, 0.85)
    sigma3 = 1.25 * (tau_d3 * 1000.0 * c3_mm) / i3_mm4
    sf3 = yield_eff_mpa / max(sigma3, 1e-6)
    defl3 = ((payload_kg * GRAVITY_M_S2 + 0.5 * m_w * GRAVITY_M_S2) * (l3**3)) / (
        3.0 * youngs_eff_mpa * i3_mm4
    )

    # J2 (Elbow) — carries forearm + wrist motor + wrist + payload
    gr_elbow = float(req.gear_ratio_elbow)
    tau_s2 = GRAVITY_M_S2 * (
        m_f * (l2_m * 0.5)
        + mm_w * l2_m
        + m_w * (l2_m + l3_m * 0.5)
        + payload_kg * (l2_m + l3_m)
    )
    i_rot2 = (
        m_f * (l2_m**2) / 3.0
        + mm_w * (l2_m**2)
        + m_w * ((l2_m + l3_m * 0.5) ** 2)
        + payload_kg * ((l2_m + l3_m) ** 2)
    )
    tau_d2 = tau_s2 + i_rot2 * alpha
    req_mot2 = tau_d2 / (gr_elbow * GEARBOX_EFFICIENCY)
    act2, cost_act2, _ = _recommend_actuator(req_mot2, tau_d2)
    i2_mm4, c2_mm = _link_section_inertia_mm4(l2, req.link_style, 1.0)
    sigma2 = 1.25 * (tau_d2 * 1000.0 * c2_mm) / i2_mm4
    sf2 = yield_eff_mpa / max(sigma2, 1e-6)
    tip_load_fore_n = GRAVITY_M_S2 * (payload_kg + m_w + mm_w + 0.5 * m_f)
    defl2 = (tip_load_fore_n * (l2**3)) / (3.0 * youngs_eff_mpa * i2_mm4)

    # J1 (Shoulder) — carries upper arm + elbow motor + forearm + wrist motor + wrist + payload
    gr_shoulder = float(req.gear_ratio_shoulder)
    tau_s1 = GRAVITY_M_S2 * (
        m_u * (l1_m * 0.5)
        + mm_e * l1_m
        + m_f * (l1_m + l2_m * 0.5)
        + mm_w * (l1_m + l2_m)
        + m_w * (l1_m + l2_m + l3_m * 0.5)
        + payload_kg * (l1_m + l2_m + l3_m)
    )
    i_rot1 = (
        m_u * (l1_m**2) / 3.0
        + mm_e * (l1_m**2)
        + m_f * ((l1_m + l2_m * 0.5) ** 2)
        + mm_w * ((l1_m + l2_m) ** 2)
        + m_w * ((l1_m + l2_m + l3_m * 0.5) ** 2)
        + payload_kg * ((l1_m + l2_m + l3_m) ** 2)
    )
    tau_d1 = tau_s1 + i_rot1 * alpha
    req_mot1 = tau_d1 / (gr_shoulder * GEARBOX_EFFICIENCY)
    act1, cost_act1, _ = _recommend_actuator(req_mot1, tau_d1)
    i1_mm4, c1_mm = _link_section_inertia_mm4(l1, req.link_style, 1.15)
    sigma1 = 1.25 * (tau_d1 * 1000.0 * c1_mm) / i1_mm4
    sf1 = yield_eff_mpa / max(sigma1, 1e-6)
    tip_load_upper_n = GRAVITY_M_S2 * (payload_kg + m_w + mm_w + m_f + mm_e + 0.5 * m_u)
    defl1 = (tip_load_upper_n * (l1**3)) / (3.0 * youngs_eff_mpa * i1_mm4)

    tcp_deflection_mm = round(defl1 + defl2 + defl3, 3)

    joints = [
        JointTorqueSummary(
            joint_id="J1",
            joint_name="J1 Hombro (Shoulder)",
            segment_length_mm=round(l1, 1),
            segment_mass_g=round(m_upper_g, 1),
            motor_mass_g=round(m_mot_shoulder_g, 1),
            static_torque_nm=round(tau_s1, 2),
            dynamic_torque_nm=round(tau_d1, 2),
            gear_ratio=round(gr_shoulder, 1),
            required_motor_torque_nm=round(req_mot1, 3),
            recommended_actuator=act1,
            safety_factor=round(sf1, 2),
        ),
        JointTorqueSummary(
            joint_id="J2",
            joint_name="J2 Codo (Elbow)",
            segment_length_mm=round(l2, 1),
            segment_mass_g=round(m_fore_g, 1),
            motor_mass_g=round(m_mot_elbow_g, 1),
            static_torque_nm=round(tau_s2, 2),
            dynamic_torque_nm=round(tau_d2, 2),
            gear_ratio=round(gr_elbow, 1),
            required_motor_torque_nm=round(req_mot2, 3),
            recommended_actuator=act2,
            safety_factor=round(sf2, 2),
        ),
        JointTorqueSummary(
            joint_id="J3",
            joint_name="J3 Muñeca (Wrist)",
            segment_length_mm=round(l3, 1),
            segment_mass_g=round(m_wrist_g, 1),
            motor_mass_g=round(m_mot_wrist_g, 1),
            static_torque_nm=round(tau_s3, 2),
            dynamic_torque_nm=round(tau_d3, 2),
            gear_ratio=round(gr_wrist, 1),
            required_motor_torque_nm=round(req_mot3, 3),
            recommended_actuator=act3,
            safety_factor=round(sf3, 2),
        ),
    ]

    # Total arm mass (printed + motors + hardware allowance)
    extra_dof_motors_kg = max(0, req.dof - 3) * mm_w + mm_s  # base yaw + wrist roll/pitch
    hardware_kg = 0.25 + 0.08 * req.dof
    total_arm_mass_kg = round(
        (printed_mass_g / 1000.0) + mm_s + mm_e + mm_w + extra_dof_motors_kg + hardware_kg,
        2,
    )

    # Complete BOM
    printed_cost_usd = round((printed_mass_g / 1000.0) * float(material["cost_per_kg_usd"]) * 1.2 + 4.5, 2)
    extra_wrist_qty = max(1, req.dof - 3)
    bearing_main_spec = "CRB Rodillos Cruzados RB4010" if payload_kg >= 2.0 else "Rodamiento 6805-2RS (25×37×7 mm)"
    bearing_main_cost = 28.0 if payload_kg >= 2.0 else 4.5

    bom: list[BOMItem] = [
        BOMItem(
            category="Impresión 3D",
            part_name=f"Estructura impresa ({req.link_style})",
            spec=f"{material['name']} — {printed_mass_g:.0f} g totales (Base + 3 Eslabones)",
            qty=1,
            unit_cost_usd=printed_cost_usd,
            total_cost_usd=printed_cost_usd,
        ),
        BOMItem(
            category="Actuadores",
            part_name=f"Actuador Base + Hombro ({act1})",
            spec=f"Relación {gr_shoulder:.0f}:1 — {req_mot1:.2f} N·m motor",
            qty=2,
            unit_cost_usd=cost_act1,
            total_cost_usd=round(cost_act1 * 2, 2),
        ),
        BOMItem(
            category="Actuadores",
            part_name=f"Actuador Codo ({act2})",
            spec=f"Relación {gr_elbow:.0f}:1 — {req_mot2:.2f} N·m motor",
            qty=1,
            unit_cost_usd=cost_act2,
            total_cost_usd=round(cost_act2, 2),
        ),
        BOMItem(
            category="Actuadores",
            part_name=f"Actuadores Muñeca ({act3})",
            spec=f"Relación {gr_wrist:.0f}:1 — {extra_wrist_qty + 1} ejes de muñeca",
            qty=extra_wrist_qty + 1,
            unit_cost_usd=cost_act3,
            total_cost_usd=round(cost_act3 * (extra_wrist_qty + 1), 2),
        ),
        BOMItem(
            category="Rodamientos",
            part_name="Rodamientos principales Base/Hombro/Codo",
            spec=bearing_main_spec,
            qty=4,
            unit_cost_usd=bearing_main_cost,
            total_cost_usd=round(bearing_main_cost * 4, 2),
        ),
        BOMItem(
            category="Rodamientos",
            part_name="Rodamientos de muñeca 608-2RS",
            spec="8×22×7 mm sellados",
            qty=4,
            unit_cost_usd=1.2,
            total_cost_usd=4.8,
        ),
        BOMItem(
            category="Transmisión",
            part_name="Correas GT2 + Poleas / Etapas Reductoras",
            spec="GT2 6/9 mm refuerzo fibra de vidrio + poleas aluminio",
            qty=2,
            unit_cost_usd=14.0,
            total_cost_usd=28.0,
        ),
        BOMItem(
            category="Tornillería",
            part_name="Kit Tornillería DIN 912 + Insertos Térmicos",
            spec="M3 / M4 / M5 acero 12.9 + insertos latón CNC",
            qty=1,
            unit_cost_usd=16.5,
            total_cost_usd=16.5,
        ),
    ]

    estimated_total_cost_usd = round(sum(item.total_cost_usd for item in bom), 2)

    # Global alerts (including printer bed/enclosure check on longest link)
    longest_link = max(l1, l2)
    global_alerts: list[AlertItem] = []
    global_alerts.extend(
        evaluate_printer_dfm_alerts(
            req.printer_id,
            material,
            longest_link + 48.0,
            56.0,
            34.0,
        )
    )
    if min(sf1, sf2, sf3) < 1.5:
        global_alerts.append(
            AlertItem(
                level="warning",
                code="ARM_LOW_STRUCTURAL_SF",
                message=(
                    f"Factor de seguridad mínimo ({min(sf1, sf2, sf3):.2f}) en hombro/codo es ajustado "
                    "para esta carga y alcance. Cambia a PA12-CF, Aluminio o reduce el alcance."
                ),
            )
        )
    if tcp_deflection_mm > 3.0:
        global_alerts.append(
            AlertItem(
                level="warning",
                code="ARM_HIGH_TCP_DEFLECTION",
                message=(
                    f"Deflexión acumulada en el TCP ({tcp_deflection_mm:.2f} mm) supera 3.0 mm; "
                    "considera arquitectura 'hollow_cobot' o material de mayor módulo elástico."
                ),
            )
        )

    return {
        "preset_name": preset_name,
        "max_reach_mm": max_reach_mm,
        "total_arm_mass_kg": total_arm_mass_kg,
        "printed_mass_g": round(printed_mass_g, 1),
        "estimated_total_cost_usd": estimated_total_cost_usd,
        "tcp_position_mm": tcp_position_mm,
        "tcp_deflection_mm": tcp_deflection_mm,
        "joints": joints,
        "bom": bom,
        "global_alerts": global_alerts,
    }
