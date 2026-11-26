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


def _link_section_props(
    length_mm: float, link_style: str, scale: float
) -> tuple[float, float, float, float]:
    """Estimate representative link section inertia I (mm^4), half-height c (mm), height h (mm), and area A (mm^2)."""
    h_mm = max(24.0, min(68.0, (24.0 + 0.11 * length_mm) * scale))
    b_mm = max(18.0, min(46.0, (16.0 + 0.07 * length_mm) * scale))
    i_solid = (b_mm * (h_mm**3)) / 12.0
    a_solid = b_mm * h_mm
    style_factor = {
        "tapered_solid": 1.00,
        "hollow_cobot": 0.78,
        "organic_ibeam": 0.74,
        "rounded_truss": 0.64,
    }.get(link_style, 0.75)
    return max(i_solid * style_factor, 1e3), h_mm * 0.5, h_mm, max(a_solid * style_factor, 40.0)


def _link_section_inertia_mm4(
    length_mm: float, link_style: str, scale: float
) -> tuple[float, float]:
    """Estimate representative link section inertia I (mm^4) and half-height c (mm)."""
    i_mm4, c_mm, _, _ = _link_section_props(length_mm, link_style, scale)
    return i_mm4, c_mm


def compute_rnea_lugre_torques(
    l_m: list[float] | tuple[float, float, float],
    m_links_kg: list[float] | tuple[float, float, float],
    m_motors_kg: list[float] | tuple[float, float, float],
    payload_kg: float,
    q_rad: list[float] | tuple[float, float, float],
    qd_rad_s: list[float] | tuple[float, float, float],
    qdd_rad_s2: list[float] | tuple[float, float, float],
    gear_type: str = "servo",
) -> dict:
    """
    Exact 3-DOF planar coupled dynamics (M(q)qdd + C(q,qd)qd + g(q)) via analytical
    Jacobians & Christoffel symbols, plus steady-state Stribeck/LuGre joint friction:
        tau_f(qd) = sign(qd) * [tau_c + (tau_s - tau_c) * exp(-(qd / v_s)^2)] + sigma2 * qd
    Returns per-joint static, inertial, Coriolis/centrifugal, friction, total torques,
    and overhead percentage vs. quasi-static gravity torque.
    """
    l1, l2, l3 = float(l_m[0]), float(l_m[1]), float(l_m[2])
    m1, m2, m3 = float(m_links_kg[0]), float(m_links_kg[1]), float(m_links_kg[2])
    # m_motors_kg = (shoulder_motor_fixed, elbow_motor_at_l1, wrist_motor_at_l2)
    mm2 = float(m_motors_kg[1]) if len(m_motors_kg) > 1 else 0.0
    mm3 = float(m_motors_kg[2]) if len(m_motors_kg) > 2 else 0.0
    mp = float(payload_kg)

    ic1 = m1 * (l1**2) / 12.0
    ic2 = m2 * (l2**2) / 12.0
    ic3 = m3 * (l3**2) / 12.0

    def _mass_matrix_and_gravity(q: tuple[float, float, float]) -> tuple[list[list[float]], list[float]]:
        th1 = q[0]
        th2 = q[0] + q[1]
        th3 = q[0] + q[1] + q[2]
        s1, c1 = math.sin(th1), math.cos(th1)
        s2, c2 = math.sin(th2), math.cos(th2)
        s3, c3 = math.sin(th3), math.cos(th3)

        # Bodies: (mass, I_c, J_vx[3], J_vz[3], J_w[3])
        bodies = [
            # Link 1 CoM
            (m1, ic1, (-0.5 * l1 * s1, 0.0, 0.0), (0.5 * l1 * c1, 0.0, 0.0), (1.0, 0.0, 0.0)),
            # Elbow motor at tip of Link 1
            (mm2, 0.0, (-l1 * s1, 0.0, 0.0), (l1 * c1, 0.0, 0.0), (1.0, 0.0, 0.0)),
            # Link 2 CoM
            (
                m2,
                ic2,
                (-l1 * s1 - 0.5 * l2 * s2, -0.5 * l2 * s2, 0.0),
                (l1 * c1 + 0.5 * l2 * c2, 0.5 * l2 * c2, 0.0),
                (1.0, 1.0, 0.0),
            ),
            # Wrist motor at tip of Link 2
            (
                mm3,
                0.0,
                (-l1 * s1 - l2 * s2, -l2 * s2, 0.0),
                (l1 * c1 + l2 * c2, l2 * c2, 0.0),
                (1.0, 1.0, 0.0),
            ),
            # Link 3 CoM
            (
                m3,
                ic3,
                (-l1 * s1 - l2 * s2 - 0.5 * l3 * s3, -l2 * s2 - 0.5 * l3 * s3, -0.5 * l3 * s3),
                (l1 * c1 + l2 * c2 + 0.5 * l3 * c3, l2 * c2 + 0.5 * l3 * c3, 0.5 * l3 * c3),
                (1.0, 1.0, 1.0),
            ),
            # Payload at TCP
            (
                mp,
                0.0,
                (-l1 * s1 - l2 * s2 - l3 * s3, -l2 * s2 - l3 * s3, -l3 * s3),
                (l1 * c1 + l2 * c2 + l3 * c3, l2 * c2 + l3 * c3, l3 * c3),
                (1.0, 1.0, 1.0),
            ),
        ]

        M = [[0.0, 0.0, 0.0] for _ in range(3)]
        g_vec = [0.0, 0.0, 0.0]
        for mass, ic, jvx, jvz, jw in bodies:
            for i in range(3):
                g_vec[i] += mass * GRAVITY_M_S2 * jvz[i]
                for j in range(3):
                    M[i][j] += mass * (jvx[i] * jvx[j] + jvz[i] * jvz[j]) + ic * jw[i] * jw[j]
        return M, g_vec

    q_t = (float(q_rad[0]), float(q_rad[1]), float(q_rad[2]))
    qd = (float(qd_rad_s[0]), float(qd_rad_s[1]), float(qd_rad_s[2]))
    qdd = (float(qdd_rad_s2[0]), float(qdd_rad_s2[1]), float(qdd_rad_s2[2]))

    M, g_vec = _mass_matrix_and_gravity(q_t)

    # Coupled inertial torque: M(q) * qdd (including off-diagonal M_ij terms)
    tau_iner = [
        sum(M[i][j] * qdd[j] for j in range(3))
        for i in range(3)
    ]

    # Central-difference dM/dq_k -> exact Christoffel symbols -> C(q, qd) * qd
    eps = 1e-5
    dM = [[[0.0] * 3 for _ in range(3)] for _ in range(3)]  # dM[k][i][j] = dM_ij / dq_k
    for k in range(3):
        qp = list(q_t)
        qm = list(q_t)
        qp[k] += eps
        qm[k] -= eps
        Mp, _ = _mass_matrix_and_gravity((qp[0], qp[1], qp[2]))
        Mm, _ = _mass_matrix_and_gravity((qm[0], qm[1], qm[2]))
        for i in range(3):
            for j in range(3):
                dM[k][i][j] = (Mp[i][j] - Mm[i][j]) / (2.0 * eps)

    tau_cor = [0.0, 0.0, 0.0]
    for i in range(3):
        acc = 0.0
        for j in range(3):
            for k in range(3):
                gamma_ijk = 0.5 * (dM[k][i][j] + dM[j][i][k] - dM[i][j][k])
                acc += gamma_ijk * qd[j] * qd[k]
        tau_cor[i] = acc

    # Steady-state Stribeck / LuGre friction per gearbox class
    friction_params = {
        "servo": (0.04, 0.050, 1.35, 0.25, 0.015),
        "planetary": (0.08, 0.065, 1.32, 0.30, 0.028),
        "harmonic": (0.18, 0.085, 1.45, 0.20, 0.060),
    }
    tc_base, tc_load, ts_ratio, v_s, sigma2 = friction_params.get(
        gear_type, friction_params["servo"]
    )

    tau_fric = [0.0, 0.0, 0.0]
    for i in range(3):
        tau_c = tc_base + tc_load * abs(g_vec[i])
        tau_s = tau_c * ts_ratio
        v = qd[i]
        if abs(v) < 1e-6:
            sgn = 1.0 if (g_vec[i] + tau_iner[i]) >= 0.0 else -1.0
            tau_fric[i] = sgn * tau_s
        else:
            sgn = 1.0 if v > 0.0 else -1.0
            stribeck = tau_c + (tau_s - tau_c) * math.exp(-((v / v_s) ** 2))
            tau_fric[i] = sgn * stribeck + sigma2 * v

    joint_ids = ("J1", "J2", "J3")
    tau_total = [
        abs(g_vec[i]) + abs(tau_iner[i]) + abs(tau_cor[i]) + abs(tau_fric[i])
        for i in range(3)
    ]
    tau_quasistatic_j1 = max(abs(g_vec[0]), 1e-4)
    overhead_pct = ((tau_total[0] - tau_quasistatic_j1) / tau_quasistatic_j1) * 100.0

    return {
        "gear_type": gear_type,
        "mass_matrix": [[round(M[i][j], 5) for j in range(3)] for i in range(3)],
        "tau_static_nm": {joint_ids[i]: round(abs(g_vec[i]), 4) for i in range(3)},
        "tau_inertial_nm": {joint_ids[i]: round(abs(tau_iner[i]), 4) for i in range(3)},
        "tau_coriolis_nm": {joint_ids[i]: round(abs(tau_cor[i]), 4) for i in range(3)},
        "tau_friction_nm": {joint_ids[i]: round(abs(tau_fric[i]), 4) for i in range(3)},
        "tau_total_nm": {joint_ids[i]: round(tau_total[i], 4) for i in range(3)},
        "overhead_vs_quasistatic_pct": round(overhead_pct, 1),
    }


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

    # Coupled 3-DOF RNEA + Stribeck/LuGre Friction (evaluated at active pose with non-zero velocity)
    gear_type = (
        "harmonic"
        if float(req.gear_ratio_shoulder) >= 40.0
        else ("planetary" if float(req.gear_ratio_shoulder) >= 10.0 else "servo")
    )
    q_rnea = (
        a1,
        math.radians(float(req.elbow_joint_deg) if abs(float(req.elbow_joint_deg)) > 5.0 else -35.0),
        math.radians(float(req.wrist_joint_deg)),
    )
    qd_rnea = (1.6, 2.0, 2.4)
    qdd_rnea = (alpha, alpha * 0.6, alpha * 0.4)
    rnea = compute_rnea_lugre_torques(
        l_m=(l1_m, l2_m, l3_m),
        m_links_kg=(m_u, m_f, m_w),
        m_motors_kg=(mm_s, mm_e, mm_w),
        payload_kg=payload_kg,
        q_rad=q_rnea,
        qd_rad_s=qd_rnea,
        qdd_rad_s2=qdd_rnea,
        gear_type=gear_type,
    )

    # Timoshenko shear modulus G_xz = E / (2*(1 + nu)), kappa = 5/6 for short links (L/h < 5)
    g_xz_mpa = youngs_eff_mpa / (2.0 * (1.0 + 0.35))
    kappa_timoshenko = 5.0 / 6.0
    timoshenko_shear_applied = False

    def _beam_deflection_mm(load_n: float, length_mm: float, i_mm4: float, h_mm: float, a_mm2: float) -> float:
        nonlocal timoshenko_shear_applied
        eb_defl = (load_n * (length_mm**3)) / (3.0 * youngs_eff_mpa * i_mm4)
        if (length_mm / max(h_mm, 1.0)) < 5.0:
            timoshenko_shear_applied = True
            shear_defl = (load_n * length_mm) / (kappa_timoshenko * g_xz_mpa * a_mm2)
            return eb_defl + shear_defl
        return eb_defl

    # J3 (Wrist)
    gr_wrist = max(4.0, float(req.gear_ratio_elbow) * 0.5)
    tau_s3 = GRAVITY_M_S2 * (m_w * (l3_m * 0.5) + payload_kg * l3_m)
    i_rot3 = m_w * (l3_m**2) / 3.0 + payload_kg * (l3_m**2)
    tau_d3 = max(
        tau_s3 + i_rot3 * alpha + rnea["tau_coriolis_nm"]["J3"] + rnea["tau_friction_nm"]["J3"],
        rnea["tau_total_nm"]["J3"],
    )
    req_mot3 = tau_d3 / (gr_wrist * GEARBOX_EFFICIENCY)
    act3, cost_act3, _ = _recommend_actuator(req_mot3, tau_d3)
    i3_mm4, c3_mm, h3_mm, a3_mm2 = _link_section_props(l3, req.link_style, 0.85)
    sigma3 = 1.25 * (tau_d3 * 1000.0 * c3_mm) / i3_mm4
    sf3 = yield_eff_mpa / max(sigma3, 1e-6)
    defl3 = _beam_deflection_mm(
        payload_kg * GRAVITY_M_S2 + 0.5 * m_w * GRAVITY_M_S2, l3, i3_mm4, h3_mm, a3_mm2
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
    tau_d2 = max(
        tau_s2 + i_rot2 * alpha + rnea["tau_coriolis_nm"]["J2"] + rnea["tau_friction_nm"]["J2"],
        rnea["tau_total_nm"]["J2"],
    )
    req_mot2 = tau_d2 / (gr_elbow * GEARBOX_EFFICIENCY)
    act2, cost_act2, _ = _recommend_actuator(req_mot2, tau_d2)
    i2_mm4, c2_mm, h2_mm, a2_mm2 = _link_section_props(l2, req.link_style, 1.0)
    sigma2 = 1.25 * (tau_d2 * 1000.0 * c2_mm) / i2_mm4
    sf2 = yield_eff_mpa / max(sigma2, 1e-6)
    tip_load_fore_n = GRAVITY_M_S2 * (payload_kg + m_w + mm_w + 0.5 * m_f)
    defl2 = _beam_deflection_mm(tip_load_fore_n, l2, i2_mm4, h2_mm, a2_mm2)

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
    tau_d1 = max(
        tau_s1 + i_rot1 * alpha + rnea["tau_coriolis_nm"]["J1"] + rnea["tau_friction_nm"]["J1"],
        rnea["tau_total_nm"]["J1"],
    )
    req_mot1 = tau_d1 / (gr_shoulder * GEARBOX_EFFICIENCY)
    act1, cost_act1, _ = _recommend_actuator(req_mot1, tau_d1)
    i1_mm4, c1_mm, h1_mm, a1_mm2 = _link_section_props(l1, req.link_style, 1.15)
    sigma1 = 1.25 * (tau_d1 * 1000.0 * c1_mm) / i1_mm4
    sf1 = yield_eff_mpa / max(sigma1, 1e-6)
    tip_load_upper_n = GRAVITY_M_S2 * (payload_kg + m_w + mm_w + m_f + mm_e + 0.5 * m_u)
    defl1 = _beam_deflection_mm(tip_load_upper_n, l1, i1_mm4, h1_mm, a1_mm2)

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

    if rnea["overhead_vs_quasistatic_pct"] > 20.0:
        cor_f_j1 = rnea["tau_coriolis_nm"]["J1"] + rnea["tau_friction_nm"]["J1"]
        cor_f_pct = (cor_f_j1 / max(rnea["tau_static_nm"]["J1"], 1e-4)) * 100.0
        global_alerts.append(
            AlertItem(
                level="warning",
                code="ARM_RNEA_LUGRE_OVERHEAD",
                message=(
                    f"Dinámica acoplada RNEA + fricción Stribeck/LuGre ({gear_type}) añade "
                    f"+{rnea['overhead_vs_quasistatic_pct']:.1f}% sobre el torque cuasi-estático en J1 "
                    f"(Coriolis + LuGre = +{cor_f_pct:.1f}%)."
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
        "rnea_dynamics": rnea,
        "timoshenko_shear_applied": timoshenko_shear_applied,
    }
