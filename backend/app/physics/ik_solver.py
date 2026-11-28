from __future__ import annotations

import math
from typing import Any

import numpy as np


def _clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))


def _rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0, 0.0], [s, c, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]], dtype=np.float64)


def _rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s, 0.0], [0.0, 1.0, 0.0, 0.0], [-s, 0.0, c, 0.0], [0.0, 0.0, 0.0, 1.0]], dtype=np.float64)


def _rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0, 0.0], [0.0, c, -s, 0.0], [0.0, s, c, 0.0], [0.0, 0.0, 0.0, 1.0]], dtype=np.float64)


def _trans(x: float, y: float, z: float) -> np.ndarray:
    return np.array([[1.0, 0.0, 0.0, x], [0.0, 1.0, 0.0, y], [0.0, 0.0, 1.0, z], [0.0, 0.0, 0.0, 1.0]], dtype=np.float64)


def solve_ik_lm_dls_redundant(
    target_xyz_mm: list[float] | tuple[float, float, float],
    link_lengths_mm: list[float] | tuple[float, ...],
    dof: int = 6,
    initial_angles_deg: list[float] | None = None,
    max_iter: int = 60,
    tol_mm: float = 1.0,
) -> dict[str, Any]:
    """
    Hybrid Levenberg-Marquardt Damped Least Squares (LM-DLS) numerical IK solver
    for redundant 6-DOF and 7-DOF articulated robotic chains.
    Uses numerical 3xN geometric Jacobian, adaptive damping lambda, and physical joint limits.
    """
    dof = max(6, min(7, int(dof)))
    target = np.array([float(target_xyz_mm[0]), float(target_xyz_mm[1]), float(target_xyz_mm[2])], dtype=np.float64)

    lens = [max(10.0, float(v)) for v in link_lengths_mm]
    if len(lens) == 3:
        base_h, l1, l2, l3 = 95.0, lens[0], lens[1], lens[2]
    elif len(lens) >= 4:
        base_h, l1, l2, l3 = lens[0], lens[1], lens[2], lens[3]
    else:
        base_h, l1, l2, l3 = 95.0, 200.0, 170.0, 75.0

    # Joint limits (rad) and FK chain definition (Z-up, +X forward when pitch=0)
    if dof == 6:
        # J0: Base Yaw (Z), J1: Shoulder Pitch (-Y so +pitch lifts +Z), J2: Elbow Pitch (-Y),
        # J3: Forearm Roll (X), J4: Wrist Pitch (-Y), J5: Flange Roll (X)
        lo_deg = np.array([-180.0, -75.0, -135.0, -180.0, -110.0, -180.0], dtype=np.float64)
        hi_deg = np.array([180.0, 105.0, 135.0, 180.0, 110.0, 180.0], dtype=np.float64)
    else:
        # 7-DOF redundant cobot (Franka/KUKA style):
        # J0: Base Yaw (Z), J1: Shoulder Pitch (-Y), J2: Upper-Arm Roll (X),
        # J3: Elbow Pitch (-Y), J4: Forearm Roll (X), J5: Wrist Pitch (-Y), J6: Flange Roll (X)
        lo_deg = np.array([-180.0, -85.0, -170.0, -140.0, -170.0, -115.0, -180.0], dtype=np.float64)
        hi_deg = np.array([180.0, 105.0, 170.0, 140.0, 170.0, 115.0, 180.0], dtype=np.float64)

    lo_rad = np.radians(lo_deg)
    hi_rad = np.radians(hi_deg)

    def _fk_xyz(q: np.ndarray) -> np.ndarray:
        T = _trans(0.0, 0.0, base_h) @ _rot_z(float(q[0]))
        if dof == 6:
            T = T @ _rot_y(-float(q[1])) @ _trans(l1, 0.0, 0.0)
            T = T @ _rot_y(-float(q[2])) @ _trans(l2 * 0.5, 0.0, 0.0)
            T = T @ _rot_x(float(q[3])) @ _trans(l2 * 0.5, 0.0, 0.0)
            T = T @ _rot_y(-float(q[4])) @ _trans(l3 * 0.5, 0.0, 0.0)
            T = T @ _rot_x(float(q[5])) @ _trans(l3 * 0.5, 0.0, 0.0)
        else:
            T = T @ _rot_y(-float(q[1])) @ _trans(l1 * 0.5, 0.0, 0.0)
            T = T @ _rot_x(float(q[2])) @ _trans(l1 * 0.5, 0.0, 0.0)
            T = T @ _rot_y(-float(q[3])) @ _trans(l2 * 0.5, 0.0, 0.0)
            T = T @ _rot_x(float(q[4])) @ _trans(l2 * 0.5, 0.0, 0.0)
            T = T @ _rot_y(-float(q[5])) @ _trans(l3 * 0.5, 0.0, 0.0)
            T = T @ _rot_x(float(q[6])) @ _trans(l3 * 0.5, 0.0, 0.0)
        return T[:3, 3]

    if initial_angles_deg is not None and len(initial_angles_deg) == dof:
        q = np.clip(np.radians(np.array(initial_angles_deg, dtype=np.float64)), lo_rad, hi_rad)
    else:
        q = np.zeros(dof, dtype=np.float64)
        q[0] = math.atan2(float(target[1]), float(target[0])) if (abs(target[0]) > 1e-6 or abs(target[1]) > 1e-6) else 0.0
        q[1] = math.radians(35.0)
        if dof == 6:
            q[2] = math.radians(-55.0)
            q[4] = math.radians(20.0)
        else:
            q[3] = math.radians(-55.0)
            q[5] = math.radians(20.0)
        q = np.clip(q, lo_rad, hi_rad)

    lam = 0.08
    eps = 1e-4
    eye3 = np.eye(3, dtype=np.float64)
    p = _fk_xyz(q)
    err_vec = target - p
    err_norm = float(np.linalg.norm(err_vec))
    iters = 0

    for it in range(1, max(1, int(max_iter)) + 1):
        iters = it
        if err_norm <= tol_mm:
            break

        # Numerical 3 x dof geometric Jacobian (central differences)
        J = np.empty((3, dof), dtype=np.float64)
        for j in range(dof):
            qp = q.copy()
            qm = q.copy()
            qp[j] += eps
            qm[j] -= eps
            J[:, j] = (_fk_xyz(qp) - _fk_xyz(qm)) / (2.0 * eps)

        # Levenberg-Marquardt DLS step: dq = J^T (J J^T + lam^2 I)^-1 e
        jjt = J @ J.T + (lam * lam) * eye3
        dq = J.T @ np.linalg.solve(jjt, err_vec)
        # Step norm cap to prevent overshoot near singularities
        dq_max = float(np.max(np.abs(dq)))
        if dq_max > 0.35:
            dq *= 0.35 / dq_max

        q_cand = np.clip(q + dq, lo_rad, hi_rad)
        p_cand = _fk_xyz(q_cand)
        err_cand_vec = target - p_cand
        err_cand = float(np.linalg.norm(err_cand_vec))

        if err_cand < err_norm:
            q = q_cand
            p = p_cand
            err_vec = err_cand_vec
            err_norm = err_cand
            lam = max(1e-3, lam * 0.55)
        else:
            lam = min(150.0, lam * 2.4)

    q_deg = [round(float(math.degrees(a)), 2) for a in q]
    j0_d = q_deg[0]
    j1_d = q_deg[1]
    j2_d = q_deg[2] if dof == 6 else q_deg[3]
    j3_d = q_deg[4] if dof == 6 else q_deg[5]

    return {
        "reachable": err_norm <= tol_mm,
        "reach_error_mm": round(err_norm, 3),
        "tcp_mm": [round(float(p[0]), 2), round(float(p[1]), 2), round(float(p[2]), 2)],
        "joint_angles_deg": q_deg,
        "j0_base_deg": round(j0_d, 1),
        "j1_shoulder_deg": round(j1_d, 1),
        "j2_elbow_deg": round(j2_d, 1),
        "j3_wrist_deg": round(j3_d, 1),
        "z_mm": round(float(p[2]), 1),
        "iterations": iters,
        "solver": "lm_dls_redundant",
    }


def solve_arm_ik(
    target_x_mm: float,
    target_y_mm: float,
    target_z_mm: float,
    l1_mm: float = 200.0,
    l2_mm: float = 170.0,
    l3_mm: float = 75.0,
    base_height_mm: float = 95.0,
    kinematics_type: str = "articulated",
    elbow_up: bool = True,
    dof: int = 4,
) -> dict[str, Any]:
    """
    Hybrid 3D Inverse Kinematics solver:
    - Fast analytical closed-form solver for 3-4 DOF Articulated, SCARA, Cartesian, and Delta.
    - Automatic Levenberg-Marquardt DLS numerical solver for redundant 6-7 DOF articulated chains.
    """
    if kinematics_type == "articulated" and int(dof) >= 6:
        return solve_ik_lm_dls_redundant(
            target_xyz_mm=[target_x_mm, target_y_mm, target_z_mm],
            link_lengths_mm=[base_height_mm, l1_mm, l2_mm, l3_mm],
            dof=int(dof),
        )
    tx = float(target_x_mm)
    ty = float(target_y_mm)
    tz = float(target_z_mm)
    l1 = max(20.0, float(l1_mm))
    l2 = max(20.0, float(l2_mm))
    l3 = max(0.0, float(l3_mm))
    h_base = max(20.0, float(base_height_mm))

    # 1. Cartesian Gantry & Delta Parallel (direct XYZ workspace clamping)
    if kinematics_type in ("cartesian_gantry", "delta_parallel"):
        max_xy = max(80.0, l1 + l2 * 0.5) if kinematics_type == "cartesian_gantry" else max(80.0, l1 * 0.85)
        min_z = 20.0
        max_z = h_base + l1 * 0.6
        cx = _clamp(tx, -max_xy, max_xy)
        cy = _clamp(ty, -max_xy, max_xy)
        cz = _clamp(tz, min_z, max_z)
        err = math.sqrt((cx - tx) ** 2 + (cy - ty) ** 2 + (cz - tz) ** 2)
        return {
            "reachable": err < 0.5,
            "reach_error_mm": round(err, 2),
            "tcp_mm": [round(cx, 1), round(cy, 1), round(cz, 1)],
            "j0_base_deg": round(_clamp(cx * 0.5, -180.0, 180.0), 1),
            "j1_shoulder_deg": round(_clamp(cy * 0.3, -60.0, 90.0), 1),
            "j2_elbow_deg": round(_clamp((cz - h_base) * 0.4, -120.0, 120.0), 1),
            "j3_wrist_deg": 0.0,
            "z_mm": round(cz, 1),
        }

    # 2. SCARA 4-DOF (2D horizontal planar IK + prismatic Z)
    if kinematics_type == "scara":
        r_raw = math.hypot(tx, ty)
        r_min = abs(l1 - l2) + 1.0
        r_max = (l1 + l2) - 1.0
        r_eff = _clamp(r_raw, r_min, r_max)
        z_eff = _clamp(tz, 0.0, h_base)

        if r_raw > 1e-6:
            ux, uy = tx / r_raw, ty / r_raw
        else:
            ux, uy = 1.0, 0.0
        px, py = ux * r_eff, uy * r_eff

        cos_j2 = _clamp((r_eff**2 - l1**2 - l2**2) / (2.0 * l1 * l2), -1.0, 1.0)
        j2_rad = -math.acos(cos_j2) if elbow_up else math.acos(cos_j2)
        phi = math.atan2(py, px)
        psi = math.atan2(l2 * math.sin(j2_rad), l1 + l2 * math.cos(j2_rad))
        j1_rad = phi - psi

        fx = l1 * math.cos(j1_rad) + l2 * math.cos(j1_rad + j2_rad)
        fy = l1 * math.sin(j1_rad) + l2 * math.sin(j1_rad + j2_rad)
        err = math.sqrt((fx - tx) ** 2 + (fy - ty) ** 2 + (z_eff - tz) ** 2)
        return {
            "reachable": err < 0.5,
            "reach_error_mm": round(err, 2),
            "tcp_mm": [round(fx, 1), round(fy, 1), round(z_eff, 1)],
            "j0_base_deg": 0.0,
            "j1_shoulder_deg": round(math.degrees(j1_rad), 1),
            "j2_elbow_deg": round(math.degrees(j2_rad), 1),
            "j3_wrist_deg": round(-math.degrees(j1_rad + j2_rad), 1),
            "z_mm": round(z_eff, 1),
        }

    # 3. Articulated 3D IK (keeps horizontal end-effector: j1 + j2 + j3 = 0)
    j0_rad = math.atan2(ty, tx) if (abs(tx) > 1e-6 or abs(ty) > 1e-6) else 0.0
    r_tcp = math.hypot(tx, ty)

    # Wrist pivot target (subtracting horizontal wrist length l3)
    r_w = max(0.0, r_tcp - l3)
    z_w = tz - h_base
    d_raw = math.hypot(r_w, z_w)

    d_min = abs(l1 - l2) + 1.0
    d_max = (l1 + l2) - 1.0
    d_eff = _clamp(d_raw, d_min, d_max)

    if d_raw > 1e-6:
        scale = d_eff / d_raw
        r_w_eff = r_w * scale
        z_w_eff = z_w * scale
    else:
        r_w_eff = d_eff
        z_w_eff = 0.0

    cos_j2 = _clamp((d_eff**2 - l1**2 - l2**2) / (2.0 * l1 * l2), -1.0, 1.0)
    j2_rad = -math.acos(cos_j2) if elbow_up else math.acos(cos_j2)
    phi = math.atan2(z_w_eff, r_w_eff)
    psi = math.atan2(l2 * math.sin(j2_rad), l1 + l2 * math.cos(j2_rad))
    j1_rad = phi - psi
    j3_rad = -(j1_rad + j2_rad)

    # Clamp to safe physical joint limits
    j0_deg = _clamp(math.degrees(j0_rad), -180.0, 180.0)
    j1_deg = _clamp(math.degrees(j1_rad), -60.0, 90.0)
    j2_deg = _clamp(math.degrees(j2_rad), -130.0, 130.0)
    j3_deg = _clamp(math.degrees(j3_rad), -90.0, 90.0)

    # Recompute achieved FK position
    a0 = math.radians(j0_deg)
    a1 = math.radians(j1_deg)
    a2 = math.radians(j1_deg + j2_deg)
    a3 = math.radians(j1_deg + j2_deg + j3_deg)

    r_fk = l1 * math.cos(a1) + l2 * math.cos(a2) + l3 * math.cos(a3)
    fx = r_fk * math.cos(a0)
    fy = r_fk * math.sin(a0)
    fz = h_base + l1 * math.sin(a1) + l2 * math.sin(a2) + l3 * math.sin(a3)
    err = math.sqrt((fx - tx) ** 2 + (fy - ty) ** 2 + (fz - tz) ** 2)

    return {
        "reachable": err < 1.0,
        "reach_error_mm": round(err, 2),
        "tcp_mm": [round(fx, 1), round(fy, 1), round(fz, 1)],
        "j0_base_deg": round(j0_deg, 1),
        "j1_shoulder_deg": round(j1_deg, 1),
        "j2_elbow_deg": round(j2_deg, 1),
        "j3_wrist_deg": round(j3_deg, 1),
        "z_mm": round(fz, 1),
    }


def compute_caliper_compensation(
    measured_m3_mm: float,
    measured_bore_mm: float,
    nominal_m3_mm: float = 3.2,
    nominal_bore_mm: float = 22.2,
    process_type: str = "msla_resin",
) -> dict[str, Any]:
    """
    Compute dimensional tolerance compensation from digital caliper measurements
    on calibration_fit_test.stl (M3 hole & bearing/horn bore).
    """
    err_m3 = float(measured_m3_mm) - float(nominal_m3_mm)
    err_bore = float(measured_bore_mm) - float(nominal_bore_mm)
    avg_err = 0.5 * (err_m3 + err_bore)

    # Radial XY hole compensation: positive value expands undersized holes
    xy_hole_comp = round(-avg_err * 0.5, 3)
    rec_clearance = round(_clamp(0.15 - avg_err * 0.5, 0.05, 0.45), 3)

    if abs(avg_err) <= 0.08:
        status = "perfect"
    elif avg_err < -0.08:
        status = "tight"
    else:
        status = "loose"

    is_msla = process_type == "msla_resin"
    if is_msla:
        exposure_delta_s = 0.0 if status == "perfect" else round(_clamp(avg_err * 1.1, -0.6, 0.6), 2)
        if status == "perfect":
            diagnosis = (
                f"Calibración excelente (error promedio {avg_err:+.2f} mm). "
                "Tus parámetros actuales en Elegoo SatelLite / Chitubox están listos para imprimir todo el brazo."
            )
        elif status == "tight":
            diagnosis = (
                f"Agujeros apretados (error promedio {avg_err:+.2f} mm por sobre-exposición). "
                f"Reduce la exposición en {exposure_delta_s:+.2f} s en SatelLite/Chitubox o aplica "
                f"XY Hole Compensation {xy_hole_comp:+.3f} mm (holgura sugerida {rec_clearance:.2f} mm)."
            )
        else:
            diagnosis = (
                f"Agujeros holgados (error promedio {avg_err:+.2f} mm por sub-exposición/contracción). "
                f"Sube la exposición {exposure_delta_s:+.2f} s en SatelLite/Chitubox o ajusta "
                f"XY Hole Compensation a {xy_hole_comp:+.3f} mm."
            )
    else:
        exposure_delta_s = 0.0
        if status == "perfect":
            diagnosis = (
                f"Calibración FDM excelente (error promedio {avg_err:+.2f} mm). "
                "Puedes imprimir las piezas directamente en Bambu Studio / OrcaSlicer."
            )
        elif status == "tight":
            diagnosis = (
                f"Agujeros apretados en FDM (error promedio {avg_err:+.2f} mm). "
                f"En OrcaSlicer / Bambu Studio configura 'XY Hole Compensation' en {xy_hole_comp:+.3f} mm "
                f"y usa holgura {rec_clearance:.2f} mm en RoboForge."
            )
        else:
            diagnosis = (
                f"Agujeros holgados en FDM (error promedio {avg_err:+.2f} mm). "
                f"En OrcaSlicer / Bambu Studio ajusta 'XY Hole Compensation' a {xy_hole_comp:+.3f} mm "
                f"y reduce la holgura a {rec_clearance:.2f} mm."
            )

    return {
        "status": status,
        "process_type": process_type,
        "error_m3_mm": round(err_m3, 3),
        "error_bore_mm": round(err_bore, 3),
        "average_error_mm": round(avg_err, 3),
        "xy_hole_compensation_mm": xy_hole_comp,
        "recommended_clearance_mm": rec_clearance,
        "exposure_delta_s": exposure_delta_s,
        "diagnosis": diagnosis,
    }
