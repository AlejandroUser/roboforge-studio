from __future__ import annotations

import math
from typing import Any


def _clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))


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
) -> dict[str, Any]:
    """
    Analytical 3D Inverse Kinematics solver for Articulated, SCARA, Cartesian,
    and Delta robotic manipulators with smooth workspace clamping.
    """
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
