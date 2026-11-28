"""RoboForge v14.5 — Impartial Adversarial Competition Referee & 100-Point Scorecard.

Evaluates any competing 3D drone/robotic design across 5 independent pillars:
  1. Flight / Performance (30 pts): TWR, safe 3D-printed Vmax, efficiency g/W
  2. Modal Resonance Margin (20 pts): 3D Hex8 eigsh vs. 1x motor RPM & BPF
  3. 20G Crash Survivability (20 pts): 20G impact safety factor
  4. Monte Carlo Reliability (20 pts): 50k-sample Pf (%) & beta index under FDM dispersion
  5. External Toolchain Manufacturing (10 pts): OrcaSlicer 2.5 (.3MF) + FreeCAD 1.1 (.STEP)
"""

from __future__ import annotations

import math
import os
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Any

import trimesh

from .adversarial_bench import (
    run_crash_20g_check,
    run_modal_resonance_check,
    run_monte_carlo_reliability,
)


def _find_exe(candidates: list[str], extra_globs: list[str]) -> Path | None:
    for name in candidates:
        found = shutil.which(name)
        if found:
            return Path(found)
    roots = [
        Path(os.environ.get("ProgramFiles", r"C:\Program Files")),
        Path(os.environ.get("LOCALAPPDATA", "")) / "Programs",
    ]
    for root in roots:
        if not root.exists():
            continue
        for pat in extra_globs:
            for hit in root.glob(pat):
                if hit.is_file():
                    return hit
    return None


def _run_external_checks(frame_stl: Path, step_path: Path | None) -> dict[str, Any]:
    orca_ok = False
    fc_ok = True if step_path is None else False

    orca = _find_exe(["orca-slicer", "orca-slicer.exe"], ["OrcaSlicer/orca-slicer.exe"])
    if orca and frame_stl.exists():
        with tempfile.TemporaryDirectory() as td:
            out_3mf = Path(td) / "referee.3mf"
            p = subprocess.run([str(orca), "--export-3mf", str(out_3mf), str(frame_stl)], capture_output=True, text=True, timeout=60)
            orca_ok = p.returncode == 0 and out_3mf.exists() and out_3mf.stat().st_size > 1000

    fc = _find_exe(["freecadcmd", "freecadcmd.exe", "FreeCADCmd.exe"], ["FreeCAD*/bin/freecadcmd.exe"])
    if fc and step_path and step_path.exists():
        with tempfile.TemporaryDirectory() as td:
            sc = Path(td) / "chk.py"
            sc.write_text(f"import Part\ns=Part.read(r'{step_path}')\nprint('FC_OK', s.isValid(), len(s.Solids))\n", encoding="utf-8")
            p = subprocess.run([str(fc), str(sc)], capture_output=True, text=True, timeout=60)
            fc_ok = p.returncode == 0 and "FC_OK True" in p.stdout

    return {"orcaslicer_ok": orca_ok, "freecad_step_ok": fc_ok}


def evaluate_competitor_adversarial(
    *,
    name: str,
    frame_stl: Path,
    auw_g: float,
    bemt_thrust_per_motor_g: float,
    efficiency_g_per_w: float,
    prop_root_sf: float,
    motor_rpm: float = 22000.0,
    blade_count: int = 3,
    e_mpa: float = 5200.0,
    yield_z_mpa: float = 45.0,
    density_g_cm3: float = 1.18,
    step_path: Path | None = None,
    run_external_tools: bool = True,
) -> dict[str, Any]:
    """Run full adversarial evaluation and return 0-100 Scorecard + discovered failure modes."""
    mesh = trimesh.load(str(frame_stl), force="mesh")
    if isinstance(mesh, trimesh.Scene):
        mesh = mesh.dump(concatenate=True)

    # 1. Flight & Safe 3D-Printed Speed
    t_tot_n = 4.0 * bemt_thrust_per_motor_g / 1000.0 * 9.80665
    w_n = auw_g / 1000.0 * 9.80665
    twr = (4.0 * bemt_thrust_per_motor_g) / max(auw_g, 1.0)
    safe_ratio = 1.0 if prop_root_sf >= 1.5 else math.sqrt(max(prop_root_sf, 0.05) / 1.5)
    t_safe_n = t_tot_n * (safe_ratio ** 2)
    f_horiz_safe = math.sqrt(max(0.0, t_safe_n**2 - w_n**2)) if t_safe_n > w_n else 0.0
    vmax_safe_kmh = math.sqrt(2.0 * f_horiz_safe / (1.225 * 0.92 * 0.0145)) * 3.6

    # 2. Modal Resonance (3D Hex8 eigsh)
    modal = run_modal_resonance_check(mesh, e_mpa=e_mpa, density_g_cm3=density_g_cm3, motor_rpm=motor_rpm, blade_count=blade_count, fix_mode="center_hub")

    # 3. 20G Crash Impact
    crash = run_crash_20g_check(mesh, e_mpa=e_mpa, yield_mpa=yield_z_mpa, total_mass_kg=auw_g / 1000.0, g_load=20.0, fix_mode="center_hub")

    # 4. Monte Carlo Reliability on governing stress (worst of prop root vs 20G frame)
    prop_root_stress = yield_z_mpa / max(prop_root_sf, 0.05)
    governing_stress = max(prop_root_stress, crash["estimated_peak_stress_mpa"])
    mc = run_monte_carlo_reliability(nominal_stress_mpa=governing_stress, yield_mean_mpa=yield_z_mpa)

    # 5. External Manufacturing Checks
    ext = _run_external_checks(frame_stl, step_path) if run_external_tools else {"orcaslicer_ok": True, "freecad_step_ok": True}

    # --- Compute transparent 0-100 Scorecard ---
    pts_flight = min(30.0, (min(vmax_safe_kmh, 260.0) / 260.0) * 18.0 + (min(efficiency_g_per_w, 4.0) / 4.0) * 12.0)
    pts_modal = 20.0 if modal["status"] == "pass" else (11.0 if modal["status"] == "warn" else 3.0)
    pts_crash = min(20.0, (min(crash["crash_safety_factor"], 2.0) / 2.0) * 20.0)
    pts_mc = max(0.0, 20.0 * (1.0 - min(mc["pf_structural_pct"], 25.0) / 25.0))
    pts_mfg = (5.0 if mesh.is_watertight else 0.0) + (3.0 if ext["orcaslicer_ok"] else 0.0) + (2.0 if ext["freecad_step_ok"] else 0.0)
    total_score = round(pts_flight + pts_modal + pts_crash + pts_mc + pts_mfg, 1)

    failures: list[str] = []
    if prop_root_sf < 1.5:
        failures.append(f"Hélice insegura a {motor_rpm:.0f} RPM (FS={prop_root_sf:.2f} < 1.5): velocidad segura limitada a {vmax_safe_kmh:.1f} km/h.")
    failures.extend(modal["warnings"])
    failures.extend(crash["warnings"])
    failures.extend(mc["warnings"])
    if not mesh.is_watertight:
        failures.append("Malla de chasis no-manifold (watertight=False).")

    return {
        "name": name,
        "total_score_100": total_score,
        "breakdown": {
            "flight_30": round(pts_flight, 1),
            "modal_20": round(pts_modal, 1),
            "crash_20g_20": round(pts_crash, 1),
            "monte_carlo_20": round(pts_mc, 1),
            "manufacturing_10": round(pts_mfg, 1),
        },
        "flight": {
            "auw_g": round(auw_g, 1),
            "twr": round(twr, 2),
            "vmax_safe_kmh": round(vmax_safe_kmh, 1),
            "efficiency_g_per_w": round(efficiency_g_per_w, 2),
            "prop_root_sf": round(prop_root_sf, 2),
        },
        "modal": modal,
        "crash_20g": crash,
        "monte_carlo": mc,
        "external": ext,
        "discovered_failures": failures,
    }


def run_extreme_drone_gauntlet(
    *,
    name: str,
    frame_stl: Path,
    auw_base_g: float,
    bemt_thrust_per_motor_g: float,
    bemt_power_per_motor_w: float,
    efficiency_g_per_w: float,
    prop_root_sf: float,
    motor_rpm: float = 22000.0,
    e_mpa: float = 5200.0,
    yield_z_mpa: float = 45.0,
    density_g_cm3: float = 1.18,
    extra_payload_g: float = 170.0,  # +170g on top of 180g = 350g heavy cine/LiDAR payload
    altitude_m: float = 2500.0,
    wind_gust_kmh: float = 55.0,
    motor_temp_c: float = 65.0,
) -> dict[str, Any]:
    """Extreme Aerospace Red-Team Drone Gauntlet (v14.6) — 5 brutal failure-discovery tests."""
    mesh = trimesh.load(str(frame_stl), force="mesh")
    if isinstance(mesh, trimesh.Scene):
        mesh = mesh.dump(concatenate=True)

    auw_heavy_g = auw_base_g + extra_payload_g
    w_n = auw_heavy_g / 1000.0 * 9.80665

    # 1. High Altitude (2,500m, rho=0.957 kg/m^3 -> -21.9% air density) + 55 km/h Crosswind + 350g Payload
    rho_alt = 1.225 * ((1.0 - 2.25577e-5 * altitude_m) ** 4.25588)
    rho_ratio = rho_alt / 1.225
    t_alt_tot_n = (4.0 * bemt_thrust_per_motor_g * rho_ratio) / 1000.0 * 9.80665
    twr_alt = t_alt_tot_n / max(w_n, 1e-6)
    v_wind_ms = wind_gust_kmh / 3.6
    f_wind_n = 0.5 * rho_alt * 0.95 * 0.0165 * (v_wind_ms ** 2)
    req_tilt_deg = math.degrees(math.atan2(f_wind_n, w_n))
    hover_throttle_alt_pct = min(150.0, (math.hypot(w_n, f_wind_n) / max(t_alt_tot_n, 1e-6)) * 100.0)
    t1_pass = twr_alt >= 2.2 and hover_throttle_alt_pct <= 65.0 and req_tilt_deg <= 38.0

    # 2. Asymmetric 40G Frontal Crash (single arm takes full AUW impact at 45 km/h)
    crash_40g = run_crash_20g_check(
        mesh,
        e_mpa=e_mpa,
        yield_mpa=yield_z_mpa,
        total_mass_kg=auw_heavy_g / 1000.0,
        g_load=40.0,
        fix_mode="center_hub",
    )
    # Single-arm concentration factor (1.75x vs symmetric 4-arm distribution)
    asym_stress_mpa = round(crash_40g["estimated_peak_stress_mpa"] * 1.75, 2)
    asym_crash_sf = round(min(20.0, yield_z_mpa / max(asym_stress_mpa, 0.1)), 2)
    t2_pass = asym_crash_sf >= 1.10

    # 3. Hot Motor (65°C thermal softening: -18% E & yield) + Full RPM Sweep Modal Resonance (12k-26k RPM = 200-433 Hz)
    thermal_factor = max(0.65, 1.0 - max(0.0, motor_temp_c - 25.0) * 0.0045)
    e_hot = e_mpa * thermal_factor
    yield_hot = yield_z_mpa * thermal_factor
    modal_hot = run_modal_resonance_check(
        mesh,
        e_mpa=e_hot,
        density_g_cm3=density_g_cm3,
        motor_rpm=motor_rpm,
        blade_count=3,
        fix_mode="center_hub",
    )
    f1_hot = modal_hot["f1_hz"]
    # Sweep band 12,000 - 26,000 RPM => 200.0 - 433.3 Hz
    in_sweep_band = any(195.0 <= f <= 440.0 for f in modal_hot["modes_hz"])
    t3_pass = (not in_sweep_band) and f1_hot >= 120.0

    # 4. Propeller Cyclic Fatigue (10^6 cycles, endurance limit Se = 0.48 * yield_hot) + 4% Blade Imbalance
    prop_stress_nom = yield_z_mpa / max(prop_root_sf, 0.05)
    prop_stress_imbal = prop_stress_nom * 1.08  # +8% dynamic stress from 4% imbalance
    endurance_limit_mpa = 0.48 * yield_hot
    fatigue_sf = round(min(10.0, endurance_limit_mpa / max(prop_stress_imbal, 0.1)), 2)
    mc_hot = run_monte_carlo_reliability(nominal_stress_mpa=prop_stress_imbal, yield_mean_mpa=yield_hot, n_samples=50000)
    t4_pass = fatigue_sf >= 1.0 and mc_hot["pf_structural_pct"] < 2.0

    # 5. Low-Battery 6S RTH (20% SoC = 21.0V vs 25.2V -> -16.7% RPM -> -30.5% thrust) at 2,500m Altitude
    low_batt_thrust_n = t_alt_tot_n * ((21.0 / 25.2) ** 2)
    twr_low_batt = round(low_batt_thrust_n / max(w_n, 1e-6), 2)
    # 3-motor emergency controlled descent ratio (3/4 thrust)
    twr_3motor = round(0.75 * twr_alt, 2)
    t5_pass = twr_low_batt >= 1.50 and twr_3motor >= 1.60

    # Extreme Gauntlet Score (0-100, 20 pts per brutal test)
    s1 = min(20.0, (min(twr_alt, 4.0) / 4.0) * 20.0) if hover_throttle_alt_pct <= 80.0 else 4.0
    s2 = min(20.0, (min(asym_crash_sf, 1.8) / 1.8) * 20.0)
    s3 = 20.0 if t3_pass else (8.0 if f1_hot >= 120.0 else 2.0)
    s4 = min(20.0, (min(fatigue_sf, 1.5) / 1.5) * 12.0 + max(0.0, 8.0 * (1.0 - min(mc_hot["pf_structural_pct"], 25.0) / 25.0)))
    s5 = min(20.0, (min(twr_low_batt, 2.5) / 2.5) * 20.0)
    total = round(s1 + s2 + s3 + s4 + s5, 1)

    failures: list[str] = []
    if not t1_pass:
        failures.append(f"ALTITUDE_WIND_FAIL: a {altitude_m:.0f}m + {wind_gust_kmh:.0f} km/h + 350g payload, TWR={twr_alt:.2f}, hover={hover_throttle_alt_pct:.1f}%.")
    if not t2_pass:
        failures.append(f"ASYM_CRASH_40G_FAIL: choque frontal asimétrico 40G en 1 brazo → σ={asym_stress_mpa:.1f} MPa, FS={asym_crash_sf:.2f} < 1.10.")
    if not t3_pass:
        failures.append(f"HOT_MODAL_SWEEP_FAIL: a {motor_temp_c:.0f}°C (E={e_hot:.0f} MPa), modos {modal_hot['modes_hz']} Hz cruzan la banda de aceleración 12k–26k RPM (200–433 Hz)!")
    if not t4_pass:
        failures.append(f"HOT_FATIGUE_FAIL: fatiga 10^6 ciclos + desbalance 4% a {motor_temp_c:.0f}°C → FS_fatiga={fatigue_sf:.2f}, Pf_caliente={mc_hot['pf_structural_pct']:.2f}%.")
    if not t5_pass:
        failures.append(f"LOW_BATT_RTH_FAIL: al 20% batería (21.0V) a {altitude_m:.0f}m → TWR={twr_low_batt:.2f} < 1.50.")

    return {
        "name": name,
        "extreme_score_100": total,
        "tests_passed": sum([t1_pass, t2_pass, t3_pass, t4_pass, t5_pass]),
        "breakdown": {
            "t1_altitude_wind_20": round(s1, 1),
            "t2_crash_40g_asym_20": round(s2, 1),
            "t3_hot_modal_sweep_20": round(s3, 1),
            "t4_hot_fatigue_mc_20": round(s4, 1),
            "t5_low_batt_rth_20": round(s5, 1),
        },
        "details": {
            "auw_heavy_g": round(auw_heavy_g, 1),
            "twr_2500m": round(twr_alt, 2),
            "hover_throttle_2500m_pct": round(hover_throttle_alt_pct, 1),
            "wind_tilt_deg": round(req_tilt_deg, 1),
            "crash_40g_asym_stress_mpa": asym_stress_mpa,
            "crash_40g_asym_sf": asym_crash_sf,
            "hot_modes_hz": modal_hot["modes_hz"],
            "in_12k_26k_sweep_band": in_sweep_band,
            "fatigue_1e6_sf": fatigue_sf,
            "hot_mc_pf_pct": mc_hot["pf_structural_pct"],
            "twr_low_batt_20pct": twr_low_batt,
            "twr_3motor_emergency": twr_3motor,
        },
        "discovered_failures": failures,
    }


__all__ = ["evaluate_competitor_adversarial", "run_extreme_drone_gauntlet"]
