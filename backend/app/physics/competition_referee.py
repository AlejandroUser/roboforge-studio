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


__all__ = ["evaluate_competitor_adversarial"]
