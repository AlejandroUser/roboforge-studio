"""Tests for Phase 3 (Hybrid 6/7-DOF LM-DLS IK) and Phase 4 (UIUC .dat Airfoil Importer)."""

from __future__ import annotations

import math

import pytest

from app.physics.ik_solver import solve_arm_ik, solve_ik_lm_dls_redundant
from app.physics.propeller_bemt import (
    AIRFOIL_LIBRARY,
    parse_uiuc_dat_airfoil,
    register_custom_airfoil_dat,
    sample_airfoil_coords,
    solve_propeller_bemt,
)


def _generate_naca2412_selig_dat() -> str:
    """Generate a clean synthetic NACA 2412 UIUC Selig-format .dat string (TE->LE->TE)."""
    m, p, t = 0.02, 0.40, 0.12
    xs = [0.5 * (1.0 - math.cos(math.pi * i / 16)) for i in range(17)]
    upper, lower = [], []
    for x in xs:
        yt = 5.0 * t * (
            0.2969 * math.sqrt(max(x, 0.0))
            - 0.1260 * x
            - 0.3516 * x**2
            + 0.2843 * x**3
            - 0.1015 * x**4
        )
        if x < p:
            yc = (m / (p * p)) * (2.0 * p * x - x * x)
        else:
            yc = (m / ((1.0 - p) ** 2)) * ((1.0 - 2.0 * p) + 2.0 * p * x - x * x)
        upper.append((x, yc + yt))
        lower.append((x, yc - yt))

    lines = ["NACA 2412 Synthetic Selig"]
    for x, y in reversed(upper):
        lines.append(f"  {x:.6f}  {y:.6f}")
    for x, y in lower[1:]:
        lines.append(f"  {x:.6f}  {y:.6f}")
    return "\n".join(lines) + "\n"


def _generate_lednicer_dat() -> str:
    """Generate a clean synthetic Lednicer-format .dat string (header + counts + 2 blocks)."""
    m, p, t = 0.04, 0.40, 0.10
    xs = [i / 12.0 for i in range(13)]
    lines = ["AG12 Synthetic Lednicer", "  13.  13.", ""]
    for x in xs:
        yt = 5.0 * t * (0.2969 * math.sqrt(max(x, 0.0)) - 0.1260 * x - 0.3516 * x**2 + 0.2843 * x**3 - 0.1015 * x**4)
        yc = (m / (p * p)) * (2.0 * p * x - x * x) if x < p else (m / ((1.0 - p) ** 2)) * ((1.0 - 2.0 * p) + 2.0 * p * x - x * x)
        lines.append(f"  {x:.6f}  {yc + yt:.6f}")
    lines.append("")
    for x in xs:
        yt = 5.0 * t * (0.2969 * math.sqrt(max(x, 0.0)) - 0.1260 * x - 0.3516 * x**2 + 0.2843 * x**3 - 0.1015 * x**4)
        yc = (m / (p * p)) * (2.0 * p * x - x * x) if x < p else (m / ((1.0 - p) ** 2)) * ((1.0 - 2.0 * p) + 2.0 * p * x - x * x)
        lines.append(f"  {x:.6f}  {yc - yt:.6f}")
    return "\n".join(lines) + "\n"


def test_lm_dls_redundant_ik_6dof_and_7dof_convergence() -> None:
    # 6-DOF reachable target
    res6 = solve_ik_lm_dls_redundant(
        target_xyz_mm=[260.0, 95.0, 210.0],
        link_lengths_mm=[95.0, 200.0, 170.0, 75.0],
        dof=6,
        max_iter=60,
        tol_mm=1.0,
    )
    assert res6["solver"] == "lm_dls_redundant"
    assert res6["reachable"] is True
    assert res6["reach_error_mm"] <= 1.0
    assert len(res6["joint_angles_deg"]) == 6

    # 7-DOF Franka-class redundant target
    res7 = solve_ik_lm_dls_redundant(
        target_xyz_mm=[380.0, -140.0, 320.0],
        link_lengths_mm=[140.0, 316.0, 384.0, 107.0],
        dof=7,
        max_iter=60,
        tol_mm=1.0,
    )
    assert res7["reachable"] is True
    assert res7["reach_error_mm"] <= 1.0
    assert len(res7["joint_angles_deg"]) == 7

    # Unreachable target must not crash or return NaN
    far = solve_ik_lm_dls_redundant(
        target_xyz_mm=[2500.0, 0.0, 500.0],
        link_lengths_mm=[95.0, 200.0, 170.0, 75.0],
        dof=6,
    )
    assert far["reachable"] is False
    assert far["reach_error_mm"] > 100.0
    assert all(math.isfinite(a) for a in far["joint_angles_deg"])

    # Hybrid dispatcher in solve_arm_ik: dof=6 uses LM-DLS, default dof=4 stays analytical
    hybrid6 = solve_arm_ik(250.0, 80.0, 200.0, dof=6)
    assert hybrid6["solver"] == "lm_dls_redundant"
    assert hybrid6["reachable"] is True
    analyt4 = solve_arm_ik(250.0, 80.0, 200.0)
    assert "solver" not in analyt4
    assert analyt4["reachable"] is True


def test_uiuc_dat_selig_and_lednicer_parse_and_register() -> None:
    selig_dat = _generate_naca2412_selig_dat()
    spec_s, pts_s = parse_uiuc_dat_airfoil(selig_dat, airfoil_id="naca2412_selig")
    assert spec_s.id == "naca2412_selig"
    assert spec_s.thickness_t == pytest.approx(0.12, abs=0.015)
    assert spec_s.camber_m == pytest.approx(0.02, abs=0.008)
    assert spec_s.cl0 > 0.15

    # Pivoted at 25% chord: x in [-0.25, +0.75] and finite printable TE thickness >= 0.032
    xs_s = [p[0] for p in pts_s]
    assert min(xs_s) == pytest.approx(-0.25, abs=0.01)
    assert max(xs_s) == pytest.approx(0.75, abs=0.01)
    te_thick = abs(pts_s[0][1] - pts_s[-1][1])
    assert te_thick >= 0.031

    # Lednicer format
    led_dat = _generate_lednicer_dat()
    info = register_custom_airfoil_dat("ag12_test", led_dat)
    assert info["id"] == "ag12_test"
    assert 8.5 <= info["thickness_pct"] <= 11.5
    assert 3.0 <= info["camber_pct"] <= 5.0
    assert info["points"] >= 30
    assert "ag12_test" in AIRFOIL_LIBRARY

    # sample_airfoil_coords and solve_propeller_bemt consume the registered custom airfoil
    coords = sample_airfoil_coords("ag12_test")
    assert len(coords) == info["points"]
    bemt = solve_propeller_bemt(diameter_inch=5.0, pitch_inch=4.0, blade_count=3, rpm=16000.0, airfoil_id="ag12_test")
    assert bemt.airfoil_id == "ag12_test"
    assert bemt.thrust_g > 200.0
    assert bemt.efficiency_g_per_w > 2.0
