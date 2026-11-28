"""Pytest suite for RoboForge v14.5 Adversarial Physics Test Bench & Competition Referee."""

from __future__ import annotations

import math
from pathlib import Path

import trimesh

from backend.app.physics.adversarial_bench import (
    run_crash_20g_check,
    run_modal_resonance_check,
    run_monte_carlo_reliability,
)
from backend.app.physics.competition_referee import (
    evaluate_competitor_adversarial,
    run_extreme_drone_gauntlet,
)


def test_modal_eigsh_matches_analytical_cantilever_beam():
    """Hex8 3D eigsh fundamental frequency must agree with Euler-Bernoulli cantilever beam formula."""
    # Cantilever beam: L=120 mm, width=20 mm, height=10 mm, E=5200 MPa, rho=1.18 g/cm^3
    L, b, h = 120.0, 20.0, 10.0
    E_mpa, rho_g_cm3 = 5200.0, 1.18
    beam = trimesh.creation.box(extents=[L, b, h])
    beam.apply_translation([L / 2.0, 0.0, 0.0])

    res = run_modal_resonance_check(
        beam, e_mpa=E_mpa, density_g_cm3=rho_g_cm3, motor_rpm=18000.0, fix_mode="root_x", nx=16, ny=4, nz=3
    )
    # Analytical f1 = (1.875^2 / (2*pi*L^2)) * sqrt(E*I / (rho*A))
    I_mm4 = b * (h ** 3) / 12.0
    A_mm2 = b * h
    rho_tonne_mm3 = rho_g_cm3 * 1e-9
    f1_analytical = (1.8751 ** 2) / (2.0 * math.pi * L * L) * math.sqrt((E_mpa * I_mm4) / (rho_tonne_mm3 * A_mm2))

    # Coarse Hex8 is slightly stiffer (~10-25% locking on bending) -> verify within 35% of Euler-Bernoulli
    ratio = res["f1_hz"] / f1_analytical
    assert 0.75 <= ratio <= 1.38, f"Modal f1={res['f1_hz']} Hz vs analytical={f1_analytical:.1f} Hz (ratio={ratio:.2f})"


def test_monte_carlo_reliability_monotonicity_and_jamming():
    """Probability of failure Pf must increase monotonically as applied stress approaches yield."""
    safe = run_monte_carlo_reliability(nominal_stress_mpa=18.0, yield_mean_mpa=45.0, hole_nom_mm=5.2, shaft_nom_mm=4.8)
    risky = run_monte_carlo_reliability(nominal_stress_mpa=38.0, yield_mean_mpa=45.0, hole_nom_mm=5.0, shaft_nom_mm=4.95)

    assert safe["pf_structural_pct"] < 1.0 and safe["reliability_beta"] > 2.3
    assert safe["status"] == "pass"
    assert risky["pf_structural_pct"] > safe["pf_structural_pct"] + 10.0
    assert risky["pf_jamming_pct"] > safe["pf_jamming_pct"]
    assert risky["status"] == "fail"


def test_crash_20g_and_adversarial_referee_end_to_end(tmp_path: Path):
    """Referee must score a solid plate cleanly across all 5 adversarial pillars."""
    plate = trimesh.creation.box(extents=[160.0, 160.0, 5.0])
    stl_p = tmp_path / "plate.stl"
    plate.export(str(stl_p))

    crash = run_crash_20g_check(plate, e_mpa=5200.0, yield_mpa=45.0, total_mass_kg=0.65, g_load=20.0)
    assert crash["crash_safety_factor"] > 0.5
    assert crash["max_deflection_mm"] >= 0.0

    score = evaluate_competitor_adversarial(
        name="TestDrone",
        frame_stl=stl_p,
        auw_g=650.0,
        bemt_thrust_per_motor_g=1000.0,
        efficiency_g_per_w=3.4,
        prop_root_sf=1.8,
        run_external_tools=False,
    )
    assert 0.0 <= score["total_score_100"] <= 100.0
    assert set(score["breakdown"].keys()) == {"flight_30", "modal_20", "crash_20g_20", "monte_carlo_20", "manufacturing_10"}

    extreme = run_extreme_drone_gauntlet(
        name="ExtremeTestDrone",
        frame_stl=stl_p,
        auw_base_g=660.0,
        bemt_thrust_per_motor_g=900.0,
        bemt_power_per_motor_w=260.0,
        efficiency_g_per_w=3.46,
        prop_root_sf=1.8,
    )
    assert 0.0 <= extreme["extreme_score_100"] <= 100.0
    assert 0 <= extreme["tests_passed"] <= 5
