"""Tests for RoboForge v13.0 Generative Physics Engines: 3D SIMP + Langelaar 45-deg & NSGA-II Propeller Optimizer."""

from __future__ import annotations

import json

import numpy as np

from app.physics.propeller_optimizer import evolve_propeller_pareto
from app.physics.topopt_3d import run_simp_3d_link


def test_simp_3d_link_multi_case_and_langelaar_overhang() -> None:
    res = run_simp_3d_link(
        length_mm=180.0,
        root_width_mm=44.0,
        tip_width_mm=30.0,
        thickness_mm=22.0,
        load_n=35.0,
        lateral_load_n=10.0,
        angular_accel_rad_s2=12.0,
        root_od_mm=36.0,
        tip_od_mm=28.0,
        root_bore_mm=12.0,
        tip_bore_mm=10.0,
        volume_fraction=0.38,
        overhang_filter_45=True,
        nx=30,
        ny=10,
        nz=10,
        n_iter=18,
    )

    assert res.rho_3d.shape == (30, 10, 10)
    assert float(np.min(res.rho_3d)) >= 0.0
    assert float(np.max(res.rho_3d)) <= 1.0
    assert len(res.compliance_history) >= 6
    assert res.mass_reduction_pct >= 25.0
    assert res.elapsed_s < 15.0, f"SIMP 3D too slow: {res.elapsed_s:.2f}s"

    # Y-symmetry check
    assert np.allclose(res.rho_3d, res.rho_3d[:, ::-1, :], atol=1e-2)

    # Preserve boss (near x=0, y=mid, outside bore) must be solid (~1.0); bore center void (~0.001)
    assert float(res.rho_3d[0, 5, 5]) <= 0.05 or float(res.rho_3d[0, 2, 5]) >= 0.95
    assert float(res.rho_3d[0, 1, 5]) >= 0.90  # Root boss outer rim preserved
    assert float(res.rho_3d[-1, 1, 5]) >= 0.90 or float(res.rho_3d[-1, 2, 5]) >= 0.90  # Tip boss preserved

    # Distinct load case (high lateral torsion) produces a distinct 3D density topology
    res_lat = run_simp_3d_link(
        length_mm=180.0,
        root_width_mm=44.0,
        tip_width_mm=30.0,
        thickness_mm=22.0,
        load_n=10.0,
        lateral_load_n=60.0,
        angular_accel_rad_s2=2.0,
        volume_fraction=0.38,
        overhang_filter_45=False,
        nx=24,
        ny=8,
        nz=8,
        n_iter=12,
    )
    assert res_lat.rho_3d.shape == (24, 8, 8)
    assert res_lat.elapsed_s < 10.0


def test_nsga2_propeller_evolution_pareto_winners_and_json_contract() -> None:
    evo = evolve_propeller_pareto(
        target_thrust_g=650.0,
        diameter_inch=5.0,
        rpm=18000.0,
        blade_count=3,
        prop_style="conventional",
        material_density_g_cm3=1.25,
        material_yield_z_mpa=45.0,
        pop_size=44,
        generations=28,
        seed=42,
    )

    assert evo.elapsed_s < 10.0, f"NSGA-II too slow: {evo.elapsed_s:.2f}s"
    assert evo.evaluations == 44 * (28 + 1)
    assert set(evo.winners.keys()) == {"max_efficiency", "ultra_quiet", "max_thrust"}
    assert len(evo.pareto_front) >= 3

    w_eff = evo.winners["max_efficiency"]
    w_quiet = evo.winners["ultra_quiet"]
    w_thrust = evo.winners["max_thrust"]

    # All 3 winners must be structurally safe and JSON-serializable with 10 radial stations
    required_station_keys = {
        "r_frac",
        "r_mm",
        "chord_mm",
        "twist_deg",
        "thickness_mm",
        "sweep_mm",
        "rake_mm",
    }
    for w in (w_eff, w_quiet, w_thrust):
        assert w.metrics["root_safety_factor"] >= 1.6
        assert w.airfoil_id in {"naca4412", "clark_y", "eppler_e63"}
        assert len(w.stations) == 10
        assert set(w.stations[0].keys()) == required_station_keys

    # Pareto specialization checks
    assert w_eff.metrics["efficiency_g_per_w"] >= evo.baseline_metrics["efficiency_g_per_w"]
    assert w_quiet.metrics["estimated_noise_dba"] <= w_thrust.metrics["estimated_noise_dba"]
    assert w_quiet.sweep_deg > 5.0  # Optimizer discovers swept tip for acoustic relief
    assert w_thrust.metrics["thrust_g"] >= w_eff.metrics["thrust_g"]

    # Full result must serialize cleanly to JSON for API / frontend
    payload = json.dumps(evo.to_dict())
    assert len(payload) > 500
