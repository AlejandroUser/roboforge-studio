"""NSGA-II Multi-Objective Evolutionary Propeller Optimizer seeded by Adkins-Liebeck Theory."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from .propeller_bemt import (
    AIR_DENSITY_KG_M3,
    AIRFOIL_LIBRARY,
    SPEED_OF_SOUND_MS,
    BladeStation,
    solve_propeller_bemt,
)

AIRFOIL_IDS: tuple[str, ...] = ("naca4412", "clark_y", "eppler_e63")

# 11-gene bounds:
# [c0, c1, c2, c3, db0, db1, db2, db3, sweep_deg, rake_deg, airfoil_idx]
GENE_LOW = np.array([0.65, 0.70, 0.55, 0.35, -8.0, -8.0, -8.0, -8.0, -10.0, -6.0, 0.0], dtype=np.float64)
GENE_HIGH = np.array([1.35, 1.45, 1.35, 1.05, 14.0, 12.0, 12.0, 10.0, 22.0, 10.0, 2.999], dtype=np.float64)
NODE_U = np.array([0.0, 0.35, 0.72, 1.0], dtype=np.float64)


@dataclass
class EvolvedPropCandidate:
    candidate_id: str
    label: str
    summary: str
    metrics: dict[str, Any]
    airfoil_id: str
    sweep_deg: float
    rake_deg: float
    stations: list[dict[str, float]] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "candidate_id": self.candidate_id,
            "label": self.label,
            "summary": self.summary,
            "metrics": self.metrics,
            "airfoil_id": self.airfoil_id,
            "sweep_deg": round(self.sweep_deg, 2),
            "rake_deg": round(self.rake_deg, 2),
            "stations": self.stations,
        }


@dataclass
class PropellerEvolutionResult:
    winners: dict[str, EvolvedPropCandidate]
    baseline_metrics: dict[str, Any]
    pareto_front: list[dict[str, Any]]
    elapsed_s: float
    generations: int
    pop_size: int
    evaluations: int

    def to_dict(self) -> dict[str, Any]:
        return {
            "winners": {k: v.to_dict() for k, v in self.winners.items()},
            "baseline_metrics": self.baseline_metrics,
            "pareto_front": self.pareto_front,
            "elapsed_s": self.elapsed_s,
            "generations": self.generations,
            "pop_size": self.pop_size,
            "evaluations": self.evaluations,
        }


def _adkins_liebeck_seed_pitch_deg(
    r_frac: np.ndarray,
    target_thrust_g: float,
    diameter_inch: float,
    rpm: float,
) -> np.ndarray:
    """
    Analytical Adkins-Liebeck minimum-induced-loss pitch seed beta(r) = atan(lambda_h / r_hat) + alpha_opt.
    """
    radius_m = (diameter_inch * 0.0254) / 2.0
    disk_area = math.pi * (radius_m**2)
    omega = max(rpm, 500.0) * 2.0 * math.pi / 60.0
    v_tip = max(omega * radius_m, 1.0)
    thrust_n = max(target_thrust_g, 50.0) / 1000.0 * 9.80665
    v_i = math.sqrt(thrust_n / max(2.0 * AIR_DENSITY_KG_M3 * disk_area, 1e-6))
    lam_h = max(0.06, min(0.32, (v_i / v_tip) * 1.12))
    # Optimal L/D angle of attack ~ 5.5 deg added on top of inflow angle
    beta_deg = np.degrees(np.arctan2(lam_h, np.maximum(r_frac, 0.14))) + 5.5
    return np.clip(beta_deg, 8.0, 56.0)


def _decode_genome_to_stations(
    genome: np.ndarray,
    diameter_inch: float,
    blade_count: int,
    target_thrust_g: float,
    rpm: float,
    hub_radius_mm: float = 6.5,
    n_stations: int = 10,
    root_thickness_boost: float = 1.0,
) -> tuple[list[BladeStation], str, float, float]:
    """Decode 11-gene vector into 10 smooth radial BladeStation objects."""
    airfoil_idx = int(np.clip(math.floor(genome[10]), 0, len(AIRFOIL_IDS) - 1))
    airfoil_id = AIRFOIL_IDS[airfoil_idx]
    spec = AIRFOIL_LIBRARY[airfoil_id]
    sweep_deg = float(genome[8])
    rake_deg = float(genome[9])

    radius_mm = (diameter_inch * 25.4) / 2.0
    r_start = max(hub_radius_mm * 1.15, radius_mm * 0.16)
    r_end = radius_mm * 0.985
    u_grid = np.linspace(0.0, 1.0, n_stations)
    r_mm_arr = r_start + u_grid * (r_end - r_start)
    r_frac_arr = r_mm_arr / radius_mm

    # Smooth cubic polynomial interpolation over the 4 control nodes
    c_nodes = genome[0:4]
    db_nodes = genome[4:8]
    c_poly = np.polyval(np.polyfit(NODE_U, c_nodes, 3), u_grid)
    db_poly = np.polyval(np.polyfit(NODE_U, db_nodes, 3), u_grid)

    base_max_chord = radius_mm * 0.24 * ((2.5 / max(blade_count, 2)) ** 0.35)
    base_planform = np.maximum(0.34, np.sin(np.pi * (0.14 + 0.78 * u_grid)) ** 0.78)
    chord_arr = np.clip(base_max_chord * base_planform * np.clip(c_poly, 0.35, 1.50), 4.0, radius_mm * 0.42)

    al_pitch = _adkins_liebeck_seed_pitch_deg(r_frac_arr, target_thrust_g, diameter_inch, rpm)
    twist_arr = np.clip(al_pitch + db_poly, 7.0, 58.0)

    # Reinforced root fillet tapering smoothly to thin high-efficiency tip
    t_factor = 1.0 + (0.95 * root_thickness_boost) * ((1.0 - u_grid) ** 1.85)
    thick_arr = np.maximum(1.15, chord_arr * spec.thickness_t * t_factor)

    span_arr = r_mm_arr - r_start
    sweep_arr = span_arr * math.tan(math.radians(sweep_deg)) * (u_grid**1.35)
    rake_arr = span_arr * math.tan(math.radians(rake_deg))

    stations = [
        BladeStation(
            r_frac=float(r_frac_arr[i]),
            r_mm=float(r_mm_arr[i]),
            chord_mm=float(chord_arr[i]),
            twist_deg=float(twist_arr[i]),
            thickness_mm=float(thick_arr[i]),
            sweep_mm=float(sweep_arr[i]),
            rake_mm=float(rake_arr[i]),
        )
        for i in range(n_stations)
    ]
    return stations, airfoil_id, sweep_deg, rake_deg


def evaluate_custom_stations_bemt(
    stations: list[BladeStation],
    diameter_inch: float,
    blade_count: int,
    rpm: float,
    airfoil_id: str,
    prop_style: str = "conventional",
    sweep_deg: float = 0.0,
    rake_deg: float = 0.0,
    hub_radius_mm: float = 6.5,
    material_density_g_cm3: float = 1.25,
    material_yield_z_mpa: float = 45.0,
) -> dict[str, Any]:
    """Fast BEMT + centrifugal/flapwise root stress + sweep-aware acoustic evaluator for custom stations."""
    spec = AIRFOIL_LIBRARY.get(airfoil_id, AIRFOIL_LIBRARY["naca4412"])
    style = "toroidal" if str(prop_style).strip().lower() == "toroidal" else "conventional"

    radius_m = (diameter_inch * 0.0254) / 2.0
    omega = max(rpm, 500.0) * 2.0 * math.pi / 60.0
    v_tip = omega * radius_m
    tip_mach = v_tip / SPEED_OF_SOUND_MS

    alpha0 = -spec.cl0 / spec.cl_alpha
    rho = AIR_DENSITY_KG_M3
    eff_blades = blade_count * (1.35 if style == "toroidal" else 1.0)
    tip_loss_bonus = 1.06 if style == "toroidal" else 1.0
    profile_drag_mult = 1.12 if style == "toroidal" else 1.0

    # Mild positive sweep slightly reduces tip-vortex induced loss and sharp BVI noise
    sweep_pos = max(0.0, sweep_deg)
    sweep_eff_bonus = 1.0 + 0.0025 * min(sweep_pos, 16.0)
    rho_mat = material_density_g_cm3 * 1000.0
    r_hub_m = hub_radius_mm / 1000.0
    dr_m = (stations[-1].r_mm - stations[0].r_mm) / 1000.0 / max(len(stations) - 1, 1)

    total_thrust_n = 0.0
    total_torque_nm = 0.0
    blade_mass_kg = 0.0
    centrifugal_n = 0.0
    root_bending_nm = 0.0
    centrifugal_relief_nm = 0.0

    for st in stations:
        r_m = st.r_mm / 1000.0
        r_hat = max(0.05, min(0.995, st.r_frac))
        c_m = st.chord_mm / 1000.0
        beta = math.radians(st.twist_deg)
        sigma_local = (eff_blades * c_m) / (math.pi * radius_m)

        F = 0.92
        lam = 0.10
        for _ in range(12):
            term_a = (sigma_local * spec.cl_alpha) / (16.0 * max(F, 0.15))
            rad = max(0.0, term_a * term_a + (sigma_local * spec.cl_alpha / (8.0 * max(F, 0.15))) * max(beta - alpha0, 0.01) * r_hat)
            lam = 0.65 * max(math.sqrt(rad) - term_a, 1e-4) + 0.35 * lam
            phi = math.atan2(lam, r_hat)
            sin_phi = max(math.sin(phi), 0.04)
            f_tip = 0.5 * eff_blades * (1.0 - r_hat) / (r_hat * sin_phi)
            f_hub = 0.5 * eff_blades * max(r_hat - (r_hub_m / radius_m), 0.01) / (r_hat * sin_phi)
            F_tip = (2.0 / math.pi) * math.acos(min(1.0, max(0.0, math.exp(-min(f_tip, 25.0)))))
            F_hub = (2.0 / math.pi) * math.acos(min(1.0, max(0.0, math.exp(-min(f_hub, 25.0)))))
            F = min(1.0, max(0.18, F_tip * F_hub * tip_loss_bonus * sweep_eff_bonus))

        phi = math.atan2(lam, r_hat)
        alpha = beta - phi
        cl_lin = spec.cl0 + spec.cl_alpha * alpha
        if cl_lin > spec.cl_max:
            ov = cl_lin - spec.cl_max
            cl = max(0.45, spec.cl_max - 0.65 * ov)
            cd = (spec.cd0 + spec.cd_k * (spec.cl_max**2) + 0.85 * (ov**1.5)) * profile_drag_mult
        elif cl_lin < -0.5:
            cl = -0.5
            cd = (spec.cd0 + 0.25) * profile_drag_mult
        else:
            cl = cl_lin
            cd = (spec.cd0 + spec.cd_k * (cl * cl)) * profile_drag_mult

        v_t = omega * r_m
        v_a = lam * v_tip
        w_sq = v_t * v_t + v_a * v_a
        dL = 0.5 * rho * w_sq * c_m * cl * dr_m
        dD = 0.5 * rho * w_sq * c_m * cd * dr_m

        dT_blade = (dL * math.cos(phi) - dD * math.sin(phi)) * F
        dQ_blade = (dL * math.sin(phi) + dD * math.cos(phi)) * r_m

        total_thrust_n += max(0.0, eff_blades * dT_blade)
        total_torque_nm += max(0.0, eff_blades * dQ_blade)

        area_m2 = 0.68 * c_m * (st.thickness_mm / 1000.0) * (1.55 if style == "toroidal" else 1.0)
        dm_kg = rho_mat * area_m2 * dr_m
        blade_mass_kg += dm_kg
        dcf = dm_kg * (omega * omega) * r_m
        centrifugal_n += dcf
        root_bending_nm += max(0.0, dT_blade) * max(0.0, r_m - r_hub_m)
        # Positive rake toward thrust direction creates restoring centrifugal moment
        centrifugal_relief_nm += dcf * (st.rake_mm / 1000.0) * 0.45

    power_w = max(0.1, total_torque_nm * omega)
    disk_area_m2 = math.pi * (radius_m**2)
    ideal_power_w = (max(total_thrust_n, 0.0) ** 1.5) / math.sqrt(2.0 * rho * disk_area_m2)
    figure_of_merit = min(0.92, ideal_power_w / power_w)
    thrust_g = total_thrust_n / 9.80665 * 1000.0
    eff_g_per_w = thrust_g / power_w

    root_st = stations[0]
    root_c_m = root_st.chord_mm / 1000.0
    root_t_m = root_st.thickness_mm / 1000.0
    root_area_m2 = max(1e-7, 0.68 * root_c_m * root_t_m * (1.8 if style == "toroidal" else 1.0))
    root_z_m3 = max(1e-10, 0.075 * root_c_m * (root_t_m**2) * (2.1 if style == "toroidal" else 1.0))

    net_bending_nm = max(0.15 * root_bending_nm, abs(root_bending_nm - centrifugal_relief_nm))
    sigma_cent_mpa = (centrifugal_n / root_area_m2) / 1e6
    sigma_bend_mpa = (net_bending_nm / root_z_m3) / 1e6
    root_stress_mpa = sigma_cent_mpa + sigma_bend_mpa
    root_sf = min(25.0, material_yield_z_mpa / max(root_stress_mpa, 0.05))

    hub_mass_kg = rho_mat * math.pi * ((r_hub_m**2) - (0.0025**2)) * 0.0065
    total_prop_mass_g = (blade_count * blade_mass_kg + max(0.0, hub_mass_kg)) * 1000.0

    # Acoustic model (dBA @ 1m): tip speed + blade loading + tip-chord thickness + sweep acoustic relief
    tip_chord_ratio = stations[-1].chord_mm / max(stations[len(stations) // 2].chord_mm, 1.0)
    loading_term = 4.2 * math.log10(max(thrust_g / 350.0, 0.25))
    taper_term = 2.4 * (tip_chord_ratio - 0.55)
    sweep_relief_dba = min(4.2, 0.21 * max(0.0, sweep_deg))
    base_noise = (
        52.0
        + 34.0 * math.log10(max(v_tip / 30.0, 0.35))
        + 3.0 * math.log10(max(blade_count, 2) / 2.0)
        + loading_term
        + taper_term
        - sweep_relief_dba
    )
    if style == "toroidal":
        base_noise -= 6.5
    estimated_noise_dba = max(42.0, min(105.0, base_noise))

    # Effective equivalent pitch at 75% radius
    st75 = stations[int(round(0.75 * (len(stations) - 1)))]
    eq_pitch_in = (2.0 * math.pi * st75.r_mm * math.tan(math.radians(st75.twist_deg))) / 25.4

    return {
        "diameter_inch": round(diameter_inch, 2),
        "pitch_inch": round(eq_pitch_in, 2),
        "blade_count": blade_count,
        "airfoil_id": spec.id,
        "prop_style": style,
        "rpm": round(rpm, 0),
        "thrust_g": round(thrust_g, 1),
        "thrust_n": round(total_thrust_n, 2),
        "torque_nm": round(total_torque_nm, 4),
        "power_w": round(power_w, 1),
        "figure_of_merit": round(figure_of_merit, 3),
        "efficiency_g_per_w": round(eff_g_per_w, 2),
        "tip_speed_ms": round(v_tip, 1),
        "tip_mach": round(tip_mach, 3),
        "total_prop_mass_g": round(total_prop_mass_g, 2),
        "centrifugal_force_n": round(centrifugal_n, 1),
        "root_stress_mpa": round(root_stress_mpa, 2),
        "root_safety_factor": round(root_sf, 2),
        "estimated_noise_dba": round(estimated_noise_dba, 1),
        "sweep_deg": round(sweep_deg, 2),
        "rake_deg": round(rake_deg, 2),
    }


def _fast_nondominated_sort(objs: np.ndarray) -> list[list[int]]:
    """Standard NSGA-II fast non-dominated sorting (all objectives minimized)."""
    n = objs.shape[0]
    S: list[list[int]] = [[] for _ in range(n)]
    n_dom = np.zeros(n, dtype=np.int32)
    fronts: list[list[int]] = [[]]

    for p in range(n):
        for q in range(n):
            if p == q:
                continue
            p_le_q = np.all(objs[p] <= objs[q])
            p_lt_q = np.any(objs[p] < objs[q])
            if p_le_q and p_lt_q:
                S[p].append(q)
            elif np.all(objs[q] <= objs[p]) and np.any(objs[q] < objs[p]):
                n_dom[p] += 1
        if n_dom[p] == 0:
            fronts[0].append(p)

    i = 0
    while fronts[i]:
        nxt: list[int] = []
        for p in fronts[i]:
            for q in S[p]:
                n_dom[q] -= 1
                if n_dom[q] == 0:
                    nxt.append(q)
        i += 1
        if nxt:
            fronts.append(nxt)
        else:
            break
    return fronts


def _crowding_distance(objs: np.ndarray, front: list[int]) -> np.ndarray:
    m = len(front)
    dist = np.zeros(m, dtype=np.float64)
    if m <= 2:
        dist[:] = 1e9
        return dist
    f_objs = objs[front]
    for obj_i in range(objs.shape[1]):
        order = np.argsort(f_objs[:, obj_i])
        dist[order[0]] = 1e9
        dist[order[-1]] = 1e9
        span = float(f_objs[order[-1], obj_i] - f_objs[order[0], obj_i])
        if span <= 1e-12:
            continue
        for k in range(1, m - 1):
            dist[order[k]] += (f_objs[order[k + 1], obj_i] - f_objs[order[k - 1], obj_i]) / span
    return dist


def _stations_to_dicts(stations: list[BladeStation]) -> list[dict[str, float]]:
    return [
        {
            "r_frac": round(st.r_frac, 4),
            "r_mm": round(st.r_mm, 3),
            "chord_mm": round(st.chord_mm, 3),
            "twist_deg": round(st.twist_deg, 2),
            "thickness_mm": round(st.thickness_mm, 3),
            "sweep_mm": round(st.sweep_mm, 3),
            "rake_mm": round(st.rake_mm, 3),
        }
        for st in stations
    ]


def evolve_propeller_pareto(
    target_thrust_g: float = 650.0,
    diameter_inch: float = 5.0,
    rpm: float = 18000.0,
    blade_count: int = 3,
    prop_style: str = "conventional",
    material_density_g_cm3: float = 1.25,
    material_yield_z_mpa: float = 45.0,
    pop_size: int = 44,
    generations: int = 28,
    seed: int = 42,
    min_root_sf: float = 1.6,
) -> PropellerEvolutionResult:
    """
    Evolve unique propeller blade geometries via NSGA-II seeded around the Adkins-Liebeck
    minimum-induced-loss curve, returning 3 Pareto-optimal winners:
    `max_efficiency`, `ultra_quiet`, and `max_thrust`.
    """
    t0 = time.perf_counter()
    rng = np.random.default_rng(seed)
    pop_size = max(16, min(120, int(pop_size)))
    generations = max(6, min(80, int(generations)))

    # Baseline conventional propeller for direct before/after comparison
    baseline_res = solve_propeller_bemt(
        diameter_inch=diameter_inch,
        pitch_inch=max(2.0, diameter_inch * 0.82),
        blade_count=blade_count,
        rpm=rpm,
        airfoil_id="naca4412",
        prop_style=prop_style,
        material_density_g_cm3=material_density_g_cm3,
        material_yield_z_mpa=material_yield_z_mpa,
        n_stations=10,
    )
    baseline_metrics = baseline_res.to_metrics_dict()

    n_genes = len(GENE_LOW)
    span = GENE_HIGH - GENE_LOW

    # Seed population: 25% near Adkins-Liebeck analytical optimum + 75% uniform exploration
    pop = rng.uniform(GENE_LOW, GENE_HIGH, size=(pop_size, n_genes))
    n_seed = max(4, pop_size // 4)
    al_center = np.array([1.0, 1.0, 0.92, 0.62, 0.0, 0.0, 0.0, 0.0, 6.0, 1.5, 1.5], dtype=np.float64)
    pop[:n_seed] = np.clip(
        al_center + rng.normal(0.0, 0.14 * span, size=(n_seed, n_genes)),
        GENE_LOW,
        GENE_HIGH,
    )

    eval_count = 0
    sf_req = max(1.6, float(min_root_sf))
    root_t_boost = 1.0 + 0.55 * max(0.0, sf_req - 1.6)

    def _eval_pop(genomes: np.ndarray) -> tuple[np.ndarray, list[dict[str, Any]], list[tuple[list[BladeStation], str, float, float]]]:
        nonlocal eval_count
        m = genomes.shape[0]
        objs = np.empty((m, 3), dtype=np.float64)
        metrics_list: list[dict[str, Any]] = []
        decoded_list: list[tuple[list[BladeStation], str, float, float]] = []
        for idx in range(m):
            stations, af_id, sw, rk = _decode_genome_to_stations(
                genomes[idx], diameter_inch, blade_count, target_thrust_g, rpm,
                root_thickness_boost=root_t_boost,
            )
            met = evaluate_custom_stations_bemt(
                stations=stations,
                diameter_inch=diameter_inch,
                blade_count=blade_count,
                rpm=rpm,
                airfoil_id=af_id,
                prop_style=prop_style,
                sweep_deg=sw,
                rake_deg=rk,
                material_density_g_cm3=material_density_g_cm3,
                material_yield_z_mpa=material_yield_z_mpa,
            )
            eval_count += 1
            # Constraint violation penalties (hard structural SF >= sf_req & Mach <= 0.72)
            pen = 0.0
            if met["root_safety_factor"] < sf_req:
                pen += (sf_req - met["root_safety_factor"]) * 120.0
            if met["tip_mach"] > 0.72:
                pen += (met["tip_mach"] - 0.72) * 100.0
            if met["thrust_g"] < 0.65 * target_thrust_g:
                pen += ((0.65 * target_thrust_g - met["thrust_g"]) / max(target_thrust_g, 1.0)) * 80.0

            # 3 minimized objectives: (-efficiency_g_per_w, noise_dba, -thrust_g)
            objs[idx, 0] = -met["efficiency_g_per_w"] + pen
            objs[idx, 1] = met["estimated_noise_dba"] + pen
            objs[idx, 2] = -met["thrust_g"] / 100.0 + pen
            metrics_list.append(met)
            decoded_list.append((stations, af_id, sw, rk))
        return objs, metrics_list, decoded_list

    objs, mets, decs = _eval_pop(pop)

    for _gen in range(generations):
        fronts = _fast_nondominated_sort(objs)
        rank = np.zeros(pop_size, dtype=np.int32)
        crowd = np.zeros(pop_size, dtype=np.float64)
        for r_idx, f in enumerate(fronts):
            cd = _crowding_distance(objs, f)
            for pos, ind in enumerate(f):
                rank[ind] = r_idx
                crowd[ind] = cd[pos]

        # Binary tournament + SBX crossover + polynomial-like mutation
        offspring = np.empty_like(pop)
        for i in range(0, pop_size, 2):
            def _tourn() -> int:
                a, b = int(rng.integers(0, pop_size)), int(rng.integers(0, pop_size))
                if rank[a] < rank[b] or (rank[a] == rank[b] and crowd[a] > crowd[b]):
                    return a
                return b

            p1, p2 = pop[_tourn()], pop[_tourn()]
            if rng.random() < 0.88:
                u_sbx = rng.random(n_genes)
                eta_c = 12.0
                beta = np.where(
                    u_sbx <= 0.5,
                    (2.0 * u_sbx) ** (1.0 / (eta_c + 1.0)),
                    (1.0 / (2.0 * (1.0 - u_sbx) + 1e-12)) ** (1.0 / (eta_c + 1.0)),
                )
                c1 = 0.5 * ((1.0 + beta) * p1 + (1.0 - beta) * p2)
                c2 = 0.5 * ((1.0 - beta) * p1 + (1.0 + beta) * p2)
            else:
                c1, c2 = p1.copy(), p2.copy()

            # Mutation
            for child in (c1, c2):
                mut_mask = rng.random(n_genes) < (1.6 / n_genes)
                if np.any(mut_mask):
                    child[mut_mask] += rng.normal(0.0, 0.12 * span[mut_mask])
                np.clip(child, GENE_LOW, GENE_HIGH, out=child)

            offspring[i] = c1
            if i + 1 < pop_size:
                offspring[i + 1] = c2

        off_objs, off_mets, off_decs = _eval_pop(offspring)
        comb_pop = np.vstack((pop, offspring))
        comb_objs = np.vstack((objs, off_objs))
        comb_mets = mets + off_mets
        comb_decs = decs + off_decs

        comb_fronts = _fast_nondominated_sort(comb_objs)
        next_idx: list[int] = []
        for f in comb_fronts:
            if len(next_idx) + len(f) <= pop_size:
                next_idx.extend(f)
            else:
                cd = _crowding_distance(comb_objs, f)
                order = np.argsort(-cd)
                needed = pop_size - len(next_idx)
                next_idx.extend(f[k] for k in order[:needed])
                break

        pop = comb_pop[next_idx]
        objs = comb_objs[next_idx]
        mets = [comb_mets[k] for k in next_idx]
        decs = [comb_decs[k] for k in next_idx]

    # Extract Pareto front and 3 distinct winners
    final_fronts = _fast_nondominated_sort(objs)
    pf_indices = final_fronts[0]

    feasible = [i for i in range(pop_size) if mets[i]["root_safety_factor"] >= sf_req]
    if not feasible:
        feasible = [i for i in range(pop_size) if mets[i]["root_safety_factor"] >= 1.6]
    pool = feasible if feasible else list(range(pop_size))

    # 1. Max Efficiency (meeting >= 85% target thrust if possible)
    eff_pool = [i for i in pool if mets[i]["thrust_g"] >= 0.85 * target_thrust_g] or pool
    idx_eff = max(eff_pool, key=lambda i: mets[i]["efficiency_g_per_w"])

    # 2. Ultra Quiet (lowest dBA, preferring >= 75% target thrust)
    quiet_pool = [i for i in pool if mets[i]["thrust_g"] >= 0.75 * target_thrust_g] or pool
    idx_quiet = min(quiet_pool, key=lambda i: (mets[i]["estimated_noise_dba"], -mets[i]["efficiency_g_per_w"]))

    # 3. Max Thrust (highest thrust within safe root SF)
    idx_thrust = max(pool, key=lambda i: mets[i]["thrust_g"])

    def _make_winner(cid: str, label: str, idx: int) -> EvolvedPropCandidate:
        stations, af_id, sw, rk = decs[idx]
        m = mets[idx]
        d_eff = m["efficiency_g_per_w"] - baseline_metrics["efficiency_g_per_w"]
        d_dba = m["estimated_noise_dba"] - baseline_metrics["estimated_noise_dba"]
        summary = (
            f"{m['thrust_g']:.0f} g · {m['efficiency_g_per_w']:.2f} g/W ({d_eff:+.2f}) · "
            f"{m['estimated_noise_dba']:.1f} dBA ({d_dba:+.1f}) · Sweep {sw:+.1f}° · FS {m['root_safety_factor']:.1f}"
        )
        return EvolvedPropCandidate(
            candidate_id=cid,
            label=label,
            summary=summary,
            metrics=m,
            airfoil_id=af_id,
            sweep_deg=sw,
            rake_deg=rk,
            stations=_stations_to_dicts(stations),
        )

    winners = {
        "max_efficiency": _make_winner("max_efficiency", "Máx. Eficiencia (g/W)", idx_eff),
        "ultra_quiet": _make_winner("ultra_quiet", "Ultra-Silenciosa (Sweep)", idx_quiet),
        "max_thrust": _make_winner("max_thrust", "Máx. Empuje Seguro", idx_thrust),
    }

    pareto_front = [
        {
            "efficiency_g_per_w": mets[i]["efficiency_g_per_w"],
            "estimated_noise_dba": mets[i]["estimated_noise_dba"],
            "thrust_g": mets[i]["thrust_g"],
            "root_safety_factor": mets[i]["root_safety_factor"],
            "airfoil_id": mets[i]["airfoil_id"],
            "sweep_deg": mets[i]["sweep_deg"],
        }
        for i in pf_indices[:24]
    ]

    elapsed_s = time.perf_counter() - t0
    return PropellerEvolutionResult(
        winners=winners,
        baseline_metrics=baseline_metrics,
        pareto_front=pareto_front,
        elapsed_s=round(elapsed_s, 3),
        generations=generations,
        pop_size=pop_size,
        evaluations=eval_count,
    )
