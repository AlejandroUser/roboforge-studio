from __future__ import annotations

import copy
import json
from functools import lru_cache
from pathlib import Path

import numpy as np

from ..schemas import AlertItem, LinkDesignRequest, VariantKind, VariantMetrics

DATA_DIR = Path(__file__).resolve().parent.parent / "data"
GRAVITY_M_S2 = 9.81

# Stress concentration factors Kt per structural architecture (mechanics-math skill)
KT_BY_VARIANT: dict[VariantKind, float] = {
    "tapered_solid": 1.15,
    "organic_ibeam": 1.25,
    "rounded_truss": 1.35,
    "hollow_cobot": 1.18,
}

# Shear efficiency factor for diagonal truss members
TRUSS_SHEAR_EFFICIENCY = 0.82


@lru_cache(maxsize=1)
def _read_materials_raw() -> dict[str, dict]:
    path = DATA_DIR / "materials.json"
    with path.open("r", encoding="utf-8") as fh:
        entries = json.load(fh)
    return {entry["id"]: entry for entry in entries}


@lru_cache(maxsize=1)
def _read_presets_raw() -> dict:
    path = DATA_DIR / "presets.json"
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def load_materials() -> dict[str, dict]:
    """Load the engineering materials database keyed by material id."""
    return copy.deepcopy(_read_materials_raw())


def load_presets() -> dict:
    """Load arm architecture and end-interface presets."""
    return copy.deepcopy(_read_presets_raw())


def _tapered_height(
    u: np.ndarray,
    root_width_mm: float,
    tip_width_mm: float,
    organic_taper: float,
) -> np.ndarray:
    """
    Compute section height h(u) along normalized coordinate u in [0, 1],
    where u=0 is the root and u=1 is the tip.
    Blends linear taper (organic_taper=0) and parabolic bionic taper (organic_taper=1).
    """
    h_linear = root_width_mm + (tip_width_mm - root_width_mm) * u
    h_parabolic = tip_width_mm + (root_width_mm - tip_width_mm) * np.power(
        np.clip(1.0 - u, 0.0, 1.0), 1.4
    )
    return (1.0 - organic_taper) * h_linear + organic_taper * h_parabolic


def _section_moment_of_inertia(
    variant_id: VariantKind,
    b_mm: float,
    h_mm: np.ndarray,
    nozzle_mm: float,
    cable_channel: bool,
) -> np.ndarray:
    """
    Second moment of area I(x) in mm^4 for each structural variant.
    b_mm is out-of-plane thickness; h_mm is in-plane section height along the link.
    """
    t_min = max(2.4, nozzle_mm * 4.0)

    if variant_id == "tapered_solid":
        inertia = (b_mm * np.power(h_mm, 3)) / 12.0
    elif variant_id == "organic_ibeam":
        t_w = min(b_mm * 0.35, max(t_min, 0.25 * b_mm))
        t_f = np.minimum(h_mm * 0.28, np.maximum(t_min, 0.18 * h_mm))
        inner_h = np.maximum(0.0, h_mm - 2.0 * t_f)
        inner_b = max(0.0, b_mm - t_w)
        inertia = (b_mm * np.power(h_mm, 3) - inner_b * np.power(inner_h, 3)) / 12.0
    elif variant_id == "rounded_truss":
        t_c = np.minimum(h_mm * 0.30, np.maximum(t_min, 0.18 * h_mm))
        inner_h = np.maximum(0.0, h_mm - 2.0 * t_c)
        inertia = TRUSS_SHEAR_EFFICIENCY * (
            b_mm * (np.power(h_mm, 3) - np.power(inner_h, 3))
        ) / 12.0
    elif variant_id == "hollow_cobot":
        t_wall = np.minimum(
            np.minimum(b_mm * 0.28, h_mm * 0.28),
            np.maximum(t_min, 0.18 * np.minimum(b_mm, h_mm)),
        )
        inner_b = np.maximum(0.0, b_mm - 2.0 * t_wall)
        inner_h = np.maximum(0.0, h_mm - 2.0 * t_wall)
        inertia = (b_mm * np.power(h_mm, 3) - inner_b * np.power(inner_h, 3)) / 12.0
    else:
        raise ValueError(f"Unsupported variant_id: {variant_id}")

    # Internal cable routing slightly reduces core inertia near the neutral axis
    if cable_channel and variant_id != "rounded_truss":
        inertia = inertia * 0.96

    return np.maximum(inertia, 1e-3)


def _simpson_integrate(y: np.ndarray, dx: float) -> float:
    """Composite Simpson's rule integration over an even number of intervals."""
    n = len(y) - 1
    if n < 2 or n % 2 != 0:
        raise ValueError("Simpson integration requires an odd number of sample points >= 3.")
    weights = np.ones(n + 1, dtype=np.float64)
    weights[1:-1:2] = 4.0
    weights[2:-1:2] = 2.0
    return float(np.sum(weights * y) * (dx / 3.0))


def _print_without_supports(
    variant_id: VariantKind,
    req: LinkDesignRequest,
    material: dict,
) -> bool:
    if material.get("category") == "cnc":
        return True
    if variant_id == "tapered_solid":
        return True
    if variant_id in ("organic_ibeam", "rounded_truss"):
        return req.print_orientation == "xy_flat"
    if variant_id == "hollow_cobot":
        # Vertical tube orientation avoids internal bridging/supports
        return req.print_orientation == "z_vertical"
    return True


def _estimate_cost_usd(
    mass_kg: float,
    material: dict,
    print_without_supports: bool,
) -> float:
    raw_material_cost = mass_kg * float(material["cost_per_kg_usd"])
    if material.get("category") == "cnc":
        # Billet stock factor + CNC setup/machining baseline
        return raw_material_cost * 2.2 + 18.0
    # FDM: filament waste/purge factor + energy/machine time + support overhead
    support_overhead = 0.0 if print_without_supports else 0.65
    return raw_material_cost * 1.15 + 0.85 + support_overhead


def evaluate_variant_metrics(
    req: LinkDesignRequest,
    variant_id: VariantKind,
    volume_mm3: float,
    baseline_mass_g: float | None,
    material: dict,
) -> tuple[VariantMetrics, list[AlertItem]]:
    """
    Evaluate structural mechanics, dynamics, and manufacturability metrics for a
    tapered robotic link variant using numerical Simpson integration.
    """
    volume_cm3 = max(float(volume_mm3) / 1000.0, 1e-4)
    density_g_cm3 = float(material["density_g_cm3"])
    mass_g = volume_cm3 * density_g_cm3
    mass_kg = mass_g / 1000.0

    if baseline_mass_g is not None and baseline_mass_g > 0.0:
        mass_savings_pct = ((baseline_mass_g - mass_g) / baseline_mass_g) * 100.0
    else:
        mass_savings_pct = 0.0

    # Anisotropic mechanical properties based on build orientation
    if req.print_orientation == "z_vertical":
        youngs_eff_mpa = float(material["youngs_z_mpa"])
        yield_eff_mpa = float(material["yield_z_mpa"])
    else:
        youngs_eff_mpa = float(material["youngs_xy_mpa"])
        yield_eff_mpa = float(material["yield_xy_mpa"])

    length_mm = float(req.length_mm)
    tip_load_n = float(req.tip_load_n)
    self_weight_n = mass_kg * GRAVITY_M_S2

    # Numerical integration via composite Simpson's rule (40 intervals -> 41 points)
    # Coordinate s measured from tip (s=0) to root (s=L)
    n_intervals = 40
    s = np.linspace(0.0, length_mm, n_intervals + 1)
    ds = length_mm / n_intervals
    u_from_root = 1.0 - (s / length_mm)  # u=0 at root, u=1 at tip

    h_profile = _tapered_height(
        u=u_from_root,
        root_width_mm=float(req.root_width_mm),
        tip_width_mm=float(req.tip_width_mm),
        organic_taper=float(req.organic_taper),
    )
    i_profile = _section_moment_of_inertia(
        variant_id=variant_id,
        b_mm=float(req.thickness_mm),
        h_mm=h_profile,
        nozzle_mm=float(req.nozzle_diameter_mm),
        cable_channel=req.cable_channel,
    )

    # Bending moment M(s) in N*mm at distance s from tip
    w_n_per_mm = self_weight_n / length_mm
    m_profile = tip_load_n * s + 0.5 * w_n_per_mm * np.power(s, 2)

    # Castigliano / unit-load integrand: M(s) * s / (E * I(s))
    integrand = (m_profile * s) / (youngs_eff_mpa * i_profile)
    tip_deflection_mm = _simpson_integrate(integrand, ds)

    # Maximum bending stress at root (s = L, last element)
    m_max_n_mm = float(m_profile[-1])
    i_root_mm4 = float(i_profile[-1])
    kt = KT_BY_VARIANT[variant_id]
    max_stress_mpa = kt * (m_max_n_mm * (float(req.root_width_mm) / 2.0)) / i_root_mm4

    safety_factor = yield_eff_mpa / max(max_stress_mpa, 1e-6)

    # Rotational inertia about root joint (kg*cm^2)
    length_cm = length_mm / 10.0
    taper_ratio = float(np.clip(req.tip_width_mm / max(req.root_width_mm, 1.0), 0.3, 1.0))
    c_dist = 0.22 + 0.11 * ((taper_ratio - 0.3) / 0.7)
    payload_mass_kg = tip_load_n / GRAVITY_M_S2
    rotational_inertia_kg_cm2 = (
        mass_kg * (length_cm**2) * c_dist + payload_mass_kg * (length_cm**2)
    )

    # Dynamic torque at root joint (N*m)
    rotational_inertia_kg_m2 = rotational_inertia_kg_cm2 * 1e-4
    length_m = length_mm / 1000.0
    dynamic_torque_nm = (
        rotational_inertia_kg_m2 * float(req.angular_accel_rad_s2)
        + (tip_load_n + 0.5 * self_weight_n) * length_m
    )

    # Specific stiffness score: (N/mm) / kg
    stiffness_n_per_mm = tip_load_n / max(tip_deflection_mm, 1e-5)
    stiffness_to_weight_score = stiffness_n_per_mm / max(mass_kg, 1e-5)

    no_supports = _print_without_supports(variant_id, req, material)
    estimated_cost_usd = _estimate_cost_usd(mass_kg, material, no_supports)

    passes_requirements = (
        safety_factor >= req.target_safety_factor
        and tip_deflection_mm <= req.max_deflection_mm
    )

    alerts: list[AlertItem] = []
    if safety_factor < 1.0:
        alerts.append(
            AlertItem(
                level="critical",
                code="STRUCTURAL_FAILURE",
                message=(
                    f"Root bending stress ({max_stress_mpa:.1f} MPa) exceeds effective yield "
                    f"({yield_eff_mpa:.1f} MPa); link will fail under load."
                ),
            )
        )
    elif safety_factor < req.target_safety_factor:
        alerts.append(
            AlertItem(
                level="warning",
                code="LOW_SAFETY_FACTOR",
                message=(
                    f"Safety factor ({safety_factor:.2f}) is below target "
                    f"({req.target_safety_factor:.2f}). Increase root width or choose a stronger material."
                ),
            )
        )

    if tip_deflection_mm > req.max_deflection_mm:
        alerts.append(
            AlertItem(
                level="warning",
                code="EXCESSIVE_DEFLECTION",
                message=(
                    f"Tip deflection ({tip_deflection_mm:.2f} mm) exceeds target "
                    f"({req.max_deflection_mm:.2f} mm)."
                ),
            )
        )

    if not no_supports and material.get("category") == "fdm":
        alerts.append(
            AlertItem(
                level="info",
                code="SUPPORTS_REQUIRED",
                message=(
                    f"Variant '{variant_id}' in '{req.print_orientation}' orientation "
                    "requires print supports for overhanging cavities."
                ),
            )
        )

    metrics = VariantMetrics(
        volume_cm3=round(volume_cm3, 2),
        mass_g=round(mass_g, 2),
        mass_savings_pct=round(mass_savings_pct, 1),
        rotational_inertia_kg_cm2=round(rotational_inertia_kg_cm2, 2),
        max_stress_mpa=round(max_stress_mpa, 2),
        effective_yield_mpa=round(yield_eff_mpa, 2),
        safety_factor=round(safety_factor, 2),
        tip_deflection_mm=round(tip_deflection_mm, 3),
        dynamic_torque_nm=round(dynamic_torque_nm, 2),
        estimated_cost_usd=round(estimated_cost_usd, 2),
        stiffness_to_weight_score=round(stiffness_to_weight_score, 2),
        print_without_supports=no_supports,
        passes_requirements=passes_requirements,
    )
    return metrics, alerts
