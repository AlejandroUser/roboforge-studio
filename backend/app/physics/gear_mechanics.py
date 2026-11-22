from __future__ import annotations

import copy
import json
import math
from functools import lru_cache
from pathlib import Path

import numpy as np

from ..schemas import (
    AlertItem,
    GearDesignRequest,
    GearVariantKind,
    VariantMetrics,
)

DATA_DIR = Path(__file__).resolve().parent.parent / "data"
CF_MATERIALS = {"petg_cf", "pa12_cf"}

# Rim/spoke stress concentration & inertia shape factors per gear variant
KT_BY_GEAR_VARIANT: dict[GearVariantKind, float] = {
    "gear_solid": 1.05,
    "gear_webbed": 1.10,
    "gear_spoked": 1.15,
}

INERTIA_FACTOR_BY_VARIANT: dict[GearVariantKind, float] = {
    "gear_solid": 0.50,
    "gear_webbed": 0.46,
    "gear_spoked": 0.42,
}


@lru_cache(maxsize=1)
def _read_gear_presets_raw() -> dict:
    path = DATA_DIR / "gear_presets.json"
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def load_gear_presets() -> dict:
    """Load gear and GT2 pulley presets keyed under 'presets'."""
    return copy.deepcopy(_read_gear_presets_raw())


def compute_gear_diameters(req: GearDesignRequest) -> tuple[float, float]:
    """
    Compute (pitch_diameter_mm, outer_diameter_mm) for spur involute gears or GT2 pulleys.
    - spur_involute: d_p = m * Z, d_o = d_p + 2*m
    - gt2_pulley:    d_p = 2.0 * Z / pi, d_o = d_p - 0.508 + 2.4 (including belt flanges)
    """
    teeth = float(req.teeth)
    if req.gear_kind == "gt2_pulley":
        d_p = (2.0 * teeth) / math.pi
        d_o = d_p - 0.508 + 2.4
    else:
        m = float(req.module_mm)
        d_p = m * teeth
        d_o = d_p + 2.0 * m
    return round(d_p, 3), round(d_o, 3)


def _lewis_form_factor(teeth: int, pressure_angle_deg: float) -> float:
    """Standard Lewis form factor Y(Z) adjusted for pressure angle."""
    y_20 = max(0.18, 0.484 - (2.87 / max(float(teeth), 10.0)))
    angle_scale = math.sin(math.radians(pressure_angle_deg)) / math.sin(math.radians(20.0))
    return max(0.15, y_20 * float(np.clip(angle_scale, 0.82, 1.18)))


def _gear_print_without_supports(
    variant_id: GearVariantKind, req: GearDesignRequest, material: dict
) -> bool:
    if material.get("category") == "cnc":
        return True
    if req.print_orientation == "z_vertical":
        return False
    # gear_solid and gear_spoked (through cutouts) print flat without supports;
    # gear_webbed has an underside recessed pocket needing bridging/supports
    if variant_id in ("gear_solid", "gear_spoked"):
        return True
    return False


def _estimate_gear_cost_usd(
    mass_kg: float, material: dict, print_without_supports: bool
) -> float:
    raw_cost = mass_kg * float(material["cost_per_kg_usd"])
    if material.get("category") == "cnc":
        return raw_cost * 2.5 + 16.0
    support_overhead = 0.0 if print_without_supports else 0.45
    return raw_cost * 1.15 + 0.55 + support_overhead


def evaluate_gear_metrics(
    req: GearDesignRequest,
    variant_id: GearVariantKind,
    volume_mm3: float,
    baseline_mass_g: float | None,
    material: dict,
) -> tuple[VariantMetrics, list[AlertItem]]:
    """
    Evaluate tooth bending stress (Lewis equation + Barth velocity factor Kv for
    involute gears; distributed belt-tooth stress for GT2 pulleys), tooth deflection,
    rotational inertia, anisotropic safety factor, and 12-station radial stress profile.
    """
    volume_cm3 = max(float(volume_mm3) / 1000.0, 1e-4)
    density_g_cm3 = float(material["density_g_cm3"])
    mass_g = volume_cm3 * density_g_cm3
    mass_kg = mass_g / 1000.0

    if baseline_mass_g is not None and baseline_mass_g > 0.0:
        mass_savings_pct = ((baseline_mass_g - mass_g) / baseline_mass_g) * 100.0
    else:
        mass_savings_pct = 0.0

    # Anisotropic mechanical properties
    if req.print_orientation == "z_vertical":
        youngs_eff_mpa = float(material["youngs_z_mpa"])
        yield_eff_mpa = float(material["yield_z_mpa"])
    else:
        youngs_eff_mpa = float(material["youngs_xy_mpa"])
        yield_eff_mpa = float(material["yield_xy_mpa"])

    d_p, _ = compute_gear_diameters(req)
    face_w = float(req.face_width_mm)
    torque_nm = float(req.torque_nm)

    # Tangential force at pitch circle (N)
    f_t_n = (2000.0 * torque_nm) / max(d_p, 1.0)

    # Pitch-line velocity (m/s) and dynamic velocity factor Kv = (6 + v) / 6
    v_m_s = (math.pi * d_p * float(req.rpm)) / 60000.0
    k_v = (6.0 + v_m_s) / 6.0
    k_t = KT_BY_GEAR_VARIANT[variant_id]

    if req.gear_kind == "spur_involute":
        m = float(req.module_mm)
        y_lewis = _lewis_form_factor(req.teeth, float(req.pressure_angle_deg))
        max_stress_mpa = k_v * k_t * (f_t_n / max(face_w * m * y_lewis, 1e-6))

        # Cantilever tooth deflection (h_t = 2.25m, root thickness t_t ~ 1.55m)
        h_t = 2.25 * m
        t_t = 1.55 * m
        i_tooth = (face_w * (t_t**3)) / 12.0
        tooth_deflection_mm = k_t * (f_t_n * (h_t**3)) / (3.0 * youngs_eff_mpa * i_tooth)
        max_allowable_deflection = max(0.20, 0.12 * m)
    else:
        # GT2 pulley: belt engages ~45% of teeth simultaneously with load-sharing factor
        engaged_teeth = max(6.0, 0.45 * float(req.teeth))
        n_eff = 0.65 * engaged_teeth
        f_per_tooth = f_t_n / n_eff
        # GT2 tooth root ~1.1 mm, height ~0.75 mm (combined bending + shear)
        max_stress_mpa = (
            k_v
            * k_t
            * (f_per_tooth * ((6.0 * 0.75) / (1.1**2) + 1.0 / 1.1))
            / max(face_w, 1.0)
        )
        i_gt2 = (face_w * (1.1**3)) / 12.0
        tooth_deflection_mm = k_t * (f_per_tooth * (0.75**3)) / (3.0 * youngs_eff_mpa * i_gt2)
        max_allowable_deflection = 0.18

    safety_factor = yield_eff_mpa / max(max_stress_mpa, 1e-6)

    # Rotational inertia about Z axis (kg*cm^2)
    r_p_cm = (d_p * 0.5) / 10.0
    c_inertia = INERTIA_FACTOR_BY_VARIANT[variant_id]
    rotational_inertia_kg_cm2 = c_inertia * mass_kg * (r_p_cm**2)

    # 12-station radial stress profile from tooth root (1.0) to hub/bore (0.0)
    u = np.linspace(0.0, 1.0, 12)
    decay = 1.4 if variant_id == "gear_solid" else 1.15
    raw_profile = np.power(np.clip(1.0 - u, 0.0, 1.0), decay)
    raw_profile[-1] = 0.0
    stress_profile = [round(float(val), 3) for val in raw_profile]

    # Stiffness-to-weight score: tangential tooth stiffness per kg
    stiffness_n_per_mm = f_t_n / max(tooth_deflection_mm, 1e-5)
    stiffness_to_weight_score = stiffness_n_per_mm / max(mass_kg, 1e-5)

    no_supports = _gear_print_without_supports(variant_id, req, material)
    estimated_cost_usd = _estimate_gear_cost_usd(mass_kg, material, no_supports)

    passes_requirements = (
        safety_factor >= req.target_safety_factor
        and tooth_deflection_mm <= max_allowable_deflection
    )

    alerts: list[AlertItem] = []
    if safety_factor < 1.0:
        alerts.append(
            AlertItem(
                level="critical",
                code="TOOTH_ROOT_FAILURE",
                message=(
                    f"Esfuerzo en raíz de diente ({max_stress_mpa:.1f} MPa) supera el límite "
                    f"elástico efectivo ({yield_eff_mpa:.1f} MPa). Aumenta módulo o ancho de cara."
                ),
            )
        )
    elif safety_factor < req.target_safety_factor:
        alerts.append(
            AlertItem(
                level="warning",
                code="LOW_SAFETY_FACTOR",
                message=(
                    f"Factor de seguridad del diente ({safety_factor:.2f}) por debajo del objetivo "
                    f"({req.target_safety_factor:.2f})."
                ),
            )
        )

    if not no_supports and material.get("category") == "fdm":
        alerts.append(
            AlertItem(
                level="info",
                code="SUPPORTS_REQUIRED",
                message=(
                    f"La variante '{variant_id}' requiere soportes o puentes para el rebaje inferior."
                ),
            )
        )

    metrics = VariantMetrics(
        volume_cm3=round(volume_cm3, 2),
        mass_g=round(mass_g, 2),
        mass_savings_pct=round(mass_savings_pct, 1),
        reference_savings_pct=None,
        rotational_inertia_kg_cm2=round(rotational_inertia_kg_cm2, 3),
        max_stress_mpa=round(max_stress_mpa, 2),
        effective_yield_mpa=round(yield_eff_mpa, 2),
        safety_factor=round(safety_factor, 2),
        tip_deflection_mm=round(tooth_deflection_mm, 4),
        dynamic_torque_nm=round(torque_nm * k_v, 2),
        estimated_cost_usd=round(estimated_cost_usd, 2),
        stiffness_to_weight_score=round(stiffness_to_weight_score, 2),
        print_without_supports=no_supports,
        passes_requirements=passes_requirements,
        stress_profile=stress_profile,
    )
    return metrics, alerts


def evaluate_gear_global_alerts(
    req: GearDesignRequest, material: dict
) -> list[AlertItem]:
    """
    Evaluate global DFM, undercutting, nozzle resolution, thermal, and FDM backlash
    alerts for involute spur gears and GT2 timing pulleys.
    """
    alerts: list[AlertItem] = []
    max_temp_c = float(material["max_temp_c"])
    mat_name = str(material["name"])
    mat_id = str(material["id"])
    category = str(material.get("category", "fdm"))

    # 1. Undercutting alert for involute gears with < 17 teeth (at 20 deg pressure angle)
    if req.gear_kind == "spur_involute" and req.teeth < 17:
        alerts.append(
            AlertItem(
                level="warning",
                code="GEAR_UNDERCUTTING_RISK",
                message=(
                    f"Engranaje involuta con Z={req.teeth} (< 17 dientes) sufre socavado "
                    "(undercutting) en la raíz, debilitando el diente. Usa ≥ 17 dientes o mayor ángulo de presión."
                ),
            )
        )

    # 2. Nozzle resolution alerts for FDM tooth geometry
    if category == "fdm":
        if req.gear_kind == "spur_involute" and req.module_mm < 2.5 * req.nozzle_diameter_mm:
            alerts.append(
                AlertItem(
                    level="warning",
                    code="GEAR_NOZZLE_RESOLUTION",
                    message=(
                        f"Módulo ({req.module_mm:.2f} mm) demasiado fino para boquilla de "
                        f"{req.nozzle_diameter_mm:.2f} mm (recomendado m ≥ {2.5 * req.nozzle_diameter_mm:.2f} mm "
                        "para definir bien el flanco involuta)."
                    ),
                )
            )
        elif req.gear_kind == "gt2_pulley" and req.nozzle_diameter_mm > 0.4:
            alerts.append(
                AlertItem(
                    level="warning",
                    code="GT2_NOZZLE_TOO_LARGE",
                    message=(
                        f"Las poleas GT2 (paso 2.0 mm) requieren boquilla ≤ 0.4 mm; con "
                        f"{req.nozzle_diameter_mm:.2f} mm los dientes quedarán redondeados y la correa saltará."
                    ),
                )
            )

        if mat_id in CF_MATERIALS and req.nozzle_diameter_mm < 0.4:
            alerts.append(
                AlertItem(
                    level="warning",
                    code="CF_NOZZLE_CLOG_RISK",
                    message=(
                        f"{mat_name} requiere boquilla endurecida ≥ 0.4 mm para evitar atascos."
                    ),
                )
            )

        if req.backlash_mm < 0.08:
            alerts.append(
                AlertItem(
                    level="info",
                    code="FDM_BACKLASH_RECOMMENDATION",
                    message=(
                        f"Backlash FDM ({req.backlash_mm:.2f} mm) muy ajustado; se recomienda "
                        "0.12–0.20 mm para compensar expansión térmica y evitar engrane forzado."
                    ),
                )
            )

        if req.print_orientation == "z_vertical":
            alerts.append(
                AlertItem(
                    level="warning",
                    code="GEAR_Z_ORIENTATION_WEAKNESS",
                    message=(
                        "Imprimir engranajes en orientación Z vertical reduce la resistencia de los "
                        "dientes y exige soportes. Imprime plano sobre la cama ('xy_flat')."
                    ),
                )
            )

    # 3. Thermal alerts
    if req.operating_temp_c >= max_temp_c:
        alerts.append(
            AlertItem(
                level="critical",
                code="THERMAL_LIMIT_EXCEEDED",
                message=(
                    f"Temperatura de operación ({req.operating_temp_c:.0f}°C) supera el límite de "
                    f"{mat_name} ({max_temp_c:.0f}°C); riesgo de deformación de dientes bajo carga."
                ),
            )
        )
    elif req.operating_temp_c >= max_temp_c - 10.0:
        alerts.append(
            AlertItem(
                level="warning",
                code="THERMAL_MARGIN_LOW",
                message=(
                    f"Temperatura de operación ({req.operating_temp_c:.0f}°C) próxima al límite "
                    f"térmico de {mat_name} ({max_temp_c:.0f}°C)."
                ),
            )
        )

    return alerts
