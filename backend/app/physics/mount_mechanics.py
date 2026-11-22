from __future__ import annotations

import copy
import json
from functools import lru_cache
from pathlib import Path

import numpy as np

from ..schemas import (
    AlertItem,
    MountActuatorType,
    MountDesignRequest,
    MountVariantKind,
    VariantMetrics,
)

DATA_DIR = Path(__file__).resolve().parent.parent / "data"

CF_MATERIALS = {"petg_cf", "pa12_cf"}

# Effective vertical plate width (mm) per actuator family
PLATE_WIDTH_MM: dict[MountActuatorType, float] = {
    "nema17": 44.0,
    "nema23": 60.0,
    "servo_mg996r": 56.0,
    "profile_2020_gusset": 24.0,
}

# Corner stress concentration factors Kt per mount architecture
KT_BY_MOUNT_VARIANT: dict[MountVariantKind, float] = {
    "mount_standard": 1.45,
    "mount_gusseted": 1.18,
    "mount_lightweight": 1.24,
}


@lru_cache(maxsize=1)
def _read_mount_presets_raw() -> dict:
    path = DATA_DIR / "mount_presets.json"
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def load_mount_presets() -> dict:
    """Load mechatronic mount presets keyed under 'actuators'."""
    return copy.deepcopy(_read_mount_presets_raw())


def _mount_section_properties(
    req: MountDesignRequest, variant_id: MountVariantKind
) -> tuple[float, float]:
    """
    Compute effective section moment of inertia I_eff (mm^4) and outer fiber
    distance c_eff (mm) at the 90-degree L-bracket corner.
    Reflects the 3x-5x real stiffness gain provided by side triangular gussets.
    """
    width_mm = PLATE_WIDTH_MM[req.actuator_type]
    t = float(req.wall_thickness_mm)
    i_plate = (width_mm * (t**3)) / 12.0

    if variant_id == "mount_standard":
        return max(i_plate, 1e-3), t * 0.5

    t_g = float(req.gusset_thickness_mm)
    eff_gusset_depth = max(
        t * 1.4,
        min(float(req.base_depth_mm) - t, float(req.shaft_height_mm) - t) * 0.65,
    )
    gusset_ratio = (2.0 * t_g) / max(width_mm, 1.0)
    depth_ratio = eff_gusset_depth / max(t, 1.0)
    gusset_gain = float(
        np.clip(1.0 + 3.2 * (gusset_ratio**0.45) * (depth_ratio**0.75), 3.0, 5.2)
    )

    if variant_id == "mount_gusseted":
        return max(i_plate * gusset_gain, 1e-3), t * 0.65

    # mount_lightweight retains ~88% of gusseted stiffness while removing low-stress web mass
    if variant_id == "mount_lightweight":
        return max(i_plate * gusset_gain * 0.88, 1e-3), t * 0.68

    raise ValueError(f"Unsupported mount variant_id: {variant_id}")


def _mount_print_without_supports(
    variant_id: MountVariantKind, req: MountDesignRequest, material: dict
) -> bool:
    if material.get("category") == "cnc":
        return True
    if variant_id in ("mount_standard", "mount_gusseted"):
        return True
    # Lightweight mount prints cleanly without supports when laid flat on its side
    return req.print_orientation == "xy_flat"


def _estimate_mount_cost_usd(
    mass_kg: float, material: dict, print_without_supports: bool
) -> float:
    raw_cost = mass_kg * float(material["cost_per_kg_usd"])
    if material.get("category") == "cnc":
        return raw_cost * 2.4 + 14.0
    support_overhead = 0.0 if print_without_supports else 0.50
    return raw_cost * 1.15 + 0.65 + support_overhead


def evaluate_mount_metrics(
    req: MountDesignRequest,
    variant_id: MountVariantKind,
    volume_mm3: float,
    baseline_mass_g: float | None,
    material: dict,
) -> tuple[VariantMetrics, list[AlertItem]]:
    """
    Evaluate structural mechanics, corner bending stress, shaft deflection,
    anisotropic safety factor, stiffness-to-weight score, and 12-station stress
    profile for a 90-degree mechatronic L-bracket mount.
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

    radial_load_n = float(req.radial_load_n)
    shaft_h_mm = float(req.shaft_height_mm)
    torque_n_mm = float(req.motor_torque_nm) * 1000.0

    # Bending moment at the 90-degree corner (N*mm)
    m_corner_n_mm = radial_load_n * shaft_h_mm + torque_n_mm

    i_eff_mm4, c_eff_mm = _mount_section_properties(req, variant_id)
    kt = KT_BY_MOUNT_VARIANT[variant_id]

    max_stress_mpa = kt * (m_corner_n_mm * c_eff_mm) / i_eff_mm4
    safety_factor = yield_eff_mpa / max(max_stress_mpa, 1e-6)

    # Tip/shaft deflection under radial load + motor reaction torque (mm)
    tip_deflection_mm = (
        (radial_load_n * (shaft_h_mm**3)) / (3.0 * youngs_eff_mpa * i_eff_mm4)
        + (torque_n_mm * (shaft_h_mm**2)) / (2.0 * youngs_eff_mpa * i_eff_mm4)
    )

    # 12-station normalized stress profile from 90-deg corner (1.0) to top of plate (0.0)
    u = np.linspace(0.0, 1.0, 12)
    m_stations = radial_load_n * shaft_h_mm * (1.0 - u) + torque_n_mm * (1.0 - 0.85 * u)
    # Gusseted variants distribute stress more evenly along the lower half
    profile_exp = 1.0 if variant_id == "mount_standard" else 0.85
    raw_profile = np.power(np.clip(m_stations / max(m_corner_n_mm, 1e-9), 0.0, 1.0), profile_exp)
    raw_profile[-1] = 0.0
    stress_profile = [round(float(v), 3) for v in raw_profile]

    # Effective inertia & reaction torque
    h_cm = shaft_h_mm / 10.0
    rotational_inertia_kg_cm2 = mass_kg * (h_cm**2) * 0.33
    dynamic_torque_nm = float(req.motor_torque_nm) + radial_load_n * (shaft_h_mm / 1000.0)

    stiffness_n_per_mm = radial_load_n / max(tip_deflection_mm, 1e-5)
    stiffness_to_weight_score = stiffness_n_per_mm / max(mass_kg, 1e-5)

    no_supports = _mount_print_without_supports(variant_id, req, material)
    estimated_cost_usd = _estimate_mount_cost_usd(mass_kg, material, no_supports)

    passes_requirements = (
        safety_factor >= req.target_safety_factor
        and tip_deflection_mm <= req.max_deflection_mm
    )

    alerts: list[AlertItem] = []
    if safety_factor < 1.0:
        alerts.append(
            AlertItem(
                level="critical",
                code="MOUNT_CORNER_FAILURE",
                message=(
                    f"Esfuerzo en la esquina 90° ({max_stress_mpa:.1f} MPa) supera el límite "
                    f"elástico efectivo ({yield_eff_mpa:.1f} MPa). Usa cartelas o mayor espesor."
                ),
            )
        )
    elif safety_factor < req.target_safety_factor:
        alerts.append(
            AlertItem(
                level="warning",
                code="LOW_SAFETY_FACTOR",
                message=(
                    f"Factor de seguridad ({safety_factor:.2f}) por debajo del objetivo "
                    f"({req.target_safety_factor:.2f})."
                ),
            )
        )

    if tip_deflection_mm > req.max_deflection_mm:
        alerts.append(
            AlertItem(
                level="warning",
                code="EXCESSIVE_DEFLECTION",
                message=(
                    f"Deflexión en el eje ({tip_deflection_mm:.3f} mm) supera el límite "
                    f"({req.max_deflection_mm:.2f} mm). Las cartelas reducen la flexión 3×–5×."
                ),
            )
        )

    metrics = VariantMetrics(
        volume_cm3=round(volume_cm3, 2),
        mass_g=round(mass_g, 2),
        mass_savings_pct=round(mass_savings_pct, 1),
        reference_savings_pct=None,
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
        stress_profile=stress_profile,
    )
    return metrics, alerts


def evaluate_mount_global_alerts(
    req: MountDesignRequest, material: dict
) -> list[AlertItem]:
    """
    Evaluate global thermal, wall-thickness/nozzle, and Z-orientation DFM alerts
    for 90-degree mechatronic mounts.
    """
    alerts: list[AlertItem] = []

    max_temp_c = float(material["max_temp_c"])
    mat_name = str(material["name"])
    mat_id = str(material["id"])
    category = str(material.get("category", "fdm"))

    # 1. Motor thermal alerts (critical when motor_temp_c >= max_temp_c - 5)
    if req.motor_temp_c >= max_temp_c - 5.0:
        alerts.append(
            AlertItem(
                level="critical",
                code="MOTOR_THERMAL_CRITICAL",
                message=(
                    f"La temperatura del motor ({req.motor_temp_c:.0f}°C) alcanza o excede el "
                    f"límite térmico seguro de {mat_name} ({max_temp_c:.0f}°C). "
                    "La carcasa del motor ablandará y deformará la montura; cambia a PETG-CF, ABS/ASA, PA12-CF o Aluminio."
                ),
            )
        )
    elif req.motor_temp_c >= max_temp_c - 12.0:
        alerts.append(
            AlertItem(
                level="warning",
                code="MOTOR_THERMAL_MARGIN_LOW",
                message=(
                    f"Margen térmico reducido ({req.motor_temp_c:.0f}°C vs HDT {max_temp_c:.0f}°C "
                    f"en {mat_name}); puede aparecer fluencia lenta bajo tensión de correa."
                ),
            )
        )

    if (
        not req.ventilation_slots
        and req.actuator_type in ("nema17", "nema23")
        and req.motor_temp_c >= 55.0
    ):
        alerts.append(
            AlertItem(
                level="info",
                code="VENTILATION_RECOMMENDED",
                message=(
                    "Activa las ranuras de ventilación térmica para mejorar la disipación de la "
                    "carcasa del motor stepper."
                ),
            )
        )

    # 2. Z-orientation delamination warning for 90-degree L-brackets
    if category == "fdm" and req.print_orientation == "z_vertical":
        yield_xy = float(material["yield_xy_mpa"])
        yield_z = float(material["yield_z_mpa"])
        loss_pct = (1.0 - yield_z / max(yield_xy, 1e-6)) * 100.0
        alerts.append(
            AlertItem(
                level="warning",
                code="MOUNT_Z_ORIENTATION_DELAMINATION",
                message=(
                    f"Imprimir una escuadra 90° en orientación Z vertical alinea las capas con el "
                    f"plano de tracción de la esquina, reduciendo la resistencia ~{loss_pct:.0f}%. "
                    "Imprime apoyada sobre un lateral ('xy_flat') para máxima resistencia."
                ),
            )
        )

    # 3. Nozzle & wall thickness DFM checks
    if category == "fdm":
        if mat_id in CF_MATERIALS and req.nozzle_diameter_mm < 0.4:
            alerts.append(
                AlertItem(
                    level="warning",
                    code="CF_NOZZLE_CLOG_RISK",
                    message=(
                        f"{mat_name} requiere una boquilla endurecida ≥ 0.4 mm para evitar atascos."
                    ),
                )
            )

        min_wall = max(3.0, req.nozzle_diameter_mm * 6.0)
        if req.wall_thickness_mm < min_wall:
            alerts.append(
                AlertItem(
                    level="warning",
                    code="MOUNT_WALL_TOO_THIN",
                    message=(
                        f"El espesor de pared ({req.wall_thickness_mm:.1f} mm) es bajo para boquilla "
                        f"de {req.nozzle_diameter_mm:.2f} mm (recomendado ≥ {min_wall:.1f} mm)."
                    ),
                )
            )

        min_gusset = req.nozzle_diameter_mm * 4.0
        if req.gusset_thickness_mm < min_gusset:
            alerts.append(
                AlertItem(
                    level="warning",
                    code="GUSSET_TOO_THIN_FOR_NOZZLE",
                    message=(
                        f"El espesor de cartela ({req.gusset_thickness_mm:.1f} mm) requiere al menos "
                        f"4 perímetros (≥ {min_gusset:.1f} mm)."
                    ),
                )
            )

    return alerts
