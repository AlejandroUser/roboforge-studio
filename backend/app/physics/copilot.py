from __future__ import annotations

import re

from ..schemas import (
    CopilotIteration,
    CopilotRequest,
    CopilotResponse,
    LinkDesignRequest,
)
from .mechanics import evaluate_variant_metrics, load_materials


def _parse_prompt_constraints(
    prompt: str, base_req: LinkDesignRequest
) -> tuple[LinkDesignRequest, list[str]]:
    """
    Parse natural-language engineering constraints in Spanish or English from
    a user prompt and apply them onto a copy of base_req.
    """
    text = prompt.lower()
    updates: dict = {}
    constraints: list[str] = []

    # 1. Architecture
    if "scara" in text:
        updates["architecture"] = "scara"
        constraints.append("Arquitectura: SCARA")
    elif "delta" in text or "biela" in text:
        updates["architecture"] = "delta_rod"
        constraints.append("Arquitectura: Biela Delta")
    elif "pata" in text or "quadruped" in text or "cuadrúpedo" in text or "leg" in text:
        updates["architecture"] = "robot_leg"
        constraints.append("Arquitectura: Pata Robótica")
    elif "articulado" in text or "6dof" in text or "6-dof" in text:
        updates["architecture"] = "articulated_6dof"
        constraints.append("Arquitectura: Articulado 6-DOF")

    # 2. Material
    material_patterns = [
        (r"pa12[\s-]*cf|nylon[\s-]*cf|nylon", "pa12_cf", "PA12-CF"),
        (r"petg[\s-]*cf", "petg_cf", "PETG-CF"),
        (r"\bpetg\b", "petg", "PETG"),
        (r"pla\+|pla\s*plus|\bpla\b", "pla_plus", "PLA+"),
        (r"\babs\b|\basa\b", "abs_asa", "ABS / ASA"),
        (r"alumin[io]*|6061", "al6061_t6", "Aluminio 6061-T6"),
        (r"placa\s*de\s*carbono|carbon\s*plate|fibra\s*de\s*carbono", "carbon_plate", "Carbon Fiber Plate"),
    ]
    for pattern, mat_id, label in material_patterns:
        if re.search(pattern, text):
            updates["material_id"] = mat_id
            constraints.append(f"Material: {label}")
            break

    # 3. Max deflection (parse BEFORE generic mm length so we don't confuse them)
    defl_match = re.search(
        r"(?:deflexi[oó]n|deflection|flecha)[^0-9]{0,18}(\d+(?:[.,]\d+)?)\s*mm",
        text,
    )
    if defl_match:
        val = float(defl_match.group(1).replace(",", "."))
        val = max(0.05, min(10.0, val))
        updates["max_deflection_mm"] = val
        constraints.append(f"Deflexión máx: ≤ {val:.2f} mm")

    # 4. Length in mm or cm (exclude the deflection match span)
    text_no_defl = (
        text[: defl_match.start()] + " " + text[defl_match.end() :]
        if defl_match
        else text
    )
    len_match = re.search(r"(\d+(?:[.,]\d+)?)\s*(mm|cm)\b", text_no_defl)
    if len_match:
        raw_val = float(len_match.group(1).replace(",", "."))
        unit = len_match.group(2)
        length_mm = raw_val * 10.0 if unit == "cm" else raw_val
        length_mm = max(60.0, min(450.0, length_mm))
        updates["length_mm"] = round(length_mm, 1)
        constraints.append(f"Longitud: {length_mm:.0f} mm")

    # 5. Load in N or kg
    load_n_match = re.search(r"(\d+(?:[.,]\d+)?)\s*(?:n|newtons?)\b", text)
    load_kg_match = re.search(r"(\d+(?:[.,]\d+)?)\s*(?:kg|kilos?)\b", text)
    if load_n_match:
        load_n = max(1.0, min(500.0, float(load_n_match.group(1).replace(",", "."))))
        updates["tip_load_n"] = round(load_n, 1)
        constraints.append(f"Carga en punta: {load_n:.1f} N")
    elif load_kg_match:
        kg = float(load_kg_match.group(1).replace(",", "."))
        load_n = max(1.0, min(500.0, kg * 9.81))
        updates["tip_load_n"] = round(load_n, 1)
        constraints.append(f"Carga en punta: {kg:.2f} kg ({load_n:.1f} N)")

    # 6. Safety factor
    sf_match = re.search(
        r"(?:fs|sf|factor(?:\s+de\s+seguridad)?|safety\s*factor)\s*(?:>=|=>|>|=|:|de)?\s*(\d+(?:[.,]\d+)?)",
        text,
    )
    if sf_match:
        sf = max(1.1, min(10.0, float(sf_match.group(1).replace(",", "."))))
        updates["target_safety_factor"] = round(sf, 2)
        constraints.append(f"Factor de Seguridad objetivo: ≥ {sf:.2f}")

    # 7. Temperature
    temp_match = re.search(r"(\d+(?:[.,]\d+)?)\s*°?\s*c\b", text)
    if temp_match:
        temp_c = max(-10.0, min(150.0, float(temp_match.group(1).replace(",", "."))))
        updates["operating_temp_c"] = round(temp_c, 1)
        constraints.append(f"Temperatura: {temp_c:.0f} °C")

    if not constraints:
        constraints.append("Optimización de rigidez/peso sobre parámetros actuales")

    return base_req.model_copy(update=updates), constraints


def _estimate_ibeam_volume_mm3(req: LinkDesignRequest) -> float:
    """Fast analytical volume proxy for organic_ibeam during closed-loop iterations."""
    avg_w = 0.5 * (req.root_width_mm + req.tip_width_mm)
    # Tapered envelope * 0.58 I-beam fill ratio + end boss rings
    span_vol = req.length_mm * avg_w * req.thickness_mm * 0.58
    boss_vol = 0.25 * 3.14159 * (
        max(0.0, req.root_end.outer_diameter_mm**2 - req.root_end.bore_diameter_mm**2)
        + max(0.0, req.tip_end.outer_diameter_mm**2 - req.tip_end.bore_diameter_mm**2)
    ) * req.thickness_mm * 0.5
    return max(span_vol + boss_vol, 5_000.0)


def run_closed_loop_copilot(req: CopilotRequest) -> CopilotResponse:
    """
    Parse natural-language engineering goals and run a closed-loop mathematical
    optimization (up to 12 iterations) using evaluate_variant_metrics to minimize
    mass while satisfying safety_factor >= target_safety_factor and
    tip_deflection_mm <= max_deflection_mm.
    """
    base_req = req.current_request or LinkDesignRequest()
    candidate, constraints = _parse_prompt_constraints(req.prompt, base_req)

    materials = load_materials()
    material = materials.get(candidate.material_id) or materials["petg_cf"]

    iterations: list[CopilotIteration] = []
    best_passing: tuple[float, LinkDesignRequest] | None = None
    best_overall: tuple[float, LinkDesignRequest] = (float("inf"), candidate)

    root_w = float(candidate.root_width_mm)
    tip_w = float(candidate.tip_width_mm)
    thick = float(candidate.thickness_mm)
    taper = float(candidate.organic_taper)

    for step in range(1, 13):
        root_w = round(max(18.0, min(110.0, root_w)), 1)
        tip_w = round(max(14.0, min(root_w * 0.85, tip_w)), 1)
        thick = round(max(10.0, min(60.0, thick)), 1)
        taper = round(max(0.2, min(0.95, taper)), 2)

        candidate = candidate.model_copy(
            update={
                "root_width_mm": root_w,
                "tip_width_mm": tip_w,
                "thickness_mm": thick,
                "organic_taper": taper,
            }
        )
        vol_mm3 = _estimate_ibeam_volume_mm3(candidate)
        metrics, _ = evaluate_variant_metrics(
            candidate, "organic_ibeam", vol_mm3, None, material
        )

        passes = metrics.passes_requirements
        iterations.append(
            CopilotIteration(
                step=step,
                root_width_mm=root_w,
                thickness_mm=thick,
                mass_g=metrics.mass_g,
                safety_factor=metrics.safety_factor,
                tip_deflection_mm=metrics.tip_deflection_mm,
                passes=passes,
            )
        )

        sf_ratio = metrics.safety_factor / max(candidate.target_safety_factor, 1e-6)
        defl_ratio = candidate.max_deflection_mm / max(metrics.tip_deflection_mm, 1e-6)
        worst_margin = min(sf_ratio, defl_ratio)

        if passes:
            if best_passing is None or metrics.mass_g < best_passing[0]:
                best_passing = (metrics.mass_g, candidate)
        else:
            deficit_score = abs(1.0 - worst_margin)
            if deficit_score < best_overall[0]:
                best_overall = (deficit_score, candidate)

        # Convergence check: passes and close to optimal mass (margin <= 1.25x)
        if passes and worst_margin <= 1.25 and step >= 3:
            break

        if not passes:
            # Increase section depth (h^3 impact) and thickness to meet stiffness & SF
            scale_needed = (1.0 / max(worst_margin, 0.15)) ** 0.36
            grow = min(1.32, max(1.06, scale_needed))
            root_w = root_w * grow
            thick = thick * min(1.18, 1.0 + 0.5 * (grow - 1.0))
            tip_w = max(tip_w, root_w * 0.58)
            taper = max(0.35, taper - 0.05)
        else:
            # Trim mass gently while staying above target SF and deflection limits
            shrink = max(0.88, min(0.96, (1.12 / max(worst_margin, 1.12)) ** 0.25))
            root_w = root_w * shrink
            thick = thick * (0.5 + 0.5 * shrink)
            tip_w = min(tip_w, root_w * 0.65)
            taper = min(0.85, taper + 0.04)

    converged = best_passing is not None
    final_req = best_passing[1] if best_passing is not None else best_overall[1]
    final_vol = _estimate_ibeam_volume_mm3(final_req)
    final_metrics, _ = evaluate_variant_metrics(
        final_req, "organic_ibeam", final_vol, None, material
    )

    if converged:
        summary = (
            f"Optimización convergida en {len(iterations)} iteraciones ({material['name']}): "
            f"raíz {final_req.root_width_mm:.0f} mm × espesor {final_req.thickness_mm:.0f} mm, "
            f"masa estimada {final_metrics.mass_g:.1f} g, FS {final_metrics.safety_factor:.2f} "
            f"(≥ {final_req.target_safety_factor:.2f}) y deflexión {final_metrics.tip_deflection_mm:.2f} mm "
            f"(≤ {final_req.max_deflection_mm:.2f} mm)."
        )
    else:
        summary = (
            f"Se alcanzó el límite geométrico tras {len(iterations)} iteraciones ({material['name']}): "
            f"FS {final_metrics.safety_factor:.2f} / deflexión {final_metrics.tip_deflection_mm:.2f} mm. "
            "Considera cambiar a PA12-CF, Aluminio 6061-T6 o Carbon Fiber Plate."
        )

    return CopilotResponse(
        summary=summary,
        parsed_constraints=constraints,
        optimized_request=final_req,
        iterations=iterations,
        converged=converged,
    )
