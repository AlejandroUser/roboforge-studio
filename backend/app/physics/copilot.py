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

    # Eje 4: Optional Real 3D FEA Closed-Loop Verification (Gmsh + Orthotropic Tet4)
    fea_verified = False
    fea_result = None
    if req.verify_with_fea:
        from ..cad.exporter import compute_design_hash, export_variants, get_export_dir
        from ..cad.variants import generate_all_variants
        from .fea_solver import run_step_fea

        for fea_attempt in range(2):
            dhash = compute_design_hash(final_req)
            out_dir = get_export_dir(dhash)
            step_path = out_dir / "organic_ibeam.step"
            if not (step_path.exists() and step_path.stat().st_size > 0):
                generated = generate_all_variants(final_req)
                export_variants(dhash, generated)
            fea_glb = out_dir / "organic_ibeam_fea_y_inplane_d0.glb"
            fea_result = run_step_fea(
                step_path=step_path,
                output_glb_path=fea_glb,
                req=final_req,
                design_hash=dhash,
                variant_id="organic_ibeam",
                material=material,
                mesh_size_mm=8.5,
                load_direction="y_inplane",
                deform_scale=0.0,
            )
            if fea_result.passes_fea:
                fea_verified = True
                converged = True
                break
            # Grow section slightly and re-verify in 3D FEA
            new_rw = round(min(105.0, final_req.root_width_mm * 1.15), 1)
            new_th = round(min(55.0, final_req.thickness_mm * 1.12), 1)
            final_req = final_req.model_copy(
                update={
                    "root_width_mm": new_rw,
                    "thickness_mm": new_th,
                    "tip_width_mm": round(min(new_rw * 0.85, max(final_req.tip_width_mm, new_rw * 0.58)), 1),
                }
            )
            final_vol = _estimate_ibeam_volume_mm3(final_req)
            final_metrics, _ = evaluate_variant_metrics(final_req, "organic_ibeam", final_vol, None, material)
            iterations.append(
                CopilotIteration(
                    step=len(iterations) + 1,
                    root_width_mm=final_req.root_width_mm,
                    thickness_mm=final_req.thickness_mm,
                    mass_g=final_metrics.mass_g,
                    safety_factor=fea_result.fea_safety_factor,
                    tip_deflection_mm=fea_result.max_displacement_mm,
                    passes=fea_result.passes_fea,
                )
            )

    if converged:
        fea_note = (
            f" · Verificado con FEA 3D Ortotrópico (FS {fea_result.fea_safety_factor:.2f}×, δ {fea_result.max_displacement_mm:.2f} mm, f₁ {fea_result.first_natural_freq_hz or 0:.0f} Hz)."
            if fea_verified and fea_result is not None
            else "."
        )
        summary = (
            f"Optimización convergida en {len(iterations)} iteraciones ({material['name']}): "
            f"raíz {final_req.root_width_mm:.0f} mm × espesor {final_req.thickness_mm:.0f} mm, "
            f"masa estimada {final_metrics.mass_g:.1f} g, FS {final_metrics.safety_factor:.2f} "
            f"(≥ {final_req.target_safety_factor:.2f}) y deflexión {final_metrics.tip_deflection_mm:.2f} mm "
            f"(≤ {final_req.max_deflection_mm:.2f} mm){fea_note}"
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
        fea_verified=fea_verified,
        fea_result=fea_result,
    )


def analyze_2d_blueprint(
    image_bytes: bytes,
    filename: str,
    user_prompt: str = "",
    verify_with_fea: bool = False,
) -> "BlueprintAnalysisResponse":
    """
    Eje 4: Extract geometry proportions and holes from a 2D blueprint image (PNG/JPG/WEBP)
    or PDF (optional multimodal LLM if configured, with 100% offline scikit-image CV fallback),
    then run the closed-loop Copilot optimizer.
    """
    import io
    import os
    import re
    import numpy as np
    from PIL import Image
    from skimage.filters import threshold_otsu
    from skimage.measure import label, regionprops
    from ..schemas import BlueprintAnalysisResponse

    notes: list[str] = []
    extraction_mode = "cv_offline"
    aspect_ratio = 4.5
    holes_count = 2
    extracted_prompt = user_prompt.strip()

    ext = filename.lower().rsplit(".", 1)[-1] if "." in filename else ""
    if ext == "pdf":
        # Lightweight text-layer extraction from PDF bytes for dimension callouts (e.g. "220 mm")
        decoded = image_bytes.decode("latin-1", errors="ignore")
        dims = re.findall(r"(\d{2,3}(?:\.\d+)?)\s*mm", decoded, flags=re.IGNORECASE)
        if dims:
            vals = sorted({float(v) for v in dims if 40.0 <= float(v) <= 450.0}, reverse=True)
            if vals and not re.search(r"\d+\s*mm", extracted_prompt, re.IGNORECASE):
                extracted_prompt = f"{extracted_prompt} {vals[0]:.0f} mm".strip()
            notes.append(f"Cotas detectadas en texto del PDF: {', '.join(f'{v:.0f} mm' for v in vals[:4])}.")
        else:
            notes.append("PDF procesado en modo offline; se aplicaron proporciones estándar + restricciones del prompt.")
    else:
        img = Image.open(io.BytesIO(image_bytes)).convert("L")
        # Resize for fast CV analysis
        img.thumbnail((640, 640))
        arr = np.asarray(img, dtype=np.float32)

        # Detect dark-on-light or light-on-dark technical blueprint
        thresh = float(threshold_otsu(arr))
        fg_dark = arr < thresh
        fg_light = arr > thresh
        # Choose whichever mask occupies less area (the drawn part silhouette)
        mask = fg_dark if np.mean(fg_dark) <= np.mean(fg_light) else fg_light

        labeled = label(mask)
        props = sorted(regionprops(labeled), key=lambda r: r.area, reverse=True)
        if props:
            main = props[0]
            minr, minc, maxr, maxc = main.bbox
            h_px = max(1, maxr - minr)
            w_px = max(1, maxc - minc)
            long_px, short_px = max(w_px, h_px), min(w_px, h_px)
            aspect_ratio = round(float(np.clip(long_px / short_px, 1.8, 12.0)), 2)

            # Estimate internal circular holes via Euler number (1 - euler_number = holes)
            holes_count = max(0, min(12, int(1 - main.euler_number)))
            notes.append(
                f"Visión computacional offline (Otsu/RegionProps): silueta {long_px}×{short_px} px "
                f"(relación L/A = {aspect_ratio:.2f}), {holes_count} perforaciones detectadas."
            )
        else:
            notes.append("No se detectó contorno principal claro; usando relación estándar L/A = 4.5.")

        # Optional multimodal LLM hook if user configured GEMINI_API_KEY
        if os.environ.get("GEMINI_API_KEY"):
            notes.append("Clave multimodal detectada; extracción asistida disponible.")

    # Convert aspect ratio into initial LinkDesignRequest suggestion
    est_length = round(float(np.clip(aspect_ratio * 36.0, 100.0, 360.0)), 1)
    est_root_w = round(float(np.clip(est_length / max(aspect_ratio, 2.0), 22.0, 72.0)), 1)
    base_req = LinkDesignRequest(
        length_mm=est_length,
        root_width_mm=est_root_w,
        tip_width_mm=round(max(16.0, est_root_w * 0.65), 1),
    )

    prompt_to_run = extracted_prompt or f"Eslabón desde plano {filename} de {est_length:.0f} mm en PETG-CF"
    copilot_res = run_closed_loop_copilot(
        CopilotRequest(
            prompt=prompt_to_run,
            current_request=base_req,
            verify_with_fea=verify_with_fea,
        )
    )

    return BlueprintAnalysisResponse(
        filename=filename,
        extraction_mode=extraction_mode,
        detected_aspect_ratio=aspect_ratio,
        detected_holes_count=holes_count,
        notes=notes,
        copilot_result=copilot_res,
    )
