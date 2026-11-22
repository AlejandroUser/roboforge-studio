"""FastAPI application for RoboForge Studio CAD generation and engineering analysis."""

from __future__ import annotations

import re
import tempfile
from pathlib import Path
from typing import Any

from fastapi import FastAPI, File, HTTPException, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from .cad.exporter import (
    EXPORTS_ROOT,
    PROJECT_ROOT,
    compute_design_hash,
    export_variants,
    load_cached_metadata,
)
from .cad.step_analyzer import analyze_step_file
from .cad.variants import generate_all_variants
from .physics.dfm_rules import evaluate_global_alerts
from .physics.mechanics import (
    evaluate_variant_metrics,
    load_materials,
    load_presets,
)
from .schemas import (
    GenerateResponse,
    LinkDesignRequest,
    StepReferenceAnalysis,
    VariantKind,
    VariantResult,
)

FRONTEND_DIR = PROJECT_ROOT / "frontend"

VARIANT_ORDER: tuple[VariantKind, ...] = (
    "tapered_solid",
    "organic_ibeam",
    "rounded_truss",
    "hollow_cobot",
    "organic_topopt",
)

HASH_PATTERN = re.compile(r"^[a-f0-9]{8,64}$")
FILENAME_PATTERN = re.compile(r"^[a-z0-9_]+\.(glb|step|stl)$")

MEDIA_TYPES: dict[str, str] = {
    ".glb": "model/gltf-binary",
    ".stl": "model/stl",
    ".step": "application/step",
}

app = FastAPI(
    title="RoboForge Studio API",
    description="Motor paramétrico CAD (build123d) y análisis estructural para eslabones robóticos.",
    version="1.0.0",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/api/health")
def health_check() -> dict[str, str]:
    return {"status": "ok", "service": "RoboForge Studio"}


@app.get("/api/materials")
def get_materials() -> dict[str, Any]:
    return load_materials()


@app.get("/api/presets")
def get_presets() -> dict[str, Any]:
    return load_presets()


@app.post("/api/analyze-step", response_model=StepReferenceAnalysis)
async def analyze_step_upload(file: UploadFile = File(...)) -> StepReferenceAnalysis:
    filename = file.filename or "referencia.step"
    ext = Path(filename).suffix.lower()
    if ext not in {".step", ".stp"}:
        raise HTTPException(
            status_code=400,
            detail="Formato no soportado. Sube un archivo .STEP o .STP válido.",
        )

    content = await file.read()
    if not content:
        raise HTTPException(status_code=400, detail="El archivo .STEP está vacío.")
    if len(content) > 25 * 1024 * 1024:
        raise HTTPException(status_code=413, detail="El archivo .STEP excede el límite de 25 MB.")

    tmp_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(suffix=ext, delete=False) as tmp:
            tmp.write(content)
            tmp_path = Path(tmp.name)
        return analyze_step_file(tmp_path, filename)
    except HTTPException:
        raise
    except Exception as exc:
        raise HTTPException(
            status_code=422,
            detail=f"No se pudo analizar la geometría del archivo .STEP: {exc}",
        ) from exc
    finally:
        if tmp_path is not None and tmp_path.exists():
            try:
                tmp_path.unlink()
            except OSError:
                pass


def _pick_recommended_variant(variants: list[VariantResult]) -> VariantKind:
    """Select the optimal variant prioritizing passing requirements and stiffness-to-weight."""
    passing = [v for v in variants if v.is_valid_solid and v.metrics.passes_requirements]
    if passing:
        best = max(
            passing,
            key=lambda v: (v.metrics.stiffness_to_weight_score, -v.metrics.mass_g),
        )
        return best.variant_id

    valid = [v for v in variants if v.is_valid_solid]
    if valid:
        best = max(
            valid,
            key=lambda v: (v.metrics.safety_factor, v.metrics.stiffness_to_weight_score),
        )
        return best.variant_id

    return "tapered_solid"


@app.post("/api/generate", response_model=GenerateResponse)
def generate_design(req: LinkDesignRequest) -> GenerateResponse:
    materials = load_materials()
    if req.material_id not in materials:
        raise HTTPException(
            status_code=400,
            detail=f"Material '{req.material_id}' no encontrado en la base de datos.",
        )
    material = materials[req.material_id]

    design_hash = compute_design_hash(req)
    cached_meta = load_cached_metadata(design_hash)

    if cached_meta is not None:
        variants_meta = cached_meta
    else:
        generated = generate_all_variants(req)
        export_variants(design_hash, generated)
        variants_meta = {
            vid: {
                "name": data["name"],
                "tagline": data["tagline"],
                "volume_mm3": data["volume_mm3"],
                "is_valid": data["is_valid"],
            }
            for vid, data in generated.items()
        }

    global_alerts = evaluate_global_alerts(req, material)

    # Evaluate baseline solid first to compute mass savings percentage
    baseline_vol = variants_meta["tapered_solid"]["volume_mm3"]
    baseline_metrics, _ = evaluate_variant_metrics(
        req=req,
        variant_id="tapered_solid",
        volume_mm3=baseline_vol,
        baseline_mass_g=None,
        material=material,
    )
    baseline_mass_g = baseline_metrics.mass_g

    variant_results: list[VariantResult] = []
    for vid in VARIANT_ORDER:
        meta = variants_meta[vid]
        metrics, alerts = evaluate_variant_metrics(
            req=req,
            variant_id=vid,
            volume_mm3=meta["volume_mm3"],
            baseline_mass_g=baseline_mass_g,
            material=material,
        )
        variant_results.append(
            VariantResult(
                variant_id=vid,
                name=meta["name"],
                tagline=meta["tagline"],
                is_valid_solid=bool(meta["is_valid"]),
                metrics=metrics,
                glb_url=f"/api/files/{design_hash}/{vid}.glb",
                step_url=f"/api/files/{design_hash}/{vid}.step",
                stl_url=f"/api/files/{design_hash}/{vid}.stl",
                alerts=alerts,
            )
        )

    recommended = _pick_recommended_variant(variant_results)

    return GenerateResponse(
        design_hash=design_hash,
        recommended_variant=recommended,
        material_name=material.get("name", req.material_id),
        global_alerts=global_alerts,
        variants=variant_results,
    )


@app.get("/api/files/{design_hash}/{filename}")
def get_exported_file(design_hash: str, filename: str) -> FileResponse:
    if not HASH_PATTERN.match(design_hash) or not FILENAME_PATTERN.match(filename):
        raise HTTPException(status_code=400, detail="Ruta de archivo inválida.")

    file_path = (EXPORTS_ROOT / design_hash / filename).resolve()
    try:
        file_path.relative_to(EXPORTS_ROOT.resolve())
    except ValueError as exc:
        raise HTTPException(status_code=403, detail="Acceso denegado.") from exc

    if not file_path.exists() or not file_path.is_file():
        raise HTTPException(status_code=404, detail="Archivo no encontrado.")

    ext = file_path.suffix.lower()
    media_type = MEDIA_TYPES.get(ext, "application/octet-stream")
    return FileResponse(path=str(file_path), media_type=media_type, filename=filename)


if FRONTEND_DIR.exists():
    app.mount("/", StaticFiles(directory=str(FRONTEND_DIR), html=True), name="frontend")
