"""FastAPI application for RoboForge Studio CAD generation and engineering analysis."""

from __future__ import annotations

import json
import re
import tempfile
import threading
import time
from pathlib import Path
from typing import Any

from fastapi import FastAPI, File, Form, HTTPException, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, PlainTextResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from .cad.arm_assembly import (
    build_and_export_arm_assembly,
    build_modular_arm_assembly,
    compute_arm_design_hash,
    compute_modular_arm_hash,
    export_arm_mujoco,
    export_arm_urdf,
    export_bom_csv,
)
from .cad.exporter import (
    EXPORTS_ROOT,
    PROJECT_ROOT,
    build_arm_manufacturing_zip,
    compute_design_hash,
    compute_gear_design_hash,
    compute_mount_design_hash,
    export_gear_variants,
    export_mount_variants,
    export_variants,
    get_export_dir,
    load_cached_gear_metadata,
    load_cached_metadata,
    load_cached_mount_metadata,
)
from .cad.bimanual_assembly import export_bimanual_urdf
from .cad.cartesian_assembly import export_cartesian_urdf
from .cad.continuum_assembly import export_continuum_urdf
from .cad.delta_assembly import export_delta_urdf
from .cad.gear_variants import generate_all_gear_variants
from .cad.hardware_parts import export_cnc_plates_dxf
from .cad.mount_variants import generate_all_mount_variants
from .cad.print_prep import generate_print_readiness_pack
from .cad.scara_assembly import export_scara_urdf
from .cad.stewart_assembly import export_stewart_urdf
from .cad.sdf_gyroid import generate_field_driven_gyroid
from .cad.sim_exporters import export_full_dof_urdf, export_topology_mujoco
from .cad.step_analyzer import analyze_step_file
from .cad.variants import generate_all_variants
from .physics.arm_compatibility import (
    auto_match_interfaces,
    evaluate_modular_arm,
    validate_arm_compatibility,
)
from .physics.arm_kinematics import evaluate_arm_assembly, load_arm_presets
from .physics.copilot import analyze_2d_blueprint, run_closed_loop_copilot
from .physics.dfm_rules import evaluate_global_alerts
from .physics.fea_solver import run_gear_fea, run_mount_fea, run_step_fea
from .physics.firmware_gen import generate_arduino_pca9685_ino
from .physics.gear_mechanics import (
    compute_gear_diameters,
    evaluate_gear_global_alerts,
    evaluate_gear_metrics,
    load_gear_presets,
)
from .physics.ik_solver import compute_caliper_compensation, solve_arm_ik
from .physics.mechanics import (
    evaluate_variant_metrics,
    load_materials,
    load_presets,
)
from .physics.mechatronics import (
    evaluate_mechatronics,
    generate_circuit_schematic_svg,
    generate_wireviz_yaml,
    get_mechatronics_catalog,
    master_bom_to_csv,
)
from .physics.mount_mechanics import (
    evaluate_mount_global_alerts,
    evaluate_mount_metrics,
    load_mount_presets,
)
from .physics.profiles import (
    delete_custom_material,
    delete_custom_printer,
    load_printers,
    save_custom_material,
    save_custom_printer,
)
from .routers.projects import router as projects_router
from .schemas import (
    ArmAssemblyRequest,
    ArmAssemblyResponse,
    BlueprintAnalysisResponse,
    CopilotRequest,
    CopilotResponse,
    CustomMaterialCreate,
    FEARequest,
    FEAResult,
    GearDesignRequest,
    GearFEARequest,
    GearGenerateResponse,
    GearVariantKind,
    GearVariantResult,
    GenerateResponse,
    GyroidRequest,
    GyroidResult,
    LinkDesignRequest,
    ModularArmRequest,
    ModularArmResponse,
    MountDesignRequest,
    MountFEARequest,
    MountGenerateResponse,
    MountVariantKind,
    MountVariantResult,
    PrinterProfile,
    PrinterProfileCreate,
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

MOUNT_VARIANT_ORDER: tuple[MountVariantKind, ...] = (
    "mount_standard",
    "mount_gusseted",
    "mount_lightweight",
)

GEAR_VARIANT_ORDER: tuple[GearVariantKind, ...] = (
    "gear_solid",
    "gear_webbed",
    "gear_spoked",
)

HASH_PATTERN = re.compile(r"^(?:(?:ref|gyr|prj)_)?[a-f0-9]{8,64}$")
FILENAME_PATTERN = re.compile(r"^[A-Za-z0-9_]+\.(glb|step|stl|3mf|urdf|xml|csv|zip|svg|yml|ino|dxf|json|md)$")

MEDIA_TYPES: dict[str, str] = {
    ".glb": "model/gltf-binary",
    ".stl": "model/stl",
    ".step": "application/step",
    ".3mf": "model/3mf",
    ".urdf": "application/xml",
    ".xml": "application/xml",
    ".csv": "text/csv; charset=utf-8",
    ".zip": "application/zip",
    ".svg": "image/svg+xml",
    ".yml": "text/yaml; charset=utf-8",
    ".ino": "text/plain; charset=utf-8",
    ".dxf": "application/dxf",
    ".json": "application/json",
    ".md": "text/markdown; charset=utf-8",
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
app.include_router(projects_router)


@app.get("/api/health")
def health_check() -> dict[str, str]:
    return {"status": "ok", "service": "RoboForge Studio"}


@app.get("/api/materials")
def get_materials() -> dict[str, Any]:
    return load_materials()


@app.post("/api/materials/custom")
def create_custom_material(mat: CustomMaterialCreate) -> dict[str, Any]:
    try:
        return save_custom_material(mat)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@app.delete("/api/materials/custom/{material_id}")
def remove_custom_material(material_id: str) -> dict[str, Any]:
    deleted = delete_custom_material(material_id)
    if not deleted:
        raise HTTPException(
            status_code=404,
            detail=f"Material personalizado '{material_id}' no encontrado o es de fábrica.",
        )
    return {"status": "deleted", "id": material_id}


@app.get("/api/printers")
def get_printers() -> dict[str, Any]:
    return load_printers()


@app.post("/api/printers", response_model=PrinterProfile)
def create_custom_printer(profile: PrinterProfileCreate) -> PrinterProfile:
    try:
        return save_custom_printer(profile)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@app.delete("/api/printers/{printer_id}")
def remove_custom_printer(printer_id: str) -> dict[str, Any]:
    deleted = delete_custom_printer(printer_id)
    if not deleted:
        raise HTTPException(
            status_code=404,
            detail=f"Impresora personalizada '{printer_id}' no encontrada o es de fábrica.",
        )
    return {"status": "deleted", "id": printer_id}


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
                threemf_url=f"/api/files/{design_hash}/{vid}.3mf",
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


@app.post("/api/fea", response_model=FEAResult)
def run_fea_validation(fea_req: FEARequest) -> FEAResult:
    materials = load_materials()
    if fea_req.request.material_id not in materials:
        raise HTTPException(
            status_code=400,
            detail=f"Material '{fea_req.request.material_id}' no encontrado en la base de datos.",
        )
    material = materials[fea_req.request.material_id]

    design_hash = (
        fea_req.design_hash
        if HASH_PATTERN.match(fea_req.design_hash)
        else compute_design_hash(fea_req.request)
    )
    out_dir = get_export_dir(design_hash)
    step_path = out_dir / f"{fea_req.variant_id}.step"

    if not (step_path.exists() and step_path.stat().st_size > 0):
        generated = generate_all_variants(fea_req.request)
        export_variants(design_hash, generated)

    if not step_path.exists():
        raise HTTPException(
            status_code=404,
            detail=f"No se encontró el archivo .STEP para la variante '{fea_req.variant_id}'.",
        )

    deform_tag = int(round(fea_req.deform_scale))
    output_glb_path = (
        out_dir / f"{fea_req.variant_id}_fea_{fea_req.load_direction}_d{deform_tag}.glb"
    )
    try:
        return run_step_fea(
            step_path=step_path,
            output_glb_path=output_glb_path,
            req=fea_req.request,
            design_hash=design_hash,
            variant_id=fea_req.variant_id,
            material=material,
            mesh_size_mm=fea_req.mesh_size_mm,
            load_direction=fea_req.load_direction,
            deform_scale=fea_req.deform_scale,
        )
    except HTTPException:
        raise
    except Exception as exc:
        raise HTTPException(
            status_code=500,
            detail=f"Error en el solver FEA 3D (Gmsh/Tet4): {exc}",
        ) from exc


def _pick_recommended_mount_variant(variants: list[MountVariantResult]) -> MountVariantKind:
    """Select the optimal mount variant prioritizing passing requirements and stiffness-to-weight."""
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

    return "mount_gusseted"


@app.get("/api/mounts/presets")
def get_mount_presets() -> dict[str, Any]:
    return load_mount_presets()


@app.post("/api/mounts/generate", response_model=MountGenerateResponse)
def generate_mount_design(req: MountDesignRequest) -> MountGenerateResponse:
    materials = load_materials()
    if req.material_id not in materials:
        raise HTTPException(
            status_code=400,
            detail=f"Material '{req.material_id}' no encontrado en la base de datos.",
        )
    material = materials[req.material_id]

    design_hash = compute_mount_design_hash(req)
    cached_meta = load_cached_mount_metadata(design_hash)

    if cached_meta is not None:
        variants_meta = cached_meta
    else:
        generated = generate_all_mount_variants(req)
        export_mount_variants(design_hash, generated)
        variants_meta = {
            vid: {
                "name": data["name"],
                "tagline": data["tagline"],
                "volume_mm3": data["volume_mm3"],
                "is_valid": data["is_valid"],
            }
            for vid, data in generated.items()
        }

    global_alerts = evaluate_mount_global_alerts(req, material)

    # Evaluate baseline (mount_standard) first for mass savings comparison
    baseline_vol = variants_meta["mount_standard"]["volume_mm3"]
    baseline_metrics, _ = evaluate_mount_metrics(
        req=req,
        variant_id="mount_standard",
        volume_mm3=baseline_vol,
        baseline_mass_g=None,
        material=material,
    )
    baseline_mass_g = baseline_metrics.mass_g

    variant_results: list[MountVariantResult] = []
    for vid in MOUNT_VARIANT_ORDER:
        meta = variants_meta[vid]
        metrics, alerts = evaluate_mount_metrics(
            req=req,
            variant_id=vid,
            volume_mm3=meta["volume_mm3"],
            baseline_mass_g=baseline_mass_g,
            material=material,
        )
        variant_results.append(
            MountVariantResult(
                variant_id=vid,
                name=meta["name"],
                tagline=meta["tagline"],
                is_valid_solid=bool(meta["is_valid"]),
                metrics=metrics,
                glb_url=f"/api/files/{design_hash}/{vid}.glb",
                step_url=f"/api/files/{design_hash}/{vid}.step",
                stl_url=f"/api/files/{design_hash}/{vid}.stl",
                threemf_url=f"/api/files/{design_hash}/{vid}.3mf",
                alerts=alerts,
            )
        )

    recommended = _pick_recommended_mount_variant(variant_results)

    return MountGenerateResponse(
        design_hash=design_hash,
        recommended_variant=recommended,
        material_name=material.get("name", req.material_id),
        global_alerts=global_alerts,
        variants=variant_results,
    )


@app.post("/api/mounts/fea", response_model=FEAResult)
def run_mount_fea_validation(fea_req: MountFEARequest) -> FEAResult:
    materials = load_materials()
    if fea_req.request.material_id not in materials:
        raise HTTPException(
            status_code=400,
            detail=f"Material '{fea_req.request.material_id}' no encontrado en la base de datos.",
        )
    material = materials[fea_req.request.material_id]

    design_hash = (
        fea_req.design_hash
        if HASH_PATTERN.match(fea_req.design_hash)
        else compute_mount_design_hash(fea_req.request)
    )
    out_dir = get_export_dir(design_hash)
    step_path = out_dir / f"{fea_req.variant_id}.step"

    if not (step_path.exists() and step_path.stat().st_size > 0):
        generated = generate_all_mount_variants(fea_req.request)
        export_mount_variants(design_hash, generated)

    if not step_path.exists():
        raise HTTPException(
            status_code=404,
            detail=f"No se encontró el archivo .STEP para la montura '{fea_req.variant_id}'.",
        )

    deform_tag = int(round(fea_req.deform_scale))
    output_glb_path = (
        out_dir / f"{fea_req.variant_id}_fea_{fea_req.load_direction}_d{deform_tag}.glb"
    )
    try:
        return run_mount_fea(
            step_path=step_path,
            output_glb_path=output_glb_path,
            req=fea_req.request,
            design_hash=design_hash,
            variant_id=fea_req.variant_id,
            material=material,
            mesh_size_mm=fea_req.mesh_size_mm,
            load_direction=fea_req.load_direction,
            deform_scale=fea_req.deform_scale,
        )
    except HTTPException:
        raise
    except Exception as exc:
        raise HTTPException(
            status_code=500,
            detail=f"Error en el solver FEA 3D de montura (Gmsh/Tet4): {exc}",
        ) from exc


def _pick_recommended_gear_variant(variants: list[GearVariantResult]) -> GearVariantKind:
    """Select the optimal transmission variant prioritizing passing requirements and low inertia."""
    passing = [v for v in variants if v.is_valid_solid and v.metrics.passes_requirements]
    if passing:
        best = max(
            passing,
            key=lambda v: (v.metrics.stiffness_to_weight_score, -v.metrics.rotational_inertia_kg_cm2),
        )
        return best.variant_id

    valid = [v for v in variants if v.is_valid_solid]
    if valid:
        best = max(
            valid,
            key=lambda v: (v.metrics.safety_factor, v.metrics.stiffness_to_weight_score),
        )
        return best.variant_id

    return "gear_webbed"


@app.get("/api/gears/presets")
def get_gear_presets() -> dict[str, Any]:
    return load_gear_presets()


@app.post("/api/gears/generate", response_model=GearGenerateResponse)
def generate_gear_design(req: GearDesignRequest) -> GearGenerateResponse:
    materials = load_materials()
    if req.material_id not in materials:
        raise HTTPException(
            status_code=400,
            detail=f"Material '{req.material_id}' no encontrado en la base de datos.",
        )
    material = materials[req.material_id]

    design_hash = compute_gear_design_hash(req)
    cached_meta = load_cached_gear_metadata(design_hash)

    if cached_meta is not None:
        variants_meta = cached_meta
    else:
        generated = generate_all_gear_variants(req)
        export_gear_variants(design_hash, generated)
        variants_meta = {
            vid: {
                "name": data["name"],
                "tagline": data["tagline"],
                "volume_mm3": data["volume_mm3"],
                "is_valid": data["is_valid"],
            }
            for vid, data in generated.items()
        }

    pitch_d, outer_d = compute_gear_diameters(req)
    global_alerts = evaluate_gear_global_alerts(req, material)

    baseline_vol = variants_meta["gear_solid"]["volume_mm3"]
    baseline_metrics, _ = evaluate_gear_metrics(
        req=req,
        variant_id="gear_solid",
        volume_mm3=baseline_vol,
        baseline_mass_g=None,
        material=material,
    )
    baseline_mass_g = baseline_metrics.mass_g

    variant_results: list[GearVariantResult] = []
    for vid in GEAR_VARIANT_ORDER:
        meta = variants_meta[vid]
        metrics, alerts = evaluate_gear_metrics(
            req=req,
            variant_id=vid,
            volume_mm3=meta["volume_mm3"],
            baseline_mass_g=baseline_mass_g,
            material=material,
        )
        variant_results.append(
            GearVariantResult(
                variant_id=vid,
                name=meta["name"],
                tagline=meta["tagline"],
                is_valid_solid=bool(meta["is_valid"]),
                metrics=metrics,
                glb_url=f"/api/files/{design_hash}/{vid}.glb",
                step_url=f"/api/files/{design_hash}/{vid}.step",
                stl_url=f"/api/files/{design_hash}/{vid}.stl",
                threemf_url=f"/api/files/{design_hash}/{vid}.3mf",
                alerts=alerts,
            )
        )

    recommended = _pick_recommended_gear_variant(variant_results)

    return GearGenerateResponse(
        design_hash=design_hash,
        recommended_variant=recommended,
        material_name=material.get("name", req.material_id),
        pitch_diameter_mm=round(pitch_d, 2),
        outer_diameter_mm=round(outer_d, 2),
        global_alerts=global_alerts,
        variants=variant_results,
    )


@app.post("/api/gears/fea", response_model=FEAResult)
def run_gear_fea_validation(fea_req: GearFEARequest) -> FEAResult:
    materials = load_materials()
    if fea_req.request.material_id not in materials:
        raise HTTPException(
            status_code=400,
            detail=f"Material '{fea_req.request.material_id}' no encontrado en la base de datos.",
        )
    material = materials[fea_req.request.material_id]

    design_hash = (
        fea_req.design_hash
        if HASH_PATTERN.match(fea_req.design_hash)
        else compute_gear_design_hash(fea_req.request)
    )
    out_dir = get_export_dir(design_hash)
    step_path = out_dir / f"{fea_req.variant_id}.step"

    if not (step_path.exists() and step_path.stat().st_size > 0):
        generated = generate_all_gear_variants(fea_req.request)
        export_gear_variants(design_hash, generated)

    if not step_path.exists():
        raise HTTPException(
            status_code=404,
            detail=f"No se encontró el archivo .STEP para el engranaje '{fea_req.variant_id}'.",
        )

    deform_tag = int(round(fea_req.deform_scale))
    output_glb_path = (
        out_dir / f"{fea_req.variant_id}_fea_{fea_req.load_direction}_d{deform_tag}.glb"
    )
    try:
        return run_gear_fea(
            step_path=step_path,
            output_glb_path=output_glb_path,
            req=fea_req.request,
            design_hash=design_hash,
            variant_id=fea_req.variant_id,
            material=material,
            mesh_size_mm=fea_req.mesh_size_mm,
            load_direction=fea_req.load_direction,
            deform_scale=fea_req.deform_scale,
        )
    except HTTPException:
        raise
    except Exception as exc:
        raise HTTPException(
            status_code=500,
            detail=f"Error en el solver FEA 3D de transmisión (Gmsh/Tet4): {exc}",
        ) from exc


@app.get("/api/arm/presets")
def get_arm_presets() -> dict[str, Any]:
    return load_arm_presets()


@app.post("/api/arm/generate", response_model=ArmAssemblyResponse)
def generate_arm_assembly(req: ArmAssemblyRequest) -> ArmAssemblyResponse:
    materials = load_materials()
    if req.material_id not in materials:
        raise HTTPException(
            status_code=400,
            detail=f"Material '{req.material_id}' no encontrado en la base de datos.",
        )
    material = materials[req.material_id]

    design_hash = compute_arm_design_hash(req)
    out_dir = get_export_dir(design_hash)

    cad_result = build_and_export_arm_assembly(
        req=req,
        material=material,
        out_dir=out_dir,
        design_hash=design_hash,
    )
    segment_volumes_mm3 = cad_result["segment_volumes_mm3"]

    kin_result = evaluate_arm_assembly(
        req=req,
        segment_volumes_mm3=segment_volumes_mm3,
        material=material,
    )

    joints = kin_result["joints"]
    bom = kin_result["bom"]

    urdf_path = out_dir / "arm_assembly.urdf"
    mujoco_path = out_dir / "arm_mujoco.xml"
    bom_csv_path = out_dir / "arm_bom.csv"

    export_arm_urdf(req, joints, urdf_path)
    export_arm_mujoco(req, joints, mujoco_path)
    export_bom_csv(bom, bom_csv_path)

    presets_data = load_arm_presets()
    preset_info = presets_data.get("presets", {}).get(req.preset, {})
    preset_name = preset_info.get("name", req.preset)

    return ArmAssemblyResponse(
        design_hash=design_hash,
        preset_name=preset_name,
        material_name=material.get("name", req.material_id),
        dof=req.dof,
        max_reach_mm=kin_result["max_reach_mm"],
        total_arm_mass_kg=kin_result["total_arm_mass_kg"],
        printed_mass_g=kin_result["printed_mass_g"],
        estimated_total_cost_usd=kin_result["estimated_total_cost_usd"],
        tcp_position_mm=kin_result["tcp_position_mm"],
        tcp_deflection_mm=kin_result["tcp_deflection_mm"],
        joints=joints,
        bom=bom,
        global_alerts=kin_result.get("global_alerts", []),
        glb_url=f"/api/files/{design_hash}/arm_assembly.glb",
        step_url=f"/api/files/{design_hash}/arm_assembly.step",
        stl_url=f"/api/files/{design_hash}/arm_assembly.stl",
        urdf_url=f"/api/files/{design_hash}/arm_assembly.urdf",
        mujoco_url=f"/api/files/{design_hash}/arm_mujoco.xml",
        bom_csv_url=f"/api/files/{design_hash}/arm_bom.csv",
    )


@app.post("/api/copilot/optimize", response_model=CopilotResponse)
def optimize_with_copilot(req: CopilotRequest) -> CopilotResponse:
    try:
        return run_closed_loop_copilot(req)
    except Exception as exc:
        raise HTTPException(
            status_code=500,
            detail=f"Error en el copiloto de optimización: {exc}",
        ) from exc


@app.post("/api/copilot/blueprint", response_model=BlueprintAnalysisResponse)
async def analyze_blueprint_endpoint(
    file: UploadFile = File(...),
    prompt: str = Form(""),
    verify_with_fea: bool = Form(False),
) -> BlueprintAnalysisResponse:
    filename = file.filename or "blueprint.png"
    ext = Path(filename).suffix.lower()
    if ext not in {".png", ".jpg", ".jpeg", ".webp", ".pdf"}:
        raise HTTPException(
            status_code=400,
            detail="Formato no soportado. Sube un plano 2D en PNG, JPG, WEBP o PDF.",
        )
    content = await file.read()
    if not content or len(content) > 20 * 1024 * 1024:
        raise HTTPException(status_code=400, detail="Archivo vacío o superior a 20 MB.")
    try:
        return analyze_2d_blueprint(
            image_bytes=content,
            filename=filename,
            user_prompt=prompt,
            verify_with_fea=verify_with_fea,
        )
    except Exception as exc:
        raise HTTPException(
            status_code=500,
            detail=f"Error analizando plano 2D: {exc}",
        ) from exc


@app.post("/api/sdf/gyroid", response_model=GyroidResult)
def generate_gyroid_sdf_endpoint(req: GyroidRequest) -> GyroidResult:
    materials = load_materials()
    if req.request.material_id not in materials:
        raise HTTPException(
            status_code=400,
            detail=f"Material '{req.request.material_id}' no encontrado en la base de datos.",
        )
    material = materials[req.request.material_id]
    from .cad.sdf_gyroid import _compute_gyroid_hash

    design_hash = _compute_gyroid_hash(req)
    out_dir = get_export_dir(design_hash)
    try:
        return generate_field_driven_gyroid(req, material, out_dir)
    except Exception as exc:
        raise HTTPException(
            status_code=500,
            detail=f"Error generando celosía implícita SDF Gyroid: {exc}",
        ) from exc


@app.get("/api/mechatronics/catalog")
def get_mechatronics_catalog_endpoint() -> dict[str, Any]:
    return get_mechatronics_catalog()


class FirmwarePoseInput(BaseModel):
    j0_base: float = 0.0
    j1_shoulder: float = 35.0
    j2_elbow: float = -55.0
    j3_wrist: float = 20.0
    gripper_pct: float = 50.0


class FirmwareInoRequest(BaseModel):
    arm_name: str = "Brazo Robótico Arduino 5.0"
    poses: list[FirmwarePoseInput] = Field(default_factory=list)
    speed_ms: int = Field(18, ge=4, le=120)


@app.post("/api/arm/firmware-ino")
def generate_arm_firmware_ino(req: FirmwareInoRequest) -> PlainTextResponse:
    ino_code = generate_arduino_pca9685_ino(
        arm_name=req.arm_name,
        poses=[p.model_dump() for p in req.poses],
        speed_ms=req.speed_ms,
    )
    return PlainTextResponse(
        content=ino_code,
        media_type="text/plain; charset=utf-8",
        headers={"Content-Disposition": 'attachment; filename="brazo_5_0_secuencia.ino"'},
    )


@app.post("/api/arm/firmware-esp32-ino")
def generate_arm_firmware_esp32_ino(req: FirmwareInoRequest) -> PlainTextResponse:
    from .physics.firmware_gen import generate_esp32_wifi_pca9685_ino

    ino_code = generate_esp32_wifi_pca9685_ino(
        arm_name=req.arm_name,
        poses=[p.model_dump() for p in req.poses],
        speed_ms=req.speed_ms,
    )
    return PlainTextResponse(
        content=ino_code,
        media_type="text/plain; charset=utf-8",
        headers={"Content-Disposition": 'attachment; filename="mars_mini_esp32_wifi.ino"'},
    )


@app.get("/api/arm/database")
def get_arm_database() -> dict[str, Any]:
    db_path = Path(__file__).resolve().parent / "data" / "arm_database.json"
    if not db_path.exists():
        return {"arms": []}
    try:
        return json.loads(db_path.read_text(encoding="utf-8"))
    except Exception:
        return {"arms": []}


# v8.0: Cinemática Inversa 3D (IK), Calibración Pie de Rey y Control USB en Vivo
class ArmIkRequest(BaseModel):
    target_x_mm: float = 200.0
    target_y_mm: float = 0.0
    target_z_mm: float = 150.0
    l1_mm: float = Field(200.0, ge=20.0, le=1500.0)
    l2_mm: float = Field(170.0, ge=20.0, le=1500.0)
    l3_mm: float = Field(75.0, ge=0.0, le=400.0)
    base_height_mm: float = Field(95.0, ge=20.0, le=800.0)
    kinematics_type: str = "articulated"
    elbow_up: bool = True


class CaliperCalibrationRequest(BaseModel):
    measured_m3_mm: float = Field(..., ge=1.5, le=6.0)
    measured_bore_mm: float = Field(..., ge=5.0, le=60.0)
    nominal_m3_mm: float = Field(3.2, ge=1.5, le=6.0)
    nominal_bore_mm: float = Field(22.2, ge=5.0, le=60.0)
    process_type: str = "msla_resin"


class HardwarePoseRequest(BaseModel):
    port: str = "VIRTUAL_SIM"
    baudrate: int = Field(115200, ge=9600, le=1000000)
    j0_base: float = 0.0
    j1_shoulder: float = 35.0
    j2_elbow: float = -55.0
    j3_wrist: float = 20.0
    gripper_pct: float = 50.0


@app.post("/api/arm/ik")
def solve_arm_ik_endpoint(req: ArmIkRequest) -> dict[str, Any]:
    return solve_arm_ik(
        target_x_mm=req.target_x_mm,
        target_y_mm=req.target_y_mm,
        target_z_mm=req.target_z_mm,
        l1_mm=req.l1_mm,
        l2_mm=req.l2_mm,
        l3_mm=req.l3_mm,
        base_height_mm=req.base_height_mm,
        kinematics_type=req.kinematics_type,
        elbow_up=req.elbow_up,
    )


@app.post("/api/print/calibrate")
def calibrate_print_caliper_endpoint(req: CaliperCalibrationRequest) -> dict[str, Any]:
    return compute_caliper_compensation(
        measured_m3_mm=req.measured_m3_mm,
        measured_bore_mm=req.measured_bore_mm,
        nominal_m3_mm=req.nominal_m3_mm,
        nominal_bore_mm=req.nominal_bore_mm,
        process_type=req.process_type,
    )


@app.get("/api/hardware/ports")
def list_hardware_ports() -> dict[str, Any]:
    ports: list[dict[str, str]] = [
        {
            "port": "VIRTUAL_SIM",
            "description": "Simulador USB RoboForge (Sin placa física)",
        }
    ]
    try:
        from serial.tools import list_ports  # type: ignore[import-not-found]

        for p in list_ports.comports():
            ports.append(
                {
                    "port": str(p.device),
                    "description": str(p.description or p.device),
                }
            )
    except Exception:
        pass
    return {"ports": ports}


_SERIAL_POOL: dict[str, Any] = {}
_SERIAL_LOCK = threading.Lock()
_SERIAL_BOOT_WAIT_S = 1.8


def _get_or_open_serial(port: str, baudrate: int) -> Any:
    """Return an open serial handle from the persistent pool, opening once if needed."""
    import serial  # type: ignore[import-not-found]

    existing = _SERIAL_POOL.get(port)
    if existing is not None and getattr(existing, "is_open", True):
        if getattr(existing, "baudrate", baudrate) == baudrate:
            return existing
        try:
            existing.close()
        except Exception:
            pass
        _SERIAL_POOL.pop(port, None)

    ser = serial.Serial(port, baudrate, timeout=0.5)
    time.sleep(_SERIAL_BOOT_WAIT_S)
    _SERIAL_POOL[port] = ser
    return ser


def _close_pooled_serial(port: str) -> bool:
    """Close and evict a serial port from the persistent pool."""
    with _SERIAL_LOCK:
        ser = _SERIAL_POOL.pop(port, None)
        if ser is None:
            return False
        try:
            ser.close()
        except Exception:
            pass
        return True


class HardwareClosePortRequest(BaseModel):
    port: str = Field(..., min_length=1, max_length=64)


@app.post("/api/hardware/close-port")
def close_hardware_port(req: HardwareClosePortRequest) -> dict[str, Any]:
    closed = _close_pooled_serial(req.port)
    return {"ok": True, "port": req.port, "closed": closed}


@app.post("/api/hardware/send-pose")
def send_hardware_pose(req: HardwarePoseRequest) -> dict[str, Any]:
    frame = (
        f"P:{int(round(req.j0_base))},{int(round(req.j1_shoulder))},"
        f"{int(round(req.j2_elbow))},{int(round(req.j3_wrist))},"
        f"{int(round(req.gripper_pct))}\n"
    )
    if req.port == "VIRTUAL_SIM":
        return {
            "ok": True,
            "frame": frame,
            "port": req.port,
            "mode": "virtual_sim",
        }
    try:
        with _SERIAL_LOCK:
            ser = _get_or_open_serial(req.port, req.baudrate)
            ser.write(frame.encode("ascii"))
            ser.flush()
        return {
            "ok": True,
            "frame": frame,
            "port": req.port,
            "mode": "serial_usb",
        }
    except Exception as exc:
        _close_pooled_serial(req.port)
        return {
            "ok": False,
            "frame": frame,
            "port": req.port,
            "mode": "serial_error",
            "error": str(exc),
        }


@app.post("/api/arm/auto-match")
def auto_match_modular_arm(req: ModularArmRequest) -> dict[str, Any]:
    matched_req, notes = auto_match_interfaces(req)
    issues = validate_arm_compatibility(matched_req)
    compatible = not any(i.severity == "critical" for i in issues)
    return {
        "compatible": compatible,
        "auto_fixed": len(notes) > 0,
        "auto_fix_notes": notes,
        "compatibility_issues": [i.model_dump() for i in issues],
        "slots": matched_req.slots.model_dump(),
        "request": matched_req.model_dump(),
    }


@app.post("/api/arm/assemble", response_model=ModularArmResponse)
def assemble_modular_arm(req: ModularArmRequest) -> ModularArmResponse:
    materials = load_materials()
    if req.material_id not in materials:
        raise HTTPException(
            status_code=400,
            detail=f"Material '{req.material_id}' no encontrado en la base de datos.",
        )

    auto_fixed = False
    auto_fix_notes: list[str] = []
    effective_req = req
    if req.auto_fix:
        effective_req, auto_fix_notes = auto_match_interfaces(req)
        auto_fixed = len(auto_fix_notes) > 0

    material = materials[effective_req.material_id]
    design_hash = compute_modular_arm_hash(effective_req)
    out_dir = get_export_dir(design_hash)

    cad_res = build_modular_arm_assembly(
        req=effective_req,
        material=material,
        output_dir=out_dir,
        design_hash=design_hash,
    )

    phys_res = evaluate_modular_arm(
        req=effective_req,
        real_volumes_mm3=cad_res["real_volumes_mm3"],
    )

    slots_used = phys_res["slots_used"]
    joints = phys_res["joints"]
    bom = phys_res["bom"]

    # Export URDF, MuJoCo XML, and BOM CSV for the modular arm
    proxy_style = (
        slots_used.upper_arm_variant
        if slots_used.upper_arm_variant in ("organic_ibeam", "hollow_cobot", "rounded_truss", "tapered_solid")
        else "organic_ibeam"
    )
    proxy_arm_req = ArmAssemblyRequest(
        preset="parol6_desktop",
        dof=max(4, min(7, int(effective_req.dof))),
        payload_kg=max(0.1, min(8.0, float(effective_req.payload_kg))),
        base_height_mm=max(40.0, min(200.0, float(effective_req.base_height_mm))),
        upper_arm_mm=max(40.0, min(1500.0, float(slots_used.upper_arm.length_mm) if slots_used.upper_arm else 200.0)),
        forearm_mm=max(40.0, min(1500.0, float(slots_used.forearm.length_mm) if slots_used.forearm else 170.0)),
        wrist_mm=max(20.0, min(160.0, float(effective_req.wrist_mm))),
        link_style=proxy_style,  # type: ignore[arg-type]
        shoulder_joint_deg=float(effective_req.joint_angles_deg[0]),
        elbow_joint_deg=float(effective_req.joint_angles_deg[1]),
        wrist_joint_deg=float(effective_req.joint_angles_deg[2]),
        max_accel_rad_s2=effective_req.max_accel_rad_s2,
        material_id=effective_req.material_id,
        printer_id=effective_req.printer_id,
    )

    urdf_path = out_dir / "arm_assembly.urdf"
    mujoco_path = out_dir / "arm_mujoco.xml"
    bom_csv_path = out_dir / "arm_bom.csv"

    export_full_dof_urdf(effective_req, phys_res, urdf_path)
    export_topology_mujoco(effective_req, phys_res, mujoco_path)
    export_bom_csv(bom, bom_csv_path)

    # v4.0: Full Mechatronics evaluation + SVG schematic + WireViz YAML + Master BOM CSV
    mechatronics = evaluate_mechatronics(effective_req, phys_res)
    circuit_svg_path = out_dir / "arm_circuit_schematic.svg"
    harness_yml_path = out_dir / "arm_harness.yml"
    master_bom_csv_path = out_dir / "arm_master_bom.csv"

    circuit_svg_path.write_text(
        generate_circuit_schematic_svg(mechatronics, effective_req.dof),
        encoding="utf-8",
    )
    harness_yml_path.write_text(
        generate_wireviz_yaml(mechatronics, effective_req.dof),
        encoding="utf-8",
    )
    master_bom_csv_path.write_text(
        master_bom_to_csv(mechatronics.master_bom),
        encoding="utf-8",
    )

    zip_included_files: list[Path] = [
        *cad_res["individual_files"],
        cad_res["step_path"],
        urdf_path,
        mujoco_path,
        bom_csv_path,
        circuit_svg_path,
        harness_yml_path,
        master_bom_csv_path,
    ]
    firmware_ino_path: Path | None = None
    if mechatronics.bus_architecture == "arduino_pca9685_i2c":
        firmware_ino_path = out_dir / "brazo_5_0_secuencia.ino"
        default_pose = {
            "j0_base": 0.0,
            "j1_shoulder": float(effective_req.joint_angles_deg[0]),
            "j2_elbow": float(effective_req.joint_angles_deg[1]),
            "j3_wrist": float(effective_req.joint_angles_deg[2]),
            "gripper_pct": 60.0,
        }
        firmware_ino_path.write_text(
            generate_arduino_pca9685_ino(
                arm_name="Brazo Robótico Arduino 5.0",
                poses=[
                    default_pose,
                    {"j0_base": -40.0, "j1_shoulder": 20.0, "j2_elbow": -40.0, "j3_wrist": 15.0, "gripper_pct": 15.0},
                    {"j0_base": 40.0, "j1_shoulder": 45.0, "j2_elbow": -65.0, "j3_wrist": 25.0, "gripper_pct": 85.0},
                ],
                speed_ms=18,
            ),
            encoding="utf-8",
        )
        zip_included_files.append(firmware_ino_path)

    firmware_esp32_ino_path: Path | None = None
    if mechatronics.bus_architecture == "esp32_pca9685_5v":
        from .physics.firmware_gen import generate_esp32_wifi_pca9685_ino

        firmware_esp32_ino_path = out_dir / "mars_mini_esp32_wifi.ino"
        default_pose = {
            "j0_base": 0.0,
            "j1_shoulder": float(effective_req.joint_angles_deg[0]),
            "j2_elbow": float(effective_req.joint_angles_deg[1]),
            "j3_wrist": float(effective_req.joint_angles_deg[2]),
            "gripper_pct": 60.0,
        }
        firmware_esp32_ino_path.write_text(
            generate_esp32_wifi_pca9685_ino(
                arm_name="RoboForge Mars-Mini (ESP32 Wi-Fi)",
                poses=[
                    default_pose,
                    {"j0_base": -35.0, "j1_shoulder": 25.0, "j2_elbow": -35.0, "j3_wrist": 10.0, "gripper_pct": 15.0},
                    {"j0_base": 35.0, "j1_shoulder": 40.0, "j2_elbow": -50.0, "j3_wrist": 15.0, "gripper_pct": 85.0},
                ],
                speed_ms=18,
            ),
            encoding="utf-8",
        )
        zip_included_files.append(firmware_esp32_ino_path)

    cnc_dxf_path: Path | None = None
    if getattr(effective_req, "cnc_plate_mode", False):
        cnc_dxf_path = out_dir / "arm_cnc_plates.dxf"
        export_cnc_plates_dxf(effective_req, cnc_dxf_path)
        zip_included_files.append(cnc_dxf_path)

    lerobot_json_path: Path | None = None
    if mechatronics.bus_architecture == "uart_serial_bus":
        from .physics.mechatronics import generate_lerobot_config_json

        lerobot_json_path = out_dir / "lerobot_config.json"
        lerobot_json_path.write_text(
            generate_lerobot_config_json(effective_req),
            encoding="utf-8",
        )
        zip_included_files.append(lerobot_json_path)

    # v7.0: 3D Print-Readiness Pack (Auto-Orient, Multi-Plate .3MF Bin-Packing, Calibration STL, PRINT_GUIDE.md)
    printers_db = load_printers()
    printer_profile = printers_db.get(
        effective_req.printer_id or "bambu_x1c",
        next(iter(printers_db.values()), {"name": "Bambu Lab X1C", "bed_x_mm": 256.0, "bed_y_mm": 256.0, "bed_z_mm": 256.0}),
    )
    printable_parts = cad_res.get("printable_parts") or {}
    print_summary, print_files = generate_print_readiness_pack(
        printable_parts=printable_parts,
        req=effective_req,
        material=material,
        printer_profile=printer_profile,
        output_dir=out_dir,
        design_hash=design_hash,
    )
    zip_included_files.extend(print_files.values())

    # Package full manufacturing kit ZIP (individual part STEP/3MF + full STEP + URDF + MuJoCo + BOM + Print Plates + Guide)
    zip_path = build_arm_manufacturing_zip(
        out_dir=out_dir,
        included_files=zip_included_files,
        zip_filename="arm_manufacturing_pack.zip",
    )

    files = {
        "glb": f"/api/files/{design_hash}/arm_assembly.glb",
        "step": f"/api/files/{design_hash}/arm_assembly.step",
        "stl": f"/api/files/{design_hash}/arm_assembly.stl",
        "urdf": f"/api/files/{design_hash}/arm_assembly.urdf",
        "mujoco": f"/api/files/{design_hash}/arm_mujoco.xml",
        "bom_csv": f"/api/files/{design_hash}/arm_bom.csv",
        "circuit_svg": f"/api/files/{design_hash}/arm_circuit_schematic.svg",
        "harness_yml": f"/api/files/{design_hash}/arm_harness.yml",
        "master_bom_csv": f"/api/files/{design_hash}/arm_master_bom.csv",
        "zip": f"/api/files/{design_hash}/{zip_path.name}",
    }
    for k, fp in print_files.items():
        files[k] = f"/api/files/{design_hash}/{fp.name}"
    if firmware_ino_path is not None:
        files["firmware_ino"] = f"/api/files/{design_hash}/{firmware_ino_path.name}"
    if firmware_esp32_ino_path is not None:
        files["firmware_esp32_ino"] = f"/api/files/{design_hash}/{firmware_esp32_ino_path.name}"
    if cnc_dxf_path is not None:
        files["cnc_dxf"] = f"/api/files/{design_hash}/{cnc_dxf_path.name}"
    if lerobot_json_path is not None:
        files["lerobot_json"] = f"/api/files/{design_hash}/{lerobot_json_path.name}"

    return ModularArmResponse(
        design_hash=design_hash,
        compatible=bool(phys_res["compatible"]),
        auto_fixed=auto_fixed,
        auto_fix_notes=auto_fix_notes,
        compatibility_issues=phys_res["compatibility_issues"],
        per_part_metrics=phys_res["per_part_metrics"],
        total_mass_g=float(phys_res["total_mass_g"]),
        reach_mm=float(phys_res["reach_mm"]),
        shoulder_torque_nm=float(phys_res["shoulder_torque_nm"]),
        min_safety_factor=float(phys_res["min_safety_factor"]),
        tcp_position_mm=phys_res["tcp_position_mm"],
        explode_vectors=cad_res["explode_vectors"],
        files=files,
        slots_used=slots_used,
        joints=joints,
        bom=bom,
        mechatronics=mechatronics,
        print_readiness=print_summary,
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


REACT_DIST_DIR = PROJECT_ROOT / "frontend-react" / "dist"
if REACT_DIST_DIR.exists() and (REACT_DIST_DIR / "index.html").exists():
    app.mount("/", StaticFiles(directory=str(REACT_DIST_DIR), html=True), name="frontend-react")
elif FRONTEND_DIR.exists():
    app.mount("/", StaticFiles(directory=str(FRONTEND_DIR), html=True), name="frontend")
