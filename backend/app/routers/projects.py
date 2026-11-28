"""Multi-project family router (v9.0–v13.0): FPV Drones, Propellers, Rovers, Gimbals & Generative Physics."""

from __future__ import annotations

import dataclasses
import hashlib
import json
from typing import Any

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

from ..cad.exporter import get_export_dir
from ..cad.organic_sdf_builder import build_organic_sdf_link, export_organic_sdf_link_files
from ..cad.organic_studio import generate_organic_studio_model
from ..families import compute_project_hash, get_family_plugin, list_family_infos
from ..physics.mechanics import load_materials
from ..physics.profiles import load_printers
from ..physics.propeller_bemt import register_custom_airfoil_dat
from ..physics.propeller_optimizer import evolve_propeller_pareto
from ..physics.topopt_3d import run_simp_3d_link
from ..schemas import (
    DFMAuditReport,
    DFMCheckItem,
    ProjectGenerateRequest,
    ProjectGenerateResponse,
)

router = APIRouter(prefix="/api/projects", tags=["projects"])


class PropellerEvolveRequest(BaseModel):
    target_thrust_g: float = Field(default=650.0, ge=100.0, le=5000.0)
    diameter_inch: float = Field(default=5.0, ge=2.0, le=12.0)
    rpm: float = Field(default=18000.0, ge=4000.0, le=38000.0)
    blade_count: int = Field(default=3, ge=2, le=4)
    prop_style: str = Field(default="conventional")
    material_id: str = Field(default="petg_cf")


class Simp3DLinkRequest(BaseModel):
    length_mm: float = Field(default=180.0, ge=80.0, le=350.0)
    root_width_mm: float = Field(default=44.0, ge=24.0, le=80.0)
    tip_width_mm: float = Field(default=32.0, ge=18.0, le=64.0)
    thickness_mm: float = Field(default=22.0, ge=12.0, le=48.0)
    payload_kg: float = Field(default=2.0, ge=0.2, le=20.0)
    angular_accel_rad_s2: float = Field(default=8.0, ge=0.0, le=40.0)
    volume_fraction: float = Field(default=0.38, ge=0.22, le=0.60)
    overhang_filter_45: bool = Field(default=True)
    material_id: str = Field(default="petg_cf")
    printer_id: str = Field(default="bambu_x1c")


class AirfoilDatImportRequest(BaseModel):
    airfoil_id: str = Field(default="custom_dat", min_length=2, max_length=48)
    dat_text: str = Field(..., min_length=20)


class OrganicStudioRequest(BaseModel):
    domain: str = Field(default="link")
    material_id: str = Field(default="pa12_cf")
    printer_id: str = Field(default="bambu_x1c")
    params: dict[str, Any] = Field(default_factory=dict)


@router.post("/organic-studio/generate")
def generate_organic_studio_endpoint(req: OrganicStudioRequest) -> dict[str, Any]:
    """Unified RoboForge v14.0 Organic Sculpting Studio across all 6 engineering domains."""
    materials = load_materials()
    printers = load_printers()
    material = materials.get(req.material_id) or materials.get("pa12_cf") or next(iter(materials.values()))
    printer = printers.get(req.printer_id) or printers.get("bambu_x1c") or next(iter(printers.values()))

    key_blob = json.dumps(req.model_dump(), sort_keys=True)
    design_hash = hashlib.sha256(("org14:" + key_blob).encode("utf-8")).hexdigest()[:16]
    output_dir = get_export_dir(design_hash)

    try:
        return generate_organic_studio_model(
            domain=req.domain,
            params=req.params,
            material=material,
            printer=printer,
            out_dir=output_dir,
            design_hash=design_hash,
        )
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"Error en Esculpido Orgánico ({req.domain}): {exc}") from exc


@router.get("/families")
def get_project_families() -> dict[str, Any]:
    """Return metadata and presets for all registered project families."""
    return {"families": list_family_infos()}


@router.post("/propeller/import-dat")
def import_airfoil_dat_endpoint(req: AirfoilDatImportRequest) -> dict[str, Any]:
    """Parse and register a custom UIUC (.dat Selig/Lednicer) airfoil profile in memory."""
    try:
        return register_custom_airfoil_dat(req.airfoil_id, req.dat_text)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@router.post("/propeller/evolve")
def evolve_propeller_endpoint(req: PropellerEvolveRequest) -> dict[str, Any]:
    """Run NSGA-II + Adkins-Liebeck multi-objective propeller evolution on local CPU."""
    materials = load_materials()
    material = materials.get(req.material_id) or materials.get("petg_cf") or next(iter(materials.values()))
    density = float(material.get("density_g_cm3", 1.25))
    yield_z = float(material.get("yield_z_mpa", material.get("yield_mpa", 45.0)))

    res = evolve_propeller_pareto(
        target_thrust_g=req.target_thrust_g,
        diameter_inch=req.diameter_inch,
        rpm=req.rpm,
        blade_count=req.blade_count,
        prop_style=req.prop_style,
        material_density_g_cm3=density,
        material_yield_z_mpa=yield_z,
    )
    raw_winners = res.winners.values() if isinstance(res.winners, dict) else res.winners
    winners_out = [
        dataclasses.asdict(w) if dataclasses.is_dataclass(w) else dict(w)
        for w in raw_winners
    ]
    return {
        "baseline_metrics": res.baseline_metrics,
        "elapsed_s": round(res.elapsed_s, 2),
        "evaluations": res.evaluations,
        "generations": res.generations,
        "pop_size": res.pop_size,
        "winners": winners_out,
        "pareto_front": res.pareto_front,
    }


@router.post("/generative/simp3d-link")
def generate_simp3d_link_endpoint(req: Simp3DLinkRequest) -> dict[str, Any]:
    """Run 3D Hex8 SIMP multi-case + Langelaar 45° + Preserve/Obstacle SDF organic link synthesis."""
    materials = load_materials()
    printers = load_printers()
    material = materials.get(req.material_id) or materials.get("petg_cf") or next(iter(materials.values()))
    printer = printers.get(req.printer_id) or printers.get("bambu_x1c") or next(iter(printers.values()))

    key_blob = json.dumps(req.model_dump(), sort_keys=True)
    design_hash = hashlib.sha256(("simp3d:" + key_blob).encode("utf-8")).hexdigest()[:16]
    output_dir = get_export_dir(design_hash)

    load_n = req.payload_kg * 9.80665
    simp = run_simp_3d_link(
        length_mm=req.length_mm,
        root_width_mm=req.root_width_mm,
        tip_width_mm=req.tip_width_mm,
        thickness_mm=req.thickness_mm,
        load_n=load_n,
        angular_accel_rad_s2=req.angular_accel_rad_s2,
        volume_fraction=req.volume_fraction,
        overhang_filter_45=req.overhang_filter_45,
    )
    mesh, stats = build_organic_sdf_link(
        rho_3d=simp.rho_3d,
        length_mm=req.length_mm,
        root_width_mm=req.root_width_mm,
        tip_width_mm=req.tip_width_mm,
        thickness_mm=req.thickness_mm,
    )

    mat_id = str(material.get("id", req.material_id))
    rgba = [48, 56, 68, 255] if "cf" in mat_id else ([188, 198, 212, 255] if "al" in mat_id else [118, 134, 152, 255])
    metallic = 0.85 if "al" in mat_id else 0.25
    roughness = 0.26 if "al" in mat_id else 0.44
    files = export_organic_sdf_link_files(
        mesh=mesh,
        material_rgba=rgba,
        metallic=metallic,
        roughness=roughness,
        out_dir=output_dir,
        stem="organic_simp3d_link",
        material_hint=mat_id,
    )

    density = float(material.get("density_g_cm3", 1.25))
    mass_g = round((float(stats["volume_mm3"]) / 1000.0) * density, 1)
    bed_x = float(printer.get("bed_x_mm", 256.0))
    bed_y = float(printer.get("bed_y_mm", 256.0))
    bed_z = float(printer.get("bed_z_mm", 256.0))
    dims = sorted([float(x) for x in mesh.extents])
    sz, sy, sx = dims[0], dims[1], dims[2]
    fits_bed = ((sx <= bed_x - 10.0 and sy <= bed_y - 10.0) or (sy <= bed_x - 10.0 and sx <= bed_y - 10.0)) and sz <= bed_z - 4.0

    checks = [
        DFMCheckItem(
            id="watertight",
            label="Malla Orgánica Watertight",
            status="pass" if stats["is_watertight"] else "fail",
            detail=f"Volumen real {stats['volume_mm3']:.0f} mm³ · {'Cerrada sin bordes abiertos' if stats['is_watertight'] else 'Bordes abiertos'}.",
        ),
        DFMCheckItem(
            id="floating_islands",
            label="Cero Islas Flotantes",
            status="pass" if stats["bodies"] == 1 else "fail",
            detail=f"{stats['bodies']} cuerpo conectado (Filtro Langelaar 45° {'ON' if req.overhang_filter_45 else 'OFF'}).",
        ),
        DFMCheckItem(
            id="fits_bed",
            label=f"Ajuste a Cama ({printer.get('name', req.printer_id)})",
            status="pass" if fits_bed else "fail",
            detail=f"Dimensiones {sx:.0f}×{sy:.0f}×{sz:.0f} mm en cama {bed_x:.0f}×{bed_y:.0f}×{bed_z:.0f} mm.",
        ),
    ]
    overall = "pass" if all(c.status == "pass" for c in checks) else "fail"
    dfm_audit = DFMAuditReport(
        status=overall,
        summary=f"Auditoría Goldfish DFM ({overall.upper()}): {sum(1 for c in checks if c.status == 'pass')}/{len(checks)} controles aprobados.",
        checks=checks,
    )

    c0 = simp.compliance_history[0] if simp.compliance_history else 1.0
    c_end = simp.compliance_history[-1] if simp.compliance_history else 1.0
    threemf_path = files.get("3mf") or files.get("threemf")
    return {
        "design_hash": design_hash,
        "name": f"Eslabón Orgánico SIMP 3D ({req.length_mm:.0f} mm · {req.payload_kg:.1f} kg)",
        "glb_url": f"/api/files/{design_hash}/{files['glb'].name}",
        "stl_url": f"/api/files/{design_hash}/{files['stl'].name}",
        "threemf_url": f"/api/files/{design_hash}/{threemf_path.name}" if threemf_path else "",
        "metrics": {
            "mass_g": mass_g,
            "volume_mm3": round(float(stats["volume_mm3"]), 1),
            "mass_reduction_pct": round(simp.mass_reduction_pct, 1),
            "final_volume_fraction": round(simp.final_volume_fraction, 3),
            "simp_elapsed_s": round(simp.elapsed_s, 2),
            "overhang_45": req.overhang_filter_45,
            "payload_kg": req.payload_kg,
            "angular_accel_rad_s2": req.angular_accel_rad_s2,
        },
        "compliance_history": [round(float(x), 4) for x in simp.compliance_history],
        "compliance_ratio": round(c_end / max(c0, 1e-9), 3),
        "dfm_audit": dfm_audit.model_dump(),
        "notes": [
            f"SIMP 3D Hex8 multi-caso ({simp.nx}×{simp.ny}×{simp.nz}, {simp.elapsed_s:.1f} s): reducción de masa {simp.mass_reduction_pct:.1f}% ({mass_g:.1f} g en {material.get('name', mat_id)}).",
            f"Interfaces Preserve/Obstacle exactas + suavizado Taubin ({'Filtro Langelaar 45° activo' if req.overhang_filter_45 else 'sin filtro 45°'}).",
        ],
    }


@router.post("/{family}/generate", response_model=ProjectGenerateResponse)
def generate_project_for_family(
    family: str,
    req: ProjectGenerateRequest,
) -> ProjectGenerateResponse:
    """Generate 3D CAD, physics, BOM, and Goldfish DFM audit for a project family."""
    plugin = get_family_plugin(family)
    if plugin is None:
        raise HTTPException(
            status_code=404,
            detail=f"Familia de proyecto desconocida: '{family}'.",
        )

    materials = load_materials()
    printers = load_printers()
    material = materials.get(req.material_id) or materials.get("petg_cf") or next(iter(materials.values()))
    printer_profile = printers.get(req.printer_id) or printers.get("bambu_x1c") or next(iter(printers.values()))

    design_hash = compute_project_hash(family, req)
    output_dir = get_export_dir(design_hash)

    try:
        result = plugin.generate(req, material, printer_profile, output_dir, design_hash)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except Exception as exc:
        raise HTTPException(
            status_code=500,
            detail=f"Error generando proyecto '{family}': {exc}",
        ) from exc

    if isinstance(result, ProjectGenerateResponse):
        return result
    return ProjectGenerateResponse.model_validate(result)
