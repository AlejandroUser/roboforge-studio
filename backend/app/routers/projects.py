"""Multi-project family router (v9.0): FPV Drones, 4WD Rovers, and Camera Gimbals."""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, HTTPException

from ..cad.exporter import get_export_dir
from ..families import compute_project_hash, get_family_plugin, list_family_infos
from ..physics.mechanics import load_materials
from ..physics.profiles import load_printers
from ..schemas import ProjectGenerateRequest, ProjectGenerateResponse

router = APIRouter(prefix="/api/projects", tags=["projects"])


@router.get("/families")
def get_project_families() -> dict[str, Any]:
    """Return metadata and presets for all registered v9.0 project families."""
    return {"families": list_family_infos()}


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
