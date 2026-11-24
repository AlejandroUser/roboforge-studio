"""Shared base protocol & export packager for RoboForge v9.0 Multi-Project Families."""

from __future__ import annotations

import csv
import hashlib
import json
import zipfile
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any

import trimesh
from build123d import Compound, Part, export_step, export_stl

from ..cad.arm_assembly import _part_to_trimesh
from ..cad.dfm_auditor import audit_assembly_dfm
from ..cad.print_prep import generate_print_readiness_pack
from ..schemas import (
    MasterBomItem,
    ProjectGenerateRequest,
    ProjectGenerateResponse,
)


def compute_project_hash(family_id: str, req: ProjectGenerateRequest) -> str:
    """Compute a deterministic `prj_<16hex>` hash for a multi-project generation request."""
    canonical = (
        f"prj_v90:{family_id}:"
        + json.dumps(req.model_dump(), sort_keys=True, separators=(",", ":"))
    )
    return "prj_" + hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:16]


def write_simple_dxf_outline(
    circles: list[tuple[float, float, float]],
    rects: list[tuple[float, float, float, float]],
    output_path: Path,
) -> Path:
    """Write a clean R12 ASCII DXF file with CNC bolt circles and plate rectangles."""
    lines = [
        "0", "SECTION", "2", "HEADER", "9", "$ACADVER", "1", "AC1009", "0", "ENDSEC",
        "0", "SECTION", "2", "ENTITIES",
    ]
    for cx, cy, r in circles:
        lines.extend([
            "0", "CIRCLE", "8", "CUT_HOLES",
            "10", f"{cx:.3f}", "20", f"{cy:.3f}", "30", "0.0",
            "40", f"{r:.3f}",
        ])
    for x0, y0, x1, y1 in rects:
        pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        for i in range(4):
            a, b = pts[i], pts[(i + 1) % 4]
            lines.extend([
                "0", "LINE", "8", "OUTLINE",
                "10", f"{a[0]:.3f}", "20", f"{a[1]:.3f}", "30", "0.0",
                "11", f"{b[0]:.3f}", "21", f"{b[1]:.3f}", "31", "0.0",
            ])
    lines.extend(["0", "ENDSEC", "0", "EOF"])
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return output_path


def _write_project_bom_csv(bom: list[MasterBomItem], output_path: Path) -> Path:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["Categoria", "Codigo", "Pieza", "Especificacion", "Cantidad", "Unit USD", "Total USD"])
        for item in bom:
            w.writerow([
                item.category_label,
                item.part_code,
                item.part_name,
                item.spec,
                item.qty,
                f"{item.unit_cost_usd:.2f}",
                f"{item.total_cost_usd:.2f}",
            ])
    return output_path


def package_project_exports(
    *,
    family_id: str,
    preset_id: str,
    name: str,
    design_hash: str,
    req: ProjectGenerateRequest,
    material: dict[str, Any],
    printer_profile: dict[str, Any],
    output_dir: Path,
    node_specs: list[tuple[str, Part | Compound, list[int], float, float]],
    printable_parts: dict[str, Part],
    explode_vectors: dict[str, list[float]],
    metrics: dict[str, Any],
    bom: list[MasterBomItem],
    notes: list[str] | None = None,
    dxf_path: Path | None = None,
) -> ProjectGenerateResponse:
    """Export GLB, STEP, STL, print-readiness plates/STLs, Goldfish DFM audit, BOM CSV, and master ZIP."""
    output_dir.mkdir(parents=True, exist_ok=True)

    # 1. Build multi-node semantic PBR GLB + keep placed meshes for DFM AABB check
    scene = trimesh.Scene()
    placed_meshes: dict[str, trimesh.Trimesh] = {}
    all_solids: list[Part | Compound] = []
    for node_name, solid, rgba, met, rough in node_specs:
        all_solids.append(solid)
        mesh = _part_to_trimesh(solid, rgba, met, rough, node_name)
        placed_meshes[node_name] = mesh
        scene.add_geometry(mesh, node_name=node_name, geom_name=node_name)

    glb_path = output_dir / "arm_assembly.glb"
    step_path = output_dir / "arm_assembly.step"
    stl_path = output_dir / "arm_assembly.stl"

    scene.export(str(glb_path), file_type="glb")
    full_compound = Compound(children=all_solids)
    export_step(full_compound, str(step_path))
    export_stl(full_compound, str(stl_path), tolerance=0.14, angular_tolerance=0.22)

    # 2. Print-readiness pack (individual part_*.stl, build_plate_*.3mf, PRINT_GUIDE.md, arm_stl_parts_pack.zip)
    class _ShimReq:
        preset = preset_id
        material_id = req.material_id
        printer_id = req.printer_id
        mechatronics = None

    _, print_files = generate_print_readiness_pack(
        printable_parts=printable_parts,
        req=_ShimReq(),
        material=material,
        printer_profile=printer_profile,
        output_dir=output_dir,
        design_hash=design_hash,
    )

    # 3. Goldfish Pre-Print DFM Audit
    printed_bom_names = [b.part_name for b in bom if b.category == "printed_cnc"]
    dfm_audit = audit_assembly_dfm(
        printable_parts=printable_parts,
        placed_meshes=placed_meshes,
        printer_profile=printer_profile,
        bom_part_names=printed_bom_names,
    )

    # 4. BOM CSV + Master Project ZIP
    bom_csv_path = _write_project_bom_csv(bom, output_dir / "project_bom.csv")
    master_zip_path = output_dir / "project_manufacturing_pack.zip"
    with zipfile.ZipFile(master_zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for p in (glb_path, step_path, stl_path, bom_csv_path):
            if p.exists():
                zf.write(p, arcname=p.name)
        if dxf_path is not None and dxf_path.exists():
            zf.write(dxf_path, arcname=dxf_path.name)
        for p in print_files.values():
            if p.exists():
                zf.write(p, arcname=p.name)

    def _url(p: Path) -> str:
        return f"/api/files/{design_hash}/{p.name}"

    files: dict[str, str] = {
        "glb": _url(glb_path),
        "step": _url(step_path),
        "stl": _url(stl_path),
        "bom_csv": _url(bom_csv_path),
        "zip": _url(master_zip_path),
    }
    if dxf_path is not None and dxf_path.exists():
        files["dxf"] = _url(dxf_path)
    for k, p in print_files.items():
        files[k] = _url(p)

    total_cost = round(sum(b.total_cost_usd for b in bom), 2)

    return ProjectGenerateResponse(
        family_id=family_id,
        preset_id=preset_id,
        design_hash=design_hash,
        name=name,
        metrics=metrics,
        bom=bom,
        total_bom_cost_usd=total_cost,
        dfm_audit=dfm_audit,
        explode_vectors=explode_vectors,
        files=files,
        notes=notes or [],
    )


class ProjectFamilyPlugin(ABC):
    """Protocol for a RoboForge v9.0 parametric 3D robotics project family."""

    family_id: str
    name: str
    subtitle: str
    icon: str

    @abstractmethod
    def get_presets(self) -> list[dict[str, Any]]:
        ...

    def get_info(self) -> dict[str, Any]:
        return {
            "id": self.family_id,
            "name": self.name,
            "subtitle": self.subtitle,
            "icon": self.icon,
            "presets": self.get_presets(),
        }

    @abstractmethod
    def generate(
        self,
        req: ProjectGenerateRequest,
        material: dict[str, Any],
        printer_profile: dict[str, Any],
        output_dir: Path,
        design_hash: str,
    ) -> ProjectGenerateResponse:
        ...
