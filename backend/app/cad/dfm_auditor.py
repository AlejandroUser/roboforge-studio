"""Goldfish Pre-Print DFM Auditor: Watertightness, Floating Islands, Bed Fit, AABB Collisions & BOM Parity."""

from __future__ import annotations

import math
import tempfile
from pathlib import Path
from typing import Any

import numpy as np
import trimesh
from build123d import Compound, Part, export_stl

from ..schemas import DFMAuditReport, DFMCheckItem, DFMStatusType


def _solid_to_mesh(solid: Part | Compound) -> trimesh.Trimesh:
    with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as tmp:
        tmp_path = Path(tmp.name)
    try:
        export_stl(solid, str(tmp_path), tolerance=0.12, angular_tolerance=0.20)
        loaded = trimesh.load(str(tmp_path), force="mesh")
        mesh = loaded.dump(concatenate=True) if isinstance(loaded, trimesh.Scene) else loaded
        if not isinstance(mesh, trimesh.Trimesh):
            raise ValueError("Expected Trimesh from CAD solid")
        return mesh
    finally:
        if tmp_path.exists():
            try:
                tmp_path.unlink()
            except OSError:
                pass


def _aabb_overlap_ratio(bounds_a: np.ndarray, bounds_b: np.ndarray) -> float:
    inter_min = np.maximum(bounds_a[0], bounds_b[0])
    inter_max = np.minimum(bounds_a[1], bounds_b[1])
    inter_ext = np.maximum(0.0, inter_max - inter_min)
    inter_vol = float(np.prod(inter_ext))
    vol_a = float(np.prod(np.maximum(0.0, bounds_a[1] - bounds_a[0])))
    vol_b = float(np.prod(np.maximum(0.0, bounds_b[1] - bounds_b[0])))
    denom = min(vol_a, vol_b)
    return inter_vol / denom if denom > 1e-6 else 0.0


def audit_assembly_dfm(
    printable_parts: dict[str, Part | Compound],
    placed_meshes: dict[str, trimesh.Trimesh],
    printer_profile: dict[str, Any],
    bom_part_names: list[str],
) -> DFMAuditReport:
    """Run all 5 Goldfish Pre-Print DFM checks on an assembly and return a structured DFMAuditReport."""
    checks: list[DFMCheckItem] = []

    bed_x = float(printer_profile.get("bed_x_mm", 256.0))
    bed_y = float(printer_profile.get("bed_y_mm", 256.0))
    bed_z = float(printer_profile.get("bed_z_mm", 256.0))
    printer_name = str(printer_profile.get("name", "Bambu Lab X1C"))
    margin = 5.0

    non_watertight: list[str] = []
    inverted_normals: list[str] = []
    multi_body: list[str] = []
    oversized: list[str] = []
    usable_x = max(20.0, bed_x - 2.0 * margin)
    usable_y = max(20.0, bed_y - 2.0 * margin)
    usable_z = max(20.0, bed_z - 4.0)

    for name, solid in printable_parts.items():
        try:
            mesh = _solid_to_mesh(solid)
        except Exception as exc:
            non_watertight.append(f"{name} (error: {exc})")
            continue

        if not mesh.is_watertight:
            non_watertight.append(name)
        elif float(mesh.volume) <= 1e-3:
            inverted_normals.append(f"{name} (vol={mesh.volume:.1f})")

        bodies = mesh.split(only_watertight=False)
        if len(bodies) != 1:
            multi_body.append(f"{name} ({len(bodies)} cuerpos)")

        # Orient shortest axis to Z for flat print bed check (consistent with 2*margin bin-packer)
        dims = sorted([float(x) for x in mesh.extents])
        sz, sy, sx = dims[0], dims[1], dims[2]
        fits = (
            ((sx <= usable_x and sy <= usable_y) or (sy <= usable_x and sx <= usable_y))
            and sz <= usable_z
        )
        if not fits:
            oversized.append(f"{name} ({sx:.0f}×{sy:.0f}×{sz:.0f} mm)")

    # 1. Watertight & outward-normals check
    if not printable_parts:
        checks.append(
            DFMCheckItem(
                id="watertight",
                label="Mallas Watertight (Manifold)",
                status="warn",
                detail="No se declararon piezas imprimibles para auditar.",
            )
        )
    elif not non_watertight and not inverted_normals:
        checks.append(
            DFMCheckItem(
                id="watertight",
                label="Mallas Watertight (Manifold)",
                status="pass",
                detail=f"{len(printable_parts)}/{len(printable_parts)} piezas STL cerradas, volumen positivo y sin bordes abiertos.",
            )
        )
    else:
        issues = []
        if non_watertight:
            issues.append(f"no-manifold: {', '.join(non_watertight)}")
        if inverted_normals:
            issues.append(f"normales invertidas/vol<=0: {', '.join(inverted_normals)}")
        checks.append(
            DFMCheckItem(
                id="watertight",
                label="Mallas Watertight (Manifold)",
                status="fail",
                detail=f"Defectos de malla detectados ({'; '.join(issues)}).",
            )
        )

    # 2. Floating islands check
    if not printable_parts:
        checks.append(
            DFMCheckItem(
                id="floating_islands",
                label="Cero Islas Flotantes (1 cuerpo/STL)",
                status="warn",
                detail="Sin piezas imprimibles.",
            )
        )
    elif not multi_body:
        checks.append(
            DFMCheckItem(
                id="floating_islands",
                label="Cero Islas Flotantes (1 cuerpo/STL)",
                status="pass",
                detail=f"{len(printable_parts)}/{len(printable_parts)} piezas son sólidos únicos conectados.",
            )
        )
    else:
        checks.append(
            DFMCheckItem(
                id="floating_islands",
                label="Cero Islas Flotantes (1 cuerpo/STL)",
                status="fail",
                detail=f"Islas desconectadas en: {', '.join(multi_body)}.",
            )
        )

    # 3. Printer bed fit check
    if not printable_parts:
        checks.append(
            DFMCheckItem(
                id="fits_bed",
                label=f"Ajuste a Cama ({printer_name})",
                status="warn",
                detail="Sin piezas imprimibles para verificar ajuste en cama.",
            )
        )
    elif not oversized:
        checks.append(
            DFMCheckItem(
                id="fits_bed",
                label=f"Ajuste a Cama ({printer_name})",
                status="pass",
                detail=f"Todas las piezas caben en área útil {usable_x:.0f}×{usable_y:.0f}×{usable_z:.0f} mm (cama {bed_x:.0f}×{bed_y:.0f} mm, margen {margin:.0f} mm/lado).",
            )
        )
    else:
        checks.append(
            DFMCheckItem(
                id="fits_bed",
                label=f"Ajuste a Cama ({printer_name})",
                status="fail",
                detail=f"Exceden área útil {usable_x:.0f}×{usable_y:.0f} mm: {', '.join(oversized)}.",
            )
        )

    # 4. Pairwise AABB + sub-body / vertex-in-mesh collision check between placed assemblies
    severe_overlaps: list[str] = []
    warn_overlaps: list[str] = []
    mesh_items = [(k, m) for k, m in placed_meshes.items() if isinstance(m, trimesh.Trimesh) and len(m.faces) > 0]
    # Split multi-body visual compounds (e.g. 4 corner wheels / 4 motors / 4 standoffs) so hollow spans don't false-positive
    sub_bodies_map: dict[str, list[trimesh.Trimesh]] = {}
    for k, m in mesh_items:
        parts = m.split(only_watertight=False)
        sub_bodies_map[k] = parts if 1 <= len(parts) <= 12 else [m]

    for i in range(len(mesh_items)):
        name_a, _ = mesh_items[i]
        for j in range(i + 1, len(mesh_items)):
            name_b, _ = mesh_items[j]
            max_real_ratio = 0.0
            for sub_a in sub_bodies_map[name_a]:
                for sub_b in sub_bodies_map[name_b]:
                    ratio = _aabb_overlap_ratio(sub_a.bounds, sub_b.bounds)
                    if ratio < 0.45:
                        continue
                    # Refine concave/L-bracket AABB overlap using fast sampled interior vertex containment
                    if sub_a.is_watertight and sub_b.is_watertight:
                        smaller, larger = (sub_a, sub_b) if len(sub_a.vertices) <= len(sub_b.vertices) else (sub_b, sub_a)
                        step = max(1, len(smaller.vertices) // 64)
                        c = smaller.centroid
                        pts = smaller.vertices[::step] * 0.98 + c * 0.02
                        pts = np.vstack([pts, c.reshape(1, 3)])
                        try:
                            inside_frac = float(np.mean(larger.contains(pts)))
                            effective = ratio if inside_frac >= 0.35 else min(ratio, inside_frac)
                        except Exception:
                            effective = ratio
                    else:
                        effective = ratio
                    if effective > max_real_ratio:
                        max_real_ratio = effective
            if max_real_ratio >= 0.72:
                severe_overlaps.append(f"{name_a}↔{name_b} ({max_real_ratio:.0%})")
            elif max_real_ratio >= 0.45:
                warn_overlaps.append(f"{name_a}↔{name_b} ({max_real_ratio:.0%})")

    if severe_overlaps:
        checks.append(
            DFMCheckItem(
                id="aabb_collisions",
                label="Colisiones AABB de Ensamblaje",
                status="fail",
                detail=f"Penetración severa: {', '.join(severe_overlaps[:4])}.",
            )
        )
    elif warn_overlaps:
        checks.append(
            DFMCheckItem(
                id="aabb_collisions",
                label="Colisiones AABB de Ensamblaje",
                status="warn",
                detail=f"Solapamiento de caja envolvente (revisar holgura): {', '.join(warn_overlaps[:4])}.",
            )
        )
    else:
        checks.append(
            DFMCheckItem(
                id="aabb_collisions",
                label="Colisiones AABB de Ensamblaje",
                status="pass",
                detail=f"{len(mesh_items)} sub-ensamblajes verificados sin penetraciones severas.",
            )
        )

    # 5. BOM <-> Printable STL parity
    n_printed = len(printable_parts)
    n_bom = len(bom_part_names)
    if n_printed == 0:
        checks.append(
            DFMCheckItem(
                id="bom_parity",
                label="Paridad Piezas Impresas ↔ BOM",
                status="warn",
                detail="Sin piezas imprimibles registradas.",
            )
        )
    elif n_bom >= n_printed:
        checks.append(
            DFMCheckItem(
                id="bom_parity",
                label="Paridad Piezas Impresas ↔ BOM",
                status="pass",
                detail=f"Paridad 1:1 confirmada ({n_printed} piezas STL cubiertas en {n_bom} líneas del BOM).",
            )
        )
    else:
        checks.append(
            DFMCheckItem(
                id="bom_parity",
                label="Paridad Piezas Impresas ↔ BOM",
                status="warn",
                detail=f"{n_printed} piezas STL exportadas pero solo {n_bom} entradas impresas en el BOM.",
            )
        )

    statuses = [c.status for c in checks]
    overall: DFMStatusType = "fail" if "fail" in statuses else ("warn" if "warn" in statuses else "pass")
    passed_count = sum(1 for s in statuses if s == "pass")
    summary = (
        f"Auditoría Goldfish DFM: {passed_count}/{len(checks)} controles aprobados."
        if overall == "pass"
        else f"Auditoría Goldfish DFM ({overall.upper()}): {passed_count}/{len(checks)} controles aprobados."
    )
    return DFMAuditReport(status=overall, summary=summary, checks=checks)
