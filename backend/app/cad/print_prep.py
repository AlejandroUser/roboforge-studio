"""3D Print-Readiness Engine: Calibration Fit Test Coupon, Auto-Orient, 2D Plate Bin-Packing (.3MF), and PRINT_GUIDE.md."""

from __future__ import annotations

import math
import tempfile
import zipfile
from pathlib import Path
from typing import Any

import numpy as np
import trimesh
from build123d import (
    BuildPart,
    BuildSketch,
    Compound,
    Cylinder,
    Location,
    Locations,
    Part,
    Plane,
    Polygon,
    RectangleRounded,
    RegularPolygon,
    extrude,
    export_stl,
)

from .threemf_exporter import CONTENT_TYPES_XML, RELS_XML, _mesh_to_3mf_object_xml

# Slicer temperature & parameter presets per material family
MATERIAL_SLICER_RECIPES: dict[str, dict[str, Any]] = {
    "pla_plus": {"nozzle_c": 215, "bed_c": 60, "infill_pct": 35, "walls": 4, "flow_cm3_h": 11.5},
    "petg": {"nozzle_c": 245, "bed_c": 80, "infill_pct": 35, "walls": 4, "flow_cm3_h": 10.0},
    "petg_cf": {"nozzle_c": 255, "bed_c": 80, "infill_pct": 40, "walls": 5, "flow_cm3_h": 9.5},
    "abs_asa": {"nozzle_c": 260, "bed_c": 105, "infill_pct": 40, "walls": 5, "flow_cm3_h": 9.5},
    "pa12_cf": {"nozzle_c": 285, "bed_c": 100, "infill_pct": 45, "walls": 5, "flow_cm3_h": 8.0},
    "tpu_95a": {"nozzle_c": 225, "bed_c": 50, "infill_pct": 25, "walls": 3, "flow_cm3_h": 5.5},
}


def build_calibration_fit_test_coupon(req: Any) -> Part:
    """Build a compact 15-min printable tolerance calibration coupon (66 × 34 × 7 mm, flat on Z=0)
    with M3/M4 tolerance step holes (+0.1/+0.2/+0.3 mm), 608 bearing / 25T horn pocket, and 5/6.35 mm shaft bores.
    """
    with BuildSketch(Plane.XY) as sk:
        RectangleRounded(66.0, 34.0, radius=3.5)
    with BuildPart() as bp:
        extrude(sk.sketch.faces()[0], amount=7.0)
    coupon = bp.part

    mech = getattr(req, "mechatronics", None)
    bus = getattr(mech, "bus_architecture", "can_fd_bus") if mech else "can_fd_bus"
    is_msla = (
        getattr(req, "printer_id", "") == "elegoo_mars_5_ultra"
        or getattr(req, "material_id", "") == "resin_abs_like"
        or bus == "esp32_pca9685_5v"
    )
    # Always keep real Ø22.2 mm 608 bearing bore (alongside M3 hex nut pocket in MSLA mode)
    big_bore_d = 22.2

    # Cutters: M3 step holes (3.1, 3.2, 3.3), M4 step holes (4.1, 4.2, 4.3), shafts (5.15, 6.50), main pocket
    holes = [
        # M3 tolerance row (y = +9.5)
        (-24.0, 9.5, 3.1),
        (-15.0, 9.5, 3.2),
        (-6.0, 9.5, 3.3),
        # M4 tolerance row (y = -9.5)
        (-24.0, -9.5, 4.1),
        (-15.0, -9.5, 4.2),
        (-6.0, -9.5, 4.3),
        # Motor shaft test holes (NEMA 17 5mm, NEMA 23 6.35mm)
        (4.0, 8.5, 5.15),
        (4.0, -8.5, 6.50),
        # Main bearing 608 / 25T horn fit bore
        (19.5, 0.0, big_bore_d),
    ]
    for hx, hy, hd in holes:
        cutter = Location((hx, hy, 3.5)) * Cylinder(radius=hd / 2.0, height=12.0)
        try:
            cand = coupon - cutter
            if cand.is_valid and cand.volume > 0:
                coupon = cand
        except Exception:
            pass

    # MSLA mode: add captive hexagonal M3 nut pocket (5.65 mm across-flats, 2.8 mm deep) on top face
    if is_msla:
        hex_r = 5.65 / math.sqrt(3.0)
        try:
            with BuildSketch(Plane.XY) as nut_sk:
                with Locations((-15.0, 0.0)):
                    RegularPolygon(radius=hex_r, side_count=6)
            with BuildPart() as nut_bp:
                extrude(nut_sk.sketch.faces()[0].moved(Location((0.0, 0.0, 4.2))), amount=3.2)
            cand = coupon - nut_bp.part
            if cand.is_valid and cand.volume > 0:
                coupon = cand
        except Exception:
            pass

    return coupon


def _orient_part_flat_on_z0(part: Part | Compound) -> tuple[trimesh.Trimesh, tuple[float, float, float]]:
    """Tessellate a CAD solid, auto-orient its shortest axis to Z (flat for printing), and ground at (0,0,0)."""
    with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as tmp:
        tmp_path = Path(tmp.name)
    try:
        export_stl(part, str(tmp_path), tolerance=0.14, angular_tolerance=0.22)
        loaded = trimesh.load(str(tmp_path), force="mesh")
        mesh = loaded.dump(concatenate=True) if isinstance(loaded, trimesh.Scene) else loaded
        if not isinstance(mesh, trimesh.Trimesh):
            raise ValueError("Expected Trimesh for print prep")
    finally:
        if tmp_path.exists():
            try:
                tmp_path.unlink()
            except OSError:
                pass

    extents = mesh.extents
    min_axis = int(np.argmin(extents))
    # Rotate shortest dimension to Z so part prints flat with minimal supports
    if min_axis == 0:
        rot = trimesh.transformations.rotation_matrix(math.pi / 2.0, [0.0, 1.0, 0.0])
        mesh.apply_transform(rot)
    elif min_axis == 1:
        rot = trimesh.transformations.rotation_matrix(math.pi / 2.0, [1.0, 0.0, 0.0])
        mesh.apply_transform(rot)

    # Align longest XY dimension along X for better shelf bin-packing
    if mesh.extents[1] > mesh.extents[0]:
        rot_z = trimesh.transformations.rotation_matrix(math.pi / 2.0, [0.0, 0.0, 1.0])
        mesh.apply_transform(rot_z)

    # Ground bounding box min corner at (0, 0, 0)
    bmin = mesh.bounds[0]
    mesh.apply_translation([-float(bmin[0]), -float(bmin[1]), -float(bmin[2])])
    size = (float(mesh.extents[0]), float(mesh.extents[1]), float(mesh.extents[2]))
    return mesh, size


def _split_solid_dovetail_x(
    solid: Part | Compound,
    clearance_mm: float = 0.20,
) -> tuple[Part, Part]:
    """Split a long CAD solid along its X midpoint with a real trapezoidal dovetail tenon (A) and
    matching mortise socket (B, +clearance_mm) using exact OpenCASCADE B-Rep booleans.
    """
    bb = solid.bounding_box()
    x0, x1 = float(bb.min.X), float(bb.max.X)
    y0, y1 = float(bb.min.Y), float(bb.max.Y)
    z0, z1 = float(bb.min.Z), float(bb.max.Z)
    sx, sy, sz = x1 - x0, y1 - y0, z1 - z0
    x_mid = 0.5 * (x0 + x1)
    y_mid = 0.5 * (y0 + y1)

    dt_len = min(14.0, max(6.0, sx * 0.055))
    neck_h = min(6.5, max(3.0, sy * 0.16))
    head_h = min(10.5, max(4.8, sy * 0.26))
    c = max(0.05, float(clearance_mm))

    left_pts = [
        (x0 - 5.0, y0 - 5.0),
        (x_mid, y0 - 5.0),
        (x_mid, y_mid - neck_h),
        (x_mid + dt_len, y_mid - head_h),
        (x_mid + dt_len, y_mid + head_h),
        (x_mid, y_mid + neck_h),
        (x_mid, y1 + 5.0),
        (x0 - 5.0, y1 + 5.0),
    ]
    right_pts = [
        (x_mid + c, y0 - 5.0),
        (x1 + 5.0, y0 - 5.0),
        (x1 + 5.0, y1 + 5.0),
        (x_mid + c, y1 + 5.0),
        (x_mid + c, y_mid + neck_h + c),
        (x_mid + dt_len + c, y_mid + head_h + c),
        (x_mid + dt_len + c, y_mid - head_h - c),
        (x_mid + c, y_mid - neck_h - c),
    ]

    with BuildSketch(Plane.XY) as sk_l:
        Polygon(*left_pts, align=None)
    with BuildPart() as bp_l:
        extrude(sk_l.sketch.faces()[0].moved(Location((0.0, 0.0, z0 - 5.0))), amount=sz + 10.0)

    with BuildSketch(Plane.XY) as sk_r:
        Polygon(*right_pts, align=None)
    with BuildPart() as bp_r:
        extrude(sk_r.sketch.faces()[0].moved(Location((0.0, 0.0, z0 - 5.0))), amount=sz + 10.0)

    half_a = solid & bp_l.part
    half_b = solid & bp_r.part
    return half_a, half_b


def _cap_planar_cut_x(half: trimesh.Trimesh, normal_sign: float) -> trimesh.Trimesh:
    """Close an open planar cut loop perpendicular to X without requiring shapely."""
    trimesh.repair.fill_holes(half)
    if half.is_watertight:
        return half
    # Find boundary edges (edges appearing only once) on the cut plane and fan-cap them
    edges = half.edges_sorted
    _, idx, counts = np.unique(edges, axis=0, return_index=True, return_counts=True)
    boundary = edges[idx[counts == 1]]
    if len(boundary) >= 3:
        b_verts = np.unique(boundary.ravel())
        center = half.vertices[b_verts].mean(axis=0)
        c_idx = len(half.vertices)
        new_verts = np.vstack([half.vertices, center.reshape(1, 3)])
        new_faces = []
        for u, v in boundary:
            p0, p1 = new_verts[u], new_verts[v]
            n = np.cross(p1 - p0, center - p0)
            if n[0] * normal_sign >= 0:
                new_faces.append([u, v, c_idx])
            else:
                new_faces.append([v, u, c_idx])
        half = trimesh.Trimesh(vertices=new_verts, faces=np.vstack([half.faces, new_faces]), process=True)
        half.fix_normals()
    return half


def _split_mesh_dovetail_x(
    mesh: trimesh.Trimesh,
    clearance_mm: float = 0.20,
) -> tuple[trimesh.Trimesh, trimesh.Trimesh]:
    """Split a flat-oriented Trimesh along X_mid into two clean watertight halves (part_A & part_B
    offset by clearance_mm) grounded at (0, 0, 0) without requiring shapely.
    """
    m = mesh.copy()
    bmin = m.bounds[0]
    m.apply_translation([-float(bmin[0]), -float(bmin[1]), -float(bmin[2])])
    sx = float(m.extents[0])
    x_mid = sx * 0.5
    c = max(0.05, float(clearance_mm))

    va, fa, _ = trimesh.intersections.slice_faces_plane(
        m.vertices, m.faces, plane_normal=[-1.0, 0.0, 0.0], plane_origin=[x_mid, 0.0, 0.0]
    )
    vb, fb, _ = trimesh.intersections.slice_faces_plane(
        m.vertices, m.faces, plane_normal=[1.0, 0.0, 0.0], plane_origin=[x_mid + c, 0.0, 0.0]
    )
    part_a = _cap_planar_cut_x(trimesh.Trimesh(vertices=va, faces=fa, process=True), normal_sign=1.0)
    part_b = _cap_planar_cut_x(trimesh.Trimesh(vertices=vb, faces=fb, process=True), normal_sign=-1.0)
    for half in (part_a, part_b):
        half.fix_normals()
        hb = half.bounds[0]
        half.apply_translation([-float(hb[0]), -float(hb[1]), -float(hb[2])])
    return part_a, part_b


def _write_multi_mesh_3mf(meshes: list[tuple[str, trimesh.Trimesh]], output_path: Path) -> Path:
    """Write a valid multi-object OPC ZIP .3MF build plate containing all placed part meshes."""
    obj_blocks: list[str] = []
    build_items: list[str] = []
    for idx, (name, mesh) in enumerate(meshes, start=1):
        obj_blocks.append(_mesh_to_3mf_object_xml(idx, name, mesh))
        build_items.append(f'    <item objectid="{idx}" />')

    model_xml = (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<model unit="millimeter" xml:lang="es-ES" '
        'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">\n'
        '  <metadata name="Application">RoboForge Studio Pro v7.0</metadata>\n'
        '  <metadata name="Description">Plato de impresión 3D auto-orientado y empaquetado para Bambu Studio / OrcaSlicer / PrusaSlicer</metadata>\n'
        "  <resources>\n"
        + "\n".join(obj_blocks)
        + "\n  </resources>\n"
        "  <build>\n"
        + "\n".join(build_items)
        + "\n  </build>\n"
        "</model>\n"
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("[Content_Types].xml", CONTENT_TYPES_XML)
        zf.writestr("_rels/.rels", RELS_XML)
        zf.writestr("3D/3dmodel.model", model_xml)
    return output_path


def generate_print_readiness_pack(
    printable_parts: dict[str, Part],
    req: Any,
    material: dict[str, Any],
    printer_profile: dict[str, Any],
    output_dir: Path,
    design_hash: str = "",
) -> tuple[Any, dict[str, Path]]:
    """Auto-orient printable parts flat on Z=0, pack them into 2D build plates (.3MF) with 8 mm gap,
    generate calibration_fit_test.stl and PRINT_GUIDE.md, and return (PrintReadinessSummary, files_map).
    """
    from ..schemas import PrintPlateInfo, PrintReadinessSummary

    output_dir.mkdir(parents=True, exist_ok=True)

    printer_name = str(printer_profile.get("name", "Bambu Lab X1C"))
    bed_x = float(printer_profile.get("bed_x_mm", 256.0))
    bed_y = float(printer_profile.get("bed_y_mm", 256.0))
    bed_z = float(printer_profile.get("bed_z_mm", 256.0))

    mat_id = str(material.get("id", getattr(req, "material_id", "petg_cf")))
    mat_name = str(material.get("name", mat_id))
    mat_category = str(material.get("category", "fdm"))
    density_g_cm3 = float(material.get("density_g_cm3", 1.29))

    is_msla = (
        str(printer_profile.get("process_type", "fdm")) == "msla_resin"
        or mat_category == "msla"
        or mat_id == "resin_abs_like"
    )
    process_type = "msla_resin" if is_msla else "fdm"

    recipe = MATERIAL_SLICER_RECIPES.get(
        mat_id,
        {"nozzle_c": 245, "bed_c": 80, "infill_pct": 40, "walls": 4, "flow_cm3_h": 9.5},
    )
    if is_msla:
        layer_h = 0.05
        wall_loops = 0
        infill_pct = 100
        infill_pattern = "Sólido MSLA"
        nozzle_c = 0
        bed_c = 0
        exposure_s: float | None = 2.3
        bottom_exposure_s: float | None = 28.0
        recommended_tilt_deg: int | None = 35
        flow_cm3_h = 18.0
    else:
        layer_h = 0.20
        wall_loops = int(recipe["walls"])
        infill_pct = int(recipe["infill_pct"])
        infill_pattern = "Gyroid"
        nozzle_c = int(recipe["nozzle_c"])
        bed_c = int(recipe["bed_c"])
        exposure_s = None
        bottom_exposure_s = None
        recommended_tilt_deg = None
        flow_cm3_h = float(recipe["flow_cm3_h"])

    warnings: list[str] = []
    if mat_category == "cnc":
        warnings.append(
            f"Material '{mat_name}' es metálico/CNC: los platos .3MF sirven para prototipado de ajuste en PLA/PETG antes de mecanizar."
        )

    # 1. Orient all printable parts flat on Z=0 and measure extents (with optional v10.0 Auto-Dovetail Split)
    oriented_items: list[tuple[str, trimesh.Trimesh, tuple[float, float, float], float]] = []
    fits_bed = True
    margin = 5.0
    xy_margin = margin
    auto_split = bool(getattr(req, "auto_dovetail_split", False))
    split_parts_count = 0
    split_part_names: list[str] = []

    def _fits(sx_: float, sy_: float, sz_: float) -> bool:
        return (
            ((sx_ <= bed_x - xy_margin and sy_ <= bed_y - xy_margin) or (sy_ <= bed_x - xy_margin and sx_ <= bed_y - xy_margin))
            and sz_ <= bed_z - 4.0
        )

    for part_name, solid in printable_parts.items():
        try:
            mesh, size = _orient_part_flat_on_z0(solid)
            sx, sy, sz = size
            if not _fits(sx, sy, sz) and auto_split and sx > bed_x - 10.0:
                # Split oversized part into two watertight dovetail halves (part_A tenon + part_B mortise +0.20mm)
                halves_ok = False
                try:
                    bb = solid.bounding_box()
                    if float(bb.size.X) >= max(float(bb.size.Y), float(bb.size.Z)) * 0.95:
                        ha_solid, hb_solid = _split_solid_dovetail_x(solid, clearance_mm=0.20)
                        ma, sa = _orient_part_flat_on_z0(ha_solid)
                        mb, sb = _orient_part_flat_on_z0(hb_solid)
                        if ma.is_watertight and mb.is_watertight:
                            halves = [(f"{part_name}_A", ma, sa), (f"{part_name}_B", mb, sb)]
                            halves_ok = True
                except Exception:
                    halves_ok = False

                if not halves_ok:
                    ma, mb = _split_mesh_dovetail_x(mesh, clearance_mm=0.20)
                    sa = (float(ma.extents[0]), float(ma.extents[1]), float(ma.extents[2]))
                    sb = (float(mb.extents[0]), float(mb.extents[1]), float(mb.extents[2]))
                    halves = [(f"{part_name}_A", ma, sa), (f"{part_name}_B", mb, sb)]

                split_parts_count += 1
                split_part_names.append(part_name)
                for h_name, h_mesh, h_size in halves:
                    h_vol = max(0.5, float(abs(h_mesh.volume)) / 1000.0)
                    if not _fits(*h_size):
                        fits_bed = False
                        warnings.append(
                            f"Mitad '{h_name}' ({h_size[0]:.0f}×{h_size[1]:.0f}×{h_size[2]:.0f} mm) aún excede la cama de {printer_name}."
                        )
                    oriented_items.append((h_name, h_mesh, h_size, h_vol))
                continue

            vol_cm3 = max(0.5, float(abs(mesh.volume)) / 1000.0)
            if not _fits(sx, sy, sz):
                fits_bed = False
                warnings.append(
                    f"Pieza '{part_name}' ({sx:.0f}×{sy:.0f}×{sz:.0f} mm) excede la cama de {printer_name} "
                    f"({bed_x:.0f}×{bed_y:.0f}×{bed_z:.0f} mm): activa Auto-Split Cola de Milano o usa impresora mayor."
                )
            oriented_items.append((part_name, mesh, size, vol_cm3))
        except Exception:
            continue

    # Sort largest footprint first for clean 2D shelf bin-packing
    oriented_items.sort(key=lambda item: item[2][0] * item[2][1], reverse=True)

    # 2. 2D Shelf Bin-Packing into Build Plates (8 mm gap between parts, unified 5 mm bed margin)
    gap = 8.0
    usable_x = max(80.0, bed_x - 2.0 * margin)
    usable_y = max(80.0, bed_y - 2.0 * margin)

    plates_meshes: list[list[tuple[str, trimesh.Trimesh, float]]] = []
    current_plate: list[tuple[str, trimesh.Trimesh, float]] = []
    cursor_x = margin
    cursor_y = margin
    row_h = 0.0

    for part_name, mesh, (sx, sy, sz), vol_cm3 in oriented_items:
        placed_mesh = mesh.copy()
        px, py = sx, sy
        # Try rotating 90° if it fits better on current shelf
        if cursor_x + px > bed_x - margin and (cursor_x + py <= bed_x - margin and cursor_y + px <= bed_y - margin):
            rot90 = trimesh.transformations.rotation_matrix(math.pi / 2.0, [0.0, 0.0, 1.0])
            placed_mesh.apply_transform(rot90)
            bmin = placed_mesh.bounds[0]
            placed_mesh.apply_translation([-float(bmin[0]), -float(bmin[1]), -float(bmin[2])])
            px, py = float(placed_mesh.extents[0]), float(placed_mesh.extents[1])

        if cursor_x + px > bed_x - margin and current_plate:
            # Move to next row
            cursor_x = margin
            cursor_y += row_h + gap
            row_h = 0.0

        if cursor_y + py > bed_y - margin and current_plate:
            # Start a new build plate
            plates_meshes.append(current_plate)
            current_plate = []
            cursor_x = margin
            cursor_y = margin
            row_h = 0.0

        placed_mesh.apply_translation([cursor_x, cursor_y, 0.0])
        current_plate.append((part_name, placed_mesh, vol_cm3))
        cursor_x += px + gap
        row_h = max(row_h, py)

    if current_plate:
        plates_meshes.append(current_plate)

    # Effective plastic fill fraction (walls + gyroid infill)
    eff_fill = min(1.0, 0.42 + 0.58 * (infill_pct / 100.0))

    exported_files: dict[str, Path] = {}
    plate_infos: list[PrintPlateInfo] = []
    total_filament_g = 0.0
    total_hours = 0.0
    individual_stl_entries: list[tuple[str, Path, tuple[float, float, float]]] = []

    # 2b. Export individual flat-oriented .STL per printable part
    for part_name, mesh, size, _ in oriented_items:
        safe_part = "".join(c if (c.isalnum() or c == "_") else "_" for c in part_name).strip("_") or "part"
        part_stl_name = f"part_{safe_part}.stl"
        part_stl_path = output_dir / part_stl_name
        mesh.export(str(part_stl_path), file_type="stl")
        exported_files[f"part_stl_{safe_part}"] = part_stl_path
        individual_stl_entries.append((part_name, part_stl_path, size))

    for p_idx, p_items in enumerate(plates_meshes, start=1):
        plate_filename = f"build_plate_{p_idx}.3mf"
        plate_path = output_dir / plate_filename
        _write_multi_mesh_3mf([(name, m) for (name, m, _) in p_items], plate_path)
        exported_files[f"build_plate_{p_idx}_3mf"] = plate_path

        p_vol_cm3 = sum(v for (_, _, v) in p_items) * eff_fill
        p_grams = round(p_vol_cm3 * density_g_cm3, 1)
        p_hours = round(max(0.35, p_vol_cm3 / max(flow_cm3_h, 2.0)), 2)
        total_filament_g += p_grams
        total_hours += p_hours

        plate_infos.append(
            PrintPlateInfo(
                plate_index=p_idx,
                filename=plate_filename,
                url=f"/api/files/{design_hash}/{plate_filename}" if design_hash else plate_filename,
                parts=[name for (name, _, _) in p_items],
                estimated_mass_g=p_grams,
                estimated_hours=p_hours,
            )
        )

    # 3. Export 15-min Calibration Fit Test Coupon STL
    coupon_part = build_calibration_fit_test_coupon(req)
    cal_path = output_dir / "calibration_fit_test.stl"
    if not (cal_path.exists() and cal_path.stat().st_size > 0):
        export_stl(coupon_part, str(cal_path), tolerance=0.06, angular_tolerance=0.12)
    exported_files["calibration_stl"] = cal_path

    # 4. Write comprehensive Spanish PRINT_GUIDE.md (MSLA SatelLite/Chitubox vs FDM Bambu/OrcaSlicer)
    plate_md_rows = "\n".join(
        f"| Plato {pi.plate_index} | `{pi.filename}` | {', '.join(pi.parts)} | {pi.estimated_mass_g:.1f} g | {pi.estimated_hours:.2f} h |"
        for pi in plate_infos
    )
    warn_md = (
        "\n".join(f"- ⚠️ {w}" for w in warnings)
        if warnings
        else "- ✅ Todas las piezas caben limpiamente en la plataforma de construcción sin necesidad de corte."
    )
    if is_msla:
        guide_md = f"""# Guía de Impresión Resina MSLA y Ensamblaje — RoboForge Studio Pro v7.1

## 1. Perfil de Impresora MSLA y Resina
- **Impresora objetivo:** {printer_name} ({bed_x:.2f} × {bed_y:.2f} × {bed_z:.1f} mm)
- **Resina recomendada:** {mat_name} (`{mat_id}`, ABS-Like resistente a impactos)
- **Estado de plataforma:** {"✅ 0 piezas fuera de placa — 100% imprimible" if fits_bed else "⚠️ Requiere dividir piezas largas"}
- **Platos totales:** {len(plate_infos)} | **Resina estimada:** {total_filament_g:.1f} g | **Tiempo total:** {total_hours:.2f} h

## 2. Receta Recomendada (Elegoo SatelLite / Chitubox / Lychee)
| Parámetro MSLA | Valor Recomendado |
| :--- | :--- |
| Altura de capa | `{layer_h:.2f} mm` (50 µm) |
| Exposición normal | `2.2 – 2.5 s` (base `{exposure_s:.1f} s` en Mars 5 Ultra) |
| Capas base (Bottom Layers) | `4 capas × {bottom_exposure_s:.0f} s` |
| Inclinación recomendada | `{recommended_tilt_deg}°` (30–45° para reducir fuerza de succión en film PFA/ACF) |
| Soportes | Soportes medios automáticos + anclajes pesados en aristas inferiores |
| Post-proceso | Lavado IPA 99% (`5 min`) + secado completo + curado UV (`3 – 5 min`) |

## 3. Paso 0 (OBLIGATORIO): Calibración en 15 Minutos
1. Imprime primero **`calibration_fit_test.stl`** plano o con soportes ligeros usando esta receta.
2. Verifica el ajuste de tornillos **M3** (`+0.1 / +0.2 / +0.3 mm`), el **bolsillo hexagonal cautivo para tuerca M3 (`5.65 mm`)** y el encaje del horn **25T** / servo.
3. Si la tuerca M3 entra muy justa, reduce la exposición normal `-0.2 s` en Elegoo SatelLite antes de lanzar el Plato 1.

## 4. Distribución de Platos MSLA (`.3MF`)
| Plato | Archivo | Piezas Incluidas | Resina Est. | Tiempo Est. |
| :--- | :--- | :--- | ---: | ---: |
{plate_md_rows}

## 5. Alertas DFM y Notas de Resina
{warn_md}

## 6. Checklist Rápido de Ensamblaje (Resina ABS-Like)
- [ ] Retirar soportes ANTES del curado UV final (sumergir 30 s en agua tibia facilita el desprendimiento).
- [ ] **Usar tornillos M3 con tuercas cautivas ISO 4032** (nunca tornillos autorroscantes directos en resina curada).
- [ ] Verificar el giro libre de los horns 25T antes de energizar la fuente `5V 5A`.
- [ ] Cargar `mars_mini_esp32_wifi.ino` en el ESP32 y conectarse a la red Wi-Fi `RoboForge-MarsMini`.
"""
    else:
        guide_md = f"""# Guía de Impresión 3D y Ensamblaje — RoboForge Studio Pro v7.1

## 1. Perfil de Impresora y Material
- **Impresora objetivo:** {printer_name} ({bed_x:.0f} × {bed_y:.0f} × {bed_z:.0f} mm)
- **Material:** {mat_name} (`{mat_id}`)
- **Estado de cama:** {"✅ Todas las piezas caben en cama" if fits_bed else "⚠️ Requiere dividir piezas largas"}
- **Platos totales:** {len(plate_infos)} | **Filamento estimado:** {total_filament_g:.1f} g | **Tiempo total:** {total_hours:.2f} h

## 2. Receta Recomendada (Bambu Studio / OrcaSlicer / PrusaSlicer)
| Parámetro | Valor Recomendado |
| :--- | :--- |
| Altura de capa | `{layer_h:.2f} mm` |
| Paredes (Wall Loops) | `{wall_loops}` (boquilla 0.4–0.6 mm) |
| Relleno estructural | `{infill_pct}%` patrón **Gyroid** |
| Temperatura boquilla | `{nozzle_c} °C` |
| Temperatura cama | `{bed_c} °C` |
| Soportes | Mínimos (piezas auto-orientadas planas sobre `Z = 0`) |

## 3. Paso 0 (OBLIGATORIO): Calibración en 15 Minutos
1. Imprime primero **`calibration_fit_test.stl`** con exactamente la receta de arriba.
2. Verifica el ajuste de tornillos **M3** (`+0.1 / +0.2 / +0.3 mm`), **M4**, eje de motor (`5.0 / 6.35 mm`) y rodamiento `608` / horn `25T`.
3. Si entra muy apretado, ajusta *XY Hole Compensation* en OrcaSlicer (`+0.05` a `+0.10 mm`) antes de lanzar el Plato 1.

## 4. Distribución de Platos de Impresión (`.3MF`)
| Plato | Archivo | Piezas Incluidas | Filamento | Tiempo Est. |
| :--- | :--- | :--- | ---: | ---: |
{plate_md_rows}

## 5. Alertas DFM y Notas de Impresión
{warn_md}

## 6. Checklist Rápido de Ensamblaje
- [ ] Limpiar rebabas/brim de los alojamientos de rodamientos y juntas.
- [ ] Instalar insertos térmicos de latón (si aplica) con soldador a `220–240 °C`.
- [ ] Verificar el giro suave de cada junta antes de apretar los tornillos ISO 4762.
- [ ] Consultar `arm_circuit_schematic.svg` y `arm_master_bom.csv` para el cableado final.
"""
    if split_parts_count > 0:
        dovetail_md = (
            f"\n## 7. Ensamblaje de Piezas Divididas con Cola de Milano (Auto-Split v10.0)\n"
            f"- **Piezas divididas automáticamente ({split_parts_count}):** {', '.join(f'`{n}`' for n in split_part_names)}\n"
            f"- Cada pieza larga fue dividida en **`*_A.stl`** (espiga trapezoidal macho) y **`*_B.stl`** (caja hembra con holgura `+0.20 mm`).\n"
            f"- Desliza la espiga verticalmente en la caja, aplica cianoacrilato / epoxi estructural 2K y prensa 15 min antes de montar en el brazo.\n"
        )
        guide_md = guide_md + dovetail_md
    guide_path = output_dir / "PRINT_GUIDE.md"
    guide_path.write_text(guide_md, encoding="utf-8")
    exported_files["print_guide_md"] = guide_path

    # 5. Build cleanly organized 3D-Printing Parts Pack ZIP (separated from Manufacturing/Engineering Kit)
    parts_table_rows = "\n".join(
        f"| `{fp.name}` | {sx:.1f} × {sy:.1f} × {sz:.1f} mm | Plana sobre `Z = 0` |"
        for _, fp, (sx, sy, sz) in individual_stl_entries
    )
    parts_readme = f"""# Paquete de Piezas para Impresión 3D — RoboForge Studio Pro v7.0

## Estructura de Carpetas
- **`01_CALIBRACION_15MIN/`** — Imprime primero `calibration_fit_test.stl` (66 × 34 × 7 mm) para validar tornillos M3/M4 y ejes antes de imprimir el brazo.
- **`02_PIEZAS_STL_SEPARADAS/`** — Las {len(individual_stl_entries)} piezas imprimibles individuales en `.STL` ya orientadas planas sobre `Z = 0` (listo para **SatelLite, Chitubox, Lychee, Cura, Bambu Studio, PrusaSlicer**).
- **`03_PLATOS_3MF_SLICER/`** — Platos multi-objeto `.3MF` empaquetados para cama `{printer_name}` ({bed_x:.0f} × {bed_y:.0f} mm) + `PRINT_GUIDE.md`.

## Piezas Incluidas ({len(individual_stl_entries)} archivos .STL)
| Archivo | Dimensiones (X × Y × Z) | Orientación |
| :--- | :---: | :--- |
{parts_table_rows}
"""
    stl_parts_zip_path = output_dir / "arm_stl_parts_pack.zip"
    with zipfile.ZipFile(stl_parts_zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf_stl:
        zf_stl.writestr("00_ORDEN_DE_IMPRESION.md", parts_readme)
        if cal_path.exists():
            zf_stl.write(cal_path, arcname=f"01_CALIBRACION_15MIN/{cal_path.name}")
        for _, fp, _ in individual_stl_entries:
            if fp.exists():
                zf_stl.write(fp, arcname=f"02_PIEZAS_STL_SEPARADAS/{fp.name}")
        for pi in plate_infos:
            p_fp = output_dir / pi.filename
            if p_fp.exists():
                zf_stl.write(p_fp, arcname=f"03_PLATOS_3MF_SLICER/{pi.filename}")
        zf_stl.write(guide_path, arcname="03_PLATOS_3MF_SLICER/PRINT_GUIDE.md")
    exported_files["stl_parts_zip"] = stl_parts_zip_path

    summary = PrintReadinessSummary(
        fits_bed=fits_bed,
        printer_name=printer_name,
        bed_mm=[round(bed_x, 1), round(bed_y, 1), round(bed_z, 1)],
        total_plates=len(plate_infos),
        total_filament_g=round(total_filament_g, 1),
        estimated_print_hours=round(total_hours, 2),
        layer_height_mm=layer_h,
        wall_loops=wall_loops,
        infill_pct=infill_pct,
        infill_pattern=infill_pattern,
        nozzle_temp_c=nozzle_c,
        bed_temp_c=bed_c,
        process_type=process_type,  # type: ignore[arg-type]
        exposure_s=exposure_s,
        bottom_exposure_s=bottom_exposure_s,
        recommended_tilt_deg=recommended_tilt_deg,
        split_parts_count=split_parts_count,
        plates=plate_infos,
        warnings=warnings,
    )
    return summary, exported_files
