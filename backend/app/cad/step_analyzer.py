"""STEP (.step / .stp) geometry analyzer using build123d & OCCT for reference reverse-engineering."""

from __future__ import annotations

import hashlib
import math
import tempfile
from pathlib import Path
from typing import Any

import trimesh
from build123d import GeomType, export_stl, import_step
from OCP.BRepAdaptor import BRepAdaptor_Surface

from ..schemas import (
    DetectedHole,
    EndInterfaceParams,
    LinkDesignRequest,
    StepReferenceAnalysis,
)
from .exporter import EXPORTS_ROOT


def _clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))


def _project_vec(vec_xyz: tuple[float, float, float], axis_idx: int) -> float:
    return vec_xyz[axis_idx]


def _classify_bolt_size(diameter_mm: float) -> tuple[str, bool]:
    """Infer nominal bolt size (M3/M4/M5) and whether it is a heat-set insert pilot hole."""
    if diameter_mm <= 3.7:
        return "M3", False
    if diameter_mm <= 4.35:
        return "M3", True
    if diameter_mm <= 4.9:
        return "M4", False
    if diameter_mm <= 5.95:
        return "M4", True
    if diameter_mm <= 6.2:
        return "M5", False
    return "M5", True


def analyze_step_file(step_path: Path, filename: str) -> StepReferenceAnalysis:
    """Import a .STEP solid, extract envelope, volume, bores/bolts, and build a suggested request."""
    shape = import_step(str(step_path))
    bb = shape.bounding_box()

    raw_sizes = [float(bb.size.X), float(bb.size.Y), float(bb.size.Z)]
    if max(raw_sizes) <= 1e-3:
        raise ValueError("El archivo .STEP no contiene geometría sólida válida.")

    # Sort axes by descending dimension: length_axis >= width_axis >= thickness_axis
    axis_order = sorted(range(3), key=lambda i: raw_sizes[i], reverse=True)
    l_idx, w_idx, t_idx = axis_order[0], axis_order[1], axis_order[2]

    bbox_length = round(raw_sizes[l_idx], 2)
    bbox_width = round(raw_sizes[w_idx], 2)
    bbox_thickness = round(raw_sizes[t_idx], 2)

    raw_volume_mm3 = float(shape.volume) if getattr(shape, "volume", None) else 0.0
    volume_cm3 = round(max(0.01, raw_volume_mm3 / 1000.0), 2)

    # Thickness axis unit vector
    t_dir = [0.0, 0.0, 0.0]
    t_dir[t_idx] = 1.0

    inward_cyls: list[dict[str, float]] = []
    outward_cyls: list[dict[str, float]] = []

    for face in shape.faces():
        if face.geom_type != GeomType.CYLINDER:
            continue
        try:
            surf = BRepAdaptor_Surface(face.wrapped)
            cyl = surf.Cylinder()
            ax_dir = cyl.Axis().Direction()
            # Keep cylinders whose axis is aligned with the part's thickness axis
            dot_t = abs(ax_dir.X() * t_dir[0] + ax_dir.Y() * t_dir[1] + ax_dir.Z() * t_dir[2])
            if dot_t < 0.85:
                continue

            radius = float(cyl.Radius())
            diameter = 2.0 * radius
            if diameter < 2.2 or diameter > 140.0:
                continue

            loc = cyl.Location()
            loc_xyz = (float(loc.X()), float(loc.Y()), float(loc.Z()))
            pt = face.center()
            n = face.normal_at(pt)
            to_ax = (loc_xyz[0] - float(pt.X), loc_xyz[1] - float(pt.Y), loc_xyz[2] - float(pt.Z))
            inward = (float(n.X) * to_ax[0] + float(n.Y) * to_ax[1] + float(n.Z) * to_ax[2]) > 0.0

            cx = _project_vec(loc_xyz, l_idx)
            cy = _project_vec(loc_xyz, w_idx)
            rec = {"x": cx, "y": cy, "d": diameter}
            if inward:
                inward_cyls.append(rec)
            else:
                outward_cyls.append(rec)
        except Exception:
            continue

    # Deduplicate inward cylindrical holes
    unique_holes: list[dict[str, float]] = []
    for rec in sorted(inward_cyls, key=lambda r: r["d"], reverse=True):
        is_dup = False
        for existing in unique_holes:
            dist = math.hypot(rec["x"] - existing["x"], rec["y"] - existing["y"])
            if dist < 1.2 and abs(rec["d"] - existing["d"]) < 0.8:
                is_dup = True
                break
        if not is_dup:
            unique_holes.append(rec)

    detected_holes = [
        DetectedHole(
            center_x_mm=round(h["x"], 2),
            center_y_mm=round(h["y"], 2),
            diameter_mm=round(h["d"], 2),
        )
        for h in unique_holes
    ]

    notes: list[str] = [
        f"Envolvente detectada: {bbox_length:.1f} × {bbox_width:.1f} × {bbox_thickness:.1f} mm "
        f"(volumen original: {volume_cm3:.2f} cm³).",
    ]

    # Identify primary end bores (diameter >= 7.5 mm) vs small bolt holes (< 7.5 mm)
    major_bores = [h for h in unique_holes if h["d"] >= 7.5]
    small_bolts = [h for h in unique_holes if 2.4 <= h["d"] < 7.5]

    def _find_coaxial_outer_diameter(bore: dict[str, float], fallback: float) -> float:
        best_d = 0.0
        for oc in outward_cyls:
            if math.hypot(oc["x"] - bore["x"], oc["y"] - bore["y"]) < 2.0 and oc["d"] > bore["d"] + 3.0:
                best_d = max(best_d, oc["d"])
        return best_d if best_d > 0.0 else fallback

    def _detect_bolt_pattern(bore: dict[str, float], outer_d: float) -> tuple[int, float, str, bool]:
        cluster = []
        for b in small_bolts:
            dist = math.hypot(b["x"] - bore["x"], b["y"] - bore["y"])
            if (bore["d"] / 2.0) < dist < (outer_d / 2.0 + 4.0):
                cluster.append((dist, b["d"]))
        if not cluster:
            return 0, _clamp(bore["d"] + 6.0, 10.0, 105.0), "M3", False
        avg_r = sum(c[0] for c in cluster) / len(cluster)
        avg_d = sum(c[1] for c in cluster) / len(cluster)
        bolt_size, heat_inserts = _classify_bolt_size(avg_d)
        return min(8, len(cluster)), _clamp(round(2.0 * avg_r, 1), 10.0, 105.0), bolt_size, heat_inserts

    if len(major_bores) >= 2:
        # Pick the pair of major bores with maximum separation along length
        best_pair: tuple[dict[str, float], dict[str, float]] | None = None
        best_dist = 0.0
        for i in range(len(major_bores)):
            for j in range(i + 1, len(major_bores)):
                d_ij = math.hypot(
                    major_bores[j]["x"] - major_bores[i]["x"],
                    major_bores[j]["y"] - major_bores[i]["y"],
                )
                if d_ij > best_dist:
                    best_dist = d_ij
                    best_pair = (major_bores[i], major_bores[j])

        assert best_pair is not None
        h1, h2 = best_pair
        # Assign larger outer/bore end as Root (End A)
        od1 = _find_coaxial_outer_diameter(h1, max(h1["d"] + 12.0, bbox_width * 0.92))
        od2 = _find_coaxial_outer_diameter(h2, max(h2["d"] + 10.0, bbox_width * 0.72))
        if (od2, h2["d"]) > (od1, h1["d"]):
            root_h, tip_h = h2, h1
            root_od, tip_od = od2, od1
        else:
            root_h, tip_h = h1, h2
            root_od, tip_od = od1, od2

        est_length = _clamp(round(best_dist, 1), 60.0, 450.0)
        r_bolts, r_pcd, r_size, r_heat = _detect_bolt_pattern(root_h, root_od)
        t_bolts, t_pcd, t_size, t_heat = _detect_bolt_pattern(tip_h, tip_od)

        root_end = EndInterfaceParams(
            preset="custom",
            outer_diameter_mm=_clamp(round(root_od, 1), 14.0, 120.0),
            bore_diameter_mm=_clamp(round(root_h["d"], 1), 3.0, 90.0),
            bolt_count=r_bolts,
            bolt_pcd_mm=r_pcd,
            bolt_size=r_size,  # type: ignore[arg-type]
            use_heat_inserts=r_heat,
        )
        tip_end = EndInterfaceParams(
            preset="custom",
            outer_diameter_mm=_clamp(round(tip_od, 1), 14.0, 120.0),
            bore_diameter_mm=_clamp(round(tip_h["d"], 1), 3.0, 90.0),
            bolt_count=t_bolts,
            bolt_pcd_mm=t_pcd,
            bolt_size=t_size,  # type: ignore[arg-type]
            use_heat_inserts=t_heat,
        )

        notes.append(
            f"Detectados {len(major_bores)} barrenos principales (distancia entre centros: {best_dist:.1f} mm) "
            f"y {len(small_bolts)} taladros de fijación."
        )
        notes.append(
            f"Extremo A estimado: Ø{root_end.bore_diameter_mm:.1f} mm (jefe Ø{root_end.outer_diameter_mm:.1f} mm, "
            f"{r_bolts} pernos) | Extremo B: Ø{tip_end.bore_diameter_mm:.1f} mm (jefe Ø{tip_end.outer_diameter_mm:.1f} mm)."
        )
    else:
        est_length = _clamp(round(max(60.0, bbox_length - bbox_width * 0.8), 1), 60.0, 450.0)
        root_od = _clamp(round(bbox_width, 1), 14.0, 120.0)
        tip_od = _clamp(round(max(16.0, bbox_width * 0.72), 1), 14.0, 120.0)
        root_bore = _clamp(round(min(22.0, root_od * 0.5), 1), 3.0, 90.0)
        tip_bore = _clamp(round(min(22.0, tip_od * 0.55), 1), 3.0, 90.0)

        root_end = EndInterfaceParams(
            preset="custom",
            outer_diameter_mm=root_od,
            bore_diameter_mm=root_bore,
            bolt_count=4 if root_od >= 34.0 else 0,
            bolt_pcd_mm=_clamp(round((root_od + root_bore) / 2.0, 1), 10.0, 105.0),
            bolt_size="M3",
            use_heat_inserts=True,
        )
        tip_end = EndInterfaceParams(
            preset="custom",
            outer_diameter_mm=tip_od,
            bore_diameter_mm=tip_bore,
            bolt_count=0,
            bolt_pcd_mm=_clamp(round((tip_od + tip_bore) / 2.0, 1), 10.0, 105.0),
            bolt_size="M3",
            use_heat_inserts=False,
        )
        notes.append(
            "No se encontraron 2 barrenos principales claramente separados; se estimaron extremos "
            "paramétricos a partir de la caja envolvente."
        )

    root_width = _clamp(round(min(bbox_width * 0.88, root_end.outer_diameter_mm * 0.92), 1), 16.0, 110.0)
    tip_width = _clamp(round(min(root_width * 0.85, tip_end.outer_diameter_mm * 0.82), 1), 12.0, 90.0)
    thickness = _clamp(round(bbox_thickness, 1), 8.0, 60.0)

    suggested = LinkDesignRequest(
        architecture="articulated_6dof",
        length_mm=est_length,
        root_width_mm=root_width,
        tip_width_mm=tip_width,
        thickness_mm=thickness,
        organic_taper=0.75,
        cable_channel=thickness >= 14.0,
        root_end=root_end,
        tip_end=tip_end,
        reference_volume_cm3=volume_cm3,
    )

    # Export translucent Ghost Overlay GLB for 3D visual comparison
    reference_glb_url: str | None = None
    try:
        ref_hash = hashlib.sha256(step_path.read_bytes()).hexdigest()[:12]
        ref_dir = EXPORTS_ROOT / f"ref_{ref_hash}"
        ref_dir.mkdir(parents=True, exist_ok=True)
        ref_glb_path = ref_dir / "reference.glb"
        if not (ref_glb_path.exists() and ref_glb_path.stat().st_size > 0):
            with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as tmp_stl:
                tmp_stl_path = Path(tmp_stl.name)
            try:
                export_stl(shape, str(tmp_stl_path), tolerance=0.12, angular_tolerance=0.2)
                loaded = trimesh.load(str(tmp_stl_path), force="mesh")
                mesh = loaded.dump(concatenate=True) if isinstance(loaded, trimesh.Scene) else loaded
                if isinstance(mesh, trimesh.Trimesh):
                    mesh.fix_normals()
                    material = trimesh.visual.material.PBRMaterial(
                        name="roboforge_step_ghost",
                        baseColorFactor=[56, 189, 248, 110],
                        metallicFactor=0.2,
                        roughnessFactor=0.35,
                        alphaMode="BLEND",
                    )
                    mesh.visual = trimesh.visual.TextureVisuals(material=material)
                    mesh.export(str(ref_glb_path), file_type="glb")
            finally:
                if tmp_stl_path.exists():
                    try:
                        tmp_stl_path.unlink()
                    except OSError:
                        pass
        if ref_glb_path.exists() and ref_glb_path.stat().st_size > 0:
            reference_glb_url = f"/api/files/ref_{ref_hash}/reference.glb"
    except Exception:
        reference_glb_url = None

    return StepReferenceAnalysis(
        filename=filename,
        bbox_length_mm=bbox_length,
        bbox_width_mm=bbox_width,
        bbox_thickness_mm=bbox_thickness,
        volume_cm3=volume_cm3,
        detected_holes=detected_holes,
        suggested_request=suggested,
        reference_glb_url=reference_glb_url,
        notes=notes,
    )
