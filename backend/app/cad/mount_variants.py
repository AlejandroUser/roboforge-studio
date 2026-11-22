"""Parametric CAD generators for Family 2: Mechatronic Mounts (NEMA 17/23, Servos, 2020)."""

from __future__ import annotations

import math
from typing import Any

from build123d import (
    Box,
    BuildPart,
    BuildSketch,
    Circle,
    Cylinder,
    Location,
    Locations,
    Part,
    Plane,
    Polygon,
    RectangleRounded,
    SlotOverall,
    extrude,
    fillet,
)

from ..schemas import MountDesignRequest, MountVariantKind
from .base_link import (
    BOLT_CLEARANCE_DIAMETERS_MM,
    HEAT_INSERT_PILOT_DIAMETERS_MM,
    round_to_nozzle,
    safe_fillet,
)


def _hole_d(bolt_size: str, use_heat_inserts: bool, clearance_mm: float) -> float:
    table = HEAT_INSERT_PILOT_DIAMETERS_MM if use_heat_inserts else BOLT_CLEARANCE_DIAMETERS_MM
    return table.get(bolt_size, 3.2) + clearance_mm


def _compute_mount_envelope(req: MountDesignRequest) -> tuple[float, float, float, float]:
    """Return (width_W, height_H, base_depth_D, shaft_z) in mm for the given actuator."""
    t = req.wall_thickness_mm
    g = req.gusset_thickness_mm
    d = req.base_depth_mm

    if req.actuator_type == "nema17":
        w = max(50.0, 44.0 + 2.0 * g)
        shaft_z = max(req.shaft_height_mm, t + 22.5)
        h = max(shaft_z + 23.0, t + 46.0)
    elif req.actuator_type == "nema23":
        w = max(64.0, 58.5 + 2.0 * g)
        shaft_z = max(req.shaft_height_mm, t + 30.0)
        h = max(shaft_z + 30.0, t + 60.0)
    elif req.actuator_type == "servo_mg996r":
        w = max(62.0, 56.0 + 2.0 * g)
        shaft_z = max(req.shaft_height_mm, t + 15.0)
        h = max(shaft_z + 16.5, t + 33.0)
    else:  # profile_2020_gusset
        w = max(32.0, 24.0 + 2.0 * g)
        shaft_z = max(req.shaft_height_mm, t + 16.0)
        h = max(shaft_z + 16.0, t + 34.0)

    return round(w, 2), round(h, 2), round(d, 2), round(shaft_z, 2)


def _build_l_bracket_body(w: float, h: float, d: float, t: float) -> Part:
    """Create the base L-bracket solid aligned to FEA coordinates:
    - Base plate: x in [-W/2, +W/2], y in [0, D], z in [0, t]
    - Vertical faceplate: x in [-W/2, +W/2], y in [0, t], z in [0, H]
    """
    base_box = Location((0.0, d / 2.0, t / 2.0)) * Box(w, d, t)
    vert_box = Location((0.0, t / 2.0, h / 2.0)) * Box(w, t, h)
    part = base_box + vert_box

    # Add a smooth internal 90-degree corner relief fillet along x at (y=t, z=t)
    inner_edges = [
        e
        for e in part.edges()
        if abs(e.center().Y - t) < 0.2 and abs(e.center().Z - t) < 0.2
    ]
    part = safe_fillet(part, inner_edges, min(3.0, t * 0.55))

    # Subtle outer top/front edge rounding for ergonomics and printability
    outer_corner_edges = [
        e
        for e in part.edges()
        if (abs(e.center().Y - d) < 0.2 and abs(abs(e.center().X) - w / 2.0) < 0.2)
        or (abs(e.center().Z - h) < 0.2 and abs(abs(e.center().X) - w / 2.0) < 0.2)
    ]
    part = safe_fillet(part, outer_corner_edges, min(3.5, t * 0.6))
    return part


def _apply_actuator_faceplate_features(
    part: Part,
    req: MountDesignRequest,
    w: float,
    h: float,
    shaft_z: float,
) -> Part:
    """Subtract motor/servo/profile mounting cutouts along Y through the vertical faceplate."""
    t = req.wall_thickness_mm
    clr = req.printer_clearance_mm
    cut_y = t + 6.0
    yc = t / 2.0
    cutters: list[Part] = []

    def _y_cyl(x: float, z: float, diameter: float) -> Part:
        return (
            Location((x, yc, z), (90.0, 0.0, 0.0))
            * Cylinder(radius=diameter / 2.0, height=cut_y)
        )

    if req.actuator_type == "nema17":
        # Central Ø22 mm pilot bore + 4×M3 on 31×31 mm square
        cutters.append(_y_cyl(0.0, shaft_z, 22.0 + 2.0 * clr))
        bolt_d = _hole_d("M3", req.use_heat_inserts, clr)
        for sx in (-15.5, 15.5):
            for sz in (-15.5, 15.5):
                cutters.append(_y_cyl(sx, shaft_z + sz, bolt_d))

    elif req.actuator_type == "nema23":
        # Central Ø38.1 mm pilot bore + 4×M5 on 47.14×47.14 mm square
        cutters.append(_y_cyl(0.0, shaft_z, 38.1 + 2.0 * clr))
        bolt_d = _hole_d("M5", req.use_heat_inserts, clr)
        half_p = 47.14 / 2.0
        for sx in (-half_p, half_p):
            for sz in (-half_p, half_p):
                cutters.append(_y_cyl(sx, shaft_z + sz, bolt_d))

    elif req.actuator_type == "servo_mg996r":
        # 40.5 × 20.2 mm rounded rectangular window + 4×M3 mounting holes
        win_w = 40.5 + 2.0 * clr
        win_h = 20.2 + 2.0 * clr
        with BuildSketch(Plane.XZ) as sk:
            with Locations((0.0, shaft_z)):
                RectangleRounded(win_w, win_h, radius=1.5)
        with BuildPart() as win_bp:
            extrude(sk.sketch.faces()[0], amount=cut_y / 2.0, both=True)
        cutters.append(Location((0.0, yc, 0.0)) * win_bp.part)

        bolt_d = _hole_d("M3", req.use_heat_inserts, clr)
        for sx in (-24.75, 24.75):
            for sz in (-5.0, 5.0):
                cutters.append(_y_cyl(sx, shaft_z + sz, bolt_d))

    else:  # profile_2020_gusset
        # 2×M5 mounting holes on the vertical leg
        bolt_d = _hole_d("M5", req.use_heat_inserts, clr)
        z_lo = max(t + 7.5, shaft_z - 8.0)
        z_hi = min(h - 7.5, shaft_z + 8.0)
        if z_hi - z_lo >= 10.0:
            cutters.append(_y_cyl(0.0, z_lo, bolt_d))
            cutters.append(_y_cyl(0.0, z_hi, bolt_d))
        else:
            cutters.append(_y_cyl(0.0, shaft_z, bolt_d))

    result = part
    for cutter in cutters:
        try:
            cand = result - cutter
            if cand.is_valid and cand.volume > 0:
                result = cand
        except Exception:
            continue
    return result


def _apply_base_plate_features(part: Part, req: MountDesignRequest, w: float, d: float) -> Part:
    """Subtract mounting holes/slots along Z through the horizontal base plate."""
    t = req.wall_thickness_mm
    g = req.gusset_thickness_mm
    clr = req.printer_clearance_mm
    cut_z = t + 6.0
    zc = t / 2.0

    # Usable interior width between lateral gussets
    inner_half_w = max(4.0, (w / 2.0) - g - 5.5)
    usable_y0 = t + 6.0
    usable_y1 = d - 6.0
    y_mid = (usable_y0 + usable_y1) / 2.0

    cutters: list[Part] = []

    def _z_cyl(x: float, y: float, diameter: float) -> Part:
        return Location((x, y, zc)) * Cylinder(radius=diameter / 2.0, height=cut_z)

    def _z_slot(x: float, y: float, length: float, width: float) -> Part:
        with BuildSketch(Plane.XY) as sk:
            with Locations((x, y)):
                SlotOverall(length, width)
        with BuildPart() as bp:
            extrude(sk.sketch.faces()[0], amount=cut_z / 2.0, both=True)
        return Location((0.0, 0.0, zc)) * bp.part

    if req.base_type == "profile_2020":
        slot_w = _hole_d("M5", req.use_heat_inserts, clr)
        slot_l = min(11.0, max(slot_w + 2.5, (usable_y1 - usable_y0) * 0.45))
        if inner_half_w >= 11.0:
            x_off = min(inner_half_w - 3.5, 16.0)
            cutters.append(_z_slot(-x_off, y_mid, slot_l, slot_w))
            cutters.append(_z_slot(x_off, y_mid, slot_l, slot_w))
        else:
            cutters.append(_z_cyl(0.0, y_mid, slot_w))

    elif req.base_type == "profile_2040":
        hole_d = _hole_d("M5", req.use_heat_inserts, clr)
        x_off = min(max(inner_half_w - 3.0, 6.0), 16.0)
        dy = min(10.0, max(5.5, (usable_y1 - usable_y0) * 0.32))
        y_coords = [y_mid - dy, y_mid + dy] if (usable_y1 - usable_y0) >= 14.0 else [y_mid]
        x_coords = [-x_off, x_off] if inner_half_w >= 9.0 else [0.0]
        for bx in x_coords:
            for by in y_coords:
                cutters.append(_z_cyl(bx, by, hole_d))

    else:  # flat_plate
        hole_d = _hole_d("M4", req.use_heat_inserts, clr)
        x_off = min(max(inner_half_w - 2.5, 6.0), w * 0.34)
        dy = min(11.0, max(5.0, (usable_y1 - usable_y0) * 0.34))
        y_coords = [y_mid - dy, y_mid + dy] if (usable_y1 - usable_y0) >= 13.0 else [y_mid]
        x_coords = [-x_off, x_off] if inner_half_w >= 8.5 else [0.0]
        for bx in x_coords:
            for by in y_coords:
                cutters.append(_z_cyl(bx, by, hole_d))

    result = part
    for cutter in cutters:
        try:
            cand = result - cutter
            if cand.is_valid and cand.volume > 0:
                result = cand
        except Exception:
            continue
    return result


def _add_side_gussets(part: Part, req: MountDesignRequest, w: float, h: float, d: float) -> Part:
    """Add left and right triangular structural gussets connecting base plate to vertical faceplate."""
    t = req.wall_thickness_mm
    g = req.gusset_thickness_mm

    y_end = max(t + 8.0, d - 1.5)
    z_top = max(t + 10.0, h - 2.0)

    # Triangular profile in YZ plane (overlap slightly into walls by 0.5mm for watertight union)
    tri_pts = [
        (t - 0.5, t - 0.5),
        (y_end, t - 0.5),
        (t - 0.5, z_top),
    ]
    with BuildSketch(Plane.YZ) as sk:
        Polygon(*tri_pts, align=None)
    tri_face = sk.sketch.faces()[0]

    # Left gusset: x in [-W/2, -W/2 + g]
    with BuildPart() as left_bp:
        extrude(tri_face.moved(Location((-w / 2.0, 0.0, 0.0))), amount=g)
    # Right gusset: x in [W/2 - g, W/2]
    with BuildPart() as right_bp:
        extrude(tri_face.moved(Location((w / 2.0 - g, 0.0, 0.0))), amount=g)

    result = part + left_bp.part + right_bp.part
    return result


def _build_mount_standard(req: MountDesignRequest) -> Part:
    """Variant 1: Clean 90-degree L-bracket with internal/external relief fillets."""
    w, h, d, shaft_z = _compute_mount_envelope(req)
    part = _build_l_bracket_body(w, h, d, req.wall_thickness_mm)
    part = _apply_actuator_faceplate_features(part, req, w, h, shaft_z)
    part = _apply_base_plate_features(part, req, w, d)
    return part


def _build_mount_gusseted(req: MountDesignRequest) -> Part:
    """Variant 2: L-bracket reinforced with dual lateral triangular structural gussets."""
    w, h, d, shaft_z = _compute_mount_envelope(req)
    part = _build_l_bracket_body(w, h, d, req.wall_thickness_mm)
    part = _add_side_gussets(part, req, w, h, d)
    part = _apply_actuator_faceplate_features(part, req, w, h, shaft_z)
    part = _apply_base_plate_features(part, req, w, d)
    return part


def _build_mount_lightweight(req: MountDesignRequest) -> Part:
    """Variant 3: Gusseted bracket with bionic gusset pockets, base lightening, and thermal vents."""
    w, h, d, shaft_z = _compute_mount_envelope(req)
    t = req.wall_thickness_mm
    g = req.gusset_thickness_mm

    part = _build_l_bracket_body(w, h, d, t)
    part = _add_side_gussets(part, req, w, h, d)

    margin = max(3.2, round_to_nozzle(3.6, req.nozzle_diameter_mm, min_lines=6))
    y_end = max(t + 8.0, d - 1.5)
    z_top = max(t + 10.0, h - 2.0)

    # 1. Rounded triangular (or circular fallback) lightening cutouts through both gussets
    leg_y = y_end - t
    leg_z = z_top - t
    hyp = math.hypot(leg_y, leg_z)
    # Incircle radius of the gusset triangle
    r_in = (leg_y * leg_z) / max(leg_y + leg_z + hyp, 1e-3)
    if r_in > margin + 2.2:
        # Inset triangle vertices strictly inside the gusset by `margin`
        p0 = (t + margin, t + margin)
        # Offset along hypotenuse normal
        y_inset = y_end - margin * (1.0 + hyp / max(leg_z, 1e-3))
        z_inset = z_top - margin * (1.0 + hyp / max(leg_y, 1e-3))
        gusset_cut_face = None

        if y_inset > p0[0] + 4.0 and z_inset > p0[1] + 4.0:
            pts = [p0, (y_inset, p0[1]), (p0[0], z_inset)]
            edge_lens = [
                math.hypot(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]),
                math.hypot(pts[2][0] - pts[1][0], pts[2][1] - pts[1][1]),
                math.hypot(pts[0][0] - pts[2][0], pts[0][1] - pts[2][1]),
            ]
            r_fil = min(3.0, min(edge_lens) * 0.24)
            try:
                with BuildSketch(Plane.YZ) as sk:
                    Polygon(*pts, align=None)
                    if r_fil >= 0.5:
                        fillet(sk.vertices(), radius=r_fil)
                gusset_cut_face = sk.sketch.faces()[0]
            except Exception:
                gusset_cut_face = None

        if gusset_cut_face is None:
            # Circular lightening hole centered at triangle incenter
            cy = t + r_in
            cz = t + r_in
            r_hole = max(2.2, r_in - margin * 0.85)
            with BuildSketch(Plane.YZ) as sk:
                with Locations((cy, cz)):
                    Circle(r_hole)
            gusset_cut_face = sk.sketch.faces()[0]

        if gusset_cut_face is not None:
            cut_x_span = g + 4.0
            for x_start in (-w / 2.0 - 2.0, w / 2.0 - g - 2.0):
                try:
                    with BuildPart() as gcut_bp:
                        extrude(
                            gusset_cut_face.moved(Location((x_start, 0.0, 0.0))),
                            amount=cut_x_span,
                        )
                    cand = part - gcut_bp.part
                    if cand.is_valid and cand.volume > 0:
                        part = cand
                except Exception:
                    continue

    # 2. Central lightening pocket/cutout in the horizontal base plate
    inner_half_w = (w / 2.0) - g - 5.0
    usable_y = (d - t) - 10.0
    if inner_half_w >= 8.0 and usable_y >= 10.0:
        y_center = t + (d - t) * 0.52
        slot_w = min(usable_y * 0.48, 12.0)
        slot_l = min(inner_half_w * 0.85, 18.0)
        if slot_l >= slot_w + 1.0 and slot_w >= 5.0:
            try:
                with BuildSketch(Plane.XY) as sk:
                    with Locations((0.0, y_center)):
                        SlotOverall(slot_l, slot_w)
                with BuildPart() as bcut_bp:
                    extrude(sk.sketch.faces()[0], amount=(t + 6.0) / 2.0, both=True)
                cand = part - (Location((0.0, 0.0, t / 2.0)) * bcut_bp.part)
                if cand.is_valid and cand.volume > 0:
                    part = cand
            except Exception:
                pass

    # 3. Optional thermal ventilation slots on the vertical faceplate
    if req.ventilation_slots and req.actuator_type in ("nema17", "nema23"):
        vent_w = 3.2
        vent_l = 10.0 if req.actuator_type == "nema17" else 14.0
        z_vent_top = h - 4.2
        if z_vent_top > shaft_z + (19.0 if req.actuator_type == "nema17" else 27.0):
            try:
                with BuildSketch(Plane.XZ) as sk:
                    with Locations((0.0, z_vent_top)):
                        SlotOverall(vent_l, vent_w)
                with BuildPart() as vcut_bp:
                    extrude(sk.sketch.faces()[0], amount=(t + 6.0) / 2.0, both=True)
                cand = part - (Location((0.0, t / 2.0, 0.0)) * vcut_bp.part)
                if cand.is_valid and cand.volume > 0:
                    part = cand
            except Exception:
                pass
        # Lateral vertical cooling slots flanking the pilot bore (outside bolt square)
        x_vent = (w / 2.0) - g - 3.6
        min_x_clear = 18.5 if req.actuator_type == "nema17" else 27.0
        if x_vent > min_x_clear:
            for sx in (-x_vent, x_vent):
                try:
                    with BuildSketch(Plane.XZ) as sk:
                        with Locations((sx, shaft_z)):
                            SlotOverall(14.0, 2.8, rotation=90.0)
                    with BuildPart() as vcut_bp:
                        extrude(sk.sketch.faces()[0], amount=(t + 6.0) / 2.0, both=True)
                    cand = part - (Location((0.0, t / 2.0, 0.0)) * vcut_bp.part)
                    if cand.is_valid and cand.volume > 0:
                        part = cand
                except Exception:
                    continue

    part = _apply_actuator_faceplate_features(part, req, w, h, shaft_z)
    part = _apply_base_plate_features(part, req, w, d)
    return part


def _part_metrics(part: Part) -> tuple[float, bool]:
    is_valid = bool(part.is_valid)
    volume_mm3 = float(part.volume) if part.volume and part.volume > 0 else 0.0
    return volume_mm3, is_valid and volume_mm3 > 0.0


def generate_all_mount_variants(
    req: MountDesignRequest,
) -> dict[MountVariantKind, dict[str, Any]]:
    """Generate all 3 mechatronic mount variants and return geometry + metadata."""
    specs: list[tuple[MountVariantKind, str, str, Any]] = [
        (
            "mount_standard",
            "Montura en L Estándar",
            "Escuadra mecatrónica limpia con radios internos y externos de alivio",
            _build_mount_standard,
        ),
        (
            "mount_gusseted",
            "Montura con Cartelas Estructurales",
            "Doble cartela triangular lateral de máxima rigidez frente a flexión y torque",
            _build_mount_gusseted,
        ),
        (
            "mount_lightweight",
            "Montura Biónica Aligerada",
            "Cartelas con bolsillos triangulares, base aligerada y ventilación térmica",
            _build_mount_lightweight,
        ),
    ]

    results: dict[MountVariantKind, dict[str, Any]] = {}
    for variant_id, name, tagline, builder in specs:
        part = builder(req)
        volume_mm3, is_valid = _part_metrics(part)
        results[variant_id] = {
            "part": part,
            "name": name,
            "tagline": tagline,
            "volume_mm3": volume_mm3,
            "is_valid": is_valid,
        }
    return results
