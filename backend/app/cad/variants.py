"""Parametric generation of the 4 structural link variants using build123d."""

from __future__ import annotations

import math
from typing import Any

from build123d import (
    BuildLine,
    BuildPart,
    BuildSketch,
    Circle,
    Face,
    Line,
    Location,
    Part,
    Plane,
    Polygon,
    SlotOverall,
    Spline,
    ThreePointArc,
    extrude,
    fillet,
    make_face,
)

from ..schemas import LinkDesignRequest, VariantKind
from .base_link import (
    apply_cable_channel,
    apply_end_features,
    create_base_extrusion,
    get_clear_span_bounds,
    get_top_bottom_outer_edges,
    make_half_width_fn,
    round_to_nozzle,
    safe_fillet,
)


def _create_tapered_pocket_face(
    req: LinkDesignRequest,
    x_start: float,
    x_end: float,
    wall_y: float,
) -> Face | None:
    """Build a smooth rounded tapered 2D pocket face strictly inside the organic contour."""
    span = x_end - x_start
    if span < 12.0:
        return None

    hw_fn = make_half_width_fn(req)
    # Sample widths to check feasibility
    r_start = hw_fn(x_start + min(6.0, span * 0.15)) - wall_y
    r_end = hw_fn(x_end - min(6.0, span * 0.15)) - wall_y
    r_mid = hw_fn((x_start + x_end) / 2.0) - wall_y

    if min(r_start, r_end, r_mid) < 2.2:
        return None

    # End semicircle centers inset by their local radii so the pocket fits strictly in [x_start, x_end]
    r_left = min(r_start, span * 0.22)
    r_right = min(r_end, span * 0.22)
    cx_left = x_start + r_left
    cx_right = x_end - r_right

    if cx_right <= cx_left + 3.0:
        # Fallback to a simple centered stadium/slot
        width_slot = max(4.0, min(r_mid * 2.0, span * 0.8))
        with BuildSketch(Plane.XY) as sk:
            with Location(((x_start + x_end) / 2.0, 0.0)):
                SlotOverall(span, width_slot)
        return sk.sketch.faces()[0]

    # Sample interior points for smooth bionic pocket boundary
    sample_s = [0.25, 0.50, 0.75]
    top_pts = [(cx_left, r_left)]
    for s in sample_s:
        xs = cx_left + s * (cx_right - cx_left)
        ys = max(2.2, hw_fn(xs) - wall_y)
        top_pts.append((xs, ys))
    top_pts.append((cx_right, r_right))

    bot_pts = [(cx_right, -r_right)]
    for s in reversed(sample_s):
        xs = cx_left + s * (cx_right - cx_left)
        ys = max(2.2, hw_fn(xs) - wall_y)
        bot_pts.append((xs, -ys))
    bot_pts.append((cx_left, -r_left))

    try:
        with BuildSketch(Plane.XY) as sk:
            with BuildLine():
                ThreePointArc((cx_left, -r_left), (cx_left - r_left, 0.0), (cx_left, r_left))
                Spline(top_pts, tangents=[(1.0, 0.0), (1.0, 0.0)])
                ThreePointArc((cx_right, r_right), (cx_right + r_right, 0.0), (cx_right, -r_right))
                Spline(bot_pts, tangents=[(-1.0, 0.0), (-1.0, 0.0)])
            make_face()
        faces = sk.sketch.faces()
        return faces[0] if faces else None
    except Exception:
        # Safe fallback: rounded slot
        width_slot = max(4.0, min(r_left, r_right, r_mid) * 1.8)
        with BuildSketch(Plane.XY) as sk:
            with Location(((x_start + x_end) / 2.0, 0.0)):
                SlotOverall(span, width_slot)
        faces = sk.sketch.faces()
        return faces[0] if faces else None


def _build_tapered_solid(req: LinkDesignRequest) -> Part:
    """Variant 1: Smooth bionic tapered baseline solid with fillets."""
    part = create_base_extrusion(req)
    fillet_r = min(2.0, req.thickness_mm * 0.14)
    part = safe_fillet(part, get_top_bottom_outer_edges(part, req.thickness_mm), fillet_r)
    part = apply_end_features(part, req)
    part = apply_cable_channel(part, req)
    return part


def _build_organic_ibeam(req: LinkDesignRequest) -> Part:
    """Variant 2: Deep smooth recessed pockets on both top and bottom faces (I-beam section)."""
    part = create_base_extrusion(req)
    fillet_r = min(1.8, req.thickness_mm * 0.12)
    part = safe_fillet(part, get_top_bottom_outer_edges(part, req.thickness_mm), fillet_r)

    x_start, x_end = get_clear_span_bounds(req)
    flange_w = max(2.4, round_to_nozzle(max(3.2, 6.0 * req.nozzle_diameter_mm), req.nozzle_diameter_mm))
    web_t = max(2.0, round_to_nozzle(max(2.4, 5.0 * req.nozzle_diameter_mm), req.nozzle_diameter_mm))

    pocket_depth = (req.thickness_mm - web_t) / 2.0
    if pocket_depth >= 1.5:
        pocket_face = _create_tapered_pocket_face(req, x_start, x_end, flange_w)
        if pocket_face is not None:
            half_t = req.thickness_mm / 2.0
            cut_len = pocket_depth + 1.5
            # Top pocket: from z = web_t / 2 up past +half_t
            with BuildPart() as top_bp:
                extrude(
                    pocket_face.moved(Location((0.0, 0.0, web_t / 2.0))),
                    amount=cut_len,
                )
            # Bottom pocket: from z = -web_t / 2 down past -half_t
            with BuildPart() as bot_bp:
                extrude(
                    pocket_face.moved(Location((0.0, 0.0, -web_t / 2.0 - cut_len))),
                    amount=cut_len,
                )
            try:
                candidate = part - top_bp.part - bot_bp.part
                if candidate.is_valid and candidate.volume > 0:
                    part = candidate
            except Exception:
                pass

    part = apply_end_features(part, req)
    # Apply cable channel only if web is thick enough or inside bosses
    if web_t >= 5.0:
        part = apply_cable_channel(part, req)
    return part


def _build_rounded_truss(req: LinkDesignRequest) -> Part:
    """Variant 3: Alternating rounded triangular/elliptical through-cutouts for minimum mass."""
    part = create_base_extrusion(req)
    fillet_r = min(1.6, req.thickness_mm * 0.11)
    part = safe_fillet(part, get_top_bottom_outer_edges(part, req.thickness_mm), fillet_r)

    x_start, x_end = get_clear_span_bounds(req)
    span = x_end - x_start
    hw_fn = make_half_width_fn(req)

    chord_w = max(2.6, round_to_nozzle(max(3.4, 7.0 * req.nozzle_diameter_mm), req.nozzle_diameter_mm))
    strut_w = max(2.4, round_to_nozzle(max(3.2, 6.0 * req.nozzle_diameter_mm), req.nozzle_diameter_mm))

    if span >= 16.0:
        avg_hw = (hw_fn(x_start + span * 0.25) + hw_fn(x_start + span * 0.75)) / 2.0
        target_pitch = max(16.0, avg_hw * 1.15)
        n_cells = max(3, min(9, int(round(span / target_pitch))))
        cell_pitch = span / n_cells
        cut_height = req.thickness_mm + 4.0

        cutters: list[Part] = []
        for i in range(n_cells):
            cx0 = x_start + i * cell_pitch
            cx1 = cx0 + cell_pitch
            xm = (cx0 + cx1) / 2.0

            # Inset horizontally by half strut_w on each side
            tx0 = cx0 + strut_w * 0.55
            tx1 = cx1 - strut_w * 0.55
            if tx1 - tx0 < 4.0:
                continue

            hw0 = hw_fn(tx0) - chord_w
            hw1 = hw_fn(tx1) - chord_w
            hwm = hw_fn(xm) - chord_w

            # Build alternating rounded triangle if vertical room is sufficient
            if min(hw0, hw1, hwm) >= 3.2 and (tx1 - tx0) >= 6.5:
                if i % 2 == 0:
                    # Base on top (+Y), apex pointing down (-Y)
                    pts = [(tx0, hw0), (tx1, hw1), (xm, -hwm)]
                else:
                    # Base on bottom (-Y), apex pointing up (+Y)
                    pts = [(tx0, -hw0), (tx1, -hw1), (xm, hwm)]

                # Compute shortest edge length to pick a safe corner fillet radius
                edge_lens = [
                    math.hypot(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]),
                    math.hypot(pts[2][0] - pts[1][0], pts[2][1] - pts[1][1]),
                    math.hypot(pts[0][0] - pts[2][0], pts[0][1] - pts[2][1]),
                ]
                r_corner = min(3.2, min(edge_lens) * 0.22)

                try:
                    with BuildSketch(Plane.XY) as sk:
                        Polygon(*pts, align=None)
                        if r_corner >= 0.4:
                            fillet(sk.vertices(), radius=r_corner)
                    face = sk.sketch.faces()[0]
                    with BuildPart() as cut_bp:
                        extrude(face, amount=cut_height / 2.0, both=True)
                    cutters.append(cut_bp.part)
                    continue
                except Exception:
                    pass

            # Fallback for narrow tip cells: clean circular cutout
            r_circ = min((tx1 - tx0) / 2.0, hwm)
            if r_circ >= 2.0:
                with BuildSketch(Plane.XY) as sk:
                    with Location((xm, 0.0)):
                        Circle(r_circ)
                with BuildPart() as cut_bp:
                    extrude(sk.sketch.faces()[0], amount=cut_height / 2.0, both=True)
                cutters.append(cut_bp.part)

        for cutter in cutters:
            try:
                candidate = part - cutter
                if candidate.is_valid and candidate.volume > 0:
                    part = candidate
            except Exception:
                continue

    part = apply_end_features(part, req)
    return part


def _build_hollow_cobot(req: LinkDesignRequest) -> Part:
    """Variant 4: Hollowed closed torsion-box cobot shell with smooth oval side windows."""
    part = create_base_extrusion(req)
    # More generous ergonomic cobot edge fillets
    fillet_r = min(3.2, req.thickness_mm * 0.20)
    part = safe_fillet(part, get_top_bottom_outer_edges(part, req.thickness_mm), fillet_r)

    x_start, x_end = get_clear_span_bounds(req)
    span = x_end - x_start
    hw_fn = make_half_width_fn(req)

    wall_y = max(2.4, round_to_nozzle(max(3.0, 6.0 * req.nozzle_diameter_mm), req.nozzle_diameter_mm))
    wall_z = max(2.0, round_to_nozzle(max(2.4, 5.0 * req.nozzle_diameter_mm), req.nozzle_diameter_mm))
    cavity_h = req.thickness_mm - 2.0 * wall_z

    # 1. Hollow internal torsion cavity along the span
    if cavity_h >= 3.0 and span >= 14.0:
        cavity_face = _create_tapered_pocket_face(req, x_start, x_end, wall_y)
        if cavity_face is not None:
            with BuildPart() as cav_bp:
                extrude(cavity_face, amount=cavity_h / 2.0, both=True)
            try:
                candidate = part - cav_bp.part
                if candidate.is_valid and candidate.volume > 0:
                    part = candidate
            except Exception:
                pass

    # 2. Smooth oval/stadium side windows cut transversely through Y (Plane.XZ)
    win_h = min(cavity_h * 0.78, req.thickness_mm - 2.0 * wall_z - 0.8)
    if win_h >= 3.0 and span >= 22.0:
        n_windows = max(2, min(4, int(span // 34.0) + 1))
        win_pitch = span / n_windows
        win_len = max(win_h + 2.0, win_pitch * 0.58)
        max_y_cut = max(req.root_width_mm, req.tip_width_mm, req.root_end.outer_diameter_mm) + 20.0

        for i in range(n_windows):
            wx = x_start + (i + 0.5) * win_pitch
            # Ensure window stays inside span margins
            effective_len = min(win_len, win_pitch - 6.0)
            if effective_len < win_h + 0.5:
                continue
            try:
                with BuildSketch(Plane.XZ) as sk:
                    with Location((wx, 0.0)):
                        SlotOverall(effective_len, win_h)
                with BuildPart() as win_bp:
                    extrude(sk.sketch.faces()[0], amount=max_y_cut / 2.0, both=True)
                candidate = part - win_bp.part
                if candidate.is_valid and candidate.volume > 0:
                    part = candidate
            except Exception:
                continue

    part = apply_end_features(part, req)
    part = apply_cable_channel(part, req)
    return part


def _part_metrics(part: Part) -> tuple[float, bool]:
    is_valid = bool(part.is_valid)
    volume_mm3 = float(part.volume) if part.volume and part.volume > 0 else 0.0
    return volume_mm3, is_valid and volume_mm3 > 0.0


def generate_all_variants(req: LinkDesignRequest) -> dict[VariantKind, dict[str, Any]]:
    """Generate all 4 parametric CAD variants and return geometry + metadata."""
    specs: list[tuple[VariantKind, str, str, Any]] = [
        (
            "tapered_solid",
            "Eslabón Biónico Sólido",
            "Perfil orgánico ahusado de máxima rigidez y resistencia base",
            _build_tapered_solid,
        ),
        (
            "organic_ibeam",
            "Viga I Orgánica Aligerada",
            "Bolsillos biónicos profundos con alma central y pestañas reforzadas",
            _build_organic_ibeam,
        ),
        (
            "rounded_truss",
            "Celosía Biónica Imprimible",
            "Recortes triangulares redondeados pasantes para masa mínima sin soportes",
            _build_rounded_truss,
        ),
        (
            "hollow_cobot",
            "Cuerpo Hueco Cobot",
            "Caja de torsión cerrada con ventanas laterales y túnel interno pasacables",
            _build_hollow_cobot,
        ),
    ]

    results: dict[VariantKind, dict[str, Any]] = {}
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
