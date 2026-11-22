"""Base parametric geometry builders for organic robotic arm links using build123d."""

from __future__ import annotations

import math
from typing import Callable

from build123d import (
    Axis,
    BuildLine,
    BuildPart,
    BuildSketch,
    Circle,
    Cylinder,
    Face,
    Line,
    Location,
    Mode,
    Part,
    Plane,
    Spline,
    ThreePointArc,
    Vector,
    extrude,
    fillet,
    make_face,
)

from ..schemas import EndInterfaceParams, LinkDesignRequest


# Bolt clearance and heat-set insert pilot diameters (mm)
BOLT_CLEARANCE_DIAMETERS_MM: dict[str, float] = {
    "M3": 3.2,
    "M4": 4.3,
    "M5": 5.3,
}

HEAT_INSERT_PILOT_DIAMETERS_MM: dict[str, float] = {
    "M3": 4.0,
    "M4": 5.6,
    "M5": 6.4,
}


def round_to_nozzle(value_mm: float, nozzle_mm: float, min_lines: int = 3) -> float:
    """Round a wall thickness to an integer multiple of nozzle diameter."""
    nozzle = max(0.2, float(nozzle_mm))
    lines = max(min_lines, round(value_mm / nozzle))
    return round(lines * nozzle, 3)


def get_end_radii(req: LinkDesignRequest) -> tuple[float, float]:
    """Compute safe effective outer boss radii (R_a at root, R_b at tip)."""
    r_a = max(
        req.root_end.outer_diameter_mm / 2.0,
        req.root_end.bore_diameter_mm / 2.0 + req.printer_clearance_mm + 2.0,
    )
    r_b = max(
        req.tip_end.outer_diameter_mm / 2.0,
        req.tip_end.bore_diameter_mm / 2.0 + req.printer_clearance_mm + 2.0,
    )
    return r_a, r_b


def compute_bitangent_geometry(
    length_mm: float,
    r_a: float,
    r_b: float,
) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]]:
    """Compute exact external bitangent points and unit tangent vector for two circles.

    Circle A is centered at (0, 0) with radius r_a.
    Circle B is centered at (length_mm, 0) with radius r_b.
    Returns (p_a_top, p_b_top, tangent_top).
    """
    sin_alpha = max(-0.92, min(0.92, (r_a - r_b) / max(length_mm, 1e-3)))
    alpha = math.asin(sin_alpha)
    cos_alpha = math.cos(alpha)

    p_a_top = (r_a * sin_alpha, r_a * cos_alpha)
    p_b_top = (length_mm + r_b * sin_alpha, r_b * cos_alpha)
    tangent_top = (cos_alpha, -sin_alpha)
    return p_a_top, p_b_top, tangent_top


def make_half_width_fn(req: LinkDesignRequest) -> Callable[[float], float]:
    """Return a smooth function y(x) giving the upper profile half-width along x in [0, L]."""
    length = req.length_mm
    r_a, r_b = get_end_radii(req)
    w_a = req.root_width_mm / 2.0
    w_b = req.tip_width_mm / 2.0
    taper = req.organic_taper

    p_a_top, p_b_top, _ = compute_bitangent_geometry(length, r_a, r_b)
    x_a, y_a = p_a_top
    x_b, y_b = p_b_top
    span = max(x_b - x_a, 1e-3)
    min_half_w = max(4.5, round_to_nozzle(3.0, req.nozzle_diameter_mm, min_lines=4))

    def half_width(x: float) -> float:
        if x <= x_a:
            dx = max(-r_a, min(r_a, x))
            return math.sqrt(max(0.0, r_a * r_a - dx * dx))
        if x >= x_b:
            dx = max(-r_b, min(r_b, x - length))
            return math.sqrt(max(0.0, r_b * r_b - dx * dx))

        s = (x - x_a) / span
        y_lin = y_a + s * (y_b - y_a)
        y_req = w_a + s * (w_b - w_a)

        # Smooth bell shape that is 0 at ends (s=0, s=1) and 1 at mid-span (s=0.5)
        bell = math.sin(math.pi * s) ** 2
        # Blend linear bitangent towards user width & apply bionic concave waist
        y_blend = y_lin + (y_req - y_lin) * (0.35 + 0.65 * taper) * bell
        concave_dip = 0.16 * taper * min(y_lin, y_blend) * bell
        return max(min_half_w, y_blend - concave_dip)

    return half_width


def create_organic_profile_face(req: LinkDesignRequest) -> Face:
    """Create the closed 2D Face of the organic tapered robotic link on the XY plane."""
    length = req.length_mm
    r_a, r_b = get_end_radii(req)
    p_a_top, p_b_top, tan_top = compute_bitangent_geometry(length, r_a, r_b)
    x_a, y_a = p_a_top
    x_b, y_b = p_b_top

    # Degenerate case where bosses almost touch or overlap
    if x_b <= x_a + 6.0:
        with BuildSketch(Plane.XY) as sk:
            Circle(r_a)
            with Location((length, 0)):
                Circle(r_b)
        return sk.sketch.faces()[0]

    hw_fn = make_half_width_fn(req)
    p_a_bot = (x_a, -y_a)
    p_b_bot = (x_b, -y_b)

    # Sample intermediate points along the span for the smooth bionic curve
    sample_s = [0.22, 0.50, 0.78]
    top_pts = [p_a_top]
    bot_pts = [p_b_bot]

    for s in sample_s:
        xs = x_a + s * (x_b - x_a)
        ys = hw_fn(xs)
        top_pts.append((xs, ys))
    top_pts.append(p_b_top)

    for s in reversed(sample_s):
        xs = x_a + s * (x_b - x_a)
        ys = hw_fn(xs)
        bot_pts.append((xs, -ys))
    bot_pts.append(p_a_bot)

    # Check if the curve is essentially straight (organic_taper ~ 0 and widths match bitangent)
    max_dev = max(
        abs(pt[1] - (y_a + ((pt[0] - x_a) / (x_b - x_a)) * (y_b - y_a)))
        for pt in top_pts[1:-1]
    )

    tan_bot = (-tan_top[0], -tan_top[1])

    with BuildSketch(Plane.XY) as sk:
        with BuildLine():
            # Left circular boss arc (from bottom tangent, around -R_a, to top tangent)
            ThreePointArc(p_a_bot, (-r_a, 0.0), p_a_top)
            # Upper organic edge
            if max_dev < 0.15:
                Line(p_a_top, p_b_top)
            else:
                Spline(top_pts, tangents=[tan_top, tan_top])
            # Right circular boss arc (from top tangent, around L + R_b, to bottom tangent)
            ThreePointArc(p_b_top, (length + r_b, 0.0), p_b_bot)
            # Lower organic edge (symmetric)
            if max_dev < 0.15:
                Line(p_b_bot, p_a_bot)
            else:
                Spline(bot_pts, tangents=[tan_bot, tan_bot])
        make_face()

    return sk.sketch.faces()[0]


def create_base_extrusion(req: LinkDesignRequest) -> Part:
    """Extrude the organic 2D link profile centered on Z in [-t/2, +t/2]."""
    face = create_organic_profile_face(req)
    with BuildPart() as bp:
        extrude(face, amount=req.thickness_mm / 2.0, both=True)
    return bp.part


def _bolt_hole_diameter(end: EndInterfaceParams, clearance_mm: float) -> float:
    if end.use_heat_inserts:
        base = HEAT_INSERT_PILOT_DIAMETERS_MM.get(end.bolt_size, 4.0)
    else:
        base = BOLT_CLEARANCE_DIAMETERS_MM.get(end.bolt_size, 3.2)
    return base + clearance_mm


def apply_end_features(part: Part, req: LinkDesignRequest) -> Part:
    """Subtract root and tip mounting bores and bolt circle patterns from the link."""
    thickness = req.thickness_mm
    cut_height = thickness + 4.0
    r_a, r_b = get_end_radii(req)

    end_specs = [
        (0.0, req.root_end, r_a),
        (req.length_mm, req.tip_end, r_b),
    ]

    cutters: list[Part] = []

    for cx, end, outer_r in end_specs:
        # Main central bore (with 2x radial clearance for slip/bearing fit)
        bore_r = (end.bore_diameter_mm / 2.0) + req.printer_clearance_mm
        bore_r = min(bore_r, outer_r - 1.5)
        if bore_r > 0.5:
            cutters.append(
                Location((cx, 0.0, 0.0)) * Cylinder(radius=bore_r, height=cut_height)
            )

        # Bolt circle pattern
        if end.bolt_count > 0:
            bolt_r = _bolt_hole_diameter(end, req.printer_clearance_mm) / 2.0
            pcd_r = end.bolt_pcd_mm / 2.0
            # Verify bolt holes fit cleanly between bore and outer perimeter
            if (pcd_r - bolt_r > bore_r + 0.5) and (pcd_r + bolt_r < outer_r - 0.5):
                angle_offset = math.pi / 4.0 if end.bolt_count == 4 else 0.0
                for i in range(end.bolt_count):
                    theta = angle_offset + (2.0 * math.pi * i) / end.bolt_count
                    bx = cx + pcd_r * math.cos(theta)
                    by = pcd_r * math.sin(theta)
                    cutters.append(
                        Location((bx, by, 0.0)) * Cylinder(radius=bolt_r, height=cut_height)
                    )

    result = part
    for cutter in cutters:
        result = result - cutter
    return result


def apply_cable_channel(part: Part, req: LinkDesignRequest) -> Part:
    """Cut an internal longitudinal cable tunnel along the X axis connecting both bores."""
    if not req.cable_channel:
        return part

    # Ensure enough top/bottom material remains around the cable tunnel
    min_wall_z = max(2.0, round_to_nozzle(2.0, req.nozzle_diameter_mm, min_lines=4))
    max_tunnel_d = req.thickness_mm - 2.0 * min_wall_z
    if max_tunnel_d < 2.8:
        return part

    tunnel_r = min(2.6, max_tunnel_d / 2.0)
    tunnel_len = req.length_mm

    # Horizontal cylinder along X axis from x=0 to x=L at (y=0, z=0)
    tunnel = (
        Location((tunnel_len / 2.0, 0.0, 0.0), (0.0, 90.0, 0.0))
        * Cylinder(radius=tunnel_r, height=tunnel_len)
    )
    try:
        cut_part = part - tunnel
        if cut_part.is_valid and cut_part.volume > 0:
            return cut_part
    except Exception:
        pass
    return part


def safe_fillet(part: Part, edges, radius: float) -> Part:
    """Safely apply a fillet with progressive radius fallbacks, never breaking solidity."""
    if not edges or radius <= 0.05:
        return part

    for factor in (1.0, 0.65, 0.35):
        r = round(radius * factor, 3)
        if r < 0.15:
            continue
        try:
            candidate = fillet(edges, radius=r)
            if candidate.is_valid and candidate.volume > 0:
                return candidate
        except Exception:
            continue
    return part


def get_top_bottom_outer_edges(part: Part, thickness_mm: float):
    """Select edges lying on the top (+t/2) or bottom (-t/2) Z planes."""
    half_t = thickness_mm / 2.0
    tol = 0.08
    return [
        e
        for e in part.edges()
        if abs(abs(e.center().Z) - half_t) < tol
    ]


def get_clear_span_bounds(req: LinkDesignRequest) -> tuple[float, float]:
    """Return safe [x_start, x_end] along X between the root and tip boss keep-out zones."""
    r_a, r_b = get_end_radii(req)
    boss_margin_a = max(3.0, round_to_nozzle(3.2, req.nozzle_diameter_mm, min_lines=5))
    boss_margin_b = max(3.0, round_to_nozzle(3.2, req.nozzle_diameter_mm, min_lines=5))

    x_start = r_a + boss_margin_a
    x_end = req.length_mm - r_b - boss_margin_b
    return x_start, x_end
