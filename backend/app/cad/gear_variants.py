"""Parametric CAD generators for Family 3: Transmission (Involute Spur Gears & GT2 Pulleys)."""

from __future__ import annotations

import math
from typing import Any

from build123d import (
    BuildPart,
    BuildSketch,
    Circle,
    Cylinder,
    Location,
    Locations,
    Mode,
    Part,
    Plane,
    Polygon,
    Rectangle,
    extrude,
    fillet,
)

from ..schemas import GearDesignRequest, GearVariantKind
from .base_link import round_to_nozzle


def _involute_gear_profile_points(req: GearDesignRequest) -> tuple[list[tuple[float, float]], float, float]:
    """Compute exact 2D involute spur gear contour points and return (points, r_root, r_outer)."""
    z = req.teeth
    m = req.module_mm
    alpha = math.radians(req.pressure_angle_deg)
    backlash = req.backlash_mm

    r_p = 0.5 * m * z
    r_b = r_p * math.cos(alpha)
    r_o = r_p + 1.0 * m
    r_r = max(2.5, r_p - 1.25 * m)

    # Half tooth angular thickness at pitch circle compensated by FDM backlash
    nominal_arc = max(0.25 * math.pi * m, 0.5 * math.pi * m - backlash)
    half_tooth_pitch = nominal_arc / (2.0 * r_p)
    inv_alpha = math.tan(alpha) - alpha
    psi_b = half_tooth_pitch + inv_alpha

    r_start = max(r_r, r_b)
    t_start = math.sqrt(max(0.0, (r_start / r_b) ** 2 - 1.0)) if r_b > 1e-6 else 0.0
    t_end = math.sqrt(max(0.0, (r_o / r_b) ** 2 - 1.0)) if r_b > 1e-6 else 0.0

    # Prevent involute flanks from crossing at sharp tips on low tooth counts
    def _inv(t_val: float) -> float:
        return t_val - math.atan(t_val)

    if _inv(t_end) >= psi_b * 0.96:
        # Bisect t_end so tip still has finite land width
        lo, hi = t_start, t_end
        target = psi_b * 0.94
        for _ in range(18):
            mid = 0.5 * (lo + hi)
            if _inv(mid) < target:
                lo = mid
            else:
                hi = mid
        t_end = lo

    n_inv = 5
    pitch_angle = 2.0 * math.pi / z
    pts: list[tuple[float, float]] = []

    for i in range(z):
        theta_c = i * pitch_angle
        # Root start on right side of tooth
        ang_r_root = theta_c - psi_b + (_inv(t_start) if r_start > r_b else 0.0)
        if r_r < r_b - 1e-4:
            # Radial fillet segment from root circle up to base circle
            pts.append((r_r * math.cos(ang_r_root), r_r * math.sin(ang_r_root)))

        # Right flank: from r_start up to r_o
        for k in range(n_inv):
            t_val = t_start + (t_end - t_start) * (k / max(n_inv - 1, 1))
            r_val = r_b * math.sqrt(1.0 + t_val * t_val)
            ang = theta_c - psi_b + _inv(t_val)
            pts.append((r_val * math.cos(ang), r_val * math.sin(ang)))

        # Left flank: from r_o down to r_start
        for k in range(n_inv - 1, -1, -1):
            t_val = t_start + (t_end - t_start) * (k / max(n_inv - 1, 1))
            r_val = r_b * math.sqrt(1.0 + t_val * t_val)
            ang = theta_c + psi_b - _inv(t_val)
            pts.append((r_val * math.cos(ang), r_val * math.sin(ang)))

        if r_r < r_b - 1e-4:
            ang_l_root = theta_c + psi_b - (_inv(t_start) if r_start > r_b else 0.0)
            pts.append((r_r * math.cos(ang_l_root), r_r * math.sin(ang_l_root)))

        # Midpoint along root valley to next tooth
        ang_valley = theta_c + 0.5 * pitch_angle
        pts.append((r_r * math.cos(ang_valley), r_r * math.sin(ang_valley)))

    return pts, r_r, r_o


def _gt2_pulley_profile_points(req: GearDesignRequest) -> tuple[list[tuple[float, float]], float, float]:
    """Compute 2D GT2 2mm-pitch timing pulley profile points and return (points, r_root, r_outer)."""
    z = req.teeth
    pitch = 2.0
    d_p = (pitch * z) / math.pi
    r_p = d_p / 2.0
    r_o = max(4.0, r_p - 0.254)
    groove_depth = 0.75 + req.backlash_mm * 0.25
    r_r = max(2.4, r_o - groove_depth)

    pitch_angle = 2.0 * math.pi / z
    pts: list[tuple[float, float]] = []

    # 6 smooth points per tooth pitch: outer land + rounded groove valley
    # Fractional phase within each tooth period and radial profile
    tooth_samples = [
        (0.00, r_o),
        (0.24, r_o),
        (0.36, r_r + groove_depth * 0.25),
        (0.50, r_r),
        (0.64, r_r + groove_depth * 0.25),
        (0.76, r_o),
    ]

    for i in range(z):
        base_ang = i * pitch_angle
        for frac, rad in tooth_samples:
            ang = base_ang + frac * pitch_angle
            pts.append((rad * math.cos(ang), rad * math.sin(ang)))

    return pts, r_r, r_o


def _build_base_gear_body(
    req: GearDesignRequest,
    compact_hub: bool = False,
) -> tuple[Part, float, float, float]:
    """Build the solid toothed gear or GT2 pulley + optional hub.
    Returns (part, r_root, r_outer, effective_hub_r).
    """
    fw = req.face_width_mm

    if req.gear_kind == "gt2_pulley":
        pts, r_r, r_o = _gt2_pulley_profile_points(req)
        flange_t = min(1.5, fw * 0.16)
        flange_r = r_o + 1.2
        with BuildSketch(Plane.XY) as sk:
            Polygon(*pts, align=None)
        tooth_face = sk.sketch.faces()[0]
        with BuildPart() as drum_bp:
            extrude(tooth_face, amount=fw)
        part = drum_bp.part
        # Bottom and top belt-retaining flanges
        bot_flange = Location((0.0, 0.0, flange_t / 2.0)) * Cylinder(radius=flange_r, height=flange_t)
        top_flange = Location((0.0, 0.0, fw - flange_t / 2.0)) * Cylinder(radius=flange_r, height=flange_t)
        part = part + bot_flange + top_flange
    else:
        pts, r_r, r_o = _involute_gear_profile_points(req)
        with BuildSketch(Plane.XY) as sk:
            Polygon(*pts, align=None)
        tooth_face = sk.sketch.faces()[0]
        with BuildPart() as gear_bp:
            extrude(tooth_face, amount=fw)
        part = gear_bp.part

    # Optional cylindrical hub on top face (z = fw .. fw + hub_height)
    bore_r = (req.bore_diameter_mm + req.printer_clearance_mm) / 2.0
    max_hub_r = max(bore_r + 1.8, r_r - 0.6)
    if compact_hub:
        max_hub_r = min(max_hub_r, max(bore_r + 2.8, r_r * 0.56))
    hub_r = min(req.hub_diameter_mm / 2.0, max_hub_r)
    if req.hub_height_mm >= 0.5 and hub_r > bore_r + 1.0:
        hub = Location((0.0, 0.0, fw + req.hub_height_mm / 2.0)) * Cylinder(
            radius=hub_r, height=req.hub_height_mm
        )
        part = part + hub
    else:
        hub_r = min(hub_r, bore_r + 2.5)

    return part, r_r, r_o, hub_r


def _apply_shaft_and_setscrew(
    part: Part,
    req: GearDesignRequest,
    r_r: float,
    hub_r: float,
) -> Part:
    """Cut central shaft bore (with optional D-shaft flat) and radial M3 set-screw hole in hub."""
    total_h = req.face_width_mm + req.hub_height_mm
    cut_h = total_h + 6.0
    zc = total_h / 2.0

    bore_r = (req.bore_diameter_mm + req.printer_clearance_mm) / 2.0
    bore_r = min(bore_r, max(1.2, r_r - 1.2))

    # Build bore cutter (D-shaft or round)
    flat = req.d_shaft_flat_mm
    if flat > 0.05 and flat < bore_r * 0.55:
        with BuildSketch(Plane.XY) as sk:
            Circle(bore_r)
            # Remove the flat chord segment on +X side so the solid keeps the D-flat
            clip_w = bore_r * 2.0
            clip_x = (bore_r - flat) + clip_w / 2.0
            with Locations((clip_x, 0.0)):
                Rectangle(clip_w, bore_r * 2.4, mode=Mode.SUBTRACT)
        with BuildPart() as bore_bp:
            extrude(sk.sketch.faces()[0], amount=cut_h / 2.0, both=True)
        bore_cutter = Location((0.0, 0.0, zc)) * bore_bp.part
    else:
        bore_cutter = Location((0.0, 0.0, zc)) * Cylinder(radius=bore_r, height=cut_h)

    try:
        cand = part - bore_cutter
        if cand.is_valid and cand.volume > 0:
            part = cand
    except Exception:
        pass

    # Radial M3 set-screw hole through hub along +X (hits D-shaft flat!)
    if req.hub_height_mm >= 4.0 and hub_r > bore_r + 1.8:
        screw_r = (2.9 + req.printer_clearance_mm) / 2.0
        screw_z = req.face_width_mm + req.hub_height_mm / 2.0
        screw_len = hub_r + 4.0
        screw_cutter = (
            Location((screw_len / 2.0, 0.0, screw_z), (0.0, 90.0, 0.0))
            * Cylinder(radius=screw_r, height=screw_len)
        )
        try:
            cand = part - screw_cutter
            if cand.is_valid and cand.volume > 0:
                part = cand
        except Exception:
            pass

    return part


def _build_gear_solid(req: GearDesignRequest) -> Part:
    """Variant 1: Solid gear or GT2 pulley baseline."""
    part, r_r, _, hub_r = _build_base_gear_body(req)
    return _apply_shaft_and_setscrew(part, req, r_r, hub_r)


def _build_gear_webbed(req: GearDesignRequest) -> Part:
    """Variant 2: Recessed annular pockets on both faces between hub and rim."""
    part, r_r, _, hub_r = _build_base_gear_body(req, compact_hub=True)
    fw = req.face_width_mm
    bore_r = (req.bore_diameter_mm + req.printer_clearance_mm) / 2.0

    rim_wall = (
        1.6
        if req.gear_kind == "gt2_pulley"
        else max(2.2, round_to_nozzle(max(2.4, 5.0 * req.nozzle_diameter_mm), req.nozzle_diameter_mm))
    )
    r_in = max(hub_r + 0.8, bore_r + 2.4)
    r_out = r_r - rim_wall

    if r_out - r_in >= 2.0 and fw >= 5.0:
        web_t = max(1.8, round_to_nozzle(fw * 0.34, req.nozzle_diameter_mm, min_lines=4))
        pocket_depth = (fw - web_t) / 2.0
        if pocket_depth >= 1.0:
            with BuildSketch(Plane.XY) as sk:
                Circle(r_out)
                Circle(r_in, mode=Mode.SUBTRACT)
            ring_face = sk.sketch.faces()[0]

            # Bottom pocket: z in [-1, pocket_depth]
            with BuildPart() as bot_bp:
                extrude(ring_face.moved(Location((0.0, 0.0, -1.0))), amount=pocket_depth + 1.0)
            # Top pocket: z in [fw - pocket_depth, fw + 1]
            with BuildPart() as top_bp:
                extrude(
                    ring_face.moved(Location((0.0, 0.0, fw - pocket_depth))),
                    amount=pocket_depth + 1.0,
                )
            for pocket in (bot_bp.part, top_bp.part):
                try:
                    cand = part - pocket
                    if cand.is_valid and cand.volume > 0:
                        part = cand
                except Exception:
                    continue

    return _apply_shaft_and_setscrew(part, req, r_r, hub_r)


def _build_gear_spoked(req: GearDesignRequest) -> Part:
    """Variant 3: Through-cut rounded bionic sector cutouts (5–6 spokes) for minimal inertia."""
    part, r_r, _, hub_r = _build_base_gear_body(req, compact_hub=True)
    fw = req.face_width_mm
    bore_r = (req.bore_diameter_mm + req.printer_clearance_mm) / 2.0

    rim_wall = (
        1.6
        if req.gear_kind == "gt2_pulley"
        else max(2.4, round_to_nozzle(max(2.6, 5.5 * req.nozzle_diameter_mm), req.nozzle_diameter_mm))
    )
    r_in = max(hub_r + 0.8, bore_r + 2.4)
    r_out = r_r - rim_wall

    carved = 0
    if r_out - r_in >= 3.2:
        n_spokes = 6 if r_out >= 22.0 else 5
        spoke_w = max(2.6, round_to_nozzle(max(3.0, 6.0 * req.nozzle_diameter_mm), req.nozzle_diameter_mm))
        sector_angle = 2.0 * math.pi / n_spokes

        # Angular half-gap for spoke width at inner & outer radii
        half_ang_in = math.asin(min(0.45, (spoke_w / 2.0) / max(r_in, 1.0)))
        half_ang_out = math.asin(min(0.45, (spoke_w / 2.0) / max(r_out, 1.0)))

        span_in = sector_angle - 2.0 * half_ang_in
        span_out = sector_angle - 2.0 * half_ang_out
        cut_h = fw + 4.0

        if span_in > 0.18 and span_out > 0.22:
            for i in range(n_spokes):
                ang_mid = i * sector_angle
                # Build rounded sector polygon (outer arc samples + inner arc samples)
                pts: list[tuple[float, float]] = []
                n_arc = 5
                for k in range(n_arc):
                    a = (ang_mid - span_out / 2.0) + span_out * (k / (n_arc - 1))
                    pts.append((r_out * math.cos(a), r_out * math.sin(a)))
                for k in range(n_arc - 1, -1, -1):
                    a = (ang_mid - span_in / 2.0) + span_in * (k / (n_arc - 1))
                    pts.append((r_in * math.cos(a), r_in * math.sin(a)))

                try:
                    with BuildSketch(Plane.XY) as sk:
                        Polygon(*pts, align=None)
                        r_fil = min(2.2, (r_out - r_in) * 0.22)
                        if r_fil >= 0.45:
                            try:
                                fillet(sk.vertices(), radius=r_fil)
                            except Exception:
                                pass
                    with BuildPart() as cut_bp:
                        extrude(
                            sk.sketch.faces()[0].moved(Location((0.0, 0.0, -2.0))),
                            amount=cut_h,
                        )
                    cand = part - cut_bp.part
                    if cand.is_valid and cand.volume > 0:
                        part = cand
                        carved += 1
                except Exception:
                    continue

    # Safe fallback for compact gears/pulleys: circular lightening holes around mid-radius
    if carved == 0 and (r_out - r_in) >= 2.2:
        r_mid = 0.5 * (r_in + r_out)
        hole_r = min((r_out - r_in) * 0.42, (math.pi * r_mid / 6.0) * 0.65)
        if hole_r >= 1.2:
            n_holes = 5
            for i in range(n_holes):
                ang = (2.0 * math.pi * i) / n_holes
                hx = r_mid * math.cos(ang)
                hy = r_mid * math.sin(ang)
                cutter = Location((hx, hy, fw / 2.0)) * Cylinder(radius=hole_r, height=fw + 4.0)
                try:
                    cand = part - cutter
                    if cand.is_valid and cand.volume > 0:
                        part = cand
                        carved += 1
                except Exception:
                    continue

    return _apply_shaft_and_setscrew(part, req, r_r, hub_r)


def _part_metrics(part: Part) -> tuple[float, bool]:
    is_valid = bool(part.is_valid)
    volume_mm3 = float(part.volume) if part.volume and part.volume > 0 else 0.0
    return volume_mm3, is_valid and volume_mm3 > 0.0


def generate_all_gear_variants(
    req: GearDesignRequest,
) -> dict[GearVariantKind, dict[str, Any]]:
    """Generate all 3 transmission variants (solid, webbed, spoked) with metadata."""
    kind_label = "Polea GT2" if req.gear_kind == "gt2_pulley" else "Engranaje Involuta"
    specs: list[tuple[GearVariantKind, str, str, Any]] = [
        (
            "gear_solid",
            f"{kind_label} Sólida",
            "Disco sólido base de máxima rigidez y resistencia torsional",
            _build_gear_solid,
        ),
        (
            "gear_webbed",
            "Alma Rebajada Aligerada",
            "Rebajes anulares en ambas caras entre cubo y corona para menor inercia",
            _build_gear_webbed,
        ),
        (
            "gear_spoked",
            "Radios Biónicos Aligerados",
            "Vaciados sectoriales redondeados pasantes para mínima inercia sin soportes",
            _build_gear_spoked,
        ),
    ]

    results: dict[GearVariantKind, dict[str, Any]] = {}
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
