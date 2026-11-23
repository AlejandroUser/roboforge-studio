"""Parametric 3D generators for cycloidal reducers, planetary gearboxes, and closed GT2/HTD belt loops."""

from __future__ import annotations

import math

from build123d import (
    BuildLine,
    BuildPart,
    BuildSketch,
    Circle,
    Compound,
    Cylinder,
    Line,
    Location,
    Locations,
    Mode,
    Part,
    Plane,
    Polygon,
    ThreePointArc,
    extrude,
    make_face,
)

_CYCLOIDAL_CACHE: dict[str, Part | Compound] = {}


def _epitrochoid_points(
    n_lobes: int,
    pin_circle_r: float,
    roller_r: float,
    eccentricity: float,
    samples_per_lobe: int = 8,
) -> list[tuple[float, float]]:
    """Sample exact 2D epitrochoid cycloidal disk contour points."""
    n_pins = n_lobes + 1
    total_pts = max(48, n_lobes * samples_per_lobe)
    pts: list[tuple[float, float]] = []
    ratio_term = pin_circle_r / max(eccentricity * n_pins, 1e-6)

    for i in range(total_pts):
        t = (2.0 * math.pi * i) / total_pts
        psi = math.atan2(
            math.sin((1 - n_pins) * t),
            ratio_term - math.cos((1 - n_pins) * t),
        )
        x = (
            pin_circle_r * math.cos(t)
            - roller_r * math.cos(t + psi)
            - eccentricity * math.cos(n_pins * t)
        )
        y = (
            pin_circle_r * math.sin(t)
            - roller_r * math.sin(t + psi)
            - eccentricity * math.sin(n_pins * t)
        )
        pts.append((x, y))
    return pts


def build_cycloidal_reducer_3d(
    ratio: int = 25,
    outer_d_mm: float = 68.0,
    thickness_mm: float = 14.0,
    clearance_mm: float = 0.2,
) -> Compound:
    """Build a realistic dual-disk 180°-phased cycloidal reducer centered at (0,0,0) with axis along Z.
    Includes outer pin-ring housing, roller pins, dual epitrochoid disks, eccentric bore, and output pins.
    """
    n_lobes = max(7, min(25, int(ratio)))
    od = round(max(40.0, float(outer_d_mm)), 1)
    t = round(max(8.0, float(thickness_mm)), 1)
    clr = round(max(0.05, float(clearance_mm)), 2)

    cache_key = f"cycloidal:{n_lobes}:{od}:{t}:{clr}"
    if cache_key in _CYCLOIDAL_CACHE:
        return _CYCLOIDAL_CACHE[cache_key]  # type: ignore[return-value]

    r_outer = od / 2.0
    pin_circle_r = r_outer * 0.78
    roller_r = max(1.6, min(3.2, (math.pi * pin_circle_r / (n_lobes + 1)) * 0.42))
    ecc = max(0.7, min(roller_r * 0.55, 1.4))
    disk_t = max(3.0, (t - 2.0) / 2.0)

    # 1. Single epitrochoid cycloidal disk with eccentric center bore + 6 output pin holes
    pts = _epitrochoid_points(
        n_lobes=n_lobes,
        pin_circle_r=pin_circle_r,
        roller_r=roller_r + clr,
        eccentricity=ecc,
        samples_per_lobe=6,
    )
    with BuildSketch(Plane.XY) as sk:
        Polygon(*pts, align=None)
        # Central eccentric bearing bore
        Circle(r_outer * 0.24, mode=Mode.SUBTRACT)
        # 6 output pin holes
        out_pcd_r = r_outer * 0.48
        out_hole_r = max(2.2, r_outer * 0.08 + ecc)
        for k in range(6):
            ang = (2.0 * math.pi * k) / 6.0
            with Locations((out_pcd_r * math.cos(ang), out_pcd_r * math.sin(ang))):
                Circle(out_hole_r, mode=Mode.SUBTRACT)
    disk_face = sk.sketch.faces()[0]

    with BuildPart() as d1_bp:
        extrude(disk_face.moved(Location((ecc, 0.0, -disk_t - 0.3))), amount=disk_t)
    # Second disk rotated 180° around Z and offset -ecc for dynamic balance
    with BuildPart() as d2_bp:
        extrude(
            disk_face.moved(Location((-ecc, 0.0, 0.3), (0.0, 0.0, 180.0))),
            amount=disk_t,
        )

    # 2. Outer pin-ring housing + visual roller pins
    ring_housing = Cylinder(radius=r_outer, height=t) - Cylinder(
        radius=pin_circle_r + roller_r * 0.4, height=t + 2.0
    )
    n_pins = n_lobes + 1
    pins: list[Part] = []
    # Cap visual roller pins to <= 20 for fast tessellation
    step = max(1, n_pins // 18)
    for k in range(0, n_pins, step):
        ang = (2.0 * math.pi * k) / n_pins
        pins.append(
            Location((pin_circle_r * math.cos(ang), pin_circle_r * math.sin(ang), 0.0))
            * Cylinder(radius=roller_r * 0.85, height=t - 0.8)
        )

    # 3. Output carrier pins (6 pins)
    out_pcd_r = r_outer * 0.48
    out_pin_r = max(1.8, r_outer * 0.075)
    for k in range(6):
        ang = (2.0 * math.pi * k) / 6.0
        pins.append(
            Location((out_pcd_r * math.cos(ang), out_pcd_r * math.sin(ang), 0.5))
            * Cylinder(radius=out_pin_r, height=t + 1.2)
        )

    reducer = Compound(children=[ring_housing, d1_bp.part, d2_bp.part, *pins])
    _CYCLOIDAL_CACHE[cache_key] = reducer
    return reducer


def _simple_gear_solid(teeth: int, r_pitch: float, thickness: float, bore_r: float) -> Part:
    """Fast toothed gear solid for planetary sun/planet stages."""
    m = (2.0 * r_pitch) / max(teeth, 6)
    r_o = r_pitch + 0.85 * m
    r_r = max(bore_r + 1.0, r_pitch - 1.0 * m)
    pitch_ang = 2.0 * math.pi / teeth
    pts: list[tuple[float, float]] = []
    for i in range(teeth):
        a0 = i * pitch_ang
        for frac, rad in (
            (0.00, r_r),
            (0.18, r_o),
            (0.42, r_o),
            (0.60, r_r),
        ):
            a = a0 + frac * pitch_ang
            pts.append((rad * math.cos(a), rad * math.sin(a)))
    with BuildSketch(Plane.XY) as sk:
        Polygon(*pts, align=None)
        if bore_r > 0.6:
            Circle(bore_r, mode=Mode.SUBTRACT)
    with BuildPart() as bp:
        extrude(sk.sketch.faces()[0], amount=thickness / 2.0, both=True)
    return bp.part


def build_planetary_reducer_3d(
    ratio: int = 10,
    outer_d_mm: float = 56.0,
    thickness_mm: float = 16.0,
) -> Compound:
    """Build a realistic planetary gearbox (sun gear + 3 planets on carrier + outer ring housing)."""
    od = round(max(36.0, float(outer_d_mm)), 1)
    t = round(max(10.0, float(thickness_mm)), 1)
    r_key = max(4, min(40, int(ratio)))

    cache_key = f"planetary:{r_key}:{od}:{t}"
    if cache_key in _CYCLOIDAL_CACHE:
        return _CYCLOIDAL_CACHE[cache_key]  # type: ignore[return-value]

    r_outer = od / 2.0
    r_ring_in = r_outer * 0.80
    r_sun = r_ring_in * 0.28
    r_planet = (r_ring_in - r_sun) / 2.0
    r_orbit = r_sun + r_planet
    gear_t = max(5.0, t - 4.0)

    # Outer flanged ring gear housing
    ring = Cylinder(radius=r_outer, height=t) - Cylinder(radius=r_ring_in, height=t + 2.0)

    # Sun gear
    sun = _simple_gear_solid(teeth=12, r_pitch=r_sun, thickness=gear_t, bore_r=max(2.5, r_sun * 0.42))

    # 3 Planet gears + carrier pins spaced at 120°
    planets: list[Part] = []
    for k in range(3):
        ang = (2.0 * math.pi * k) / 3.0
        px = r_orbit * math.cos(ang)
        py = r_orbit * math.sin(ang)
        pl = _simple_gear_solid(teeth=14, r_pitch=r_planet, thickness=gear_t, bore_r=max(2.0, r_planet * 0.35))
        planets.append(Location((px, py, 0.0)) * pl)
        planets.append(Location((px, py, 0.0)) * Cylinder(radius=max(1.8, r_planet * 0.30), height=t - 1.0))

    # Planet carrier ring on +Z side
    carrier_z = (t / 2.0) - 1.5
    carrier = Location((0.0, 0.0, carrier_z)) * (
        Cylinder(radius=r_orbit + 3.0, height=2.6) - Cylinder(radius=max(3.0, r_sun * 0.9), height=4.0)
    )

    reducer = Compound(children=[ring, sun, carrier, *planets])
    _CYCLOIDAL_CACHE[cache_key] = reducer
    return reducer


def build_closed_belt_loop_3d(
    center_dist_mm: float = 120.0,
    r1_mm: float = 14.0,
    r2_mm: float = 24.0,
    width_mm: float = 6.0,
) -> Part:
    """Build a 3D closed GT2/HTD belt loop tangent to pulley 1 at (0,0) and pulley 2 at (C,0)."""
    c = round(max(20.0, float(center_dist_mm)), 1)
    r1 = round(max(4.0, float(r1_mm)), 1)
    r2 = round(max(4.0, float(r2_mm)), 1)
    w = round(max(3.0, float(width_mm)), 1)
    belt_t = 1.8

    cache_key = f"belt:{c}:{r1}:{r2}:{w}"
    if cache_key in _CYCLOIDAL_CACHE:
        return _CYCLOIDAL_CACHE[cache_key]  # type: ignore[return-value]

    def _stadium_face(ra: float, rb: float) -> object:
        sin_a = max(-0.9, min(0.9, (ra - rb) / max(c, 1e-3)))
        cos_a = math.sqrt(max(1e-6, 1.0 - sin_a * sin_a))
        p1_top = (ra * sin_a, ra * cos_a)
        p2_top = (c + rb * sin_a, rb * cos_a)
        p2_bot = (c + rb * sin_a, -rb * cos_a)
        p1_bot = (ra * sin_a, -ra * cos_a)
        with BuildSketch(Plane.XY) as sk:
            with BuildLine():
                ThreePointArc(p1_bot, (-ra, 0.0), p1_top)
                Line(p1_top, p2_top)
                ThreePointArc(p2_top, (c + rb, 0.0), p2_bot)
                Line(p2_bot, p1_bot)
            make_face()
        return sk.sketch.faces()[0]

    outer_face = _stadium_face(r1 + belt_t, r2 + belt_t)
    inner_face = _stadium_face(r1, r2)

    with BuildPart() as out_bp:
        extrude(outer_face, amount=w / 2.0, both=True)
    with BuildPart() as in_bp:
        extrude(inner_face, amount=(w + 2.0) / 2.0, both=True)

    belt = out_bp.part - in_bp.part
    _CYCLOIDAL_CACHE[cache_key] = belt
    return belt
