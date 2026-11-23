"""Parametric 3D mechanical hardware builders (ISO 4762 screws, ball bearings, motors, MCU box)."""

from __future__ import annotations

import math

from build123d import (
    BuildPart,
    BuildSketch,
    Cylinder,
    Location,
    Mode,
    Part,
    Plane,
    RectangleRounded,
    RegularPolygon,
    extrude,
)

# In-memory cache keyed by (kind, spec) so hardware solids build once and reuse instantaneously
_HARDWARE_CACHE: dict[str, Part] = {}

ISO4762_SPECS: dict[str, tuple[float, float, float, float, float]] = {
    # metric: (shank_d, head_d, head_h, hex_af, hex_depth)
    "M3": (3.0, 5.5, 3.0, 2.5, 1.6),
    "M4": (4.0, 7.0, 4.0, 3.0, 2.2),
    "M5": (5.0, 8.5, 5.0, 4.0, 2.8),
    "M6": (6.0, 10.0, 6.0, 5.0, 3.3),
}

BEARING_SPECS: dict[str, tuple[float, float, float]] = {
    # code: (inner_d, outer_d, width)
    "608": (8.0, 22.0, 7.0),
    "6002": (15.0, 32.0, 9.0),
    "6805": (25.0, 37.0, 7.0),
    "6808": (40.0, 52.0, 7.0),
}


def build_iso4762_screw(metric: str = "M3", length_mm: float = 12.0) -> Part:
    """Build a realistic ISO 4762 socket-head cap screw.
    Seating plane under head is at z = 0; shank extends to -length_mm, head to +head_h.
    """
    m_key = metric.upper() if metric.upper() in ISO4762_SPECS else "M3"
    length = round(max(4.0, float(length_mm)), 1)
    cache_key = f"screw:{m_key}:{length}"
    if cache_key in _HARDWARE_CACHE:
        return _HARDWARE_CACHE[cache_key]

    shank_d, head_d, head_h, hex_af, hex_depth = ISO4762_SPECS[m_key]

    shank = Location((0.0, 0.0, -length / 2.0)) * Cylinder(radius=shank_d / 2.0, height=length)
    head = Location((0.0, 0.0, head_h / 2.0)) * Cylinder(radius=head_d / 2.0, height=head_h)
    screw = shank + head

    # Recessed hexagonal Allen socket at top of head
    hex_r = hex_af / math.sqrt(3.0)
    try:
        with BuildSketch(Plane.XY) as sk:
            RegularPolygon(radius=hex_r, side_count=6)
        with BuildPart() as hex_bp:
            extrude(
                sk.sketch.faces()[0].moved(Location((0.0, 0.0, head_h - hex_depth))),
                amount=hex_depth + 0.6,
            )
        cand = screw - hex_bp.part
        if cand.is_valid and cand.volume > 0:
            screw = cand
    except Exception:
        pass

    _HARDWARE_CACHE[cache_key] = screw
    return screw


def build_ball_bearing(bearing_code: str = "608") -> Part:
    """Build a realistic deep-groove ball bearing centered at (0, 0, 0) with axis along Z."""
    code = bearing_code if bearing_code in BEARING_SPECS else "608"
    cache_key = f"bearing:{code}"
    if cache_key in _HARDWARE_CACHE:
        return _HARDWARE_CACHE[cache_key]

    id_mm, od_mm, width = BEARING_SPECS[code]
    r_in = id_mm / 2.0
    r_out = od_mm / 2.0
    radial_span = max(2.0, r_out - r_in)

    # Outer race, inner race, and slightly recessed seal/shield ring
    race_w = radial_span * 0.28
    outer_race = Cylinder(radius=r_out, height=width) - Cylinder(radius=r_out - race_w, height=width + 2.0)
    inner_race = Cylinder(radius=r_in + race_w, height=width) - Cylinder(radius=r_in, height=width + 2.0)
    shield_h = max(1.5, width - 1.4)
    shield = Cylinder(radius=r_out - race_w + 0.25, height=shield_h) - Cylinder(
        radius=r_in + race_w - 0.25, height=shield_h + 2.0
    )

    bearing = outer_race + inner_race + shield
    _HARDWARE_CACHE[cache_key] = bearing
    return bearing


def _build_nema_body(
    side_mm: float,
    length_mm: float,
    pilot_d: float,
    pilot_h: float,
    shaft_d: float,
    shaft_len: float,
    has_encoder_cap: bool = False,
) -> Part:
    """Build a NEMA square stepper body with chamfered corners, pilot boss, shaft, and optional rear encoder."""
    corner_r = max(2.5, side_mm * 0.08)
    with BuildSketch(Plane.XY) as sk:
        RectangleRounded(side_mm, side_mm, radius=corner_r)
    face = sk.sketch.faces()[0]

    # Body extends along -Z from z=0 down to z=-length_mm
    with BuildPart() as body_bp:
        extrude(face.moved(Location((0.0, 0.0, -length_mm))), amount=length_mm)
    motor = body_bp.part

    # Front pilot boss and output shaft along +Z
    pilot = Location((0.0, 0.0, pilot_h / 2.0)) * Cylinder(radius=pilot_d / 2.0, height=pilot_h)
    shaft = Location((0.0, 0.0, shaft_len / 2.0)) * Cylinder(radius=shaft_d / 2.0, height=shaft_len)
    motor = motor + pilot + shaft

    # Closed-loop servo rear encoder / driver PCB housing (e.g. MKS SERVO42D / 57D)
    if has_encoder_cap:
        cap_len = 11.0
        with BuildSketch(Plane.XY) as cap_sk:
            RectangleRounded(side_mm * 0.92, side_mm * 0.92, radius=corner_r * 0.8)
        with BuildPart() as cap_bp:
            extrude(cap_sk.sketch.faces()[0].moved(Location((0.0, 0.0, -length_mm - cap_len))), amount=cap_len)
        motor = motor + cap_bp.part

    return motor


def build_motor_3d(motor_id: str = "nema17_48mm") -> Part:
    """Build a realistic 3D motor solid (NEMA 17/23 or cylindrical BLDC QDD).
    Mounting faceplate lies on z = 0, body extends along -Z, pilot & shaft along +Z.
    """
    cache_key = f"motor:{motor_id}"
    if cache_key in _HARDWARE_CACHE:
        return _HARDWARE_CACHE[cache_key]

    mid = (motor_id or "nema17_48mm").lower()

    if "nema23" in mid:
        motor = _build_nema_body(
            side_mm=57.2,
            length_mm=76.0,
            pilot_d=38.1,
            pilot_h=2.0,
            shaft_d=6.35,
            shaft_len=21.0,
            has_encoder_cap=("mks" in mid or "servo" in mid or "closed" in mid),
        )
    elif "bldc_qdd_8308" in mid or "8308" in mid:
        # Heavy cylindrical QDD actuator (Ø92 × 42 mm)
        body = Location((0.0, 0.0, -21.0)) * Cylinder(radius=46.0, height=42.0)
        collar = Location((0.0, 0.0, -5.0)) * Cylinder(radius=48.0, height=10.0)
        pilot = Location((0.0, 0.0, 1.5)) * Cylinder(radius=21.0, height=3.0)
        shaft = Location((0.0, 0.0, 8.0)) * Cylinder(radius=7.0, height=16.0)
        motor = body + collar + pilot + shaft
    elif "bldc" in mid or "qdd" in mid or "5010" in mid:
        # Compact cylindrical QDD actuator (Ø56 × 32 mm)
        body = Location((0.0, 0.0, -16.0)) * Cylinder(radius=28.0, height=32.0)
        collar = Location((0.0, 0.0, -4.0)) * Cylinder(radius=29.5, height=8.0)
        pilot = Location((0.0, 0.0, 1.25)) * Cylinder(radius=14.0, height=2.5)
        shaft = Location((0.0, 0.0, 7.0)) * Cylinder(radius=5.0, height=14.0)
        motor = body + collar + pilot + shaft
    else:
        # Default: NEMA 17 (42.3 × 42.3 × 48 mm)
        motor = _build_nema_body(
            side_mm=42.3,
            length_mm=48.0,
            pilot_d=22.0,
            pilot_h=2.0,
            shaft_d=5.0,
            shaft_len=20.0,
            has_encoder_cap=("mks" in mid or "servo" in mid or "closed" in mid),
        )

    _HARDWARE_CACHE[cache_key] = motor
    return motor


def build_controller_box() -> Part:
    """Build a compact industrial STM32/CAN-FD controller box with XT60 power and CAN-FD bus ports.
    Rests on z = 0..26 mm, centered in XY.
    """
    cache_key = "controller_box:v1"
    if cache_key in _HARDWARE_CACHE:
        return _HARDWARE_CACHE[cache_key]

    bx, by, bz = 74.0, 52.0, 26.0
    with BuildSketch(Plane.XY) as sk:
        RectangleRounded(bx, by, radius=4.5)
    with BuildPart() as base_bp:
        extrude(sk.sketch.faces()[0], amount=bz - 4.0)

    # Stepped top lid
    with BuildSketch(Plane.XY) as lid_sk:
        RectangleRounded(bx - 3.0, by - 3.0, radius=3.5)
    with BuildPart() as lid_bp:
        extrude(lid_sk.sketch.faces()[0].moved(Location((0.0, 0.0, bz - 4.0))), amount=4.0)

    # XT60 power connector shroud on +X side
    with BuildSketch(Plane.YZ) as xt_sk:
        RectangleRounded(16.0, 9.0, radius=1.5)
    with BuildPart() as xt_bp:
        extrude(xt_sk.sketch.faces()[0].moved(Location((bx / 2.0 - 1.0, 0.0, 12.0))), amount=9.0)

    # CAN-FD / USB-C bus ports on -Y front side
    port1 = Location((-14.0, -by / 2.0 - 2.5, 12.0), (90.0, 0.0, 0.0)) * Cylinder(radius=4.5, height=7.0)
    port2 = Location((14.0, -by / 2.0 - 2.5, 12.0), (90.0, 0.0, 0.0)) * Cylinder(radius=4.5, height=7.0)

    box = base_bp.part + lid_bp.part + xt_bp.part + port1 + port2
    _HARDWARE_CACHE[cache_key] = box
    return box
