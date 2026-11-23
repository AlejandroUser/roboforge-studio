"""Parametric 3D mechanical hardware builders (ISO 4762 screws, ball bearings, motors, MCU box)."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from build123d import (
    Box,
    BuildPart,
    BuildSketch,
    Circle,
    Compound,
    Cone,
    Cylinder,
    Location,
    Locations,
    Mode,
    Part,
    Plane,
    Polygon,
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

    if "mg90" in mid:
        return build_servo_mg90s_3d()

    if "sts3215" in mid:
        return build_servo_sts3215_3d()

    if "servo_td" in mid or "servo_ds" in mid or "mg996" in mid:
        return build_servo_3d(motor_id)

    if "nema14" in mid:
        return _build_nema_body(
            side_mm=35.2,
            length_mm=34.0,
            pilot_d=22.0,
            pilot_h=2.0,
            shaft_d=5.0,
            shaft_len=18.0,
            has_encoder_cap=False,
        )

    if "damiao" in mid and "4340" in mid:
        body = Location((0.0, 0.0, -23.0)) * Cylinder(radius=39.0, height=46.0)
        collar = Location((0.0, 0.0, -5.0)) * Cylinder(radius=41.0, height=10.0)
        pilot = Location((0.0, 0.0, 1.5)) * Cylinder(radius=22.0, height=3.0)
        shaft = Location((0.0, 0.0, 7.0)) * Cylinder(radius=8.0, height=14.0)
        motor = body + collar + pilot + shaft
        _HARDWARE_CACHE[cache_key] = motor
        return motor

    if "damiao" in mid or "4310" in mid:
        body = Location((0.0, 0.0, -18.0)) * Cylinder(radius=28.0, height=36.0)
        collar = Location((0.0, 0.0, -4.0)) * Cylinder(radius=29.5, height=8.0)
        pilot = Location((0.0, 0.0, 1.5)) * Cylinder(radius=16.0, height=3.0)
        shaft = Location((0.0, 0.0, 6.0)) * Cylinder(radius=6.0, height=12.0)
        motor = body + collar + pilot + shaft
        _HARDWARE_CACHE[cache_key] = motor
        return motor

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


def build_servo_3d(motor_id: str = "servo_td8120mg") -> Part:
    """Build a realistic standard RC servo (TD-8120MG / DS3218 / MG996R: 40 × 20 × 40.5 mm).
    Output 25T spline shaft is centered at (0, 0) extending along +Z (z = 0..+5 mm),
    top gearbox face lies at z = 0, body extends along -Z (z = -38..0 mm), offset +10 mm in X.
    """
    cache_key = f"servo:{motor_id}"
    if cache_key in _HARDWARE_CACHE:
        return _HARDWARE_CACHE[cache_key]

    # Body centered at x = +10 mm so the output spline sits at x = 0 (real 10 mm from front edge)
    with BuildSketch(Plane.XY) as body_sk:
        with Locations((10.0, 0.0)):
            RectangleRounded(40.0, 20.0, radius=1.8)
    with BuildPart() as body_bp:
        extrude(body_sk.sketch.faces()[0].moved(Location((0.0, 0.0, -38.0))), amount=38.0)

    # Mounting ears flange (54 × 20 × 2.6 mm) at z = -10.6..-8.0 mm with 4×Ø4.2 mm mounting holes
    with BuildSketch(Plane.XY) as ear_sk:
        with Locations((10.0, 0.0)):
            RectangleRounded(54.0, 20.0, radius=1.6)
    with BuildPart() as ear_bp:
        extrude(ear_sk.sketch.faces()[0].moved(Location((0.0, 0.0, -10.6))), amount=2.6)

    ears = ear_bp.part
    for hx in (-14.5, 34.5):
        for hy in (-5.0, 5.0):
            hole = Location((hx, hy, -9.3)) * Cylinder(radius=2.1, height=6.0)
            try:
                ears = ears - hole
            except Exception:
                pass

    # Upper output collar + 25T spline shaft (Ø6 mm) along +Z
    collar = Location((0.0, 0.0, 1.0)) * Cylinder(radius=5.8, height=2.0)
    spline = Location((0.0, 0.0, 3.0)) * Cylinder(radius=3.0, height=6.0)

    servo = body_bp.part + ears + collar + spline
    _HARDWARE_CACHE[cache_key] = servo
    return servo


def build_servo_mg90s_3d() -> Part:
    """Build a realistic MG90S micro-servo (22.8 × 12.2 × 28.5 mm, mounting ears 32.5 mm).
    Output spline shaft centered at (0, 0) along +Z, body extends along -Z.
    """
    cache_key = "servo:mg90s"
    if cache_key in _HARDWARE_CACHE:
        return _HARDWARE_CACHE[cache_key]

    with BuildSketch(Plane.XY) as body_sk:
        with Locations((5.4, 0.0)):
            RectangleRounded(22.8, 12.2, radius=1.2)
    with BuildPart() as body_bp:
        extrude(body_sk.sketch.faces()[0].moved(Location((0.0, 0.0, -25.0))), amount=25.0)

    # Mounting ears (32.5 × 12.2 × 2.0 mm) with 2×Ø2.2 mm holes
    with BuildSketch(Plane.XY) as ear_sk:
        with Locations((5.4, 0.0)):
            RectangleRounded(32.5, 12.2, radius=1.0)
    with BuildPart() as ear_bp:
        extrude(ear_sk.sketch.faces()[0].moved(Location((0.0, 0.0, -8.5))), amount=2.0)

    ears = ear_bp.part
    for hx in (-8.3, 19.1):
        try:
            ears = ears - (Location((hx, 0.0, -7.5)) * Cylinder(radius=1.15, height=5.0))
        except Exception:
            pass

    collar = Location((0.0, 0.0, 1.0)) * Cylinder(radius=4.5, height=2.0)
    spline = Location((0.0, 0.0, 2.5)) * Cylinder(radius=2.4, height=4.0)

    servo = body_bp.part + ears + collar + spline
    _HARDWARE_CACHE[cache_key] = servo
    return servo


def build_servo_horn_25t() -> Part:
    """Build a realistic Ø20 × 4 mm circular aluminum 25T servo horn with central hub & 4×M3 PCD 14 mm."""
    cache_key = "horn_25t:v1"
    if cache_key in _HARDWARE_CACHE:
        return _HARDWARE_CACHE[cache_key]

    disc = Location((0.0, 0.0, 2.0)) * Cylinder(radius=10.0, height=4.0)
    hub = Location((0.0, 0.0, 2.5)) * Cylinder(radius=4.5, height=5.8)
    horn = disc + hub

    # Center M3 spline screw bore + 4×M3 mounting holes on PCD 14 mm (r = 7 mm)
    cutters = [Location((0.0, 0.0, 2.5)) * Cylinder(radius=1.6, height=10.0)]
    for i in range(4):
        ang = i * (math.pi / 2.0)
        cutters.append(
            Location((7.0 * math.cos(ang), 7.0 * math.sin(ang), 2.0))
            * Cylinder(radius=1.55, height=8.0)
        )
    for cutter in cutters:
        try:
            cand = horn - cutter
            if cand.is_valid and cand.volume > 0:
                horn = cand
        except Exception:
            pass

    _HARDWARE_CACHE[cache_key] = horn
    return horn


def build_dual_servo_clevis(
    width_mm: float = 56.0,
    depth_mm: float = 44.0,
    height_mm: float = 48.0,
) -> Part:
    """Build a U-shaped wrapping dual-servo clevis bracket centered at (0, 0, 0) with joint axis along Y.
    Features 2 lateral cheeks with recessed pockets for opposed TD-8120MG servos, bottom U-bridge,
    wall stiffener ribs, and M4 mounting holes.
    """
    w = round(max(36.0, float(width_mm)), 1)
    d = round(max(30.0, float(depth_mm)), 1)
    h = round(max(34.0, float(height_mm)), 1)
    cache_key = f"clevis:{w}:{d}:{h}"
    if cache_key in _HARDWARE_CACHE:
        return _HARDWARE_CACHE[cache_key]

    cheek_t = 5.5
    bridge_h = 7.0

    # Side cheek profile in XZ plane
    with BuildSketch(Plane.XZ) as cheek_sk:
        RectangleRounded(d, h, radius=5.0)
    cheek_face = cheek_sk.sketch.faces()[0]

    # Left (-Y) and Right (+Y) cheeks
    with BuildPart() as left_bp:
        extrude(cheek_face.moved(Location((0.0, -w / 2.0, 0.0))), amount=cheek_t)
    with BuildPart() as right_bp:
        extrude(cheek_face.moved(Location((0.0, w / 2.0 - cheek_t, 0.0))), amount=cheek_t)

    # Bottom U-bridge connecting both cheeks
    bridge = Location((0.0, 0.0, -h / 2.0 + bridge_h / 2.0)) * Box(d - 4.0, w, bridge_h)
    clevis = left_bp.part + right_bp.part + bridge

    # Cut servo clearance windows (22 × 22 mm central hub + 4×M4 mounting holes per cheek) along Y
    cut_y = w + 10.0
    hub_window = Location((0.0, 0.0, 0.0), (90.0, 0.0, 0.0)) * Cylinder(radius=10.5, height=cut_y)
    try:
        cand = clevis - hub_window
        if cand.is_valid and cand.volume > 0:
            clevis = cand
    except Exception:
        pass

    # 4×M4 mounting holes flanking the servo ear span on both cheeks
    for sx in (-15.0, 15.0):
        for sz in (-6.0, 6.0):
            m4_hole = Location((sx, 0.0, sz), (90.0, 0.0, 0.0)) * Cylinder(radius=2.15, height=cut_y)
            try:
                cand = clevis - m4_hole
                if cand.is_valid and cand.volume > 0:
                    clevis = cand
            except Exception:
                pass

    _HARDWARE_CACHE[cache_key] = clevis
    return clevis


def build_gripper_parts(compact: bool = False) -> tuple[Part, Part, Part]:
    """Build a 3-part articulated 2-finger robotic gripper (~65 mm standard or <= 50 mm compact MG90S):
    Returns (gripper_base, gripper_finger_left, gripper_finger_right) so the frontend
    can animate opening/closing the fingers smoothly at 60 FPS along ±Y.
    """
    tag = "v2_compact" if compact else "v2"
    cache_keys = (f"gripper_base:{tag}", f"gripper_finger_left:{tag}", f"gripper_finger_right:{tag}")
    if all(k in _HARDWARE_CACHE for k in cache_keys):
        return (
            _HARDWARE_CACHE[cache_keys[0]],
            _HARDWARE_CACHE[cache_keys[1]],
            _HARDWARE_CACHE[cache_keys[2]],
        )

    s = 0.74 if compact else 1.0

    # 1. Base + central micro-servo actuator housing + pivot pins
    with BuildSketch(Plane.YZ) as base_sk:
        RectangleRounded(44.0 * s, 24.0 * s, radius=2.5 * s)
    with BuildPart() as base_bp:
        extrude(base_sk.sketch.faces()[0], amount=14.0 * s)

    act_box = Location((23.0 * s, 0.0, 0.0)) * Box(22.0 * s, 22.0 * s, 18.0 * s)
    pin_top = Location((16.0 * s, 12.0 * s, 0.0)) * Cylinder(radius=2.2 * s, height=15.0 * s)
    pin_bot = Location((16.0 * s, -12.0 * s, 0.0)) * Cylinder(radius=2.2 * s, height=15.0 * s)
    gripper_base = base_bp.part + act_box + pin_top + pin_bot

    # 2. Serrated gripping finger polygons (right on +Y, left on -Y)
    jaw_pts_right = [
        (12.0 * s, 10.0 * s),
        (26.0 * s, 20.0 * s),
        (65.0 * s, 16.0 * s),
        (65.0 * s, 10.0 * s),
        (56.0 * s, 11.5 * s),
        (52.0 * s, 9.5 * s),
        (46.0 * s, 11.5 * s),
        (42.0 * s, 9.5 * s),
        (36.0 * s, 11.5 * s),
        (26.0 * s, 14.0 * s),
        (12.0 * s, 5.0 * s),
    ]
    jaw_pts_left = [(x, -y) for (x, y) in jaw_pts_right]

    with BuildSketch(Plane.XY) as jr_sk:
        Polygon(*jaw_pts_right, align=None)
    with BuildPart() as jr_bp:
        extrude(jr_sk.sketch.faces()[0], amount=5.5 * s, both=True)
    gripper_finger_right = jr_bp.part

    with BuildSketch(Plane.XY) as jl_sk:
        Polygon(*jaw_pts_left, align=None)
    with BuildPart() as jl_bp:
        extrude(jl_sk.sketch.faces()[0], amount=5.5 * s, both=True)
    gripper_finger_left = jl_bp.part

    _HARDWARE_CACHE[cache_keys[0]] = gripper_base
    _HARDWARE_CACHE[cache_keys[1]] = gripper_finger_left
    _HARDWARE_CACHE[cache_keys[2]] = gripper_finger_right
    return gripper_base, gripper_finger_left, gripper_finger_right


def build_gripper_3d() -> Part:
    """Backwards-compatible wrapper returning the complete fused 2-finger gripper solid."""
    cache_key = "gripper_3d:v1"
    if cache_key in _HARDWARE_CACHE:
        return _HARDWARE_CACHE[cache_key]

    base, left, right = build_gripper_parts()
    gripper = base + left + right
    _HARDWARE_CACHE[cache_key] = gripper
    return gripper


def build_bearing_crb() -> Part:
    """Build a Crossed Roller Bearing RU42 (ID 20 mm, OD 70 mm, W 12 mm) with mounting bolt circles."""
    cache_key = "bearing_crb:ru42"
    if cache_key in _HARDWARE_CACHE:
        return _HARDWARE_CACHE[cache_key]

    r_in, r_out, w = 10.0, 35.0, 12.0
    outer_ring = Cylinder(radius=r_out, height=w) - Cylinder(radius=23.5, height=w + 2.0)
    inner_ring = Cylinder(radius=22.0, height=w) - Cylinder(radius=r_in, height=w + 2.0)
    seal = Cylinder(radius=23.8, height=w - 2.0) - Cylinder(radius=21.7, height=w)
    crb = outer_ring + inner_ring + seal

    # 6×M4 mounting holes on outer ring (PCD 60 mm) and inner ring (PCD 30 mm)
    for k in range(6):
        ang = (2.0 * math.pi * k) / 6.0
        for pcd_r in (30.0, 15.0):
            hole = Location((pcd_r * math.cos(ang), pcd_r * math.sin(ang), 0.0)) * Cylinder(
                radius=2.2, height=w + 4.0
            )
            try:
                crb = crb - hole
            except Exception:
                pass

    _HARDWARE_CACHE[cache_key] = crb
    return crb


def build_bearing_tapered_30205() -> Part:
    """Build a tapered roller bearing 30205 (ID 25 mm, OD 52 mm, W 16.25 mm)."""
    cache_key = "bearing_tapered:30205"
    if cache_key in _HARDWARE_CACHE:
        return _HARDWARE_CACHE[cache_key]

    r_in, r_out, w = 12.5, 26.0, 16.25
    outer_cup = Cylinder(radius=r_out, height=w) - Cylinder(radius=22.0, height=w + 2.0)
    inner_cone = Cylinder(radius=16.5, height=w) - Cylinder(radius=r_in, height=w + 2.0)
    cage = Location((0.0, 0.0, -w * 0.4)) * Cone(
        bottom_radius=21.8, top_radius=19.2, height=w * 0.8
    ) - Cylinder(radius=16.0, height=w + 2.0)
    bearing = outer_cup + inner_cone + cage

    _HARDWARE_CACHE[cache_key] = bearing
    return bearing


def build_homing_sensor() -> Part:
    """Build an M8 inductive proximity homing sensor (Ø8 × 32 mm barrel + dual hex locknuts + LED cap)."""
    cache_key = "homing_sensor:m8"
    if cache_key in _HARDWARE_CACHE:
        return _HARDWARE_CACHE[cache_key]

    barrel = Cylinder(radius=4.0, height=30.0)
    tip = Location((0.0, 0.0, 16.0)) * Cylinder(radius=3.6, height=3.0)
    led_cap = Location((0.0, 0.0, -16.5)) * Cylinder(radius=4.4, height=4.0)

    with BuildSketch(Plane.XY) as nut_sk:
        RegularPolygon(radius=6.8, side_count=6)
    nut_face = nut_sk.sketch.faces()[0]
    with BuildPart() as n1_bp:
        extrude(nut_face.moved(Location((0.0, 0.0, 4.0))), amount=3.2)
    with BuildPart() as n2_bp:
        extrude(nut_face.moved(Location((0.0, 0.0, -6.0))), amount=3.2)

    sensor = barrel + tip + led_cap + n1_bp.part + n2_bp.part
    _HARDWARE_CACHE[cache_key] = sensor
    return sensor


def build_servo_sts3215_3d() -> Part:
    """Build a Feetech STS3215 serial bus servo (45.2 × 24.7 × 35 mm) with dual front/rear structural horns."""
    cache_key = "servo:sts3215"
    if cache_key in _HARDWARE_CACHE:
        return _HARDWARE_CACHE[cache_key]

    # Body offset +11 mm in X so front output spline is centered at (0, 0)
    with BuildSketch(Plane.XY) as body_sk:
        with Locations((11.0, 0.0)):
            RectangleRounded(45.2, 24.7, radius=2.0)
    with BuildPart() as body_bp:
        extrude(body_sk.sketch.faces()[0].moved(Location((0.0, 0.0, -35.0))), amount=35.0)

    # Front output spline horn (+Z) and rear structural idler horn (-Z)
    front_horn = Location((0.0, 0.0, 2.5)) * Cylinder(radius=9.5, height=5.0)
    rear_horn = Location((0.0, 0.0, -37.5)) * Cylinder(radius=9.5, height=5.0)

    # 3-pin UART serial bus connector boss on back edge (+X)
    bus_port = Location((34.5, 0.0, -18.0)) * Box(4.5, 11.0, 8.0)

    servo = body_bp.part + front_horn + rear_horn + bus_port
    _HARDWARE_CACHE[cache_key] = servo
    return servo


def build_wrist_3dof_parts() -> tuple[Part, Part, Part]:
    """Build a realistic 3-DOF articulated wrist (J4 Roll, J5 Pitch, J6 Tool Flange) along +X.
    Returns (wrist_j4_roll, wrist_j5_pitch, wrist_j6_flange) with origin at J4 entry (0, 0, 0).
    """
    keys = ("wrist3:j4", "wrist3:j5", "wrist3:j6")
    if all(k in _HARDWARE_CACHE for k in keys):
        return (
            _HARDWARE_CACHE[keys[0]],
            _HARDWARE_CACHE[keys[1]],
            _HARDWARE_CACHE[keys[2]],
        )

    # J4 Roll housing (x = 0..28 mm)
    j4_tube = Location((14.0, 0.0, 0.0), (0.0, 90.0, 0.0)) * Cylinder(radius=15.0, height=28.0)
    j4_ring = Location((26.0, 0.0, 0.0), (0.0, 90.0, 0.0)) * Cylinder(radius=17.5, height=6.0)
    wrist_j4_roll = j4_tube + j4_ring

    # J5 Pitch clevis & pivot housing (x = 28..58 mm)
    j5_base = Location((36.0, 0.0, 0.0)) * Box(16.0, 28.0, 24.0)
    j5_pivot = Location((48.0, 0.0, 0.0), (90.0, 0.0, 0.0)) * Cylinder(radius=13.0, height=30.0)
    j5_neck = Location((56.0, 0.0, 0.0), (0.0, 90.0, 0.0)) * Cylinder(radius=11.0, height=16.0)
    wrist_j5_pitch = j5_base + j5_pivot + j5_neck

    # J6 ISO 4-bolt rotating tool flange (x = 64..74 mm)
    j6_disc = Location((69.0, 0.0, 0.0), (0.0, 90.0, 0.0)) * Cylinder(radius=19.0, height=9.0)
    for k in range(4):
        ang = math.pi / 4.0 + k * (math.pi / 2.0)
        hy = 13.0 * math.cos(ang)
        hz = 13.0 * math.sin(ang)
        bolt = Location((69.0, hy, hz), (0.0, 90.0, 0.0)) * Cylinder(radius=2.2, height=14.0)
        try:
            j6_disc = j6_disc - bolt
        except Exception:
            pass
    wrist_j6_flange = j6_disc

    _HARDWARE_CACHE[keys[0]] = wrist_j4_roll
    _HARDWARE_CACHE[keys[1]] = wrist_j5_pitch
    _HARDWARE_CACHE[keys[2]] = wrist_j6_flange
    return wrist_j4_roll, wrist_j5_pitch, wrist_j6_flange


def export_cnc_plates_dxf(req: Any, output_path: Path) -> Path:
    """Generate a valid ASCII DXF R12 file with 2.5D Aluminum 6061-T6 side-plate profiles and bore circles."""
    slots = getattr(req, "slots", None)
    upper = getattr(slots, "upper_arm", None) if slots else None
    forearm = getattr(slots, "forearm", None) if slots else None

    l1 = float(getattr(upper, "length_mm", 200.0) if upper else 200.0)
    r1_a = float(getattr(upper.root_end, "outer_diameter_mm", 46.0) / 2.0) if upper else 23.0
    r1_b = float(getattr(upper.tip_end, "outer_diameter_mm", 34.0) / 2.0) if upper else 17.0
    b1_a = float(getattr(upper.root_end, "bore_diameter_mm", 22.0) / 2.0) if upper else 11.0
    b1_b = float(getattr(upper.tip_end, "bore_diameter_mm", 16.0) / 2.0) if upper else 8.0

    l2 = float(getattr(forearm, "length_mm", 170.0) if forearm else 170.0)
    r2_a = float(getattr(forearm.root_end, "outer_diameter_mm", 36.0) / 2.0) if forearm else 18.0
    r2_b = float(getattr(forearm.tip_end, "outer_diameter_mm", 28.0) / 2.0) if forearm else 14.0
    b2_a = float(getattr(forearm.root_end, "bore_diameter_mm", 16.0) / 2.0) if forearm else 8.0
    b2_b = float(getattr(forearm.tip_end, "bore_diameter_mm", 12.0) / 2.0) if forearm else 6.0

    entities: list[str] = []

    def _dxf_line(x1: float, y1: float, x2: float, y2: float, layer: str = "OUTLINE") -> None:
        entities.extend(
            [
                "0", "LINE",
                "8", layer,
                "10", f"{x1:.4f}", "20", f"{y1:.4f}", "30", "0.0",
                "11", f"{x2:.4f}", "21", f"{y2:.4f}", "31", "0.0",
            ]
        )

    def _dxf_circle(cx: float, cy: float, r: float, layer: str = "HOLES") -> None:
        entities.extend(
            [
                "0", "CIRCLE",
                "8", layer,
                "10", f"{cx:.4f}", "20", f"{cy:.4f}", "30", "0.0",
                "40", f"{r:.4f}",
            ]
        )

    def _dxf_arc(cx: float, cy: float, r: float, a0: float, a1: float, layer: str = "OUTLINE") -> None:
        entities.extend(
            [
                "0", "ARC",
                "8", layer,
                "10", f"{cx:.4f}", "20", f"{cy:.4f}", "30", "0.0",
                "40", f"{r:.4f}",
                "50", f"{a0:.4f}",
                "51", f"{a1:.4f}",
            ]
        )

    def _emit_plate(y_off: float, length: float, ra: float, rb: float, ba: float, bb: float) -> None:
        # Outer tapered plate boundary (two semicircular arcs + top/bottom tangent lines)
        _dxf_arc(0.0, y_off, ra, 90.0, 270.0)
        _dxf_arc(length, y_off, rb, 270.0, 90.0)
        _dxf_line(0.0, y_off + ra, length, y_off + rb)
        _dxf_line(0.0, y_off - ra, length, y_off - rb)
        # Main bores
        _dxf_circle(0.0, y_off, ba, "BORES")
        _dxf_circle(length, y_off, bb, "BORES")
        # 4×M4 mounting bolt circles on root boss
        pcd_r = (ra + ba) * 0.5
        for k in range(4):
            ang = math.pi / 4.0 + k * (math.pi / 2.0)
            _dxf_circle(pcd_r * math.cos(ang), y_off + pcd_r * math.sin(ang), 2.15, "HOLES")
        # Central lightening holes along plate neutral axis
        for frac in (0.35, 0.55, 0.72):
            cx = length * frac
            r_local = ra + (rb - ra) * frac
            r_cut = max(3.5, r_local * 0.42)
            _dxf_circle(cx, y_off, r_cut, "POCKETS")

    # Plate 1: Upper Arm CNC Side Plate (Al 6061-T6)
    _emit_plate(0.0, l1, r1_a, r1_b, b1_a, b1_b)
    # Plate 2: Forearm CNC Side Plate (offset in Y by +85 mm for clean nesting)
    _emit_plate(max(85.0, r1_a + r2_a + 25.0), l2, r2_a, r2_b, b2_a, b2_b)

    dxf_lines = [
        "0", "SECTION",
        "2", "HEADER",
        "9", "$ACADVER", "1", "AC1009",
        "9", "$INSUNITS", "70", "4",
        "0", "ENDSEC",
        "0", "SECTION",
        "2", "ENTITIES",
        *entities,
        "0", "ENDSEC",
        "0", "EOF",
    ]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(dxf_lines) + "\n", encoding="utf-8")
    return output_path


def build_four_bar_parallelogram_rod(length_mm: float = 200.0) -> Part:
    """Build a secondary 4-bar parallelogram linkage rod from (0, 0) to (length_mm, 0) in XY plane."""
    length = round(max(40.0, float(length_mm)), 1)
    cache_key = f"four_bar_rod:{length}"
    if cache_key in _HARDWARE_CACHE:
        return _HARDWARE_CACHE[cache_key]

    rod_h = 8.0
    rod_t = 6.0
    bar = Location((length / 2.0, 0.0, 0.0)) * Box(length, rod_h, rod_t)
    eye_a = Cylinder(radius=7.5, height=rod_t) - Cylinder(radius=3.2, height=rod_t + 4.0)
    eye_b = Location((length, 0.0, 0.0)) * (
        Cylinder(radius=7.5, height=rod_t) - Cylinder(radius=3.2, height=rod_t + 4.0)
    )
    rod = bar + eye_a + eye_b
    _HARDWARE_CACHE[cache_key] = rod
    return rod


def build_vacuum_nozzle() -> Part:
    """Build a pneumatic vacuum pick-and-place nozzle with brass fitting and silicone suction cup.
    Mounting face at z = 0, nozzle extends downward along -Z to z = -42 mm.
    """
    cache_key = "vacuum_nozzle:v1"
    if cache_key in _HARDWARE_CACHE:
        return _HARDWARE_CACHE[cache_key]

    fitting = Location((0.0, 0.0, -5.0)) * Cylinder(radius=7.5, height=10.0)
    stem = Location((0.0, 0.0, -21.0)) * Cylinder(radius=3.2, height=24.0)
    cup = Location((0.0, 0.0, -42.0)) * Cone(bottom_radius=9.5, top_radius=3.8, height=10.0)
    nozzle = fitting + stem + cup
    _HARDWARE_CACHE[cache_key] = nozzle
    return nozzle


