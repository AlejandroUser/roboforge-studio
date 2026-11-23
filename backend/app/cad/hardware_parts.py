"""Parametric 3D mechanical hardware builders (ISO 4762 screws, ball bearings, motors, MCU box)."""

from __future__ import annotations

import math

from build123d import (
    Box,
    BuildPart,
    BuildSketch,
    Compound,
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

    if "servo_td" in mid or "servo_ds" in mid or "mg996" in mid:
        return build_servo_3d(motor_id)

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


def build_gripper_parts() -> tuple[Part, Part, Part]:
    """Build a 3-part articulated 2-finger robotic gripper (~65 mm long along +X):
    Returns (gripper_base, gripper_finger_left, gripper_finger_right) so the frontend
    can animate opening/closing the fingers smoothly at 60 FPS along ±Y.
    """
    cache_keys = ("gripper_base:v2", "gripper_finger_left:v2", "gripper_finger_right:v2")
    if all(k in _HARDWARE_CACHE for k in cache_keys):
        return (
            _HARDWARE_CACHE[cache_keys[0]],
            _HARDWARE_CACHE[cache_keys[1]],
            _HARDWARE_CACHE[cache_keys[2]],
        )

    # 1. Base + central micro-servo actuator housing + pivot pins
    with BuildSketch(Plane.YZ) as base_sk:
        RectangleRounded(44.0, 24.0, radius=3.0)
    with BuildPart() as base_bp:
        extrude(base_sk.sketch.faces()[0], amount=14.0)

    act_box = Location((23.0, 0.0, 0.0)) * Box(22.0, 22.0, 18.0)
    pin_top = Location((16.0, 12.0, 0.0)) * Cylinder(radius=2.5, height=16.0)
    pin_bot = Location((16.0, -12.0, 0.0)) * Cylinder(radius=2.5, height=16.0)
    gripper_base = base_bp.part + act_box + pin_top + pin_bot

    # 2. Serrated gripping finger polygons (right on +Y, left on -Y)
    jaw_pts_right = [
        (12.0, 10.0),
        (26.0, 20.0),
        (65.0, 16.0),
        (65.0, 10.0),
        (56.0, 11.5),
        (52.0, 9.5),
        (46.0, 11.5),
        (42.0, 9.5),
        (36.0, 11.5),
        (26.0, 14.0),
        (12.0, 5.0),
    ]
    jaw_pts_left = [(x, -y) for (x, y) in jaw_pts_right]

    with BuildSketch(Plane.XY) as jr_sk:
        Polygon(*jaw_pts_right, align=None)
    with BuildPart() as jr_bp:
        extrude(jr_sk.sketch.faces()[0], amount=6.0, both=True)
    gripper_finger_right = jr_bp.part

    with BuildSketch(Plane.XY) as jl_sk:
        Polygon(*jaw_pts_left, align=None)
    with BuildPart() as jl_bp:
        extrude(jl_sk.sketch.faces()[0], amount=6.0, both=True)
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

