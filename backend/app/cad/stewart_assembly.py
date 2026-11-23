"""Parametric 6-DOF Stewart Platform / Hexapod (6-UPS Parallel) 3D CAD assembler + URDF exporter."""

from __future__ import annotations

import math
import xml.sax.saxutils as saxutils
from pathlib import Path
from typing import Any

import trimesh
from build123d import (
    BuildPart,
    BuildSketch,
    Compound,
    Location,
    Part,
    Plane,
    RegularPolygon,
    Sphere,
    extrude,
    export_step,
    export_stl,
)

from ..schemas import JointTorqueSummary, ModularArmRequest
from .arm_assembly import _export_part_step_and_3mf, _part_to_trimesh
from .delta_assembly import _cylinder_between
from .hardware_parts import build_controller_box


def build_stewart_hexapod_assembly(
    req: ModularArmRequest,
    material: dict[str, Any],
    output_dir: Path,
    design_hash: str,
) -> dict[str, Any]:
    """Build a 6-DOF Stewart Platform / Hexapod with hexagonal base, 6 telescopic linear actuators
    (cylinder barrel + chrome piston rod + universal spherical ball joints), and mobile top platform.
    """
    from ..physics.arm_compatibility import resolve_default_slots

    output_dir.mkdir(parents=True, exist_ok=True)
    slots = resolve_default_slots(req)

    r_base = 128.0
    r_plat = 82.0
    z_stroke = float(getattr(req, "z_stroke_mm", 90.0) or 90.0)
    z_plat = max(130.0, req.base_height_mm + z_stroke * 0.75 + float(req.joint_angles_deg[0]) * 0.4)

    # 1. Hexagonal Stationary Base Plate (z = 0..14 mm)
    with BuildSketch(Plane.XY) as b_sk:
        RegularPolygon(radius=r_base, side_count=6)
    with BuildPart() as b_bp:
        extrude(b_sk.sketch.faces()[0], amount=14.0)
    stewart_base = b_bp.part

    # 2. Hexagonal Mobile Upper Platform (z = z_plat..z_plat+12 mm)
    with BuildSketch(Plane.XY) as p_sk:
        RegularPolygon(radius=r_plat, side_count=6)
    with BuildPart() as p_bp:
        extrude(p_sk.sketch.faces()[0].moved(Location((0.0, 0.0, z_plat))), amount=12.0)
    stewart_platform = p_bp.part

    # 3. Six 6-UPS Telescopic Linear Actuator Legs (paired anchors on base & platform)
    base_angles_deg = [-12.0, 12.0, 108.0, 132.0, 228.0, 252.0]
    plat_angles_deg = [-48.0, 48.0, 72.0, 168.0, 192.0, 288.0]

    legs: list[Compound] = []
    explode_vectors: dict[str, list[float]] = {
        "stewart_base": [0.0, 0.0, -38.0],
        "stewart_platform": [0.0, 0.0, 62.0],
        "controller_box": [-58.0, 0.0, -24.0],
    }

    for i in range(6):
        ba = math.radians(base_angles_deg[i])
        pa = math.radians(plat_angles_deg[i])
        p_b = (r_base * 0.82 * math.cos(ba), r_base * 0.82 * math.sin(ba), 14.0)
        p_p = (r_plat * 0.82 * math.cos(pa), r_plat * 0.82 * math.sin(pa), z_plat)
        p_mid = (
            p_b[0] + 0.56 * (p_p[0] - p_b[0]),
            p_b[1] + 0.56 * (p_p[1] - p_b[1]),
            p_b[2] + 0.56 * (p_p[2] - p_b[2]),
        )

        barrel = _cylinder_between(p_b, p_mid, radius=9.5)
        piston = _cylinder_between(p_mid, p_p, radius=5.2)
        ball_b = Location(p_b) * Sphere(9.0)
        ball_p = Location(p_p) * Sphere(7.5)
        legs.append(Compound(children=[barrel, piston, ball_b, ball_p]))

        mid_ang = 0.5 * (ba + pa)
        explode_vectors[f"stewart_leg_{i + 1}"] = [
            round(42.0 * math.cos(mid_ang), 2),
            round(42.0 * math.sin(mid_ang), 2),
            12.0,
        ]

    controller_box = Location((-(r_base + 52.0), 0.0, 0.0)) * build_controller_box()

    real_volumes_mm3 = {
        "base_pedestal": float(max(stewart_base.volume, 1.0)),
        "shoulder_mount": float(max(stewart_platform.volume, 1.0)),
        "shoulder_gear": float(max(legs[0].volume, 1.0)),
        "upper_arm_link": float(max(sum(lg.volume for lg in legs[:3]), 1.0)),
        "elbow_gear": float(max(legs[3].volume, 1.0)),
        "forearm_link": float(max(sum(lg.volume for lg in legs[3:]), 1.0)),
        "wrist_tube": float(max(stewart_platform.volume * 0.5, 1.0)),
    }

    printable_parts: dict[str, Part] = {
        "stewart_base": stewart_base,
        "stewart_platform": stewart_platform,
    }

    individual_files: list[Path] = []
    for solid, stem, vhint in (
        (stewart_base, "stewart_base", "tapered_solid"),
        (stewart_platform, "stewart_platform", "mount_standard"),
    ):
        s_p, m_p = _export_part_step_and_3mf(solid, output_dir, stem, vhint)
        individual_files.extend([s_p, m_p])

    full_compound = Compound(children=[stewart_base, stewart_platform, controller_box, *legs])
    step_path = output_dir / "arm_assembly.step"
    stl_path = output_dir / "arm_assembly.stl"
    glb_path = output_dir / "arm_assembly.glb"

    if not (step_path.exists() and step_path.stat().st_size > 0):
        export_step(full_compound, str(step_path))
    if not (stl_path.exists() and stl_path.stat().st_size > 0):
        export_stl(full_compound, str(stl_path), tolerance=0.14, angular_tolerance=0.22)

    if not (glb_path.exists() and glb_path.stat().st_size > 0):
        scene = trimesh.Scene()
        node_specs: list[tuple[str, Part | Compound, list[int], float, float]] = [
            ("stewart_base", stewart_base, [76, 118, 168, 255], 0.82, 0.26),
            ("stewart_platform", stewart_platform, [84, 166, 208, 255], 0.82, 0.24),
            ("controller_box", controller_box, [34, 62, 56, 255], 0.42, 0.48),
        ]
        for i in range(6):
            node_specs.append((f"stewart_leg_{i + 1}", legs[i], [208, 216, 228, 255], 0.92, 0.18))
        for node_name, solid, rgba, met, rough in node_specs:
            mesh = _part_to_trimesh(solid, rgba, met, rough, node_name)
            scene.add_geometry(mesh, node_name=node_name, geom_name=node_name)
        scene.export(str(glb_path), file_type="glb")

    return {
        "real_volumes_mm3": real_volumes_mm3,
        "explode_vectors": explode_vectors,
        "individual_files": individual_files,
        "printable_parts": printable_parts,
        "step_path": step_path,
        "stl_path": stl_path,
        "glb_path": glb_path,
        "slots_used": slots,
        "design_hash": design_hash,
    }


def export_stewart_urdf(
    req: ModularArmRequest,
    joints: list[JointTorqueSummary],
    output_path: Path,
) -> Path:
    """Export a valid ROS 2 URDF describing the 6-DOF Stewart Hexapod with 6 prismatic actuator legs."""
    z_stroke = float(getattr(req, "z_stroke_mm", 90.0) or 90.0) / 1000.0
    leg_xml = []
    for i in range(6):
        ang = i * (math.pi / 3.0)
        x = 0.105 * math.cos(ang)
        y = 0.105 * math.sin(ang)
        leg_xml.append(
            f'  <joint name="stewart_leg_{i + 1}_prismatic" type="prismatic">\n'
            f'    <parent link="stewart_base"/><child link="stewart_leg_{i + 1}"/>\n'
            f'    <origin xyz="{x:.4f} {y:.4f} 0.014" rpy="0 0 0"/><axis xyz="0 0 1"/>\n'
            f'    <limit lower="0.0" upper="{z_stroke:.4f}" effort="120.0" velocity="0.35"/>\n'
            f'  </joint>\n'
            f'  <link name="stewart_leg_{i + 1}"/>'
        )
    urdf_xml = (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        f'<robot name="roboforge_stewart_hexapod_{saxutils.escape(req.material_id)}">\n'
        '  <link name="stewart_base">\n'
        '    <inertial><origin xyz="0 0 0.007" rpy="0 0 0"/><mass value="2.2"/>'
        '<inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.03"/></inertial>\n'
        '  </link>\n'
        + "\n".join(leg_xml)
        + '\n  <joint name="stewart_platform_virtual" type="fixed">\n'
        '    <parent link="stewart_leg_1"/><child link="stewart_platform"/>\n'
        '    <origin xyz="0 0 0.16" rpy="0 0 0"/>\n'
        '  </joint>\n'
        '  <link name="stewart_platform"/>\n'
        "</robot>\n"
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(urdf_xml, encoding="utf-8")
    return output_path
