"""Parametric 4-DOF Cartesian Gantry (P-P-P-R: LumenPnP / OpenPnP) 3D CAD assembler + URDF exporter."""

from __future__ import annotations

import xml.sax.saxutils as saxutils
from pathlib import Path
from typing import Any

import trimesh
from build123d import (
    Box,
    Compound,
    Cylinder,
    Location,
    Part,
    export_step,
    export_stl,
)

from ..schemas import JointTorqueSummary, ModularArmRequest
from .arm_assembly import _export_part_step_and_3mf, _part_to_trimesh
from .hardware_parts import build_controller_box, build_motor_3d, build_vacuum_nozzle


def build_cartesian_gantry_assembly(
    req: ModularArmRequest,
    material: dict[str, Any],
    output_dir: Path,
    design_hash: str,
) -> dict[str, Any]:
    """Build a 4-DOF Cartesian Gantry (X-Y-Z prismatic + rotary vacuum nozzle) standing on Z=0."""
    from ..physics.arm_compatibility import resolve_default_slots

    output_dir.mkdir(parents=True, exist_ok=True)
    slots = resolve_default_slots(req)

    x_span = max(240.0, float(slots.upper_arm.length_mm) * 1.45 if slots.upper_arm else 300.0)
    y_span = max(220.0, float(slots.forearm.length_mm) * 1.45 if slots.forearm else 260.0)
    z_stroke = float(getattr(req, "z_stroke_mm", 80.0) or 80.0)
    frame_h = max(140.0, req.base_height_mm + z_stroke * 0.65)
    p = 20.0  # 2020 aluminum extrusion profile width

    # 1. Rigid 2020 Aluminum Extrusion Frame (4 legs + 4 perimeter beams + 2 top Y rails)
    frame_parts: list[Part] = []
    # Lower perimeter rectangle at z = p/2
    frame_parts.append(Location((0.0, -y_span / 2.0, p / 2.0)) * Box(x_span, p, p))
    frame_parts.append(Location((0.0, +y_span / 2.0, p / 2.0)) * Box(x_span, p, p))
    frame_parts.append(Location((-x_span / 2.0, 0.0, p / 2.0)) * Box(p, y_span, p))
    frame_parts.append(Location((+x_span / 2.0, 0.0, p / 2.0)) * Box(p, y_span, p))
    # 4 corner vertical columns
    for sx in (-x_span / 2.0, +x_span / 2.0):
        for sy in (-y_span / 2.0, +y_span / 2.0):
            frame_parts.append(Location((sx, sy, frame_h / 2.0)) * Box(p, p, frame_h))
    # 2 elevated Y-axis rails along X = ±x_span/2 at z = frame_h - p/2
    frame_parts.append(Location((-x_span / 2.0, 0.0, frame_h - p / 2.0)) * Box(p, y_span, p))
    frame_parts.append(Location((+x_span / 2.0, 0.0, frame_h - p / 2.0)) * Box(p, y_span, p))
    gantry_frame_2020 = Compound(children=frame_parts)

    # 2. Moving Y-axis Bridge (spans X at y = y_pos, z = frame_h + p/2)
    y_pos = max(-y_span * 0.25, min(y_span * 0.25, float(req.joint_angles_deg[1]) * 1.2))
    bridge_beam = Location((0.0, y_pos, frame_h + p / 2.0)) * Box(x_span + p, p, p)
    truck_l = Location((-x_span / 2.0, y_pos, frame_h + 2.0)) * Box(26.0, 44.0, 26.0)
    truck_r = Location((+x_span / 2.0, y_pos, frame_h + 2.0)) * Box(26.0, 44.0, 26.0)
    gantry_y_bridge = bridge_beam + truck_l + truck_r

    # 3. Moving X-axis Carriage sliding along bridge
    x_pos = max(-x_span * 0.28, min(x_span * 0.28, float(req.joint_angles_deg[0]) * 1.5))
    gantry_x_carriage = Location((x_pos, y_pos + 16.0, frame_h + p / 2.0)) * Box(46.0, 22.0, 48.0)

    # 4. Moving Z-axis Tool Head + Vacuum Nozzle
    z_head_z = frame_h - 14.0
    z_rail = Location((x_pos, y_pos + 32.0, z_head_z)) * Box(28.0, 16.0, max(64.0, z_stroke * 0.85))
    z_collar = Location((x_pos, y_pos + 32.0, z_head_z - 28.0)) * Cylinder(radius=10.0, height=22.0)
    gantry_z_head = z_rail + z_collar

    vacuum_nozzle = Location((x_pos, y_pos + 32.0, z_head_z - 39.0)) * build_vacuum_nozzle()

    # 5. X, Y, Z Stepper Motors + Controller Box
    nema17 = build_motor_3d("nema17_48mm")
    motor_x = Location((-x_span / 2.0 - 12.0, y_pos, frame_h + p / 2.0), (0.0, 90.0, 0.0)) * nema17
    motor_y = Location((-x_span / 2.0, -y_span / 2.0 - 10.0, frame_h - p / 2.0), (-90.0, 0.0, 0.0)) * nema17
    motor_z = Location((x_pos, y_pos + 32.0, frame_h + 44.0), (180.0, 0.0, 0.0)) * build_motor_3d("nema14_34mm")
    controller_box = Location((-x_span / 2.0 - 50.0, 0.0, 0.0)) * build_controller_box()

    real_volumes_mm3 = {
        "base_pedestal": float(max(gantry_frame_2020.volume, 1.0)),
        "shoulder_mount": float(max(gantry_y_bridge.volume, 1.0)),
        "shoulder_gear": float(max(gantry_x_carriage.volume, 1.0)),
        "upper_arm_link": float(max(gantry_y_bridge.volume, 1.0)),
        "elbow_gear": float(max(gantry_z_head.volume, 1.0)),
        "forearm_link": float(max(gantry_x_carriage.volume, 1.0)),
        "wrist_tube": float(max(gantry_z_head.volume + vacuum_nozzle.volume, 1.0)),
    }

    individual_files: list[Path] = []
    for solid, stem, vhint in (
        (gantry_y_bridge, "gantry_y_bridge", "mount_gusseted"),
        (gantry_x_carriage, "gantry_x_carriage", "mount_standard"),
        (gantry_z_head, "gantry_z_head", "tapered_solid"),
    ):
        s_p, m_p = _export_part_step_and_3mf(solid, output_dir, stem, vhint)
        individual_files.extend([s_p, m_p])

    full_compound = Compound(
        children=[
            gantry_frame_2020,
            gantry_y_bridge,
            gantry_x_carriage,
            gantry_z_head,
            vacuum_nozzle,
            motor_x,
            motor_y,
            motor_z,
            controller_box,
        ]
    )
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
            ("gantry_frame_2020", gantry_frame_2020, [192, 200, 212, 255], 0.88, 0.24),
            ("gantry_y_bridge", gantry_y_bridge, [82, 134, 186, 255], 0.78, 0.28),
            ("gantry_x_carriage", gantry_x_carriage, [56, 178, 198, 255], 0.65, 0.30),
            ("gantry_z_head", gantry_z_head, [72, 86, 108, 255], 0.76, 0.28),
            ("vacuum_nozzle", vacuum_nozzle, [218, 176, 62, 255], 0.82, 0.26),
            ("motor_x", motor_x, [32, 36, 44, 255], 0.58, 0.42),
            ("motor_y", motor_y, [32, 36, 44, 255], 0.58, 0.42),
            ("motor_z", motor_z, [36, 42, 52, 255], 0.60, 0.40),
            ("controller_box", controller_box, [34, 62, 56, 255], 0.42, 0.48),
        ]
        for node_name, solid, rgba, met, rough in node_specs:
            mesh = _part_to_trimesh(solid, rgba, met, rough, node_name)
            scene.add_geometry(mesh, node_name=node_name, geom_name=node_name)
        scene.export(str(glb_path), file_type="glb")

    explode_vectors: dict[str, list[float]] = {
        "gantry_frame_2020": [0.0, 0.0, -30.0],
        "gantry_y_bridge": [0.0, 0.0, 38.0],
        "gantry_x_carriage": [0.0, 42.0, 42.0],
        "gantry_z_head": [0.0, 74.0, 18.0],
        "vacuum_nozzle": [0.0, 74.0, -34.0],
        "motor_x": [-56.0, 0.0, 38.0],
        "motor_y": [0.0, -56.0, 0.0],
        "motor_z": [0.0, 74.0, 78.0],
        "controller_box": [-58.0, 0.0, -24.0],
    }

    printable_parts: dict[str, Part] = {
        "gantry_y_bridge": gantry_y_bridge,
        "gantry_x_carriage": gantry_x_carriage,
        "gantry_z_head": gantry_z_head,
        "vacuum_nozzle": vacuum_nozzle,
    }

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


def export_cartesian_urdf(
    req: ModularArmRequest,
    joints: list[JointTorqueSummary],
    output_path: Path,
) -> Path:
    """Export a valid ROS 2 URDF with 3 orthogonal prismatic joints (X, Y, Z) + 1 revolute nozzle joint."""
    z_stroke = float(getattr(req, "z_stroke_mm", 80.0) or 80.0) / 1000.0
    urdf_xml = f"""<?xml version="1.0" encoding="UTF-8"?>
<robot name="roboforge_cartesian_gantry_{saxutils.escape(req.material_id)}">
  <link name="gantry_frame_link">
    <inertial><origin xyz="0 0 0.08" rpy="0 0 0"/><mass value="2.2"/><inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.03"/></inertial>
  </link>
  <joint name="joint_y_prismatic" type="prismatic">
    <parent link="gantry_frame_link"/><child link="y_bridge_link"/>
    <origin xyz="0 0 0.16" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-0.12" upper="0.12" effort="40.0" velocity="0.6"/>
  </joint>
  <link name="y_bridge_link">
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="0.65"/><inertia ixx="0.004" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.004"/></inertial>
  </link>
  <joint name="joint_x_prismatic" type="prismatic">
    <parent link="y_bridge_link"/><child link="x_carriage_link"/>
    <origin xyz="0 0.02 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-0.14" upper="0.14" effort="30.0" velocity="0.8"/>
  </joint>
  <link name="x_carriage_link">
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="0.32"/><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial>
  </link>
  <joint name="joint_z_prismatic" type="prismatic">
    <parent link="x_carriage_link"/><child link="z_head_link"/>
    <origin xyz="0 0.015 -0.02" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-{z_stroke:.4f}" upper="0.0" effort="25.0" velocity="0.4"/>
  </joint>
  <link name="z_head_link">
    <inertial><origin xyz="0 0 -0.02" rpy="0 0 0"/><mass value="0.18"/><inertia ixx="0.0004" ixy="0" ixz="0" iyy="0.0004" iyz="0" izz="0.0002"/></inertial>
  </link>
  <joint name="joint_tool_roll" type="revolute">
    <parent link="z_head_link"/><child link="vacuum_nozzle_link"/>
    <origin xyz="0 0 -0.04" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.1416" upper="3.1416" effort="2.0" velocity="12.0"/>
  </joint>
  <link name="vacuum_nozzle_link"/>
</robot>
"""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(urdf_xml, encoding="utf-8")
    return output_path
