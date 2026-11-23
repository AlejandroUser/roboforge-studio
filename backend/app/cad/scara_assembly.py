"""Parametric 4-DOF SCARA (R-R-P-R) 3D CAD assembler + URDF exporter with real prismatic Z joint."""

from __future__ import annotations

import math
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
from .arm_assembly import (
    _export_part_step_and_3mf,
    _get_or_build_subpart,
    _hex_to_rgba,
    _part_to_trimesh,
)
from .cycloidal_variants import build_closed_belt_loop_3d
from .hardware_parts import (
    build_ball_bearing,
    build_controller_box,
    build_homing_sensor,
    build_motor_3d,
)
from .variants import (
    _build_hollow_cobot,
    _build_organic_ibeam,
    _build_organic_topopt,
    _build_rounded_truss,
    _build_tapered_solid,
)


def build_scara_arm_assembly(
    req: ModularArmRequest,
    material: dict[str, Any],
    output_dir: Path,
    design_hash: str,
) -> dict[str, Any]:
    """Build a real 4-DOF SCARA (R-R-P-R) 3D assembly standing on Z=0 with twin linear rails,
    T8 leadscrew, sliding Z-carriage, horizontal R1/R2 arms, closed GT2 belt loop, and Z-quill.
    """
    from ..physics.arm_compatibility import resolve_default_slots

    output_dir.mkdir(parents=True, exist_ok=True)
    slots = resolve_default_slots(req)
    assert slots.upper_arm is not None and slots.forearm is not None

    link_builders = {
        "tapered_solid": _build_tapered_solid,
        "organic_ibeam": _build_organic_ibeam,
        "rounded_truss": _build_rounded_truss,
        "hollow_cobot": _build_hollow_cobot,
        "organic_topopt": _build_organic_topopt,
    }

    arm1_raw = _get_or_build_subpart(
        "link",
        slots.upper_arm_variant,
        slots.upper_arm,
        link_builders.get(slots.upper_arm_variant, _build_organic_ibeam),
    )
    arm2_raw = _get_or_build_subpart(
        "link",
        slots.forearm_variant,
        slots.forearm,
        link_builders.get(slots.forearm_variant, _build_hollow_cobot),
    )

    l1 = float(slots.upper_arm.length_mm)
    l2 = float(slots.forearm.length_mm)
    t1 = float(slots.upper_arm.thickness_mm)
    t2 = float(slots.forearm.thickness_mm)
    z_stroke = float(getattr(req, "z_stroke_mm", 120.0) or 120.0)
    col_h = max(180.0, req.base_height_mm + z_stroke + 60.0)
    z_car = req.base_height_mm + z_stroke * 0.55

    # Horizontal SCARA joint angles (around +Z axis)
    a1_deg = float(req.joint_angles_deg[0])
    a2_deg = a1_deg + float(req.joint_angles_deg[1])
    a1 = math.radians(a1_deg)
    a2 = math.radians(a2_deg)

    # 1. Heavy bolted base plate + vertical SCARA column (z = 0..col_h)
    base_plate = Location((0.0, 0.0, 7.0)) * Box(130.0, 120.0, 14.0)
    col_body = Location((-24.0, 0.0, col_h / 2.0)) * Box(46.0, 56.0, col_h)
    base_pedestal = base_plate + col_body

    # 2. Twin MGN12 linear rails + central T8 leadscrew along front of vertical column (x = 0)
    rail_l = Location((2.0, -18.0, col_h / 2.0)) * Box(8.0, 12.0, col_h - 24.0)
    rail_r = Location((2.0, 18.0, col_h / 2.0)) * Box(8.0, 12.0, col_h - 24.0)
    leadscrew = Location((4.0, 0.0, col_h / 2.0)) * Cylinder(radius=4.0, height=col_h - 16.0)
    linear_rail_z = Compound(children=[rail_l, rail_r, leadscrew])

    # 3. Sliding Z-carriage riding on rails at z = z_car
    car_block = Location((16.0, 0.0, z_car)) * Box(26.0, 62.0, 48.0)
    car_boss = Location((36.0, 0.0, z_car)) * Cylinder(radius=24.0, height=34.0)
    z_carriage = car_block + car_boss

    # Joint 1 pivot on front of Z-carriage
    p1_xy = (36.0, 0.0)
    p2_xy = (p1_xy[0] + l1 * math.cos(a1), p1_xy[1] + l1 * math.sin(a1))
    p3_xy = (p2_xy[0] + l2 * math.cos(a2), p2_xy[1] + l2 * math.sin(a2))

    # 4. Horizontal SCARA Arm 1 (in XY plane at z = z_car + 18)
    z_arm1 = z_car + 18.0 + t1 / 2.0
    scara_arm_1 = Location((p1_xy[0], p1_xy[1], z_arm1), (0.0, 0.0, a1_deg)) * arm1_raw

    # 5. Closed 3D GT2 Belt Loop coupling J1 to J2 above Arm 1
    belt_raw = build_closed_belt_loop_3d(center_dist_mm=l1, r1_mm=16.0, r2_mm=14.0, width_mm=7.0)
    z_belt = z_arm1 + t1 / 2.0 + 5.5
    belt_scara = Location((p1_xy[0], p1_xy[1], z_belt), (0.0, 0.0, a1_deg)) * belt_raw

    # 6. Horizontal SCARA Arm 2 (in XY plane below Arm 1 at z = z_arm1 - t1/2 - t2/2 - 2.0)
    z_arm2 = z_arm1 - t1 / 2.0 - t2 / 2.0 - 2.0
    scara_arm_2 = Location((p2_xy[0], p2_xy[1], z_arm2), (0.0, 0.0, a2_deg)) * arm2_raw

    # 7. Precision vertical Z-quill spline shaft + TCP flange at tip of Arm 2
    quill_h = max(90.0, z_stroke * 0.9)
    z_quill = Location((p3_xy[0], p3_xy[1], z_arm2 - quill_h * 0.25)) * Cylinder(
        radius=7.5, height=quill_h
    )
    tcp_flange = Location((p3_xy[0], p3_xy[1], z_arm2 - quill_h * 0.75 - 4.0)) * Cylinder(
        radius=16.0, height=8.0
    )

    # 8. Vertical NEMA motors + bearings + M8 homing sensor + controller box
    mech_cfg = getattr(req, "mechatronics", None)
    joint_acts = getattr(mech_cfg, "joint_actuators", None) or {}
    j1_motor_id = getattr(joint_acts.get("J1"), "motor_id", None) or "nema17_48mm"
    j2_motor_id = getattr(joint_acts.get("J2"), "motor_id", None) or "nema17_48mm"

    # Motors oriented vertically (shaft down along -Z: rotate 180° around X)
    rot_down = Location((0.0, 0.0, 0.0), (180.0, 0.0, 0.0))
    motor_j1_placed = Location((p1_xy[0], p1_xy[1], z_belt + 6.0)) * (rot_down * build_motor_3d(j1_motor_id))
    motor_j2_placed = Location((p2_xy[0], p2_xy[1], z_arm1 + t1 / 2.0 + 4.0)) * (
        rot_down * build_motor_3d(j2_motor_id)
    )

    bearing_j1 = Location((p1_xy[0], p1_xy[1], z_arm1)) * build_ball_bearing("6002")
    bearing_j2 = Location((p2_xy[0], p2_xy[1], z_arm2)) * build_ball_bearing("608")
    homing_sensor_z = Location((14.0, 34.0, z_car + 16.0), (0.0, 90.0, 0.0)) * build_homing_sensor()
    controller_box = Location((-95.0, 0.0, 0.0)) * build_controller_box()

    real_volumes_mm3 = {
        "base_pedestal": float(max(base_pedestal.volume, 1.0)),
        "shoulder_mount": float(max(z_carriage.volume, 1.0)),
        "shoulder_gear": float(max(belt_scara.volume, 1.0)),
        "upper_arm_link": float(max(arm1_raw.volume, 1.0)),
        "elbow_gear": float(max(bearing_j2.volume, 1.0)),
        "forearm_link": float(max(arm2_raw.volume, 1.0)),
        "wrist_tube": float(max(z_quill.volume + tcp_flange.volume, 1.0)),
    }

    # Export individual STEP/3MF files
    individual_files: list[Path] = []
    for part_solid, stem, vhint in (
        (base_pedestal, "base_pedestal", "tapered_solid"),
        (z_carriage, "z_carriage", "mount_gusseted"),
        (arm1_raw, "scara_arm_1", slots.upper_arm_variant),
        (arm2_raw, "scara_arm_2", slots.forearm_variant),
        (belt_scara, "belt_scara", "gear_solid"),
    ):
        s_p, m_p = _export_part_step_and_3mf(part_solid, output_dir, stem, vhint)
        individual_files.extend([s_p, m_p])

    full_compound = Compound(
        children=[
            base_pedestal,
            linear_rail_z,
            z_carriage,
            scara_arm_1,
            belt_scara,
            scara_arm_2,
            z_quill,
            tcp_flange,
            motor_j1_placed,
            motor_j2_placed,
            bearing_j1,
            bearing_j2,
            homing_sensor_z,
            controller_box,
        ]
    )
    step_path = output_dir / "arm_assembly.step"
    stl_path = output_dir / "arm_assembly.stl"
    glb_path = output_dir / "arm_assembly.glb"

    if not (step_path.exists() and step_path.stat().st_size > 0):
        export_step(full_compound, str(step_path))
    if not (stl_path.exists() and stl_path.stat().st_size > 0):
        export_stl(full_compound, str(stl_path), tolerance=0.12, angular_tolerance=0.20)

    if not (glb_path.exists() and glb_path.stat().st_size > 0):
        link_rgba = _hex_to_rgba(str(material.get("color_hex", "")), [62, 184, 204, 255])
        scene = trimesh.Scene()
        node_specs: list[tuple[str, Part | Compound, list[int], float, float]] = [
            ("base_pedestal", base_pedestal, [64, 76, 94, 255], 0.72, 0.32),
            ("linear_rail_z", linear_rail_z, [215, 222, 232, 255], 0.94, 0.16),
            ("z_carriage", z_carriage, [88, 128, 174, 255], 0.76, 0.28),
            ("scara_arm_1", scara_arm_1, link_rgba, 0.55, 0.30),
            ("belt_scara", belt_scara, [34, 36, 40, 255], 0.12, 0.82),
            ("scara_arm_2", scara_arm_2, link_rgba, 0.55, 0.30),
            ("z_quill", z_quill, [215, 222, 232, 255], 0.95, 0.14),
            ("tcp_flange", tcp_flange, [242, 182, 58, 255], 0.80, 0.26),
            ("motor_j1_shoulder", motor_j1_placed, [32, 36, 44, 255], 0.58, 0.42),
            ("motor_j2_elbow", motor_j2_placed, [32, 36, 44, 255], 0.58, 0.42),
            ("bearing_j1", bearing_j1, [215, 222, 232, 255], 0.96, 0.14),
            ("bearing_j2", bearing_j2, [215, 222, 232, 255], 0.96, 0.14),
            ("homing_sensor_z", homing_sensor_z, [212, 168, 62, 255], 0.85, 0.26),
            ("controller_box", controller_box, [34, 62, 56, 255], 0.42, 0.48),
        ]
        for node_name, solid, rgba, met, rough in node_specs:
            mesh = _part_to_trimesh(solid, rgba, met, rough, node_name)
            scene.add_geometry(mesh, node_name=node_name, geom_name=node_name)
        scene.export(str(glb_path), file_type="glb")

    explode_vectors: dict[str, list[float]] = {
        "base_pedestal": [0.0, 0.0, -35.0],
        "linear_rail_z": [18.0, 0.0, 0.0],
        "z_carriage": [32.0, 0.0, 12.0],
        "scara_arm_1": [round(36.0 * math.cos(a1), 2), round(36.0 * math.sin(a1), 2), 28.0],
        "belt_scara": [round(36.0 * math.cos(a1), 2), round(36.0 * math.sin(a1), 2), 62.0],
        "scara_arm_2": [
            round(36.0 * math.cos(a1) + 40.0 * math.cos(a2), 2),
            round(36.0 * math.sin(a1) + 40.0 * math.sin(a2), 2),
            -26.0,
        ],
        "z_quill": [
            round(36.0 * math.cos(a1) + 40.0 * math.cos(a2), 2),
            round(36.0 * math.sin(a1) + 40.0 * math.sin(a2), 2),
            -68.0,
        ],
        "tcp_flange": [
            round(36.0 * math.cos(a1) + 40.0 * math.cos(a2), 2),
            round(36.0 * math.sin(a1) + 40.0 * math.sin(a2), 2),
            -95.0,
        ],
        "motor_j1_shoulder": [24.0, 0.0, 88.0],
        "motor_j2_elbow": [round(36.0 * math.cos(a1), 2), round(36.0 * math.sin(a1), 2), 82.0],
        "bearing_j1": [24.0, 0.0, 42.0],
        "bearing_j2": [round(36.0 * math.cos(a1), 2), round(36.0 * math.sin(a1), 2), -14.0],
        "homing_sensor_z": [0.0, 40.0, 14.0],
        "controller_box": [-55.0, 0.0, -20.0],
    }

    printable_parts: dict[str, Part] = {
        "base_pedestal": base_pedestal,
        "z_carriage": z_carriage,
        "scara_arm_1": arm1_raw,
        "scara_arm_2": arm2_raw,
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


def export_scara_urdf(
    req: ModularArmRequest,
    joints: list[JointTorqueSummary],
    output_path: Path,
) -> Path:
    """Export a valid ROS 2 / Gazebo URDF XML for a 4-DOF SCARA robot with a real prismatic Z joint."""
    from ..physics.arm_compatibility import resolve_default_slots

    slots = resolve_default_slots(req)
    l1 = (float(slots.upper_arm.length_mm) if slots.upper_arm else 200.0) / 1000.0
    l2 = (float(slots.forearm.length_mm) if slots.forearm else 160.0) / 1000.0
    z_stroke = float(getattr(req, "z_stroke_mm", 120.0) or 120.0) / 1000.0
    l_base = float(req.base_height_mm) / 1000.0

    urdf_xml = f"""<?xml version="1.0" encoding="UTF-8"?>
<robot name="roboforge_scara_{saxutils.escape(req.material_id)}">
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 {l_base / 2.0:.4f}" rpy="0 0 0" />
      <mass value="1.2000" />
      <inertia ixx="0.004" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.003" />
    </inertial>
    <visual>
      <origin xyz="0 0 {l_base / 2.0:.4f}" rpy="0 0 0" />
      <geometry><box size="0.12 0.12 {l_base:.4f}" /></geometry>
    </visual>
  </link>

  <joint name="joint_3_z_prismatic" type="prismatic">
    <parent link="base_link" />
    <child link="z_carriage_link" />
    <origin xyz="0.036 0 {l_base:.4f}" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="0.0" upper="{z_stroke:.4f}" effort="80.0" velocity="0.25" />
  </joint>

  <link name="z_carriage_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="0.4500" />
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001" />
    </inertial>
  </link>

  <joint name="joint_1_scara_r1" type="revolute">
    <parent link="z_carriage_link" />
    <child link="scara_arm_1_link" />
    <origin xyz="0 0 0.025" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="-2.3562" upper="2.3562" effort="18.0" velocity="4.5" />
  </joint>

  <link name="scara_arm_1_link">
    <inertial>
      <origin xyz="{l1 / 2.0:.4f} 0 0" rpy="0 0 0" />
      <mass value="0.3200" />
      <inertia ixx="0.0008" ixy="0" ixz="0" iyy="0.0018" iyz="0" izz="0.0018" />
    </inertial>
    <visual>
      <origin xyz="{l1 / 2.0:.4f} 0 0" rpy="0 0 0" />
      <geometry><box size="{l1:.4f} 0.040 0.022" /></geometry>
    </visual>
  </link>

  <joint name="joint_2_scara_r2" type="revolute">
    <parent link="scara_arm_1_link" />
    <child link="scara_arm_2_link" />
    <origin xyz="{l1:.4f} 0 -0.024" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="-2.5307" upper="2.5307" effort="12.0" velocity="6.0" />
  </joint>

  <link name="scara_arm_2_link">
    <inertial>
      <origin xyz="{l2 / 2.0:.4f} 0 0" rpy="0 0 0" />
      <mass value="0.2400" />
      <inertia ixx="0.0005" ixy="0" ixz="0" iyy="0.0011" iyz="0" izz="0.0011" />
    </inertial>
    <visual>
      <origin xyz="{l2 / 2.0:.4f} 0 0" rpy="0 0 0" />
      <geometry><box size="{l2:.4f} 0.032 0.018" /></geometry>
    </visual>
  </link>

  <joint name="joint_4_tool_roll" type="revolute">
    <parent link="scara_arm_2_link" />
    <child link="tcp_flange" />
    <origin xyz="{l2:.4f} 0 -0.060" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="-3.1416" upper="3.1416" effort="4.0" velocity="8.0" />
  </joint>

  <link name="tcp_flange" />
</robot>
"""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(urdf_xml, encoding="utf-8")
    return output_path
