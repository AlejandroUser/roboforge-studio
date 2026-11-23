"""Parametric 4-DOF Parallel Delta (ABB IRB 360 FlexPicker style) 3D CAD assembler + URDF exporter."""

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
    Sphere,
    export_step,
    export_stl,
)

from ..schemas import JointTorqueSummary, ModularArmRequest
from .arm_assembly import _export_part_step_and_3mf, _part_to_trimesh
from .hardware_parts import build_controller_box, build_motor_3d, build_vacuum_nozzle


def _cylinder_between(p_a: tuple[float, float, float], p_b: tuple[float, float, float], radius: float) -> Part:
    """Create a cylinder connecting 3D points p_a and p_b cleanly."""
    dx = p_b[0] - p_a[0]
    dy = p_b[1] - p_a[1]
    dz = p_b[2] - p_a[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length < 1e-3:
        return Cylinder(radius=radius, height=1.0)
    mid = ((p_a[0] + p_b[0]) / 2.0, (p_a[1] + p_b[1]) / 2.0, (p_a[2] + p_b[2]) / 2.0)
    # Spherical angles to align local +Z with (dx, dy, dz)
    phi = math.degrees(math.acos(max(-1.0, min(1.0, dz / length))))
    theta = math.degrees(math.atan2(dy, dx))
    # Rotate around Y by phi, then around Z by theta
    cyl = Location((0.0, 0.0, 0.0), (0.0, phi, 0.0)) * Cylinder(radius=radius, height=length)
    cyl = Location((0.0, 0.0, 0.0), (0.0, 0.0, theta)) * cyl
    return Location(mid) * cyl


def build_delta_parallel_assembly(
    req: ModularArmRequest,
    material: dict[str, Any],
    output_dir: Path,
    design_hash: str,
) -> dict[str, Any]:
    """Build a 4-DOF Parallel Delta robot (top triangular frame, 3 motors, 3 biceps, 3 parallel carbon
    forearm rod pairs with spherical ball joints, moving platform, central Z-shaft, and vacuum nozzle).
    """
    from ..physics.arm_compatibility import resolve_default_slots

    output_dir.mkdir(parents=True, exist_ok=True)
    slots = resolve_default_slots(req)

    r_top = 110.0
    r_plat = 42.0
    l_bicep = max(95.0, min(180.0, float(slots.upper_arm.length_mm) * 0.65 if slots.upper_arm else 125.0))
    z_top = max(290.0, req.base_height_mm + 210.0)
    z_plat = max(75.0, z_top - 220.0 + float(req.joint_angles_deg[0]) * 0.8)

    # 1. Top stationary frame ring + 3 structural support legs standing on Z = 0
    top_ring = Location((0.0, 0.0, z_top)) * (
        Cylinder(radius=r_top + 14.0, height=16.0) - Cylinder(radius=r_top - 22.0, height=20.0)
    )
    frame_legs: list[Part] = [top_ring]
    for i in range(3):
        leg_ang = math.radians(60.0 + i * 120.0)
        lx = (r_top + 6.0) * math.cos(leg_ang)
        ly = (r_top + 6.0) * math.sin(leg_ang)
        frame_legs.append(Location((lx, ly, z_top / 2.0)) * Cylinder(radius=9.0, height=z_top))
        frame_legs.append(Location((lx, ly, 5.0)) * Cylinder(radius=18.0, height=10.0))
    delta_top_frame = Compound(children=frame_legs)

    # 2. Moving End-Effector Platform + Central Telescopic Z-Rotation Shaft + Vacuum Nozzle
    delta_platform = Location((0.0, 0.0, z_plat)) * Cylinder(radius=r_plat, height=12.0)
    z_shaft_len = max(60.0, z_top - z_plat)
    delta_z_shaft = Location((0.0, 0.0, (z_top + z_plat) / 2.0)) * Cylinder(radius=5.5, height=z_shaft_len)
    vacuum_nozzle = Location((0.0, 0.0, z_plat - 6.0)) * build_vacuum_nozzle()

    # 3. Three 120°-symmetric parallel kinematic chains (Motor + Bicep + Parallel Carbon Forearm Pair)
    motor_raw = build_motor_3d("nema23_76mm" if req.payload_kg >= 1.5 else "nema17_48mm")
    bicep_angle = math.radians(22.0)

    motors: list[Part] = []
    biceps: list[Part] = []
    forearms: list[Compound] = []
    explode_vectors: dict[str, list[float]] = {
        "delta_top_frame": [0.0, 0.0, 48.0],
        "delta_platform": [0.0, 0.0, -55.0],
        "delta_z_shaft": [0.0, 0.0, -25.0],
        "vacuum_nozzle": [0.0, 0.0, -92.0],
        "controller_box": [-55.0, 0.0, -20.0],
    }

    for i in range(3):
        ang = math.radians(i * 120.0)
        ca, sa = math.cos(ang), math.sin(ang)
        # Tangent unit vector for parallel rod spacing (±18 mm)
        tx, ty = -sa, ca

        # Shoulder pivot on top frame
        p_sh = (r_top * ca, r_top * sa, z_top - 4.0)
        # Elbow point at end of bicep
        r_elbow = r_top + l_bicep * math.cos(bicep_angle)
        z_elbow = z_top - 4.0 - l_bicep * math.sin(bicep_angle)
        p_el = (r_elbow * ca, r_elbow * sa, z_elbow)

        # Platform attachment point
        p_pl = (r_plat * ca, r_plat * sa, z_plat)

        # Motor mounted tangentially at p_sh
        m_placed = Location(p_sh) * (
            Location((0.0, 0.0, 0.0), (0.0, 0.0, math.degrees(ang) + 90.0))
            * (Location((0.0, 0.0, 0.0), (90.0, 0.0, 0.0)) * motor_raw)
        )
        motors.append(m_placed)

        # Bicep arm connecting p_sh -> p_el
        bicep_bar = _cylinder_between(p_sh, p_el, radius=7.5)
        bicep_bar = bicep_bar + Location(p_sh) * Sphere(11.0) + Location(p_el) * Sphere(9.5)
        biceps.append(bicep_bar)

        # Parallel carbon forearm rod pair (±16 mm tangential offset) with spherical ball joints
        off = 16.0
        rod_a_p0 = (p_el[0] + off * tx, p_el[1] + off * ty, p_el[2])
        rod_a_p1 = (p_pl[0] + off * tx, p_pl[1] + off * ty, p_pl[2])
        rod_b_p0 = (p_el[0] - off * tx, p_el[1] - off * ty, p_el[2])
        rod_b_p1 = (p_pl[0] - off * tx, p_pl[1] - off * ty, p_pl[2])

        rod_a = _cylinder_between(rod_a_p0, rod_a_p1, radius=3.2)
        rod_b = _cylinder_between(rod_b_p0, rod_b_p1, radius=3.2)
        balls = [
            Location(pt) * Sphere(5.5)
            for pt in (rod_a_p0, rod_a_p1, rod_b_p0, rod_b_p1)
        ]
        forearms.append(Compound(children=[rod_a, rod_b, *balls]))

        idx = i + 1
        explode_vectors[f"delta_motor_{idx}"] = [round(45.0 * ca, 2), round(45.0 * sa, 2), 48.0]
        explode_vectors[f"delta_bicep_{idx}"] = [round(58.0 * ca, 2), round(58.0 * sa, 2), 18.0]
        explode_vectors[f"delta_forearms_{idx}"] = [round(52.0 * ca, 2), round(52.0 * sa, 2), -26.0]

    controller_box = Location((-(r_top + 65.0), 0.0, 0.0)) * build_controller_box()

    real_volumes_mm3 = {
        "base_pedestal": float(max(delta_top_frame.volume, 1.0)),
        "shoulder_mount": float(max(delta_platform.volume, 1.0)),
        "shoulder_gear": float(max(biceps[0].volume, 1.0)),
        "upper_arm_link": float(max(sum(b.volume for b in biceps), 1.0)),
        "elbow_gear": float(max(forearms[0].volume, 1.0)),
        "forearm_link": float(max(sum(f.volume for f in forearms), 1.0)),
        "wrist_tube": float(max(delta_z_shaft.volume + vacuum_nozzle.volume, 1.0)),
    }

    individual_files: list[Path] = []
    for solid, stem, vhint in (
        (delta_platform, "delta_platform", "mount_standard"),
        (biceps[0], "delta_bicep_1", "tapered_solid"),
    ):
        s_p, m_p = _export_part_step_and_3mf(solid, output_dir, stem, vhint)
        individual_files.extend([s_p, m_p])

    full_compound = Compound(
        children=[
            delta_top_frame,
            delta_platform,
            delta_z_shaft,
            vacuum_nozzle,
            controller_box,
            *motors,
            *biceps,
            *forearms,
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
            ("delta_top_frame", delta_top_frame, [88, 134, 188, 255], 0.82, 0.26),
            ("delta_platform", delta_platform, [92, 168, 212, 255], 0.80, 0.26),
            ("delta_z_shaft", delta_z_shaft, [215, 222, 232, 255], 0.94, 0.16),
            ("vacuum_nozzle", vacuum_nozzle, [218, 176, 62, 255], 0.82, 0.26),
            ("controller_box", controller_box, [34, 62, 56, 255], 0.42, 0.48),
        ]
        for i in range(3):
            idx = i + 1
            node_specs.append((f"delta_motor_{idx}", motors[i], [32, 36, 44, 255], 0.58, 0.42))
            node_specs.append((f"delta_bicep_{idx}", biceps[i], [74, 176, 194, 255], 0.72, 0.28))
            node_specs.append((f"delta_forearms_{idx}", forearms[i], [34, 38, 44, 255], 0.42, 0.34))

        for node_name, solid, rgba, met, rough in node_specs:
            mesh = _part_to_trimesh(solid, rgba, met, rough, node_name)
            scene.add_geometry(mesh, node_name=node_name, geom_name=node_name)
        scene.export(str(glb_path), file_type="glb")

    return {
        "real_volumes_mm3": real_volumes_mm3,
        "explode_vectors": explode_vectors,
        "individual_files": individual_files,
        "step_path": step_path,
        "stl_path": stl_path,
        "glb_path": glb_path,
        "slots_used": slots,
        "design_hash": design_hash,
    }


def export_delta_urdf(
    req: ModularArmRequest,
    joints: list[JointTorqueSummary],
    output_path: Path,
) -> Path:
    """Export a valid ROS 2 URDF describing the 4-DOF Parallel Delta robot kinematic branches."""
    urdf_xml = f"""<?xml version="1.0" encoding="UTF-8"?>
<robot name="roboforge_delta_parallel_{saxutils.escape(req.material_id)}">
  <link name="delta_top_frame">
    <inertial><origin xyz="0 0 0.30" rpy="0 0 0"/><mass value="2.5"/><inertia ixx="0.03" ixy="0" ixz="0" iyy="0.03" iyz="0" izz="0.04"/></inertial>
  </link>
  <joint name="delta_arm_1_joint" type="revolute">
    <parent link="delta_top_frame"/><child link="delta_bicep_1"/>
    <origin xyz="0.11 0 0.30" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-0.6" upper="1.3" effort="25.0" velocity="8.0"/>
  </joint>
  <link name="delta_bicep_1"><inertial><origin xyz="0.06 0 0" rpy="0 0 0"/><mass value="0.18"/><inertia ixx="0.0004" ixy="0" ixz="0" iyy="0.0004" iyz="0" izz="0.0004"/></inertial></link>
  <joint name="delta_arm_2_joint" type="revolute">
    <parent link="delta_top_frame"/><child link="delta_bicep_2"/>
    <origin xyz="-0.055 0.095 0.30" rpy="0 0 2.0944"/><axis xyz="0 1 0"/>
    <limit lower="-0.6" upper="1.3" effort="25.0" velocity="8.0"/>
  </joint>
  <link name="delta_bicep_2"><inertial><origin xyz="0.06 0 0" rpy="0 0 0"/><mass value="0.18"/><inertia ixx="0.0004" ixy="0" ixz="0" iyy="0.0004" iyz="0" izz="0.0004"/></inertial></link>
  <joint name="delta_arm_3_joint" type="revolute">
    <parent link="delta_top_frame"/><child link="delta_bicep_3"/>
    <origin xyz="-0.055 -0.095 0.30" rpy="0 0 -2.0944"/><axis xyz="0 1 0"/>
    <limit lower="-0.6" upper="1.3" effort="25.0" velocity="8.0"/>
  </joint>
  <link name="delta_bicep_3"><inertial><origin xyz="0.06 0 0" rpy="0 0 0"/><mass value="0.18"/><inertia ixx="0.0004" ixy="0" ixz="0" iyy="0.0004" iyz="0" izz="0.0004"/></inertial></link>
  <joint name="delta_z_tool_roll" type="revolute">
    <parent link="delta_top_frame"/><child link="delta_platform"/>
    <origin xyz="0 0 0.09" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.1416" upper="3.1416" effort="4.0" velocity="15.0"/>
  </joint>
  <link name="delta_platform"/>
</robot>
"""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(urdf_xml, encoding="utf-8")
    return output_path
