"""Parametric Surgical Continuum PCC (Piecewise Constant Curvature) 3D CAD assembler + URDF exporter."""

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
from .arm_assembly import _export_part_step_and_3mf, _part_to_trimesh
from .delta_assembly import _cylinder_between


def _pcc_frame(s_norm: float, arc_len: float, bend_rad: float, z0: float) -> tuple[tuple[float, float, float], float]:
    """Return 3D point (x, 0, z) and tangent pitch angle (rad from +Z toward +X) along a PCC circular arc."""
    s = s_norm * arc_len
    if abs(bend_rad) < 1e-3:
        return (0.0, 0.0, z0 + s), 0.0
    r_curv = arc_len / bend_rad
    theta = s_norm * bend_rad
    x = r_curv * (1.0 - math.cos(theta))
    z = z0 + r_curv * math.sin(theta)
    return (x, 0.0, z), theta


def build_continuum_pcc_assembly(
    req: ModularArmRequest,
    material: dict[str, Any],
    output_dir: Path,
    design_hash: str,
) -> dict[str, Any]:
    """Build a Piecewise Constant Curvature (PCC) continuum surgical robot:
    - continuum_actuator_base: cylindrical 3-tendon winch drive housing
    - continuum_backbone: curved Nitinol central spine swept along a PCC arc
    - continuum_spacer_disks: 8 PEEK spacer disks distributed normal to the PCC curve
    - continuum_tendons: 3 tensioned micro-tendons at 120° passing through all 8 spacer disks
    - micro_forceps: articulated surgical micro-forceps jaws at the distal tip
    """
    from ..physics.arm_compatibility import resolve_default_slots

    output_dir.mkdir(parents=True, exist_ok=True)
    slots = resolve_default_slots(req)

    base_h = min(70.0, max(45.0, float(req.base_height_mm) * 0.65))
    arc_len = max(85.0, min(180.0, float(slots.upper_arm.length_mm) if slots.upper_arm else 110.0))
    bend_deg = max(-85.0, min(85.0, float(req.joint_angles_deg[0]) if abs(req.joint_angles_deg[0]) > 5 else 42.0))
    bend_rad = math.radians(bend_deg)

    # 1. Cylindrical 3-Tendon Winch Actuator Base
    base_cyl = Location((0.0, 0.0, base_h / 2.0)) * Cylinder(radius=28.0, height=base_h)
    base_flange = Location((0.0, 0.0, 4.0)) * Cylinder(radius=34.0, height=8.0)
    continuum_actuator_base = base_cyl + base_flange

    # 2. Curved Nitinol Backbone (segmented smooth cylinders along exact PCC arc)
    n_seg = 16
    spine_pts = [_pcc_frame(i / n_seg, arc_len, bend_rad, base_h)[0] for i in range(n_seg + 1)]
    spine_segs = [
        _cylinder_between(spine_pts[i], spine_pts[i + 1], radius=1.6)
        for i in range(n_seg)
    ]
    continuum_backbone = Compound(children=spine_segs)

    # 3. 8 Precision PEEK Spacer Disks distributed along the PCC curve
    n_disks = 8
    disk_r = 9.0
    tendon_r_pcd = 6.4
    disk_solids: list[Part] = []
    tendon_pts_per_cable: list[list[tuple[float, float, float]]] = [[], [], []]

    for d_idx in range(n_disks + 1):
        s_norm = d_idx / n_disks
        pt, theta = _pcc_frame(s_norm, arc_len, bend_rad, base_h)
        # Normal plane vectors in XZ: local X' = (cos(theta), 0, -sin(theta)), local Y' = (0, 1, 0)
        nx_x, nx_z = math.cos(theta), -math.sin(theta)
        if d_idx >= 1:
            disk = Location(pt) * (
                Location((0.0, 0.0, 0.0), (0.0, math.degrees(theta), 0.0))
                * Cylinder(radius=disk_r, height=2.2)
            )
            disk_solids.append(disk)

        for c_idx in range(3):
            ang = math.radians(c_idx * 120.0)
            lx = tendon_r_pcd * math.cos(ang)
            ly = tendon_r_pcd * math.sin(ang)
            wx = pt[0] + lx * nx_x
            wy = pt[1] + ly
            wz = pt[2] + lx * nx_z
            tendon_pts_per_cable[c_idx].append((wx, wy, wz))

    continuum_spacer_disks = Compound(children=disk_solids)

    # 4. 3 Tensioned Micro-Tendons spaced at 120° passing cleanly through all 8 spacer disks
    tendon_segs: list[Part] = []
    for cable_pts in tendon_pts_per_cable:
        for i in range(len(cable_pts) - 1):
            tendon_segs.append(_cylinder_between(cable_pts[i], cable_pts[i + 1], radius=0.55))
    continuum_tendons = Compound(children=tendon_segs)

    # 5. Distal Surgical Micro-Forceps Jaws at the tip of the PCC curve
    tip_pt, tip_theta = _pcc_frame(1.0, arc_len, bend_rad, base_h)
    jaw_hub = Location((0.0, 0.0, 4.0)) * Cylinder(radius=4.5, height=8.0)
    jaw_l = Location((0.0, -3.0, 14.0), (14.0, 0.0, 0.0)) * Box(3.0, 2.2, 16.0)
    jaw_r = Location((0.0, +3.0, 14.0), (-14.0, 0.0, 0.0)) * Box(3.0, 2.2, 16.0)
    micro_forceps = Location(tip_pt) * (
        Location((0.0, 0.0, 0.0), (0.0, math.degrees(tip_theta), 0.0)) * (jaw_hub + jaw_l + jaw_r)
    )

    real_volumes_mm3 = {
        "base_pedestal": float(max(continuum_actuator_base.volume, 1.0)),
        "shoulder_mount": float(max(continuum_spacer_disks.volume, 1.0)),
        "shoulder_gear": float(max(continuum_tendons.volume, 1.0)),
        "upper_arm_link": float(max(continuum_backbone.volume + continuum_spacer_disks.volume, 1.0)),
        "elbow_gear": float(max(continuum_tendons.volume, 1.0)),
        "forearm_link": float(max(continuum_backbone.volume, 1.0)),
        "wrist_tube": float(max(micro_forceps.volume, 1.0)),
    }

    individual_files: list[Path] = []
    for solid, stem, vhint in (
        (continuum_actuator_base, "continuum_actuator_base", "tapered_solid"),
        (micro_forceps, "micro_forceps", "tapered_solid"),
    ):
        s_p, m_p = _export_part_step_and_3mf(solid, output_dir, stem, vhint)
        individual_files.extend([s_p, m_p])

    full_compound = Compound(
        children=[
            continuum_actuator_base,
            continuum_backbone,
            continuum_spacer_disks,
            continuum_tendons,
            micro_forceps,
        ]
    )
    step_path = output_dir / "arm_assembly.step"
    stl_path = output_dir / "arm_assembly.stl"
    glb_path = output_dir / "arm_assembly.glb"

    if not (step_path.exists() and step_path.stat().st_size > 0):
        export_step(full_compound, str(step_path))
    if not (stl_path.exists() and stl_path.stat().st_size > 0):
        export_stl(full_compound, str(stl_path), tolerance=0.10, angular_tolerance=0.18)

    if not (glb_path.exists() and glb_path.stat().st_size > 0):
        scene = trimesh.Scene()
        node_specs: list[tuple[str, Part | Compound, list[int], float, float]] = [
            ("continuum_actuator_base", continuum_actuator_base, [52, 62, 78, 255], 0.72, 0.30),
            ("continuum_backbone", continuum_backbone, [222, 228, 236, 255], 0.94, 0.16),
            ("continuum_spacer_disks", continuum_spacer_disks, [218, 174, 72, 255], 0.35, 0.42),
            ("continuum_tendons", continuum_tendons, [186, 196, 210, 255], 0.88, 0.24),
            ("micro_forceps", micro_forceps, [210, 218, 228, 255], 0.95, 0.15),
        ]
        for node_name, solid, rgba, met, rough in node_specs:
            mesh = _part_to_trimesh(solid, rgba, met, rough, node_name)
            scene.add_geometry(mesh, node_name=node_name, geom_name=node_name)
        scene.export(str(glb_path), file_type="glb")

    explode_vectors: dict[str, list[float]] = {
        "continuum_actuator_base": [0.0, 0.0, -32.0],
        "continuum_backbone": [0.0, 0.0, 14.0],
        "continuum_spacer_disks": [0.0, 32.0, 22.0],
        "continuum_tendons": [0.0, -32.0, 22.0],
        "micro_forceps": [round(40.0 * math.sin(tip_theta), 2), 0.0, round(40.0 * math.cos(tip_theta), 2)],
    }

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


def export_continuum_urdf(
    req: ModularArmRequest,
    joints: list[JointTorqueSummary],
    output_path: Path,
) -> Path:
    """Export a valid ROS 2 URDF approximating the PCC continuum backbone as articulated micro-segments."""
    urdf_xml = f"""<?xml version="1.0" encoding="UTF-8"?>
<robot name="roboforge_continuum_pcc_{saxutils.escape(req.material_id)}">
  <link name="continuum_actuator_base">
    <inertial><origin xyz="0 0 0.03" rpy="0 0 0"/><mass value="0.35"/><inertia ixx="0.0005" ixy="0" ixz="0" iyy="0.0005" iyz="0" izz="0.0005"/></inertial>
  </link>
  <joint name="pcc_seg_1_bend" type="revolute">
    <parent link="continuum_actuator_base"/><child link="pcc_seg_1"/>
    <origin xyz="0 0 0.055" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-1.48" upper="1.48" effort="1.2" velocity="4.0"/>
  </joint>
  <link name="pcc_seg_1"><inertial><origin xyz="0 0 0.025" rpy="0 0 0"/><mass value="0.03"/><inertia ixx="0.00002" ixy="0" ixz="0" iyy="0.00002" iyz="0" izz="0.00001"/></inertial></link>
  <joint name="pcc_seg_2_bend" type="revolute">
    <parent link="pcc_seg_1"/><child link="micro_forceps"/>
    <origin xyz="0 0 0.055" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-1.48" upper="1.48" effort="0.8" velocity="4.0"/>
  </joint>
  <link name="micro_forceps"/>
</robot>
"""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(urdf_xml, encoding="utf-8")
    return output_path
