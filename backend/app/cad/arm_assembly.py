"""Parametric full robotic arm assembly (4-6 DOF) CAD builder + URDF, MuJoCo, and BOM CSV exporters."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import tempfile
import xml.sax.saxutils as saxutils
from pathlib import Path
from typing import Any

import trimesh
from build123d import (
    Compound,
    Cylinder,
    Location,
    Part,
    export_step,
    export_stl,
)

from ..schemas import (
    ArmAssemblyRequest,
    BOMItem,
    EndInterfaceParams,
    JointTorqueSummary,
    LinkDesignRequest,
    ModularArmRequest,
)
from .cycloidal_variants import (
    build_bevel_differential_wrist,
    build_capstan_drum_and_tendons,
    build_cycloidal_reducer_3d,
    build_planetary_reducer_3d,
)
from .gear_variants import _build_gear_solid, _build_gear_spoked, _build_gear_webbed
from .hardware_parts import (
    build_ball_bearing,
    build_bearing_crb,
    build_bearing_tapered_30205,
    build_controller_box,
    build_dual_servo_clevis,
    build_four_bar_parallelogram_rod,
    build_gripper_3d,
    build_gripper_parts,
    build_homing_sensor,
    build_iso4762_screw,
    build_motor_3d,
    build_servo_3d,
    build_servo_horn_25t,
    build_vacuum_nozzle,
    build_wrist_3dof_parts,
)
from .mount_variants import (
    _build_mount_gusseted,
    _build_mount_lightweight,
    _build_mount_standard,
    _compute_mount_envelope,
)
from .threemf_exporter import compute_modifier_box_for_mesh, export_smart_3mf
from .variants import (
    _build_hollow_cobot,
    _build_organic_ibeam,
    _build_organic_topopt,
    _build_rounded_truss,
    _build_tapered_solid,
)

# In-memory cache of generated build123d sub-parts for < 1s modular re-assembly
_SUBPART_CACHE: dict[str, Part] = {}


def compute_arm_design_hash(req: ArmAssemblyRequest) -> str:
    """Compute a deterministic 16-char SHA-256 hex digest for an ArmAssemblyRequest."""
    canonical = "arm:" + json.dumps(req.model_dump(), sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:16]


def _hex_to_rgba(hex_color: str, fallback: list[int]) -> list[int]:
    try:
        s = hex_color.strip().lstrip("#")
        if len(s) == 6:
            return [int(s[0:2], 16), int(s[2:4], 16), int(s[4:6], 16), 255]
    except Exception:
        pass
    return fallback


def _part_to_trimesh(part: Part | Compound, rgba: list[int], metallic: float, roughness: float, name: str) -> trimesh.Trimesh:
    with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as tmp:
        tmp_path = Path(tmp.name)
    try:
        export_stl(part, str(tmp_path), tolerance=0.14, angular_tolerance=0.22)
        loaded = trimesh.load(str(tmp_path), force="mesh")
        mesh = loaded.dump(concatenate=True) if isinstance(loaded, trimesh.Scene) else loaded
        if not isinstance(mesh, trimesh.Trimesh):
            raise ValueError(f"Expected Trimesh for {name}")
        mesh.fix_normals()
        mat = trimesh.visual.material.PBRMaterial(
            name=name,
            baseColorFactor=rgba,
            metallicFactor=metallic,
            roughnessFactor=roughness,
        )
        mesh.visual = trimesh.visual.TextureVisuals(material=mat)
        return mesh
    finally:
        if tmp_path.exists():
            try:
                tmp_path.unlink()
            except OSError:
                pass


def _build_styled_segment(
    length_mm: float,
    root_w: float,
    tip_w: float,
    thickness: float,
    style: str,
) -> Part:
    """Build a single styled robotic arm segment along +X in the XY plane."""
    req = LinkDesignRequest(
        length_mm=max(60.0, min(450.0, length_mm)),
        root_width_mm=max(16.0, min(110.0, root_w)),
        tip_width_mm=max(12.0, min(90.0, tip_w)),
        thickness_mm=max(10.0, min(60.0, thickness)),
        organic_taper=0.72,
        cable_channel=False,
        root_end=EndInterfaceParams(
            preset="custom",
            outer_diameter_mm=max(20.0, min(110.0, root_w * 1.12)),
            bore_diameter_mm=max(8.0, min(50.0, root_w * 0.42)),
            bolt_count=0,
            bolt_pcd_mm=max(14.0, root_w * 0.7),
            bolt_size="M3",
            use_heat_inserts=False,
        ),
        tip_end=EndInterfaceParams(
            preset="custom",
            outer_diameter_mm=max(16.0, min(95.0, tip_w * 1.12)),
            bore_diameter_mm=max(6.0, min(40.0, tip_w * 0.42)),
            bolt_count=0,
            bolt_pcd_mm=max(12.0, tip_w * 0.7),
            bolt_size="M3",
            use_heat_inserts=False,
        ),
    )
    builders = {
        "tapered_solid": _build_tapered_solid,
        "organic_ibeam": _build_organic_ibeam,
        "rounded_truss": _build_rounded_truss,
        "hollow_cobot": _build_hollow_cobot,
    }
    builder = builders.get(style, _build_hollow_cobot)
    return builder(req)


def _place_link_in_xz(link: Part, origin: tuple[float, float, float], pitch_deg: float) -> Part:
    """Place a link built in XY (+X along length, Z thickness) into XZ plane at pitch_deg from +X toward +Z."""
    # 1. Rotate +90° around X -> length stays +X, width goes to +Z, thickness goes to -Y
    # 2. Rotate -pitch_deg around Y -> maps +X to (cos(pitch), 0, sin(pitch))
    # 3. Translate to origin
    x_rot = Location((0.0, 0.0, 0.0), (90.0, 0.0, 0.0)) * link
    y_rot = Location((0.0, 0.0, 0.0), (0.0, -pitch_deg, 0.0)) * x_rot
    return Location(origin) * y_rot


def _build_base_pedestal(
    base_height_mm: float,
    payload_scale: float,
    compact_msla: bool = False,
) -> Part:
    """Build a bolted cylindrical robot base pedestal standing on z = 0..base_height_mm.
    When compact_msla=True (Mars-Mini / Elegoo Mars 5 Ultra), constrains flange to Ø64 mm
    and adds an Ø3.5 mm anti-suction-cup resin drain hole.
    """
    if compact_msla:
        flange_h = min(8.0, base_height_mm * 0.16)
        col_r = 21.0
        flange_r = 32.0  # Exact Ø64.0 mm flange fits 77.76 mm MSLA plate with >13 mm margin
        flange = Location((0.0, 0.0, flange_h / 2.0)) * Cylinder(radius=flange_r, height=flange_h)
        column = Location((0.0, 0.0, base_height_mm / 2.0)) * Cylinder(radius=col_r, height=base_height_mm)
        base = flange + column
        # 4×M3 anchor bolt holes (Ø3.25 mm)
        pcd_r = 26.5
        for i in range(4):
            ang = math.pi / 4.0 + i * (math.pi / 2.0)
            hole = Location((pcd_r * math.cos(ang), pcd_r * math.sin(ang), flange_h / 2.0)) * Cylinder(
                radius=1.625, height=flange_h + 4.0
            )
            try:
                base = base - hole
            except Exception:
                pass
        # Anti-suction-cup Ø3.5 mm resin drain hole through center
        try:
            base = base - (Location((0.0, 0.0, base_height_mm / 2.0)) * Cylinder(radius=1.75, height=base_height_mm + 4.0))
        except Exception:
            pass
        # Real Ø22.2 mm × 7.2 mm 608 bearing counterbore seat on top of column
        try:
            bearing_seat = Location((0.0, 0.0, base_height_mm - 3.6)) * Cylinder(radius=11.1, height=7.4)
            cand = base - bearing_seat
            if cand.is_valid and cand.volume > 0:
                base = cand
        except Exception:
            pass
        return base

    flange_h = min(12.0, base_height_mm * 0.16)
    col_r = 32.0 * payload_scale
    flange_r = col_r + 14.0

    flange = Location((0.0, 0.0, flange_h / 2.0)) * Cylinder(radius=flange_r, height=flange_h)
    column = Location((0.0, 0.0, base_height_mm / 2.0)) * Cylinder(radius=col_r, height=base_height_mm)
    base = flange + column

    # 4×M5 anchor bolt holes around mounting flange
    pcd_r = col_r + 7.5
    for i in range(4):
        ang = math.pi / 4.0 + i * (math.pi / 2.0)
        bx = pcd_r * math.cos(ang)
        by = pcd_r * math.sin(ang)
        hole = Location((bx, by, flange_h / 2.0)) * Cylinder(radius=2.75, height=flange_h + 4.0)
        try:
            base = base - hole
        except Exception:
            pass
    return base


def _build_joint_housing(center: tuple[float, float, float], radius: float, length_y: float) -> Part:
    """Build a cylindrical actuator/reducer housing centered at `center` with axis along Y."""
    housing = (
        Location(center, (90.0, 0.0, 0.0))
        * Cylinder(radius=radius, height=length_y)
    )
    cap_r = radius * 0.72
    cap = (
        Location(center, (90.0, 0.0, 0.0))
        * Cylinder(radius=cap_r, height=length_y + 6.0)
    )
    return housing + cap


def _build_wrist_and_tcp(
    p_wrist: tuple[float, float, float],
    wrist_mm: float,
    pitch_deg: float,
    payload_scale: float,
) -> tuple[Part, Part]:
    """Build the wrist tube and ISO-style 4-bolt TCP tool flange."""
    tube_r = 13.0 * payload_scale
    flange_r = 18.5 * payload_scale
    flange_h = 8.0
    tube_len = max(12.0, wrist_mm - flange_h)

    # Local frame: tube along +X from x=0 to x=tube_len, flange from x=tube_len to x=wrist_mm
    tube_local = (
        Location((tube_len / 2.0, 0.0, 0.0), (0.0, 90.0, 0.0))
        * Cylinder(radius=tube_r, height=tube_len)
    )
    flange_local = (
        Location((tube_len + flange_h / 2.0, 0.0, 0.0), (0.0, 90.0, 0.0))
        * Cylinder(radius=flange_r, height=flange_h)
    )
    # 4×M4 TCP bolt holes
    pcd_r = flange_r * 0.68
    for i in range(4):
        ang = math.pi / 4.0 + i * (math.pi / 2.0)
        hy = pcd_r * math.cos(ang)
        hz = pcd_r * math.sin(ang)
        bolt = (
            Location((tube_len + flange_h / 2.0, hy, hz), (0.0, 90.0, 0.0))
            * Cylinder(radius=2.2, height=flange_h + 4.0)
        )
        try:
            flange_local = flange_local - bolt
        except Exception:
            pass

    wrist_part = Location(p_wrist) * (Location((0.0, 0.0, 0.0), (0.0, -pitch_deg, 0.0)) * tube_local)
    flange_part = Location(p_wrist) * (Location((0.0, 0.0, 0.0), (0.0, -pitch_deg, 0.0)) * flange_local)
    return wrist_part, flange_part


def build_and_export_arm_assembly(
    req: ArmAssemblyRequest,
    material: dict[str, Any],
    out_dir: Path,
    design_hash: str,
) -> dict[str, Any]:
    """Build articulated 3D robotic arm assembly, export STEP/STL/GLB, and return segment volumes."""
    out_dir.mkdir(parents=True, exist_ok=True)

    # Scale cross-section cleanly with payload
    scale = max(0.82, min(1.35, 0.88 + 0.12 * math.sqrt(req.payload_kg)))

    a1_deg = req.shoulder_joint_deg
    a2_deg = a1_deg + req.elbow_joint_deg
    a3_deg = a2_deg + req.wrist_joint_deg

    a1 = math.radians(a1_deg)
    a2 = math.radians(a2_deg)

    p0 = (0.0, 0.0, req.base_height_mm)
    p1 = (p0[0] + req.upper_arm_mm * math.cos(a1), 0.0, p0[2] + req.upper_arm_mm * math.sin(a1))
    p2 = (p1[0] + req.forearm_mm * math.cos(a2), 0.0, p1[2] + req.forearm_mm * math.sin(a2))

    # 1. Base pedestal
    base_part = _build_base_pedestal(req.base_height_mm, scale)

    # 2. Upper arm & Forearm styled bionic links
    upper_t = round(24.0 * scale, 1)
    forearm_t = round(19.0 * scale, 1)

    upper_raw = _build_styled_segment(
        length_mm=req.upper_arm_mm,
        root_w=44.0 * scale,
        tip_w=34.0 * scale,
        thickness=upper_t,
        style=req.link_style,
    )
    upper_part = _place_link_in_xz(upper_raw, p0, a1_deg)

    forearm_raw = _build_styled_segment(
        length_mm=req.forearm_mm,
        root_w=34.0 * scale,
        tip_w=24.0 * scale,
        thickness=forearm_t,
        style=req.link_style,
    )
    forearm_part = _place_link_in_xz(forearm_raw, p1, a2_deg)

    # 3. Joint housings (Shoulder, Elbow, Wrist)
    j_shoulder = _build_joint_housing(p0, radius=27.0 * scale, length_y=upper_t + 24.0)
    j_elbow = _build_joint_housing(p1, radius=21.0 * scale, length_y=upper_t + 18.0)
    j_wrist = _build_joint_housing(p2, radius=16.0 * scale, length_y=forearm_t + 14.0)
    joints_compound = Compound(children=[j_shoulder, j_elbow, j_wrist])

    # 4. Wrist tube + TCP flange
    wrist_tube, tcp_flange = _build_wrist_and_tcp(p2, req.wrist_mm, a3_deg, scale)

    # Compute exact solid volumes (mm³) for physics/BOM evaluation
    segment_volumes_mm3 = {
        "base": float(max(base_part.volume, 1.0)),
        "upper_arm": float(max(upper_part.volume, 1.0)),
        "forearm": float(max(forearm_part.volume, 1.0)),
        "wrist": float(max(wrist_tube.volume + tcp_flange.volume, 1.0)),
    }

    # Export combined STEP & STL
    full_assembly = Compound(
        children=[base_part, upper_part, forearm_part, j_shoulder, j_elbow, j_wrist, wrist_tube, tcp_flange]
    )
    step_path = out_dir / "arm_assembly.step"
    stl_path = out_dir / "arm_assembly.stl"
    glb_path = out_dir / "arm_assembly.glb"

    if not (step_path.exists() and step_path.stat().st_size > 0):
        export_step(full_assembly, str(step_path))
    if not (stl_path.exists() and stl_path.stat().st_size > 0):
        export_stl(full_assembly, str(stl_path), tolerance=0.14, angular_tolerance=0.22)

    # Export hierarchical multi-material GLB scene for 60 FPS real-time FK in Three.js
    if not (glb_path.exists() and glb_path.stat().st_size > 0):
        import numpy as np
        from trimesh.transformations import rotation_matrix, translation_matrix

        link_rgba = _hex_to_rgba(str(material.get("color_hex", "")), [62, 184, 204, 255])
        joint_rgba = [42, 48, 62, 255]

        # Local zero-pose meshes (each segment starts at origin along +X in XZ plane)
        upper_local = _place_link_in_xz(upper_raw, (0.0, 0.0, 0.0), 0.0)
        forearm_local = _place_link_in_xz(forearm_raw, (0.0, 0.0, 0.0), 0.0)
        j_shoulder_local = _build_joint_housing((0.0, 0.0, 0.0), radius=27.0 * scale, length_y=upper_t + 24.0)
        j_elbow_local = _build_joint_housing((0.0, 0.0, 0.0), radius=21.0 * scale, length_y=upper_t + 18.0)
        j_wrist_local = _build_joint_housing((0.0, 0.0, 0.0), radius=16.0 * scale, length_y=forearm_t + 14.0)
        wrist_local, tcp_local = _build_wrist_and_tcp((0.0, 0.0, 0.0), req.wrist_mm, 0.0, scale)

        scene = trimesh.Scene()
        scene.add_geometry(
            _part_to_trimesh(base_part, [74, 88, 108, 255], 0.65, 0.35, "arm_base"),
            node_name="base_pedestal",
            parent_node_name="world",
        )

        # Joint 1: Shoulder pivot at (0, 0, base_height_mm), rotated around Y by -shoulder_joint_deg
        t_j1 = translation_matrix([0.0, 0.0, float(req.base_height_mm)]) @ rotation_matrix(
            math.radians(-req.shoulder_joint_deg), [0.0, 1.0, 0.0]
        )
        scene.graph.update(frame_to="joint_shoulder", frame_from="world", matrix=t_j1)
        scene.add_geometry(
            _part_to_trimesh(j_shoulder_local, joint_rgba, 0.72, 0.30, "shoulder_housing"),
            node_name="shoulder_housing",
            parent_node_name="joint_shoulder",
        )
        scene.add_geometry(
            _part_to_trimesh(upper_local, link_rgba, 0.55, 0.32, "upper_arm_link"),
            node_name="upper_arm_link",
            parent_node_name="joint_shoulder",
        )

        # Joint 2: Elbow pivot at (upper_arm_mm, 0, 0) in shoulder frame
        t_j2 = translation_matrix([float(req.upper_arm_mm), 0.0, 0.0]) @ rotation_matrix(
            math.radians(-req.elbow_joint_deg), [0.0, 1.0, 0.0]
        )
        scene.graph.update(frame_to="joint_elbow", frame_from="joint_shoulder", matrix=t_j2)
        scene.add_geometry(
            _part_to_trimesh(j_elbow_local, joint_rgba, 0.72, 0.30, "elbow_housing"),
            node_name="elbow_housing",
            parent_node_name="joint_elbow",
        )
        scene.add_geometry(
            _part_to_trimesh(forearm_local, link_rgba, 0.55, 0.32, "forearm_link"),
            node_name="forearm_link",
            parent_node_name="joint_elbow",
        )

        # Joint 3: Wrist pivot at (forearm_mm, 0, 0) in elbow frame
        t_j3 = translation_matrix([float(req.forearm_mm), 0.0, 0.0]) @ rotation_matrix(
            math.radians(-req.wrist_joint_deg), [0.0, 1.0, 0.0]
        )
        scene.graph.update(frame_to="joint_wrist", frame_from="joint_elbow", matrix=t_j3)
        scene.add_geometry(
            _part_to_trimesh(j_wrist_local, joint_rgba, 0.72, 0.30, "wrist_housing"),
            node_name="wrist_housing",
            parent_node_name="joint_wrist",
        )
        scene.add_geometry(
            _part_to_trimesh(wrist_local, link_rgba, 0.55, 0.32, "wrist_tube"),
            node_name="wrist_tube",
            parent_node_name="joint_wrist",
        )
        scene.add_geometry(
            _part_to_trimesh(tcp_local, [242, 182, 58, 255], 0.70, 0.28, "tcp_flange"),
            node_name="tcp_flange",
            parent_node_name="joint_wrist",
        )
        scene.export(str(glb_path), file_type="glb")

    return {
        "segment_volumes_mm3": segment_volumes_mm3,
        "step_path": step_path,
        "stl_path": stl_path,
        "glb_path": glb_path,
        "design_hash": design_hash,
    }


def export_arm_urdf(
    req: ArmAssemblyRequest,
    joints: list[JointTorqueSummary],
    output_path: Path,
) -> Path:
    """Export a valid ROS 2 / Gazebo URDF XML file for the articulated robotic arm."""
    l_base = req.base_height_mm / 1000.0
    l_upper = req.upper_arm_mm / 1000.0
    l_fore = req.forearm_mm / 1000.0
    l_wrist = req.wrist_mm / 1000.0

    # Extract segment masses (kg) and joint efforts (N·m) from JointTorqueSummary
    j_map = {j.joint_id.upper(): j for j in joints}
    j1 = j_map.get("J1") or (joints[0] if joints else None)
    j2 = j_map.get("J2") or (joints[1] if len(joints) > 1 else j1)
    j3 = j_map.get("J3") or (joints[2] if len(joints) > 2 else j2)

    m_upper = max(0.05, (j1.segment_mass_g if j1 else 250.0) / 1000.0)
    m_fore = max(0.04, (j2.segment_mass_g if j2 else 180.0) / 1000.0)
    m_wrist = max(0.03, (j3.segment_mass_g if j3 else 100.0) / 1000.0)

    eff1 = max(5.0, round((j1.dynamic_torque_nm if j1 else 15.0) * 1.5, 2))
    eff2 = max(3.0, round((j2.dynamic_torque_nm if j2 else 8.0) * 1.5, 2))
    eff3 = max(1.5, round((j3.dynamic_torque_nm if j3 else 3.0) * 1.5, 2))

    def _box_inertia(m: float, lx: float, ly: float, lz: float) -> str:
        ixx = max(1e-5, m * (ly * ly + lz * lz) / 12.0)
        iyy = max(1e-5, m * (lx * lx + lz * lz) / 12.0)
        izz = max(1e-5, m * (lx * lx + ly * ly) / 12.0)
        return f'ixx="{ixx:.6f}" ixy="0" ixz="0" iyy="{iyy:.6f}" iyz="0" izz="{izz:.6f}"'

    urdf_xml = f"""<?xml version="1.0" encoding="UTF-8"?>
<robot name="roboforge_{saxutils.escape(req.preset)}">
  <material name="roboforge_link">
    <color rgba="0.24 0.72 0.80 1.0" />
  </material>
  <material name="roboforge_dark">
    <color rgba="0.18 0.22 0.28 1.0" />
  </material>
  <material name="roboforge_amber">
    <color rgba="0.95 0.71 0.23 1.0" />
  </material>

  <link name="base_link">
    <inertial>
      <origin xyz="0 0 {l_base / 2.0:.4f}" rpy="0 0 0" />
      <mass value="0.6500" />
      <inertia {_box_inertia(0.65, 0.08, 0.08, l_base)} />
    </inertial>
    <visual>
      <origin xyz="0 0 {l_base / 2.0:.4f}" rpy="0 0 0" />
      <geometry><cylinder radius="0.035" length="{l_base:.4f}" /></geometry>
      <material name="roboforge_dark" />
    </visual>
    <collision>
      <origin xyz="0 0 {l_base / 2.0:.4f}" rpy="0 0 0" />
      <geometry><cylinder radius="0.035" length="{l_base:.4f}" /></geometry>
    </collision>
  </link>

  <joint name="joint_1_shoulder" type="revolute">
    <parent link="base_link" />
    <child link="upper_arm_link" />
    <origin xyz="0 0 {l_base:.4f}" rpy="0 0 0" />
    <axis xyz="0 -1 0" />
    <limit lower="-1.0472" upper="1.5708" effort="{eff1}" velocity="3.14" />
  </joint>

  <link name="upper_arm_link">
    <inertial>
      <origin xyz="{l_upper / 2.0:.4f} 0 0" rpy="0 0 0" />
      <mass value="{m_upper:.4f}" />
      <inertia {_box_inertia(m_upper, l_upper, 0.028, 0.044)} />
    </inertial>
    <visual>
      <origin xyz="{l_upper / 2.0:.4f} 0 0" rpy="0 0 0" />
      <geometry><box size="{l_upper:.4f} 0.026 0.042" /></geometry>
      <material name="roboforge_link" />
    </visual>
    <collision>
      <origin xyz="{l_upper / 2.0:.4f} 0 0" rpy="0 0 0" />
      <geometry><box size="{l_upper:.4f} 0.026 0.042" /></geometry>
    </collision>
  </link>

  <joint name="joint_2_elbow" type="revolute">
    <parent link="upper_arm_link" />
    <child link="forearm_link" />
    <origin xyz="{l_upper:.4f} 0 0" rpy="0 0 0" />
    <axis xyz="0 -1 0" />
    <limit lower="-2.2689" upper="2.2689" effort="{eff2}" velocity="4.0" />
  </joint>

  <link name="forearm_link">
    <inertial>
      <origin xyz="{l_fore / 2.0:.4f} 0 0" rpy="0 0 0" />
      <mass value="{m_fore:.4f}" />
      <inertia {_box_inertia(m_fore, l_fore, 0.022, 0.032)} />
    </inertial>
    <visual>
      <origin xyz="{l_fore / 2.0:.4f} 0 0" rpy="0 0 0" />
      <geometry><box size="{l_fore:.4f} 0.022 0.032" /></geometry>
      <material name="roboforge_link" />
    </visual>
    <collision>
      <origin xyz="{l_fore / 2.0:.4f} 0 0" rpy="0 0 0" />
      <geometry><box size="{l_fore:.4f} 0.022 0.032" /></geometry>
    </collision>
  </link>

  <joint name="joint_3_wrist" type="revolute">
    <parent link="forearm_link" />
    <child link="wrist_link" />
    <origin xyz="{l_fore:.4f} 0 0" rpy="0 0 0" />
    <axis xyz="0 -1 0" />
    <limit lower="-1.5708" upper="1.5708" effort="{eff3}" velocity="5.0" />
  </joint>

  <link name="wrist_link">
    <inertial>
      <origin xyz="{l_wrist / 2.0:.4f} 0 0" rpy="0 0 0" />
      <mass value="{m_wrist:.4f}" />
      <inertia {_box_inertia(m_wrist, l_wrist, 0.020, 0.020)} />
    </inertial>
    <visual>
      <origin xyz="{l_wrist / 2.0:.4f} 0 0" rpy="0 1.5708 0" />
      <geometry><cylinder radius="0.015" length="{l_wrist:.4f}" /></geometry>
      <material name="roboforge_amber" />
    </visual>
    <collision>
      <origin xyz="{l_wrist / 2.0:.4f} 0 0" rpy="0 1.5708 0" />
      <geometry><cylinder radius="0.015" length="{l_wrist:.4f}" /></geometry>
    </collision>
  </link>

  <joint name="tcp_fixed_joint" type="fixed">
    <parent link="wrist_link" />
    <child link="tcp_flange" />
    <origin xyz="{l_wrist:.4f} 0 0" rpy="0 0 0" />
  </joint>

  <link name="tcp_flange" />
</robot>
"""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(urdf_xml, encoding="utf-8")
    return output_path


def export_arm_mujoco(
    req: ArmAssemblyRequest,
    joints: list[JointTorqueSummary],
    output_path: Path,
) -> Path:
    """Export a valid MuJoCo MJCF XML model for the articulated robotic arm."""
    l_base = req.base_height_mm / 1000.0
    l_upper = req.upper_arm_mm / 1000.0
    l_fore = req.forearm_mm / 1000.0
    l_wrist = req.wrist_mm / 1000.0

    j_map = {j.joint_id.upper(): j for j in joints}
    j1 = j_map.get("J1") or (joints[0] if joints else None)
    j2 = j_map.get("J2") or (joints[1] if len(joints) > 1 else j1)
    j3 = j_map.get("J3") or (joints[2] if len(joints) > 2 else j2)

    m_upper = max(0.05, (j1.segment_mass_g if j1 else 250.0) / 1000.0)
    m_fore = max(0.04, (j2.segment_mass_g if j2 else 180.0) / 1000.0)
    m_wrist = max(0.03, (j3.segment_mass_g if j3 else 100.0) / 1000.0)

    eff1 = max(5.0, round((j1.dynamic_torque_nm if j1 else 15.0) * 1.5, 2))
    eff2 = max(3.0, round((j2.dynamic_torque_nm if j2 else 8.0) * 1.5, 2))
    eff3 = max(1.5, round((j3.dynamic_torque_nm if j3 else 3.0) * 1.5, 2))

    mjcf_xml = f"""<?xml version="1.0" encoding="UTF-8"?>
<mujoco model="roboforge_{saxutils.escape(req.preset)}">
  <compiler angle="radian" coordinate="local" />
  <option timestep="0.002" gravity="0 0 -9.81" />

  <worldbody>
    <light pos="0.5 -0.8 1.5" dir="-0.2 0.4 -1" diffuse="0.9 0.9 0.9" />
    <geom name="floor" type="plane" size="1 1 0.1" rgba="0.12 0.14 0.18 1" />

    <body name="base_link" pos="0 0 0">
      <geom name="base_geom" type="cylinder" pos="0 0 {l_base / 2.0:.4f}" size="0.035 {l_base / 2.0:.4f}" rgba="0.28 0.34 0.42 1" mass="0.65" />

      <body name="upper_arm" pos="0 0 {l_base:.4f}">
        <joint name="j1_shoulder" type="hinge" axis="0 -1 0" range="-1.0472 1.5708" damping="0.4" />
        <geom name="upper_arm_geom" type="capsule" fromto="0 0 0 {l_upper:.4f} 0 0" size="0.018" rgba="0.24 0.72 0.80 1" mass="{m_upper:.4f}" />

        <body name="forearm" pos="{l_upper:.4f} 0 0">
          <joint name="j2_elbow" type="hinge" axis="0 -1 0" range="-2.2689 2.2689" damping="0.3" />
          <geom name="forearm_geom" type="capsule" fromto="0 0 0 {l_fore:.4f} 0 0" size="0.014" rgba="0.24 0.72 0.80 1" mass="{m_fore:.4f}" />

          <body name="wrist" pos="{l_fore:.4f} 0 0">
            <joint name="j3_wrist" type="hinge" axis="0 -1 0" range="-1.5708 1.5708" damping="0.15" />
            <geom name="wrist_geom" type="capsule" fromto="0 0 0 {l_wrist:.4f} 0 0" size="0.012" rgba="0.95 0.71 0.23 1" mass="{m_wrist:.4f}" />
            <site name="tcp_site" pos="{l_wrist:.4f} 0 0" size="0.008" rgba="1 0.3 0.2 1" />
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <motor name="act_shoulder" joint="j1_shoulder" gear="1" ctrllimited="true" ctrlrange="-{eff1} {eff1}" />
    <motor name="act_elbow" joint="j2_elbow" gear="1" ctrllimited="true" ctrlrange="-{eff2} {eff2}" />
    <motor name="act_wrist" joint="j3_wrist" gear="1" ctrllimited="true" ctrlrange="-{eff3} {eff3}" />
  </actuator>
</mujoco>
"""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(mjcf_xml, encoding="utf-8")
    return output_path


def export_bom_csv(bom: list[BOMItem], output_path: Path) -> Path:
    """Export the Bill of Materials (BOM) to a UTF-8 CSV file."""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(
            [
                "Categoria",
                "Pieza",
                "Especificacion",
                "Cantidad",
                "Costo Unitario (USD)",
                "Costo Total (USD)",
            ]
        )
        for item in bom:
            writer.writerow(
                [
                    item.category,
                    item.part_name,
                    item.spec,
                    item.qty,
                    f"{item.unit_cost_usd:.2f}",
                    f"{item.total_cost_usd:.2f}",
                ]
            )
    return output_path


def compute_modular_arm_hash(req: ModularArmRequest) -> str:
    """Compute a deterministic 16-char SHA-256 hex digest for a ModularArmRequest."""
    canonical = "modarm_v90:" + json.dumps(req.model_dump(), sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:16]


def _get_or_build_subpart(kind: str, variant: str, spec_model: Any, builder_fn: Any) -> Part:
    """Build or retrieve a cached build123d Part for a modular arm slot."""
    key_raw = f"{kind}:{variant}:" + json.dumps(spec_model.model_dump(), sort_keys=True, separators=(",", ":"))
    key = hashlib.sha256(key_raw.encode("utf-8")).hexdigest()[:20]
    cached = _SUBPART_CACHE.get(key)
    if cached is not None:
        return cached
    part = builder_fn(spec_model)
    if len(_SUBPART_CACHE) > 96:
        _SUBPART_CACHE.clear()
    _SUBPART_CACHE[key] = part
    return part


def _export_part_step_and_3mf(part: Part, out_dir: Path, stem: str, variant_hint: str) -> tuple[Path, Path]:
    """Export an individual solid to <stem>.step and smart <stem>.3mf."""
    step_path = out_dir / f"{stem}.step"
    threemf_path = out_dir / f"{stem}.3mf"
    if not (step_path.exists() and step_path.stat().st_size > 0):
        export_step(part, str(step_path))
    if not (threemf_path.exists() and threemf_path.stat().st_size > 0):
        with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as tmp:
            tmp_stl = Path(tmp.name)
        try:
            export_stl(part, str(tmp_stl), tolerance=0.10, angular_tolerance=0.18)
            mod_center, mod_size = compute_modifier_box_for_mesh(tmp_stl, variant_hint)
            export_smart_3mf(
                stl_path=tmp_stl,
                output_3mf_path=threemf_path,
                part_name=f"RoboForge_{stem}",
                modifier_box_center=mod_center,
                modifier_box_size=mod_size,
            )
        finally:
            if tmp_stl.exists():
                try:
                    tmp_stl.unlink()
                except OSError:
                    pass
    return step_path, threemf_path


def build_modular_arm_assembly(
    req: ModularArmRequest,
    material: dict[str, Any],
    output_dir: Path,
    design_hash: str,
) -> dict[str, Any]:
    """Assemble real CAD solids from variants.py, mount_variants.py, and gear_variants.py
    using local mounting frames and axial Y-layers to avoid coplanar collisions.
    Delegates to specialized multi-topology assemblers for SCARA, Cartesian Gantry, Delta Parallel, and Continuum PCC.
    """
    kin = getattr(req, "kinematics_type", "articulated")
    if kin == "scara":
        from .scara_assembly import build_scara_arm_assembly

        return build_scara_arm_assembly(req, material, output_dir, design_hash)
    if kin == "cartesian_gantry":
        from .cartesian_assembly import build_cartesian_gantry_assembly

        return build_cartesian_gantry_assembly(req, material, output_dir, design_hash)
    if kin == "delta_parallel":
        from .delta_assembly import build_delta_parallel_assembly

        return build_delta_parallel_assembly(req, material, output_dir, design_hash)
    if kin == "continuum_pcc":
        from .continuum_assembly import build_continuum_pcc_assembly

        return build_continuum_pcc_assembly(req, material, output_dir, design_hash)
    if kin == "soft_pneumatic_bellows":
        from .continuum_assembly import build_soft_pneumatic_bellows_assembly

        return build_soft_pneumatic_bellows_assembly(req, material, output_dir, design_hash)
    if kin == "bimanual_dual_arm":
        from .bimanual_assembly import build_bimanual_arm_assembly

        return build_bimanual_arm_assembly(req, material, output_dir, design_hash)
    if kin == "stewart_hexapod":
        from .stewart_assembly import build_stewart_hexapod_assembly

        return build_stewart_hexapod_assembly(req, material, output_dir, design_hash)

    from ..physics.arm_compatibility import resolve_default_slots

    output_dir.mkdir(parents=True, exist_ok=True)
    slots = resolve_default_slots(req)

    assert slots.shoulder_mount is not None
    assert slots.shoulder_gear is not None
    assert slots.upper_arm is not None
    assert slots.elbow_gear is not None
    assert slots.forearm is not None

    link_builders = {
        "tapered_solid": _build_tapered_solid,
        "organic_ibeam": _build_organic_ibeam,
        "rounded_truss": _build_rounded_truss,
        "hollow_cobot": _build_hollow_cobot,
        "organic_topopt": _build_organic_topopt,
    }
    mount_builders = {
        "mount_standard": _build_mount_standard,
        "mount_gusseted": _build_mount_gusseted,
        "mount_lightweight": _build_mount_lightweight,
    }
    gear_builders = {
        "gear_solid": _build_gear_solid,
        "gear_webbed": _build_gear_webbed,
        "gear_spoked": _build_gear_spoked,
    }

    # 1. Instantiate real CAD sub-parts (with sub-part hash caching)
    scale = max(0.82, min(1.35, 0.88 + 0.12 * math.sqrt(req.payload_kg)))
    is_compact_msla = (
        float(req.base_height_mm) <= 66.0
        or (req.printer_id or "") == "elegoo_mars_5_ultra"
        or req.material_id == "resin_abs_like"
    )
    base_raw = _build_base_pedestal(req.base_height_mm, scale, compact_msla=is_compact_msla)

    mount_raw = _get_or_build_subpart(
        "mount",
        slots.shoulder_mount_variant,
        slots.shoulder_mount,
        mount_builders.get(slots.shoulder_mount_variant, _build_mount_gusseted),
    )
    sh_gear_raw = _get_or_build_subpart(
        "gear",
        slots.shoulder_gear_variant,
        slots.shoulder_gear,
        gear_builders.get(slots.shoulder_gear_variant, _build_gear_spoked),
    )
    upper_raw = _get_or_build_subpart(
        "link",
        slots.upper_arm_variant,
        slots.upper_arm,
        link_builders.get(slots.upper_arm_variant, _build_organic_ibeam),
    )
    el_gear_raw = _get_or_build_subpart(
        "gear",
        slots.elbow_gear_variant,
        slots.elbow_gear,
        gear_builders.get(slots.elbow_gear_variant, _build_gear_webbed),
    )
    forearm_raw = _get_or_build_subpart(
        "link",
        slots.forearm_variant,
        slots.forearm,
        link_builders.get(slots.forearm_variant, _build_hollow_cobot),
    )

    # 2. Kinematics & Axial Y-Layer Offsets (prevents coplanar overlap & Z-fighting)
    a1_deg, a2_rel_deg, a3_rel_deg = (
        float(req.joint_angles_deg[0]),
        float(req.joint_angles_deg[1]),
        float(req.joint_angles_deg[2]),
    )
    a2_deg = a1_deg + a2_rel_deg
    a3_deg = a2_deg + a3_rel_deg
    a1 = math.radians(a1_deg)
    a2 = math.radians(a2_deg)
    a3 = math.radians(a3_deg)

    l1 = float(slots.upper_arm.length_mm)
    l2 = float(slots.forearm.length_mm)
    upper_t = float(slots.upper_arm.thickness_mm)
    forearm_t = float(slots.forearm.thickness_mm)
    clearance = 1.2

    y_upper = 0.0
    y_mount_face = -(upper_t / 2.0 + clearance)
    y_sh_gear = +(upper_t / 2.0 + clearance)
    y_el_gear = -(upper_t / 2.0 + clearance)
    y_forearm = +(upper_t / 2.0 + forearm_t / 2.0 + clearance)

    p0 = (0.0, y_upper, float(req.base_height_mm))
    p1 = (p0[0] + l1 * math.cos(a1), y_upper, p0[2] + l1 * math.sin(a1))
    p1_fore = (p1[0], y_forearm, p1[2])
    p2_fore = (p1_fore[0] + l2 * math.cos(a2), y_forearm, p1_fore[2] + l2 * math.sin(a2))

    # Place shoulder mount: rotate 180° around Z so faceplate faces +Y and base extends along -Y
    _, _, _, mount_shaft_z = _compute_mount_envelope(slots.shoulder_mount)
    mount_rot = Location((0.0, 0.0, 0.0), (0.0, 0.0, 180.0)) * mount_raw
    shoulder_mount_placed = Location((0.0, y_mount_face, float(req.base_height_mm) - mount_shaft_z)) * mount_rot

    # Place shoulder gear coaxially at P0 on +Y side (rotate -90° around X so gear +Z aligns with +Y)
    shoulder_gear_placed = Location((0.0, y_sh_gear, float(req.base_height_mm))) * (
        Location((0.0, 0.0, 0.0), (-90.0, 0.0, 0.0)) * sh_gear_raw
    )

    # Place upper arm link in XZ plane at y = 0
    upper_arm_placed = _place_link_in_xz(upper_raw, p0, a1_deg)

    # Place elbow gear coaxially at P1 on -Y side (rotate +90° around X so gear +Z aligns with -Y)
    elbow_gear_placed = Location((p1[0], y_el_gear, p1[2])) * (
        Location((0.0, 0.0, 0.0), (90.0, 0.0, 0.0)) * el_gear_raw
    )

    # Place forearm link in XZ plane at y = y_forearm
    forearm_placed = _place_link_in_xz(forearm_raw, p1_fore, a2_deg)

    # Wrist tube + TCP flange coaxial with forearm
    wrist_tube_placed, tcp_flange_placed = _build_wrist_and_tcp(p2_fore, req.wrist_mm, a3_deg, scale)

    # Joint hardware (bearing collars & shafts spanning axial layers cleanly)
    sh_hw_len = upper_t + float(slots.shoulder_gear.face_width_mm) + float(slots.shoulder_mount.wall_thickness_mm) + 12.0
    el_hw_len = upper_t + forearm_t + float(slots.elbow_gear.face_width_mm) + 10.0
    wr_hw_len = forearm_t + 14.0

    joint_hw_shoulder = _build_joint_housing(
        (0.0, 0.0, float(req.base_height_mm)),
        radius=max(10.0, float(slots.upper_arm.root_end.bore_diameter_mm) * 0.48),
        length_y=sh_hw_len,
    )
    joint_hw_elbow = _build_joint_housing(
        (p1[0], y_forearm * 0.4, p1[2]),
        radius=max(8.0, float(slots.forearm.root_end.bore_diameter_mm) * 0.48),
        length_y=el_hw_len,
    )
    joint_hw_wrist = _build_joint_housing(
        p2_fore,
        radius=max(7.0, float(slots.forearm.tip_end.bore_diameter_mm) * 0.46),
        length_y=wr_hw_len,
    )

    # 2b. v4.0 / v4.1 Real 3D Hardware: Motors / Dual RC Servos, 25T Horns, Bearings, Gripper, Fasteners & MCU Box
    mech_cfg = getattr(req, "mechatronics", None)
    joint_acts = getattr(mech_cfg, "joint_actuators", None) or {}
    bus_arch = getattr(mech_cfg, "bus_architecture", "can_fd_bus")
    default_j1_motor = (
        "nema23_76mm" if slots.shoulder_mount.actuator_type == "nema23" else "nema17_48mm"
    )
    j1_spec = joint_acts.get("J1") or joint_acts.get("j1")
    j2_spec = joint_acts.get("J2") or joint_acts.get("j2")
    j3_spec = joint_acts.get("J3") or joint_acts.get("j3")
    j1_motor_id = getattr(j1_spec, "motor_id", None) or default_j1_motor
    j2_motor_id = getattr(j2_spec, "motor_id", None) or "nema17_48mm"
    j3_motor_id = getattr(j3_spec, "motor_id", None) or "bldc_qdd_5010"

    j1_parallel = int(getattr(j1_spec, "parallel_count", 1) or 1)
    j2_parallel = int(getattr(j2_spec, "parallel_count", 1) or 1)
    j3_parallel = int(getattr(j3_spec, "parallel_count", 1) or 1)

    def _is_rc_servo(mid: str) -> bool:
        ml = (mid or "").lower()
        return ("servo_td" in ml) or ("servo_ds" in ml) or ("mg996" in ml) or ("mg90" in ml) or ("sts3215" in ml)

    use_rc_servos = (
        bus_arch in ("arduino_pca9685_i2c", "uart_serial_bus", "esp32_pca9685_5v")
        or _is_rc_servo(j1_motor_id)
        or _is_rc_servo(j2_motor_id)
        or _is_rc_servo(j3_motor_id)
    )
    is_direct_servo_mode = (
        bus_arch in ("arduino_pca9685_i2c", "uart_serial_bus", "esp32_pca9685_5v")
        or (
            _is_rc_servo(j1_motor_id)
            and (getattr(j1_spec, "reducer_id", "direct") or "direct") == "direct"
        )
    )

    # Motors: local motor faceplate is z=0, body extends along -Z, shaft along +Z.
    # Rotate -90° around X -> local +Z maps to +Y (shaft points into joint along +Y, body along -Y).
    # Rotate +90° around X -> local +Z maps to -Y (opposed mirror servo on +Y side).
    rot_z_to_plus_y = Location((0.0, 0.0, 0.0), (-90.0, 0.0, 0.0))
    rot_z_to_minus_y = Location((0.0, 0.0, 0.0), (90.0, 0.0, 0.0))

    horn_nodes: list[tuple[str, Part | Compound, list[float]]] = []
    horn_25t = build_servo_horn_25t() if use_rc_servos else None

    # J1 Shoulder motor (supports dual opposed parallel servos + 25T red aluminum horns)
    j1_y_neg = y_mount_face - float(slots.shoulder_mount.wall_thickness_mm)
    j1_m1 = Location((0.0, j1_y_neg, float(req.base_height_mm))) * (
        rot_z_to_plus_y * build_motor_3d(j1_motor_id)
    )
    if _is_rc_servo(j1_motor_id) and j1_parallel >= 2:
        j1_y_pos = y_sh_gear + float(slots.shoulder_gear.face_width_mm) + 2.0
        j1_m2 = Location((0.0, j1_y_pos, float(req.base_height_mm))) * (
            rot_z_to_minus_y * build_servo_3d(j1_motor_id)
        )
        motor_j1_placed: Part | Compound = Compound(children=[j1_m1, j1_m2])
        if horn_25t is not None:
            h1_a = Location((0.0, j1_y_neg + 5.5, float(req.base_height_mm))) * (rot_z_to_plus_y * horn_25t)
            h1_b = Location((0.0, j1_y_pos - 5.5, float(req.base_height_mm))) * (rot_z_to_minus_y * horn_25t)
            horn_nodes.append(("horn_25t_j1", Compound(children=[h1_a, h1_b]), [0.0, -72.0, 0.0]))
    else:
        motor_j1_placed = j1_m1
        if _is_rc_servo(j1_motor_id) and horn_25t is not None:
            h1_a = Location((0.0, j1_y_neg + 5.5, float(req.base_height_mm))) * (rot_z_to_plus_y * horn_25t)
            horn_nodes.append(("horn_25t_j1", h1_a, [0.0, -72.0, 0.0]))

    # J2 Elbow motor (supports dual opposed parallel servos + 25T red aluminum horns)
    j2_y_neg = y_el_gear - float(slots.elbow_gear.face_width_mm) - 1.5
    j2_m1 = Location((p1[0], j2_y_neg, p1[2])) * (rot_z_to_plus_y * build_motor_3d(j2_motor_id))
    if _is_rc_servo(j2_motor_id) and j2_parallel >= 2:
        j2_y_pos = y_forearm + forearm_t / 2.0 + 3.0
        j2_m2 = Location((p1[0], j2_y_pos, p1[2])) * (rot_z_to_minus_y * build_servo_3d(j2_motor_id))
        motor_j2_placed: Part | Compound = Compound(children=[j2_m1, j2_m2])
        if horn_25t is not None:
            h2_a = Location((p1[0], j2_y_neg + 5.5, p1[2])) * (rot_z_to_plus_y * horn_25t)
            h2_b = Location((p1[0], j2_y_pos - 5.5, p1[2])) * (rot_z_to_minus_y * horn_25t)
            horn_nodes.append(
                (
                    "horn_25t_j2",
                    Compound(children=[h2_a, h2_b]),
                    [round(32.0 * math.cos(a1), 2), -74.0, round(32.0 * math.sin(a1), 2)],
                )
            )
    else:
        motor_j2_placed = j2_m1
        if _is_rc_servo(j2_motor_id) and horn_25t is not None:
            h2_a = Location((p1[0], j2_y_neg + 5.5, p1[2])) * (rot_z_to_plus_y * horn_25t)
            horn_nodes.append(
                (
                    "horn_25t_j2",
                    h2_a,
                    [round(32.0 * math.cos(a1), 2), -74.0, round(32.0 * math.sin(a1), 2)],
                )
            )

    # J3 Wrist motor
    j3_y_neg = y_forearm - forearm_t / 2.0 - 2.0
    motor_j3_placed: Part | Compound = Location((p2_fore[0], j3_y_neg, p2_fore[2])) * (
        rot_z_to_plus_y * build_motor_3d(j3_motor_id)
    )
    if _is_rc_servo(j3_motor_id) and horn_25t is not None:
        h3_a = Location((p2_fore[0], j3_y_neg + 5.5, p2_fore[2])) * (rot_z_to_plus_y * horn_25t)
        horn_nodes.append(
            (
                "horn_25t_j3",
                h3_a,
                [
                    round(32.0 * math.cos(a1) + 42.0 * math.cos(a2), 2),
                    -26.0,
                    round(32.0 * math.sin(a1) + 42.0 * math.sin(a2), 2),
                ],
            )
        )

    # Optional 3-part Articulated Robotic Gripper + Dual-Servo Wrapping Clevises for Direct-Drive Servo / Mars-Mini arms
    gripper_parts_placed: tuple[Part, Part, Part] | None = None
    if use_rc_servos:
        g_base_raw, g_left_raw, g_right_raw = build_gripper_parts(compact=is_compact_msla)
        g_shift = Location((max(10.0, req.wrist_mm - 6.0), 0.0, 0.0))
        g_pose = Location(p2_fore) * Location((0.0, 0.0, 0.0), (0.0, -a3_deg, 0.0)) * g_shift
        gripper_parts_placed = (
            g_pose * g_base_raw,
            g_pose * g_left_raw,
            g_pose * g_right_raw,
        )

    clevis_shoulder_placed: Part | None = None
    clevis_elbow_placed: Part | None = None
    if is_direct_servo_mode:
        clevis_sh_w = min(54.0, max(40.0, upper_t + 22.0)) if is_compact_msla else max(46.0, upper_t + 26.0)
        clevis_el_w = min(52.0, max(38.0, upper_t + forearm_t + 12.0)) if is_compact_msla else max(44.0, upper_t + forearm_t + 16.0)
        clevis_d = 38.0 if is_compact_msla else 44.0
        clevis_h = 40.0 if is_compact_msla else 46.0
        clevis_bolt_d = 3.25 if is_compact_msla else 4.3
        clevis_shoulder_placed = Location((0.0, 0.0, float(req.base_height_mm))) * build_dual_servo_clevis(
            width_mm=clevis_sh_w, depth_mm=clevis_d, height_mm=clevis_h, bolt_d_mm=clevis_bolt_d
        )
        clevis_elbow_placed = Location((p1[0], y_forearm * 0.45, p1[2])) * (
            Location((0.0, 0.0, 0.0), (0.0, -a1_deg, 0.0))
            * build_dual_servo_clevis(width_mm=clevis_el_w, depth_mm=clevis_d - 2.0, height_mm=clevis_h - 2.0, bolt_d_mm=clevis_bolt_d)
        )

    # Real Ball Bearings (or CRB RU42 / Tapered 30205 on heavy/industrial arms)
    sh_bore = float(slots.upper_arm.root_end.bore_diameter_mm)
    b1_code = "6808" if sh_bore >= 42.0 else ("6002" if sh_bore >= 28.0 else "608")
    el_bore = float(slots.forearm.root_end.bore_diameter_mm)
    b2_code = "6002" if el_bore >= 28.0 else "608"

    use_heavy_bearings = (bus_arch == "teensy_dm542") or (req.payload_kg >= 2.5)
    bearing_j1_placed = Location((0.0, 0.0, float(req.base_height_mm))) * (
        rot_z_to_plus_y * (build_bearing_crb() if use_heavy_bearings else build_ball_bearing(b1_code))
    )
    bearing_j2_placed = Location((p1[0], y_forearm * 0.5, p1[2])) * (
        rot_z_to_plus_y * (build_bearing_tapered_30205() if use_heavy_bearings else build_ball_bearing(b2_code))
    )
    bearing_j3_placed = Location(p2_fore) * (
        rot_z_to_plus_y * build_ball_bearing("608")
    )

    # v5.0: Real 3D Cycloidal / Planetary reducers when selected in mechatronics config
    j1_reducer_id = str(getattr(j1_spec, "reducer_id", "") or "").lower()
    j2_reducer_id = str(getattr(j2_spec, "reducer_id", "") or "").lower()

    extra_v5_nodes: list[tuple[str, Part | Compound, list[int], float, float, list[float]]] = []
    replace_sh_gear = False
    replace_el_gear = False

    if not is_direct_servo_mode:
        if j1_reducer_id.startswith("cycloidal"):
            cyc_ratio = 40 if "40" in j1_reducer_id else 25
            cyc_j1 = Location((0.0, y_sh_gear + 7.0, float(req.base_height_mm))) * (
                rot_z_to_plus_y * build_cycloidal_reducer_3d(ratio=cyc_ratio, outer_d_mm=68.0, thickness_mm=14.0)
            )
            extra_v5_nodes.append(("cycloidal_j1", cyc_j1, [196, 206, 218, 255], 0.88, 0.22, [0.0, 58.0, 0.0]))
            replace_sh_gear = True
        elif j1_reducer_id.startswith("planetary"):
            plan_ratio = 20 if "20" in j1_reducer_id else 10
            plan_j1 = Location((0.0, y_sh_gear + 8.0, float(req.base_height_mm))) * (
                rot_z_to_plus_y * build_planetary_reducer_3d(ratio=plan_ratio, outer_d_mm=58.0, thickness_mm=16.0)
            )
            extra_v5_nodes.append(("planetary_j1", plan_j1, [178, 192, 210, 255], 0.84, 0.26, [0.0, 58.0, 0.0]))
            replace_sh_gear = True

        if j2_reducer_id.startswith("cycloidal"):
            cyc_ratio = 40 if "40" in j2_reducer_id else 25
            cyc_j2 = Location((p1[0], y_el_gear - 7.0, p1[2])) * (
                rot_z_to_minus_y * build_cycloidal_reducer_3d(ratio=cyc_ratio, outer_d_mm=56.0, thickness_mm=13.0)
            )
            extra_v5_nodes.append(
                ("cycloidal_j2", cyc_j2, [196, 206, 218, 255], 0.88, 0.22, [round(32.0 * math.cos(a1), 2), -58.0, round(32.0 * math.sin(a1), 2)])
            )
            replace_el_gear = True
        elif j2_reducer_id.startswith("planetary"):
            plan_ratio = 20 if "20" in j2_reducer_id else 10
            plan_j2 = Location((p1[0], y_el_gear - 7.0, p1[2])) * (
                rot_z_to_minus_y * build_planetary_reducer_3d(ratio=plan_ratio, outer_d_mm=50.0, thickness_mm=14.0)
            )
            extra_v5_nodes.append(
                ("planetary_j2", plan_j2, [178, 192, 210, 255], 0.84, 0.26, [round(32.0 * math.cos(a1), 2), -58.0, round(32.0 * math.sin(a1), 2)])
            )
            replace_el_gear = True

        # Shift J2 elbow motor outward in -Y so it clears planetary_j2 / cycloidal_j2 cleanly
        if replace_el_gear:
            motor_j2_placed = Location((0.0, -28.0, 0.0)) * motor_j2_placed

    # v5.0: 7-DOF Redundant Upper-Arm Roll Joint Collar (OpenArm 7-DOF)
    if req.dof >= 7:
        p_mid = (p0[0] + 0.5 * l1 * math.cos(a1), y_upper, p0[2] + 0.5 * l1 * math.sin(a1))
        roll_collar_local = Location((0.0, 0.0, 0.0), (0.0, 90.0, 0.0)) * Cylinder(
            radius=max(20.0, upper_t * 0.92), height=28.0
        )
        roll_j3_placed = Location(p_mid) * (Location((0.0, 0.0, 0.0), (0.0, -a1_deg, 0.0)) * roll_collar_local)
        extra_v5_nodes.append(
            (
                "upper_arm_roll_j3",
                roll_j3_placed,
                [84, 148, 198, 255],
                0.78,
                0.28,
                [round(38.0 * math.cos(a1), 2), 0.0, round(38.0 * math.sin(a1), 2)],
            )
        )

    # v5.0: Inductive homing sensor on industrial arms (Teensy / AR4)
    if use_heavy_bearings:
        hs_placed = Location((28.0, 0.0, float(req.base_height_mm) - 24.0), (0.0, 90.0, 0.0)) * build_homing_sensor()
        extra_v5_nodes.append(("homing_sensor_j1", hs_placed, [216, 172, 58, 255], 0.85, 0.25, [36.0, 0.0, -24.0]))

    # v5.0: 3-DOF Articulated Wrist (J4 Roll, J5 Pitch, J6 Tool Flange)
    wrist_3dof_placed: tuple[Part, Part, Part] | None = None
    if getattr(req, "wrist_3dof", False) and gripper_parts_placed is None and not getattr(req, "bevel_differential_wrist", False):
        w4_raw, w5_raw, w6_raw = build_wrist_3dof_parts()
        w_pose = Location(p2_fore) * Location((0.0, 0.0, 0.0), (0.0, -a3_deg, 0.0))
        wrist_3dof_placed = (w_pose * w4_raw, w_pose * w5_raw, w_pose * w6_raw)

    # v6.0: Remote Base Motors (Capstan / Tendon Drive — Barrett WAM / AMBIDEX)
    if getattr(req, "remote_base_motors", False):
        # Relocate J2 & J3 motors onto the fixed base pedestal to minimize moving arm inertia!
        base_m_z = max(32.0, float(req.base_height_mm) - 34.0)
        motor_j2_placed = Location((0.0, -38.0, base_m_z)) * (rot_z_to_plus_y * build_motor_3d(j2_motor_id))
        motor_j3_placed = Location((0.0, +38.0, base_m_z)) * (rot_z_to_minus_y * build_motor_3d(j3_motor_id))
        cap1, tendons1 = build_capstan_drum_and_tendons(drum_radius_mm=24.0, width_mm=16.0, span_mm=l1)
        cap2, _ = build_capstan_drum_and_tendons(drum_radius_mm=18.0, width_mm=14.0, span_mm=l2)
        cap1_placed = Location((0.0, y_sh_gear + 8.0, float(req.base_height_mm))) * (rot_z_to_plus_y * cap1)
        cap2_placed = Location((p1[0], y_el_gear - 7.0, p1[2])) * (rot_z_to_minus_y * cap2)
        tendons_placed = Location((0.0, y_sh_gear + 8.0, float(req.base_height_mm))) * (
            Location((0.0, 0.0, 0.0), (0.0, -a1_deg, 0.0)) * tendons1
        )
        extra_v5_nodes.append(("capstan_drum_j1", cap1_placed, [196, 204, 216, 255], 0.88, 0.22, [0.0, 62.0, 0.0]))
        extra_v5_nodes.append(
            ("capstan_drum_j2", cap2_placed, [196, 204, 216, 255], 0.88, 0.22, [round(32.0 * math.cos(a1), 2), -62.0, round(32.0 * math.sin(a1), 2)])
        )
        extra_v5_nodes.append(
            ("tendon_cables", tendons_placed, [224, 232, 240, 255], 0.90, 0.20, [round(24.0 * math.cos(a1), 2), 72.0, round(24.0 * math.sin(a1), 2)])
        )
        replace_sh_gear = True
        replace_el_gear = True

    # v6.0: Four-Bar Parallelogram Linkage Rod (Dobot Magician / uArm Swift Pro / Mars-Mini)
    parallelogram_rod_placed: Part | None = None
    if getattr(req, "four_bar_parallelogram", False):
        rod_raw = build_four_bar_parallelogram_rod(l1)
        # Parallel to upper_arm_link, offset +24 mm perpendicular to arm axis in XZ and +Y side
        p_rod_origin = (
            p0[0] - 22.0 * math.sin(a1),
            -(upper_t / 2.0 + 7.0),
            p0[2] + 22.0 * math.cos(a1),
        )
        rod_placed = _place_link_in_xz(rod_raw, p_rod_origin, a1_deg)
        parallelogram_rod_placed = rod_raw
        extra_v5_nodes.append(
            (
                "parallelogram_rod",
                rod_placed,
                [152, 184, 218, 255],
                0.82,
                0.26,
                [round(32.0 * math.cos(a1), 2), -36.0, round(32.0 * math.sin(a1) + 28.0, 2)],
            )
        )

    # v6.0: Bevel Differential Wrist (Thor 6-DOF / Niryo Ned2)
    if getattr(req, "bevel_differential_wrist", False):
        bevel_raw = build_bevel_differential_wrist()
        bevel_placed = Location(p2_fore) * (Location((0.0, 0.0, 0.0), (0.0, -a3_deg, 0.0)) * bevel_raw)
        extra_v5_nodes.append(
            (
                "bevel_wrist_diff",
                bevel_placed,
                [186, 198, 214, 255],
                0.88,
                0.22,
                [
                    round(32.0 * math.cos(a1) + 42.0 * math.cos(a2) + 55.0 * math.cos(a3), 2),
                    48.0,
                    round(32.0 * math.sin(a1) + 42.0 * math.sin(a2) + 55.0 * math.sin(a3), 2),
                ],
            )
        )

    # v6.0: Vacuum Pick-and-Place Nozzle EOAT
    if getattr(req, "vacuum_eoat", False):
        vac_tip = (
            p2_fore[0] + req.wrist_mm * math.cos(a3),
            y_forearm,
            p2_fore[2] + req.wrist_mm * math.sin(a3),
        )
        vac_placed = Location(vac_tip) * build_vacuum_nozzle()
        extra_v5_nodes.append(
            (
                "vacuum_nozzle",
                vac_placed,
                [218, 176, 62, 255],
                0.82,
                0.26,
                [
                    round(32.0 * math.cos(a1) + 42.0 * math.cos(a2) + 84.0 * math.cos(a3), 2),
                    48.0,
                    round(32.0 * math.sin(a1) + 42.0 * math.sin(a2) + 84.0 * math.sin(a3) - 24.0, 2),
                ],
            )
        )

    # ISO 4762 Fasteners Kit: real socket-head cap screws placed on base flange, shoulder mount, elbow, and TCP PCDs
    fastener_solids: list[Part] = []
    # Base pedestal 4×M5 anchor screws (head up along +Z at z = flange_h)
    base_flange_h = min(12.0, req.base_height_mm * 0.16)
    base_pcd_r = 32.0 * scale + 7.5
    m5_base = build_iso4762_screw("M5", 16.0)
    for i in range(4):
        ang = math.pi / 4.0 + i * (math.pi / 2.0)
        fastener_solids.append(
            Location((base_pcd_r * math.cos(ang), base_pcd_r * math.sin(ang), base_flange_h)) * m5_base
        )

    # Shoulder mount 4× motor faceplate bolts (M5 for NEMA23, M3 for NEMA17)
    is_n23 = slots.shoulder_mount.actuator_type == "nema23"
    m_sh = build_iso4762_screw("M5" if is_n23 else "M3", 14.0)
    sh_half_sq = 23.57 if is_n23 else 15.5
    for sx in (-sh_half_sq, sh_half_sq):
        for sz in (-sh_half_sq, sh_half_sq):
            # Head sits on +Y face of mount faceplate (z_local -> +Y)
            fastener_solids.append(
                Location((sx, y_mount_face, float(req.base_height_mm) + sz)) * (rot_z_to_plus_y * m_sh)
            )

    # Elbow 4×M3 joint ring bolts
    m3_el = build_iso4762_screw("M3", 12.0)
    el_pcd_r = max(10.0, float(slots.forearm.root_end.bolt_pcd_mm) / 2.0)
    for i in range(4):
        ang = math.pi / 4.0 + i * (math.pi / 2.0)
        fastener_solids.append(
            Location(
                (
                    p1[0] + el_pcd_r * math.cos(ang),
                    y_forearm + forearm_t / 2.0,
                    p1[2] + el_pcd_r * math.sin(ang),
                )
            )
            * (rot_z_to_plus_y * m3_el)
        )

    # TCP 4×M4 tool flange bolts
    m4_tcp = build_iso4762_screw("M4", 10.0)
    tcp_pcd_r = (18.5 * scale) * 0.68
    rot_z_to_plus_x = Location((0.0, 0.0, 0.0), (0.0, 90.0, 0.0))
    for i in range(4):
        ang = math.pi / 4.0 + i * (math.pi / 2.0)
        local_bolt = Location((req.wrist_mm, tcp_pcd_r * math.cos(ang), tcp_pcd_r * math.sin(ang))) * (
            rot_z_to_plus_x * m4_tcp
        )
        fastener_solids.append(
            Location(p2_fore) * (Location((0.0, 0.0, 0.0), (0.0, -a3_deg, 0.0)) * local_bolt)
        )

    fasteners_kit = Compound(children=fastener_solids)

    # Industrial STM32 / CAN-FD Controller Box mounted cleanly beside the base pedestal
    base_flange_r = 32.0 * scale + 14.0
    controller_box_placed = Location((-(base_flange_r + 42.0), 0.0, 0.0)) * build_controller_box()

    # Real CAD volumes per slot (mm³)
    real_volumes_mm3 = {
        "base_pedestal": float(max(base_raw.volume, 1.0)),
        "shoulder_mount": float(max(mount_raw.volume, 1.0)),
        "shoulder_gear": float(max(sh_gear_raw.volume, 1.0)),
        "upper_arm_link": float(max(upper_raw.volume, 1.0)),
        "elbow_gear": float(max(el_gear_raw.volume, 1.0)),
        "forearm_link": float(max(forearm_raw.volume, 1.0)),
        "wrist_tube": float(max(wrist_tube_placed.volume + tcp_flange_placed.volume, 1.0)),
    }

    # 3. Export individual print-ready STEP + 3MF files for each component
    individual_files: list[Path] = []
    indiv_specs = [
        (base_raw, "base_pedestal", "tapered_solid"),
        (mount_raw, "shoulder_mount", slots.shoulder_mount_variant),
        (sh_gear_raw, "shoulder_gear", slots.shoulder_gear_variant),
        (upper_raw, "upper_arm_link", slots.upper_arm_variant),
        (el_gear_raw, "elbow_gear", slots.elbow_gear_variant),
        (forearm_raw, "forearm_link", slots.forearm_variant),
        (wrist_tube_placed + tcp_flange_placed, "wrist_tcp", "tapered_solid"),
    ]
    for part_solid, stem, vhint in indiv_specs:
        s_path, m_path = _export_part_step_and_3mf(part_solid, output_dir, stem, vhint)
        individual_files.extend([s_path, m_path])

    # 4. Export combined assembly STEP, STL, and semantic multi-node GLB
    compound_children: list[Part | Compound] = [
        base_raw,
        shoulder_mount_placed,
        upper_arm_placed,
        forearm_placed,
        joint_hw_shoulder,
        joint_hw_elbow,
        joint_hw_wrist,
        motor_j1_placed,
        motor_j2_placed,
        motor_j3_placed,
        bearing_j1_placed,
        bearing_j2_placed,
        bearing_j3_placed,
        fasteners_kit,
        controller_box_placed,
    ]
    if wrist_3dof_placed is None:
        compound_children.append(wrist_tube_placed)
    if is_direct_servo_mode and clevis_shoulder_placed is not None and clevis_elbow_placed is not None:
        compound_children.extend([clevis_shoulder_placed, clevis_elbow_placed])
    else:
        if not replace_sh_gear:
            compound_children.append(shoulder_gear_placed)
        if not replace_el_gear:
            compound_children.append(elbow_gear_placed)

    if gripper_parts_placed is not None:
        compound_children.extend(list(gripper_parts_placed))
    elif wrist_3dof_placed is not None:
        compound_children.extend(list(wrist_3dof_placed))
    else:
        compound_children.append(tcp_flange_placed)

    for _, h_solid, _ in horn_nodes:
        compound_children.append(h_solid)
    for _, v5_solid, _, _, _, _ in extra_v5_nodes:
        compound_children.append(v5_solid)

    full_compound = Compound(children=compound_children)
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
            ("base_pedestal", base_raw, [68, 82, 102, 255], 0.72, 0.32),
            ("shoulder_mount", shoulder_mount_placed, [92, 134, 182, 255], 0.78, 0.28),
            ("upper_arm_link", upper_arm_placed, link_rgba, 0.52, 0.32),
            ("forearm_link", forearm_placed, link_rgba, 0.52, 0.32),
            ("joint_hw_shoulder", joint_hw_shoulder, [42, 48, 62, 255], 0.78, 0.28),
            ("joint_hw_elbow", joint_hw_elbow, [42, 48, 62, 255], 0.78, 0.28),
            ("joint_hw_wrist", joint_hw_wrist, [42, 48, 62, 255], 0.78, 0.28),
            # v4.0 Hardware semantic nodes
            ("motor_j1_shoulder", motor_j1_placed, [32, 36, 44, 255], 0.58, 0.42),
            ("motor_j2_elbow", motor_j2_placed, [32, 36, 44, 255], 0.58, 0.42),
            ("motor_j3_wrist", motor_j3_placed, [36, 42, 52, 255], 0.62, 0.38),
            ("bearing_j1", bearing_j1_placed, [215, 222, 232, 255], 0.96, 0.14),
            ("bearing_j2", bearing_j2_placed, [215, 222, 232, 255], 0.96, 0.14),
            ("bearing_j3", bearing_j3_placed, [215, 222, 232, 255], 0.96, 0.14),
            ("fasteners_kit", fasteners_kit, [110, 118, 128, 255], 0.88, 0.26),
            ("controller_box", controller_box_placed, [34, 62, 56, 255], 0.42, 0.48),
        ]
        if wrist_3dof_placed is None:
            node_specs.append(("wrist_tube", wrist_tube_placed, link_rgba, 0.55, 0.30))
        # Direct-drive servo mode: wrapping dual-servo clevises instead of spur gears
        if is_direct_servo_mode and clevis_shoulder_placed is not None and clevis_elbow_placed is not None:
            node_specs.append(("clevis_shoulder", clevis_shoulder_placed, [52, 72, 98, 255], 0.48, 0.38))
            node_specs.append(("clevis_elbow", clevis_elbow_placed, [52, 72, 98, 255], 0.48, 0.38))
        else:
            if not replace_sh_gear:
                node_specs.append(("shoulder_gear", shoulder_gear_placed, [224, 174, 72, 255], 0.74, 0.28))
            if not replace_el_gear:
                node_specs.append(("elbow_gear", elbow_gear_placed, [196, 152, 68, 255], 0.74, 0.28))

        # End effector: 3-part articulated gripper, 3-DOF articulated wrist, or ISO TCP flange
        if gripper_parts_placed is not None:
            g_base, g_left, g_right = gripper_parts_placed
            node_specs.append(("gripper_base", g_base, [44, 52, 64, 255], 0.45, 0.40))
            node_specs.append(("gripper_finger_left", g_left, [120, 134, 150, 255], 0.72, 0.30))
            node_specs.append(("gripper_finger_right", g_right, [120, 134, 150, 255], 0.72, 0.30))
        elif wrist_3dof_placed is not None:
            w4, w5, w6 = wrist_3dof_placed
            node_specs.append(("wrist_j4_roll", w4, [72, 140, 184, 255], 0.72, 0.30))
            node_specs.append(("wrist_j5_pitch", w5, [54, 68, 88, 255], 0.75, 0.28))
            node_specs.append(("wrist_j6_flange", w6, [242, 182, 58, 255], 0.84, 0.24))
        else:
            node_specs.append(("tcp_flange", tcp_flange_placed, [242, 182, 58, 255], 0.80, 0.26))

        # v4.1 Red anodized 25T aluminum servo horns
        for h_name, h_solid, _ in horn_nodes:
            node_specs.append((h_name, h_solid, [220, 38, 38, 255], 0.88, 0.24))

        # v5.0 Cycloidal/Planetary reducers, 7-DOF roll joint, and homing sensor
        for v5_name, v5_solid, v5_rgba, v5_met, v5_rough, _ in extra_v5_nodes:
            node_specs.append((v5_name, v5_solid, v5_rgba, v5_met, v5_rough))

        for node_name, solid, rgba, met, rough in node_specs:
            mesh = _part_to_trimesh(solid, rgba, met, rough, node_name)
            scene.add_geometry(mesh, node_name=node_name, geom_name=node_name)
        scene.export(str(glb_path), file_type="glb")

    # 5. Explode vectors in exact GLB spatial units (mm) for 3D Exploded View slider
    u1_x, u1_z = math.cos(a1), math.sin(a1)
    u2_x, u2_z = math.cos(a2), math.sin(a2)
    u3_x, u3_z = math.cos(a3), math.sin(a3)

    tcp_explode = [
        round(32.0 * u1_x + 42.0 * u2_x + 70.0 * u3_x, 2),
        48.0,
        round(32.0 * u1_z + 42.0 * u2_z + 70.0 * u3_z, 2),
    ]
    explode_vectors: dict[str, list[float]] = {
        "base_pedestal": [0.0, 0.0, -40.0],
        "shoulder_mount": [0.0, -52.0, 0.0],
        "shoulder_gear": [0.0, 56.0, 0.0],
        "clevis_shoulder": [0.0, 0.0, 26.0],
        "upper_arm_link": [round(32.0 * u1_x, 2), 0.0, round(32.0 * u1_z, 2)],
        "elbow_gear": [round(32.0 * u1_x, 2), -56.0, round(32.0 * u1_z, 2)],
        "clevis_elbow": [round(32.0 * u1_x, 2), 18.0, round(32.0 * u1_z + 24.0, 2)],
        "forearm_link": [round(32.0 * u1_x + 42.0 * u2_x, 2), 48.0, round(32.0 * u1_z + 42.0 * u2_z, 2)],
        "wrist_tube": [
            round(32.0 * u1_x + 42.0 * u2_x + 36.0 * u3_x, 2),
            48.0,
            round(32.0 * u1_z + 42.0 * u2_z + 36.0 * u3_z, 2),
        ],
        "tcp_flange": tcp_explode,
        "wrist_j4_roll": [
            round(32.0 * u1_x + 42.0 * u2_x + 42.0 * u3_x, 2),
            48.0,
            round(32.0 * u1_z + 42.0 * u2_z + 42.0 * u3_z, 2),
        ],
        "wrist_j5_pitch": [
            round(32.0 * u1_x + 42.0 * u2_x + 68.0 * u3_x, 2),
            48.0,
            round(32.0 * u1_z + 42.0 * u2_z + 68.0 * u3_z, 2),
        ],
        "wrist_j6_flange": [
            round(32.0 * u1_x + 42.0 * u2_x + 96.0 * u3_x, 2),
            48.0,
            round(32.0 * u1_z + 42.0 * u2_z + 96.0 * u3_z, 2),
        ],
        "gripper_base": tcp_explode,
        "gripper_finger_left": [tcp_explode[0], round(tcp_explode[1] - 24.0, 2), tcp_explode[2]],
        "gripper_finger_right": [tcp_explode[0], round(tcp_explode[1] + 24.0, 2), tcp_explode[2]],
        "joint_hw_shoulder": [0.0, -22.0, 0.0],
        "joint_hw_elbow": [round(32.0 * u1_x, 2), 20.0, round(32.0 * u1_z, 2)],
        "joint_hw_wrist": [round(32.0 * u1_x + 42.0 * u2_x, 2), 76.0, round(32.0 * u1_z + 42.0 * u2_z, 2)],
        # v4.0 Hardware explode vectors
        "motor_j1_shoulder": [0.0, -96.0, 0.0],
        "motor_j2_elbow": [round(32.0 * u1_x, 2), -98.0, round(32.0 * u1_z, 2)],
        "motor_j3_wrist": [round(32.0 * u1_x + 42.0 * u2_x, 2), -18.0, round(32.0 * u1_z + 42.0 * u2_z, 2)],
        "bearing_j1": [0.0, 28.0, 0.0],
        "bearing_j2": [round(32.0 * u1_x, 2), 24.0, round(32.0 * u1_z, 2)],
        "bearing_j3": [round(32.0 * u1_x + 42.0 * u2_x, 2), 84.0, round(32.0 * u1_z + 42.0 * u2_z, 2)],
        "fasteners_kit": [0.0, 0.0, 32.0],
        "controller_box": [-58.0, 0.0, -24.0],
    }
    for h_name, _, h_vec in horn_nodes:
        explode_vectors[h_name] = h_vec
    for v5_name, _, _, _, _, v5_vec in extra_v5_nodes:
        explode_vectors[v5_name] = v5_vec

    printable_parts: dict[str, Part] = {
        "base_pedestal": base_raw,
        "upper_arm_link": upper_raw,
        "forearm_link": forearm_raw,
    }
    if wrist_3dof_placed is None:
        printable_parts["wrist_tube"] = wrist_tube_placed
    if parallelogram_rod_placed is not None:
        printable_parts["parallelogram_rod"] = parallelogram_rod_placed
    if not is_direct_servo_mode:
        printable_parts["shoulder_mount"] = mount_raw
        if not replace_sh_gear:
            printable_parts["shoulder_gear"] = sh_gear_raw
        if not replace_el_gear:
            printable_parts["elbow_gear"] = el_gear_raw
    elif clevis_shoulder_placed is not None and clevis_elbow_placed is not None:
        printable_parts["clevis_shoulder"] = clevis_shoulder_placed
        printable_parts["clevis_elbow"] = clevis_elbow_placed

    # Always include end-effector / wrist printable parts (gripper or 3-DOF wrist or TCP flange)
    if gripper_parts_placed is not None:
        g_base, g_left, g_right = gripper_parts_placed
        printable_parts["gripper_base"] = g_base
        printable_parts["gripper_finger_left"] = g_left
        printable_parts["gripper_finger_right"] = g_right
    elif wrist_3dof_placed is not None:
        w4, w5, w6 = wrist_3dof_placed
        printable_parts["wrist_j4_roll"] = w4
        printable_parts["wrist_j5_pitch"] = w5
        printable_parts["wrist_j6_flange"] = w6
    else:
        printable_parts["tcp_flange"] = tcp_flange_placed

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

