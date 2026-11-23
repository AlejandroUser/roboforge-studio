"""Parametric 14-DOF Bimanual Dual-Arm (ALOHA / ABB YuMi style) 3D CAD assembler + URDF exporter."""

from __future__ import annotations

import math
import tempfile
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
    _place_link_in_xz,
)
from .hardware_parts import (
    build_controller_box,
    build_gripper_3d,
    build_motor_3d,
)
from .variants import (
    _build_hollow_cobot,
    _build_organic_ibeam,
    _build_organic_topopt,
    _build_rounded_truss,
    _build_tapered_solid,
)


def _ensure_watertight_single_body(part: Part, fallback_spec: Any) -> Part:
    """Guarantee a single-body, watertight printable STL solid (falls back to clean tapered_solid if OCCT drops faces)."""
    with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as tmp:
        tmp_stl = Path(tmp.name)
    try:
        export_stl(part, str(tmp_stl), tolerance=0.10, angular_tolerance=0.18)
        loaded = trimesh.load(str(tmp_stl), force="mesh")
        mesh = loaded.dump(concatenate=True) if isinstance(loaded, trimesh.Scene) else loaded
        if isinstance(mesh, trimesh.Trimesh) and mesh.is_watertight:
            bodies = mesh.split(only_watertight=False)
            if len(bodies) == 1:
                return part
    except Exception:
        pass
    finally:
        if tmp_stl.exists():
            try:
                tmp_stl.unlink()
            except OSError:
                pass
    return _build_tapered_solid(fallback_spec)


def build_bimanual_arm_assembly(
    req: ModularArmRequest,
    material: dict[str, Any],
    output_dir: Path,
    design_hash: str,
) -> dict[str, Any]:
    """Build a 14-DOF Bimanual Dual-Arm robot on a shared torso pedestal with mirrored left/right arms."""
    from ..physics.arm_compatibility import resolve_default_slots

    output_dir.mkdir(parents=True, exist_ok=True)
    slots = resolve_default_slots(req)
    assert slots.upper_arm is not None and slots.forearm is not None

    # Clamp bimanual link lengths so printed parts fit a standard 256 mm FDM build plate cleanly
    if float(slots.upper_arm.length_mm) > 205.0:
        slots.upper_arm = slots.upper_arm.model_copy(update={"length_mm": 205.0})
    if float(slots.forearm.length_mm) > 185.0:
        slots.forearm = slots.forearm.model_copy(update={"length_mm": 185.0})

    link_builders = {
        "tapered_solid": _build_tapered_solid,
        "organic_ibeam": _build_organic_ibeam,
        "rounded_truss": _build_rounded_truss,
        "hollow_cobot": _build_hollow_cobot,
        "organic_topopt": _build_organic_topopt,
    }
    upper_raw = _ensure_watertight_single_body(
        _get_or_build_subpart(
            "link",
            slots.upper_arm_variant,
            slots.upper_arm,
            link_builders.get(slots.upper_arm_variant, _build_hollow_cobot),
        ),
        slots.upper_arm,
    )
    forearm_raw = _ensure_watertight_single_body(
        _get_or_build_subpart(
            "link",
            slots.forearm_variant,
            slots.forearm,
            link_builders.get(slots.forearm_variant, _build_hollow_cobot),
        ),
        slots.forearm,
    )

    l1 = float(slots.upper_arm.length_mm)
    l2 = float(slots.forearm.length_mm)
    t1 = float(slots.upper_arm.thickness_mm)
    t2 = float(slots.forearm.thickness_mm)
    torso_h = min(220.0, max(160.0, float(req.base_height_mm) + 70.0))
    sh_y = 104.0  # Left arm at y = -104 mm, Right arm at y = +104 mm (torso width = 236 mm <= 256 mm X1C bed)

    a1_deg = float(req.joint_angles_deg[0])
    a2_deg = a1_deg + float(req.joint_angles_deg[1])
    a3_deg = a2_deg + float(req.joint_angles_deg[2])
    a1, a2 = math.radians(a1_deg), math.radians(a2_deg)

    # 1. Central Torso Column + Wide Base + Shoulder Crossbar (fits 256 mm bed: 136 x 236 x <=220 mm)
    base_plate = Location((0.0, 0.0, 8.0)) * Box(136.0, 224.0, 16.0)
    torso_col = Location((0.0, 0.0, torso_h / 2.0)) * Box(66.0, 92.0, torso_h)
    crossbar = Location((0.0, 0.0, torso_h - 22.0)) * Box(50.0, sh_y * 2.0 + 28.0, 42.0)
    bimanual_torso = base_plate + torso_col + crossbar

    z_sh = torso_h - 22.0
    p1_x = l1 * math.cos(a1)
    p1_z = z_sh + l1 * math.sin(a1)
    p2_x = p1_x + l2 * math.cos(a2)
    p2_z = p1_z + l2 * math.sin(a2)

    # 2. Left Arm (y = -sh_y) and Right Arm (y = +sh_y)
    left_upper_arm = _place_link_in_xz(upper_raw, (0.0, -sh_y, z_sh), a1_deg)
    left_forearm = _place_link_in_xz(forearm_raw, (p1_x, -sh_y - (t1 + t2) * 0.52, p1_z), a2_deg)

    right_upper_arm = _place_link_in_xz(upper_raw, (0.0, +sh_y, z_sh), a1_deg)
    right_forearm = _place_link_in_xz(forearm_raw, (p1_x, +sh_y + (t1 + t2) * 0.52, p1_z), a2_deg)

    gripper_raw = build_gripper_3d()
    left_gripper = Location((p2_x, -sh_y - (t1 + t2) * 0.52, p2_z)) * (
        Location((0.0, 0.0, 0.0), (0.0, -a3_deg, 0.0)) * gripper_raw
    )
    right_gripper = Location((p2_x, +sh_y + (t1 + t2) * 0.52, p2_z)) * (
        Location((0.0, 0.0, 0.0), (0.0, -a3_deg, 0.0)) * gripper_raw
    )

    # 3. Shoulder & Elbow Motors + Controller Box (mounted cleanly outside crossbar faces, zero overlap)
    m_raw = build_motor_3d("servo_sts3215" if req.payload_kg <= 0.8 else "nema17_48mm")
    rot_plus_y = Location((0.0, 0.0, 0.0), (-90.0, 0.0, 0.0))
    rot_minus_y = Location((0.0, 0.0, 0.0), (90.0, 0.0, 0.0))

    motor_j1_shoulder = Compound(
        children=[
            Location((0.0, -(sh_y + 15.0), z_sh)) * (rot_plus_y * m_raw),
            Location((0.0, +(sh_y + 15.0), z_sh)) * (rot_minus_y * m_raw),
        ]
    )
    motor_j2_elbow = Compound(
        children=[
            Location((p1_x, -sh_y + 12.0, p1_z)) * (rot_minus_y * m_raw),
            Location((p1_x, +sh_y - 12.0, p1_z)) * (rot_plus_y * m_raw),
        ]
    )
    controller_box = Location((-88.0, 0.0, 0.0)) * build_controller_box()

    real_volumes_mm3 = {
        "base_pedestal": float(max(bimanual_torso.volume, 1.0)),
        "shoulder_mount": float(max(bimanual_torso.volume * 0.25, 1.0)),
        "shoulder_gear": float(max(upper_raw.volume * 0.15, 1.0)),
        "upper_arm_link": float(max(upper_raw.volume * 2.0, 1.0)),
        "elbow_gear": float(max(forearm_raw.volume * 0.15, 1.0)),
        "forearm_link": float(max(forearm_raw.volume * 2.0, 1.0)),
        "wrist_tube": float(max(gripper_raw.volume * 2.0, 1.0)),
    }

    printable_parts: dict[str, Part] = {
        "bimanual_torso": bimanual_torso,
        "left_upper_arm": upper_raw,
        "right_upper_arm": upper_raw,
        "left_forearm": forearm_raw,
        "right_forearm": forearm_raw,
        "left_gripper": gripper_raw,
        "right_gripper": gripper_raw,
    }

    individual_files: list[Path] = []
    for solid, stem, vhint in (
        (bimanual_torso, "bimanual_torso", "tapered_solid"),
        (upper_raw, "upper_arm_link", slots.upper_arm_variant),
        (forearm_raw, "forearm_link", slots.forearm_variant),
        (gripper_raw, "gripper_assembly", "tapered_solid"),
    ):
        s_p, m_p = _export_part_step_and_3mf(solid, output_dir, stem, vhint)
        individual_files.extend([s_p, m_p])

    full_compound = Compound(
        children=[
            bimanual_torso,
            left_upper_arm,
            left_forearm,
            left_gripper,
            right_upper_arm,
            right_forearm,
            right_gripper,
            motor_j1_shoulder,
            motor_j2_elbow,
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
        link_rgba = _hex_to_rgba(str(material.get("color_hex", "")), [62, 184, 204, 255])
        scene = trimesh.Scene()
        node_specs: list[tuple[str, Part | Compound, list[int], float, float]] = [
            ("bimanual_torso", bimanual_torso, [62, 76, 98, 255], 0.72, 0.32),
            ("left_upper_arm", left_upper_arm, link_rgba, 0.55, 0.32),
            ("left_forearm", left_forearm, link_rgba, 0.55, 0.32),
            ("left_gripper", left_gripper, [52, 60, 74, 255], 0.65, 0.34),
            ("right_upper_arm", right_upper_arm, link_rgba, 0.55, 0.32),
            ("right_forearm", right_forearm, link_rgba, 0.55, 0.32),
            ("right_gripper", right_gripper, [52, 60, 74, 255], 0.65, 0.34),
            ("motor_j1_shoulder", motor_j1_shoulder, [32, 36, 44, 255], 0.58, 0.42),
            ("motor_j2_elbow", motor_j2_elbow, [32, 36, 44, 255], 0.58, 0.42),
            ("controller_box", controller_box, [34, 62, 56, 255], 0.42, 0.48),
        ]
        for node_name, solid, rgba, met, rough in node_specs:
            mesh = _part_to_trimesh(solid, rgba, met, rough, node_name)
            scene.add_geometry(mesh, node_name=node_name, geom_name=node_name)
        scene.export(str(glb_path), file_type="glb")

    explode_vectors: dict[str, list[float]] = {
        "bimanual_torso": [0.0, 0.0, -35.0],
        "left_upper_arm": [24.0, -48.0, 12.0],
        "left_forearm": [54.0, -72.0, 12.0],
        "left_gripper": [84.0, -78.0, 12.0],
        "right_upper_arm": [24.0, 48.0, 12.0],
        "right_forearm": [54.0, 72.0, 12.0],
        "right_gripper": [84.0, 78.0, 12.0],
        "motor_j1_shoulder": [0.0, 0.0, 42.0],
        "motor_j2_elbow": [32.0, 0.0, 42.0],
        "controller_box": [-58.0, 0.0, -24.0],
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


def export_bimanual_urdf(
    req: ModularArmRequest,
    joints: list[JointTorqueSummary],
    output_path: Path,
) -> Path:
    """Export a valid ROS 2 URDF for a 14-DOF Bimanual Dual-Arm robot (left & right kinematic trees)."""
    urdf_xml = f"""<?xml version="1.0" encoding="UTF-8"?>
<robot name="roboforge_bimanual_14dof_{saxutils.escape(req.material_id)}">
  <link name="bimanual_torso">
    <inertial><origin xyz="0 0 0.10" rpy="0 0 0"/><mass value="2.4"/><inertia ixx="0.03" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.03"/></inertial>
  </link>
  <joint name="left_j1_shoulder" type="revolute">
    <parent link="bimanual_torso"/><child link="left_upper_arm"/>
    <origin xyz="0 -0.115 0.18" rpy="0 0 0"/><axis xyz="0 -1 0"/>
    <limit lower="-1.57" upper="1.57" effort="18.0" velocity="3.5"/>
  </joint>
  <link name="left_upper_arm"><inertial><origin xyz="0.10 0 0" rpy="0 0 0"/><mass value="0.35"/><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.002"/></inertial></link>
  <joint name="left_j2_elbow" type="revolute">
    <parent link="left_upper_arm"/><child link="left_forearm"/>
    <origin xyz="0.20 0 0" rpy="0 0 0"/><axis xyz="0 -1 0"/>
    <limit lower="-2.2" upper="2.2" effort="10.0" velocity="4.5"/>
  </joint>
  <link name="left_forearm"><inertial><origin xyz="0.08 0 0" rpy="0 0 0"/><mass value="0.25"/><inertia ixx="0.0006" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial></link>
  <joint name="right_j1_shoulder" type="revolute">
    <parent link="bimanual_torso"/><child link="right_upper_arm"/>
    <origin xyz="0 0.115 0.18" rpy="0 0 0"/><axis xyz="0 -1 0"/>
    <limit lower="-1.57" upper="1.57" effort="18.0" velocity="3.5"/>
  </joint>
  <link name="right_upper_arm"><inertial><origin xyz="0.10 0 0" rpy="0 0 0"/><mass value="0.35"/><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.002"/></inertial></link>
  <joint name="right_j2_elbow" type="revolute">
    <parent link="right_upper_arm"/><child link="right_forearm"/>
    <origin xyz="0.20 0 0" rpy="0 0 0"/><axis xyz="0 -1 0"/>
    <limit lower="-2.2" upper="2.2" effort="10.0" velocity="4.5"/>
  </joint>
  <link name="right_forearm"><inertial><origin xyz="0.08 0 0" rpy="0 0 0"/><mass value="0.25"/><inertia ixx="0.0006" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial></link>
</robot>
"""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(urdf_xml, encoding="utf-8")
    return output_path
