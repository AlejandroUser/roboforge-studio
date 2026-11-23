from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from ..physics.arm_compatibility import resolve_default_slots
from ..schemas import ModularArmRequest


def _urdf_link(
    name: str,
    mass_kg: float,
    geom_xml: str,
    origin_xyz: str = "0 0 0",
    rgba: str = "0.25 0.55 0.85 1.0",
) -> str:
    m = max(0.02, float(mass_kg))
    ixx = round(m * 0.0025, 6)
    return (
        f'  <link name="{name}">\n'
        f"    <inertial>\n"
        f'      <origin xyz="{origin_xyz}" rpy="0 0 0"/>\n'
        f'      <mass value="{m:.4f}"/>\n'
        f'      <inertia ixx="{ixx}" ixy="0" ixz="0" iyy="{ixx}" iyz="0" izz="{ixx}"/>\n'
        f"    </inertial>\n"
        f"    <visual>\n"
        f'      <origin xyz="{origin_xyz}" rpy="0 0 0"/>\n'
        f"      <geometry>{geom_xml}</geometry>\n"
        f'      <material name="mat_{name}"><color rgba="{rgba}"/></material>\n'
        f"    </visual>\n"
        f"    <collision>\n"
        f'      <origin xyz="{origin_xyz}" rpy="0 0 0"/>\n'
        f"      <geometry>{geom_xml}</geometry>\n"
        f"    </collision>\n"
        f"  </link>"
    )


def _urdf_joint(
    name: str,
    jtype: str,
    parent: str,
    child: str,
    xyz: str,
    axis: str,
    lower: float,
    upper: float,
    effort: float = 25.0,
    velocity: float = 3.0,
) -> str:
    return (
        f'  <joint name="{name}" type="{jtype}">\n'
        f'    <parent link="{parent}"/>\n'
        f'    <child link="{child}"/>\n'
        f'    <origin xyz="{xyz}" rpy="0 0 0"/>\n'
        f'    <axis xyz="{axis}"/>\n'
        f'    <limit lower="{lower:.4f}" upper="{upper:.4f}" effort="{effort:.2f}" velocity="{velocity:.2f}"/>\n'
        f"  </joint>"
    )


def export_full_dof_urdf(
    req: ModularArmRequest,
    eval_result: dict[str, Any],
    output_path: Path,
) -> Path:
    """
    Generate a valid ROS 2 / Gazebo URDF XML with exact movable joint counts and
    types matching req.dof and req.kinematics_type across all 8 topologies.
    """
    slots = resolve_default_slots(req)
    l1 = (float(slots.upper_arm.length_mm) if slots.upper_arm else 200.0) / 1000.0
    l2 = (float(slots.forearm.length_mm) if slots.forearm else 170.0) / 1000.0
    l3 = float(req.wrist_mm) / 1000.0
    h_base = float(req.base_height_mm) / 1000.0
    z_stroke = float(req.z_stroke_mm) / 1000.0
    effort = max(5.0, float(eval_result.get("shoulder_torque_nm", 15.0)) * 1.5)
    ktype = req.kinematics_type

    links: list[str] = []
    joints: list[str] = []

    if ktype == "scara":
        links.append(_urdf_link("base_link", 0.85, f'<cylinder radius="0.045" length="{h_base:.4f}"/>', f"0 0 {h_base/2:.4f}", "0.22 0.25 0.30 1"))
        links.append(_urdf_link("scara_arm_1", 0.35, f'<box size="{l1:.4f} 0.036 0.024"/>', f"{l1/2:.4f} 0 0"))
        links.append(_urdf_link("scara_arm_2", 0.28, f'<box size="{l2:.4f} 0.030 0.022"/>', f"{l2/2:.4f} 0 0"))
        links.append(_urdf_link("z_quill", 0.18, f'<cylinder radius="0.010" length="{z_stroke:.4f}"/>', f"0 0 {-z_stroke/2:.4f}", "0.80 0.82 0.85 1"))
        links.append(_urdf_link("tcp_tool", 0.08, '<cylinder radius="0.014" length="0.020"/>', "0 0 -0.01", "0.95 0.65 0.10 1"))
        joints.append(_urdf_joint("joint_1_scara_j1", "revolute", "base_link", "scara_arm_1", f"0 0 {h_base:.4f}", "0 0 1", -2.35, 2.35, effort))
        joints.append(_urdf_joint("joint_2_scara_j2", "revolute", "scara_arm_1", "scara_arm_2", f"{l1:.4f} 0 0", "0 0 1", -2.50, 2.50, effort * 0.6))
        joints.append(_urdf_joint("joint_3_z_prismatic", "prismatic", "scara_arm_2", "z_quill", f"{l2:.4f} 0 0", "0 0 1", -z_stroke, 0.0, 40.0, 0.5))
        joints.append(_urdf_joint("joint_4_tool_yaw", "revolute", "z_quill", "tcp_tool", f"0 0 {-z_stroke:.4f}", "0 0 1", -3.1416, 3.1416, 5.0))

    elif ktype == "cartesian_gantry":
        span_x = max(0.18, l1)
        span_y = max(0.18, l2)
        links.append(_urdf_link("gantry_frame", 1.80, f'<box size="{span_x:.4f} {span_y:.4f} 0.025"/>', "0 0 0.0125", "0.75 0.78 0.82 1"))
        links.append(_urdf_link("gantry_y_bridge", 0.55, f'<box size="0.030 {span_y:.4f} 0.030"/>', f"0 0 {h_base:.4f}"))
        links.append(_urdf_link("gantry_x_carriage", 0.25, '<box size="0.045 0.045 0.040"/>'))
        links.append(_urdf_link("gantry_z_head", 0.18, '<cylinder radius="0.012" length="0.080"/>'))
        links.append(_urdf_link("vacuum_nozzle", 0.05, '<cylinder radius="0.008" length="0.030"/>', "0 0 -0.015", "0.95 0.65 0.10 1"))
        joints.append(_urdf_joint("joint_x_prismatic", "prismatic", "gantry_frame", "gantry_y_bridge", "0 0 0", "1 0 0", -span_x / 2, span_x / 2, 50.0, 0.6))
        joints.append(_urdf_joint("joint_y_prismatic", "prismatic", "gantry_y_bridge", "gantry_x_carriage", f"0 0 {h_base:.4f}", "0 1 0", -span_y / 2, span_y / 2, 40.0, 0.6))
        joints.append(_urdf_joint("joint_z_prismatic", "prismatic", "gantry_x_carriage", "gantry_z_head", "0 0 0", "0 0 1", -z_stroke, 0.0, 35.0, 0.5))
        joints.append(_urdf_joint("joint_nozzle_yaw", "revolute", "gantry_z_head", "vacuum_nozzle", "0 0 -0.04", "0 0 1", -3.1416, 3.1416, 3.0))

    elif ktype == "delta_parallel":
        links.append(_urdf_link("delta_top_frame", 1.10, '<cylinder radius="0.110" length="0.024"/>', f"0 0 {h_base:.4f}", "0.22 0.35 0.55 1"))
        for k in range(3):
            ang = math.radians(k * 120.0)
            ax = f"{-math.sin(ang):.4f} {math.cos(ang):.4f} 0"
            bx = 0.095 * math.cos(ang)
            by = 0.095 * math.sin(ang)
            links.append(_urdf_link(f"delta_bicep_{k+1}", 0.16, f'<box size="{l1*0.65:.4f} 0.016 0.016"/>', f"{l1*0.32:.4f} 0 0"))
            joints.append(_urdf_joint(f"joint_delta_{k+1}", "revolute", "delta_top_frame", f"delta_bicep_{k+1}", f"{bx:.4f} {by:.4f} {h_base:.4f}", ax, -0.7, 1.2, effort))
        links.append(_urdf_link("delta_platform", 0.22, '<cylinder radius="0.042" length="0.016"/>', "0 0 0", "0.95 0.65 0.10 1"))
        joints.append(_urdf_joint("joint_delta_z_platform", "prismatic", "delta_top_frame", "delta_platform", f"0 0 {h_base - 0.12:.4f}", "0 0 1", -z_stroke, 0.0, 30.0, 1.0))

    elif ktype == "stewart_hexapod":
        links.append(_urdf_link("stewart_base", 1.60, '<cylinder radius="0.120" length="0.020"/>', "0 0 0.01", "0.25 0.35 0.50 1"))
        for k in range(6):
            ang = math.radians(k * 60.0)
            px = 0.095 * math.cos(ang)
            py = 0.095 * math.sin(ang)
            links.append(_urdf_link(f"stewart_leg_{k+1}", 0.24, f'<cylinder radius="0.009" length="{h_base:.4f}"/>', f"0 0 {h_base/2:.4f}", "0.80 0.84 0.88 1"))
            joints.append(_urdf_joint(f"joint_leg_{k+1}_prismatic", "prismatic", "stewart_base", f"stewart_leg_{k+1}", f"{px:.4f} {py:.4f} 0.02", "0 0 1", 0.0, z_stroke, 80.0, 0.4))

    elif ktype == "bimanual_dual_arm":
        links.append(_urdf_link("bimanual_torso", 2.40, f'<box size="0.14 0.28 {h_base:.4f}"/>', f"0 0 {h_base/2:.4f}", "0.20 0.24 0.30 1"))
        for side, sign in (("left", 1.0), ("right", -1.0)):
            segs = [
                (f"{side}_j1_shoulder_yaw", f"{side}_link_1", "0 0 1", f"0 {sign*0.14:.4f} {h_base:.4f}", 0.04, 0.28),
                (f"{side}_j2_shoulder_pitch", f"{side}_link_2", "0 1 0", "0 0 0.03", l1 * 0.5, 0.32),
                (f"{side}_j3_upper_roll", f"{side}_link_3", "1 0 0", f"{l1*0.5:.4f} 0 0", l1 * 0.5, 0.26),
                (f"{side}_j4_elbow_pitch", f"{side}_link_4", "0 1 0", f"{l1*0.5:.4f} 0 0", l2, 0.24),
                (f"{side}_j5_wrist_roll", f"{side}_link_5", "1 0 0", f"{l2:.4f} 0 0", l3 * 0.5, 0.14),
                (f"{side}_j6_wrist_pitch", f"{side}_link_6", "0 1 0", f"{l3*0.5:.4f} 0 0", l3 * 0.5, 0.12),
                (f"{side}_j7_tcp_roll", f"{side}_gripper", "1 0 0", f"{l3*0.5:.4f} 0 0", 0.04, 0.10),
            ]
            parent = "bimanual_torso"
            for jname, lname, axis, xyz, seg_len, seg_m in segs:
                links.append(_urdf_link(lname, seg_m, f'<box size="{seg_len:.4f} 0.028 0.024"/>', f"{seg_len/2:.4f} 0 0"))
                joints.append(_urdf_joint(jname, "revolute", parent, lname, xyz, axis, -2.6, 2.6, effort * 0.6))
                parent = lname

    elif ktype in ("continuum_pcc", "soft_pneumatic_bellows"):
        n_segs = 6
        seg_l = max(0.012, l1 / n_segs)
        links.append(_urdf_link("continuum_base", 0.45, f'<cylinder radius="0.032" length="{h_base:.4f}"/>', f"0 0 {h_base/2:.4f}", "0.20 0.24 0.30 1"))
        parent = "continuum_base"
        for k in range(1, n_segs + 1):
            lname = f"continuum_seg_{k}"
            z_off = h_base if k == 1 else seg_l
            axis = "0 1 0" if k % 2 == 1 else "1 0 0"
            links.append(_urdf_link(lname, 0.04, f'<cylinder radius="0.012" length="{seg_l:.4f}"/>', f"0 0 {seg_l/2:.4f}", "0.05 0.72 0.85 1"))
            joints.append(_urdf_joint(f"joint_cont_{k}", "revolute", parent, lname, f"0 0 {z_off:.4f}", axis, -0.55, 0.55, 4.0))
            parent = lname

    else:
        # Articulated 4..7 DOF (exact joint count == req.dof!)
        n_dof = max(4, min(7, int(req.dof)))
        links.append(_urdf_link("base_link", 0.90, f'<cylinder radius="0.048" length="{h_base:.4f}"/>', f"0 0 {h_base/2:.4f}", "0.22 0.25 0.30 1"))
        chain_spec: list[tuple[str, str, str, str, float, float]] = [
            ("joint_1_base_yaw", "turret_link", "0 0 1", f"0 0 {h_base:.4f}", 0.035, 0.30),
            ("joint_2_shoulder_pitch", "upper_arm_link", "0 1 0", "0 0 0.035", l1, 0.42),
            ("joint_3_elbow_pitch", "forearm_link", "0 1 0", f"{l1:.4f} 0 0", l2, 0.30),
            ("joint_4_wrist_pitch", "wrist_pitch_link", "0 1 0", f"{l2:.4f} 0 0", l3 * 0.5, 0.14),
            ("joint_5_wrist_roll", "wrist_roll_link", "1 0 0", f"{l3*0.5:.4f} 0 0", l3 * 0.5, 0.12),
            ("joint_6_tcp_roll", "tcp_flange_link", "1 0 0", f"{l3*0.5:.4f} 0 0", 0.025, 0.08),
            ("joint_7_redundant_roll", "tcp_tool_link", "0 1 0", "0.025 0 0", 0.025, 0.06),
        ]
        parent = "base_link"
        for jname, lname, axis, xyz, seg_len, seg_m in chain_spec[:n_dof]:
            links.append(_urdf_link(lname, seg_m, f'<box size="{seg_len:.4f} 0.034 0.026"/>', f"{seg_len/2:.4f} 0 0"))
            joints.append(_urdf_joint(jname, "revolute", parent, lname, xyz, axis, -2.6, 2.6, effort))
            parent = lname

    urdf_xml = (
        '<?xml version="1.0" encoding="utf-8"?>\n'
        f'<robot name="roboforge_{ktype}_{req.dof}dof">\n'
        + "\n".join(links)
        + "\n"
        + "\n".join(joints)
        + "\n</robot>\n"
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(urdf_xml, encoding="utf-8")
    return output_path


def export_topology_mujoco(
    req: ModularArmRequest,
    eval_result: dict[str, Any],
    output_path: Path,
) -> Path:
    """
    Generate a valid MuJoCo MJCF XML faithful to each kinematic topology:
    - articulated: exact req.dof hinge joints (4..7)
    - scara: 2 hinge Z + 1 slide Z + 1 hinge tool
    - cartesian_gantry: 3 orthogonal slide joints (X, Y, Z) + 1 hinge nozzle
    - delta_parallel: 3 parallel arms + platform + <equality><connect/> loop closure
    - stewart_hexapod: 6 slide legs + platform + <equality><connect/> loop closure
    - bimanual_dual_arm: shared torso + 7 left + 7 right = 14 hinge joints
    - continuum_pcc / soft_pneumatic_bellows: 6 segmented bodies + <tendon> actuators
    """
    slots = resolve_default_slots(req)
    l1 = (float(slots.upper_arm.length_mm) if slots.upper_arm else 200.0) / 1000.0
    l2 = (float(slots.forearm.length_mm) if slots.forearm else 170.0) / 1000.0
    l3 = float(req.wrist_mm) / 1000.0
    h_base = float(req.base_height_mm) / 1000.0
    z_stroke = float(req.z_stroke_mm) / 1000.0
    ktype = req.kinematics_type

    grav_z = 0.0 if req.environment_mode == "space_zero_g" else -float(req.gravity_m_s2)
    world_lines: list[str] = ['    <light pos="0 0 2" dir="0 0 -1"/>']
    equality_lines: list[str] = []
    tendon_lines: list[str] = []
    actuator_lines: list[str] = []

    if ktype == "scara":
        world_lines.append(
            f'    <body name="base_pedestal" pos="0 0 0">\n'
            f'      <geom type="cylinder" size="0.045 {h_base/2:.4f}" pos="0 0 {h_base/2:.4f}"/>\n'
            f'      <body name="scara_arm_1" pos="0 0 {h_base:.4f}">\n'
            f'        <joint name="j1_scara" type="hinge" axis="0 0 1" range="-2.4 2.4"/>\n'
            f'        <geom type="box" size="{l1/2:.4f} 0.018 0.012" pos="{l1/2:.4f} 0 0" mass="0.35"/>\n'
            f'        <body name="scara_arm_2" pos="{l1:.4f} 0 0">\n'
            f'          <joint name="j2_scara" type="hinge" axis="0 0 1" range="-2.5 2.5"/>\n'
            f'          <geom type="box" size="{l2/2:.4f} 0.015 0.011" pos="{l2/2:.4f} 0 0" mass="0.28"/>\n'
            f'          <body name="z_quill" pos="{l2:.4f} 0 0">\n'
            f'            <joint name="j3_z_slide" type="slide" axis="0 0 1" range="{-z_stroke:.4f} 0"/>\n'
            f'            <geom type="cylinder" size="0.010 {z_stroke/2:.4f}" pos="0 0 {-z_stroke/2:.4f}" mass="0.18"/>\n'
            f'            <body name="tcp_tool" pos="0 0 {-z_stroke:.4f}">\n'
            f'              <joint name="j4_tool_yaw" type="hinge" axis="0 0 1" range="-3.14 3.14"/>\n'
            f'              <geom type="cylinder" size="0.014 0.010" mass="0.08"/>\n'
            f"            </body>\n"
            f"          </body>\n"
            f"        </body>\n"
            f"      </body>\n"
            f"    </body>"
        )
        for jn in ("j1_scara", "j2_scara", "j3_z_slide", "j4_tool_yaw"):
            actuator_lines.append(f'    <motor name="act_{jn}" joint="{jn}" gear="20"/>')

    elif ktype == "cartesian_gantry":
        sx, sy = max(0.18, l1), max(0.18, l2)
        world_lines.append(
            f'    <body name="gantry_frame" pos="0 0 0">\n'
            f'      <geom type="box" size="{sx/2:.4f} {sy/2:.4f} 0.012" pos="0 0 0.012"/>\n'
            f'      <body name="gantry_y_bridge" pos="0 0 {h_base:.4f}">\n'
            f'        <joint name="j_x_slide" type="slide" axis="1 0 0" range="{-sx/2:.4f} {sx/2:.4f}"/>\n'
            f'        <geom type="box" size="0.015 {sy/2:.4f} 0.015" mass="0.55"/>\n'
            f'        <body name="gantry_x_carriage" pos="0 0 0">\n'
            f'          <joint name="j_y_slide" type="slide" axis="0 1 0" range="{-sy/2:.4f} {sy/2:.4f}"/>\n'
            f'          <geom type="box" size="0.022 0.022 0.020" mass="0.25"/>\n'
            f'          <body name="gantry_z_head" pos="0 0 0">\n'
            f'            <joint name="j_z_slide" type="slide" axis="0 0 1" range="{-z_stroke:.4f} 0"/>\n'
            f'            <geom type="cylinder" size="0.012 0.040" mass="0.18"/>\n'
            f'            <body name="vacuum_nozzle" pos="0 0 -0.04">\n'
            f'              <joint name="j_nozzle_yaw" type="hinge" axis="0 0 1" range="-3.14 3.14"/>\n'
            f'              <geom type="cylinder" size="0.008 0.015" mass="0.05"/>\n'
            f"            </body>\n"
            f"          </body>\n"
            f"        </body>\n"
            f"      </body>\n"
            f"    </body>"
        )
        for jn in ("j_x_slide", "j_y_slide", "j_z_slide", "j_nozzle_yaw"):
            actuator_lines.append(f'    <motor name="act_{jn}" joint="{jn}" gear="20"/>')

    elif ktype == "delta_parallel":
        world_lines.append(
            f'    <body name="delta_top_frame" pos="0 0 {h_base:.4f}">\n'
            f'      <geom type="cylinder" size="0.110 0.012"/>'
        )
        for k in range(3):
            ang = math.radians(k * 120.0)
            bx, by = 0.095 * math.cos(ang), 0.095 * math.sin(ang)
            ax = f"{-math.sin(ang):.4f} {math.cos(ang):.4f} 0"
            world_lines.append(
                f'      <body name="delta_bicep_{k+1}" pos="{bx:.4f} {by:.4f} 0">\n'
                f'        <joint name="j_delta_{k+1}" type="hinge" axis="{ax}" range="-0.7 1.2"/>\n'
                f'        <geom type="capsule" fromto="0 0 0 {l1*0.5*math.cos(ang):.4f} {l1*0.5*math.sin(ang):.4f} -0.05" size="0.008" mass="0.14"/>\n'
                f'        <body name="delta_forearm_{k+1}" pos="{l1*0.5*math.cos(ang):.4f} {l1*0.5*math.sin(ang):.4f} -0.05">\n'
                f'          <joint name="j_delta_ball_{k+1}" type="ball"/>\n'
                f'          <geom type="capsule" fromto="0 0 0 0 0 -0.14" size="0.004" mass="0.06"/>\n'
                f"        </body>\n"
                f"      </body>"
            )
            equality_lines.append(
                f'    <connect body1="delta_forearm_{k+1}" body2="delta_platform" anchor="0 0 -0.14"/>'
            )
            actuator_lines.append(f'    <motor name="act_delta_{k+1}" joint="j_delta_{k+1}" gear="25"/>')
        world_lines.append("    </body>")
        world_lines.append(
            f'    <body name="delta_platform" pos="0 0 {max(0.05, h_base - 0.19):.4f}">\n'
            f'      <joint name="j_platform_z" type="slide" axis="0 0 1" range="{-z_stroke:.4f} 0.05"/>\n'
            f'      <geom type="cylinder" size="0.040 0.008" mass="0.22"/>\n'
            f"    </body>"
        )

    elif ktype == "stewart_hexapod":
        world_lines.append('    <body name="stewart_base" pos="0 0 0.01">\n      <geom type="cylinder" size="0.120 0.010"/>')
        for k in range(6):
            ang = math.radians(k * 60.0)
            px, py = 0.095 * math.cos(ang), 0.095 * math.sin(ang)
            world_lines.append(
                f'      <body name="stewart_leg_{k+1}" pos="{px:.4f} {py:.4f} 0.01">\n'
                f'        <joint name="j_leg_{k+1}_slide" type="slide" axis="0 0 1" range="0 {z_stroke:.4f}"/>\n'
                f'        <geom type="cylinder" size="0.009 {h_base/2:.4f}" pos="0 0 {h_base/2:.4f}" mass="0.22"/>\n'
                f"      </body>"
            )
            equality_lines.append(
                f'    <connect body1="stewart_leg_{k+1}" body2="stewart_platform" anchor="0 0 {h_base:.4f}"/>'
            )
            actuator_lines.append(f'    <motor name="act_leg_{k+1}" joint="j_leg_{k+1}_slide" gear="40"/>')
        world_lines.append("    </body>")
        world_lines.append(
            f'    <body name="stewart_platform" pos="0 0 {h_base + 0.02:.4f}">\n'
            f'      <freejoint name="j_platform_free"/>\n'
            f'      <geom type="cylinder" size="0.095 0.010" mass="0.65"/>\n'
            f"    </body>"
        )

    elif ktype == "bimanual_dual_arm":
        world_lines.append(
            f'    <body name="bimanual_torso" pos="0 0 {h_base/2:.4f}">\n'
            f'      <geom type="box" size="0.07 0.14 {h_base/2:.4f}" mass="2.4"/>'
        )
        for side, sign in (("left", 1.0), ("right", -1.0)):
            segs = [
                (f"{side}_j1", "0 0 1", f"0 {sign*0.14:.4f} {h_base/2:.4f}", 0.04),
                (f"{side}_j2", "0 1 0", "0 0 0.03", l1 * 0.5),
                (f"{side}_j3", "1 0 0", f"{l1*0.5:.4f} 0 0", l1 * 0.5),
                (f"{side}_j4", "0 1 0", f"{l1*0.5:.4f} 0 0", l2),
                (f"{side}_j5", "1 0 0", f"{l2:.4f} 0 0", l3 * 0.5),
                (f"{side}_j6", "0 1 0", f"{l3*0.5:.4f} 0 0", l3 * 0.5),
                (f"{side}_j7", "1 0 0", f"{l3*0.5:.4f} 0 0", 0.04),
            ]
            indent = "      "
            for idx, (jn, ax, pos_s, sl) in enumerate(segs):
                pad = indent + "  " * idx
                world_lines.append(
                    f'{pad}<body name="{jn}_body" pos="{pos_s}">\n'
                    f'{pad}  <joint name="{jn}" type="hinge" axis="{ax}" range="-2.6 2.6"/>\n'
                    f'{pad}  <geom type="box" size="{sl/2:.4f} 0.014 0.012" pos="{sl/2:.4f} 0 0" mass="0.22"/>'
                )
                actuator_lines.append(f'    <motor name="act_{jn}" joint="{jn}" gear="20"/>')
            for idx in reversed(range(len(segs))):
                world_lines.append(f"{indent + '  ' * idx}</body>")
        world_lines.append("    </body>")

    elif ktype in ("continuum_pcc", "soft_pneumatic_bellows"):
        n_segs = 6
        seg_l = max(0.012, l1 / n_segs)
        world_lines.append(
            f'    <body name="continuum_base" pos="0 0 0">\n'
            f'      <geom type="cylinder" size="0.030 {h_base/2:.4f}" pos="0 0 {h_base/2:.4f}"/>'
        )
        for k in range(1, n_segs + 1):
            pad = "      " + "  " * (k - 1)
            z_off = h_base if k == 1 else seg_l
            ax = "0 1 0" if k % 2 == 1 else "1 0 0"
            world_lines.append(
                f'{pad}<body name="seg_{k}" pos="0 0 {z_off:.4f}">\n'
                f'{pad}  <joint name="j_seg_{k}" type="hinge" axis="{ax}" range="-0.55 0.55" stiffness="1.2" damping="0.15"/>\n'
                f'{pad}  <geom type="cylinder" size="0.011 {seg_l/2:.4f}" pos="0 0 {seg_l/2:.4f}" mass="0.035"/>\n'
                f'{pad}  <site name="s1_{k}" pos="0.008 0 {seg_l/2:.4f}" size="0.002"/>\n'
                f'{pad}  <site name="s2_{k}" pos="-0.004 0.007 {seg_l/2:.4f}" size="0.002"/>\n'
                f'{pad}  <site name="s3_{k}" pos="-0.004 -0.007 {seg_l/2:.4f}" size="0.002"/>'
            )
        for k in reversed(range(1, n_segs + 1)):
            world_lines.append(f"{'      ' + '  ' * (k - 1)}</body>")
        world_lines.append("    </body>")
        for t_idx in (1, 2, 3):
            sites = "\n".join(f'      <site site="s{t_idx}_{k}"/>' for k in range(1, n_segs + 1))
            tendon_lines.append(f'    <spatial name="tendon_{t_idx}">\n{sites}\n    </spatial>')
            actuator_lines.append(f'    <general name="act_tendon_{t_idx}" tendon="tendon_{t_idx}" gainprm="15"/>')

    else:
        # Articulated 4..7 DOF (exact joint count == req.dof!)
        n_dof = max(4, min(7, int(req.dof)))
        world_lines.append(
            f'    <body name="base_pedestal" pos="0 0 0">\n'
            f'      <geom type="cylinder" size="0.048 {h_base/2:.4f}" pos="0 0 {h_base/2:.4f}"/>'
        )
        chain = [
            ("j1_base_yaw", "0 0 1", f"0 0 {h_base:.4f}", 0.035, 0.30),
            ("j2_shoulder_pitch", "0 1 0", "0 0 0.035", l1, 0.42),
            ("j3_elbow_pitch", "0 1 0", f"{l1:.4f} 0 0", l2, 0.30),
            ("j4_wrist_pitch", "0 1 0", f"{l2:.4f} 0 0", l3 * 0.5, 0.14),
            ("j5_wrist_roll", "1 0 0", f"{l3*0.5:.4f} 0 0", l3 * 0.5, 0.12),
            ("j6_tcp_roll", "1 0 0", f"{l3*0.5:.4f} 0 0", 0.025, 0.08),
            ("j7_redundant_roll", "0 1 0", "0.025 0 0", 0.025, 0.06),
        ][:n_dof]
        for idx, (jn, ax, pos_s, sl, sm) in enumerate(chain):
            pad = "      " + "  " * idx
            world_lines.append(
                f'{pad}<body name="{jn}_body" pos="{pos_s}">\n'
                f'{pad}  <joint name="{jn}" type="hinge" axis="{ax}" range="-2.6 2.6"/>\n'
                f'{pad}  <geom type="box" size="{sl/2:.4f} 0.017 0.013" pos="{sl/2:.4f} 0 0" mass="{sm:.3f}"/>'
            )
            actuator_lines.append(f'    <motor name="act_{jn}" joint="{jn}" gear="25"/>')
        for idx in reversed(range(len(chain))):
            world_lines.append(f"{'      ' + '  ' * idx}</body>")
        world_lines.append("    </body>")

    sections: list[str] = [
        f'<mujoco model="roboforge_{ktype}_{req.dof}dof">',
        f'  <option gravity="0 0 {grav_z:.2f}" timestep="0.002"/>',
        "  <worldbody>",
        *world_lines,
        "  </worldbody>",
    ]
    if equality_lines:
        sections.extend(["  <equality>", *equality_lines, "  </equality>"])
    if tendon_lines:
        sections.extend(["  <tendon>", *tendon_lines, "  </tendon>"])
    if actuator_lines:
        sections.extend(["  <actuator>", *actuator_lines, "  </actuator>"])
    sections.append("</mujoco>\n")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(sections), encoding="utf-8")
    return output_path
