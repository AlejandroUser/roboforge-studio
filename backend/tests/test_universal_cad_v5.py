"""Pytest suite for RoboForge Studio Pro v5.0 Universal CAD Engine:
Cycloidal/Planetary reducers, GT2 closed belts, SCARA 4-DOF + prismatic URDF, 7-DOF, 3-DOF wrist,
CRB/30205 bearings, STS3215, CNC DXF R12, and LeRobot JSON export.
"""

from __future__ import annotations

import io
import json
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

import trimesh

from backend.app.cad.cycloidal_variants import (
    build_closed_belt_loop_3d,
    build_cycloidal_reducer_3d,
    build_planetary_reducer_3d,
)
from backend.app.cad.hardware_parts import (
    build_bearing_crb,
    build_bearing_tapered_30205,
    build_homing_sensor,
    build_servo_sts3215_3d,
    build_wrist_3dof_parts,
    export_cnc_plates_dxf,
)
from backend.app.schemas import JointActuatorSpec, MechatronicsConfig, ModularArmRequest


def test_v5_reducers_belts_bearings_sts3215_wrist_and_dxf(tmp_path: Path):
    """All new v5.0 CAD primitives and DXF R12 exporter must produce valid geometry."""
    cyc = build_cycloidal_reducer_3d(ratio=25, outer_d_mm=68.0, thickness_mm=14.0)
    assert cyc.volume > 15_000.0

    plan = build_planetary_reducer_3d(ratio=10, outer_d_mm=56.0, thickness_mm=16.0)
    assert plan.volume > 10_000.0

    belt = build_closed_belt_loop_3d(center_dist_mm=120.0, r1_mm=14.0, r2_mm=24.0, width_mm=6.0)
    assert belt.is_valid and belt.volume > 1_000.0

    crb = build_bearing_crb()
    assert crb.is_valid and crb.volume > 10_000.0

    tapered = build_bearing_tapered_30205()
    assert tapered.is_valid and tapered.volume > 8_000.0

    sensor = build_homing_sensor()
    assert sensor.is_valid and sensor.volume > 1_000.0

    sts = build_servo_sts3215_3d()
    assert sts.is_valid and sts.volume > 25_000.0

    w4, w5, w6 = build_wrist_3dof_parts()
    assert w4.is_valid and w4.volume > 5_000.0
    assert w5.is_valid and w5.volume > 5_000.0
    assert w6.is_valid and w6.volume > 3_000.0

    dxf_path = export_cnc_plates_dxf(ModularArmRequest(), tmp_path / "plates.dxf")
    dxf_txt = dxf_path.read_text(encoding="utf-8")
    assert "SECTION" in dxf_txt and "ENTITIES" in dxf_txt and "CIRCLE" in dxf_txt and "EOF" in dxf_txt


def test_v5_scara_4dof_assembly_and_prismatic_urdf():
    """4-DOF SCARA must build linear rails, z_carriage, scara_arm_1/2, belt_scara, z_quill, and prismatic URDF."""
    from fastapi.testclient import TestClient

    from backend.app.main import app

    client = TestClient(app)
    req = ModularArmRequest(
        dof=4,
        kinematics_type="scara",
        z_stroke_mm=140.0,
        payload_kg=0.8,
        material_id="petg_cf",
        auto_fix=True,
    )
    resp = client.post("/api/arm/assemble", json=req.model_dump())
    assert resp.status_code == 200, resp.text
    data = resp.json()

    r_glb = client.get(data["files"]["glb"])
    assert r_glb.status_code == 200
    scene = trimesh.load(io.BytesIO(r_glb.content), file_type="glb", force="scene")
    nodes = set(scene.graph.nodes_geometry)

    for must_node in ("base_pedestal", "linear_rail_z", "z_carriage", "scara_arm_1", "belt_scara", "scara_arm_2", "z_quill"):
        assert must_node in nodes, f"Missing SCARA node '{must_node}' in {sorted(nodes)}"
        assert must_node in data["explode_vectors"]

    # URDF must declare a real prismatic joint along Z
    r_urdf = client.get(data["files"]["urdf"])
    assert r_urdf.status_code == 200
    urdf_root = ET.fromstring(r_urdf.text)
    prismatic_joints = [j for j in urdf_root.findall("joint") if j.attrib.get("type") == "prismatic"]
    assert len(prismatic_joints) >= 1, "SCARA URDF must include a real prismatic Z joint"


def test_v5_cycloidal_7dof_wrist3dof_and_cnc_dxf_and_lerobot():
    """Verify 7-DOF + cycloidal/planetary reducers + 3-DOF wrist + CNC DXF + LeRobot JSON in ZIP."""
    from fastapi.testclient import TestClient

    from backend.app.main import app

    client = TestClient(app)

    # Case A: 7-DOF Cycloidal + 3-DOF Wrist + CNC DXF
    req_7dof = ModularArmRequest(
        dof=7,
        kinematics_type="articulated",
        wrist_3dof=True,
        cnc_plate_mode=True,
        payload_kg=2.6,
        material_id="al6061_t6",
        auto_fix=True,
        mechatronics=MechatronicsConfig(
            bus_architecture="can_fd_bus",
            system_voltage_v=24.0,
            joint_actuators={
                "J1": JointActuatorSpec(motor_id="damiao_dm_j4340", reducer_id="cycloidal_25", target_rpm=25.0),
                "J2": JointActuatorSpec(motor_id="damiao_dm_j4310", reducer_id="planetary_10", target_rpm=30.0),
                "J3": JointActuatorSpec(motor_id="bldc_qdd_5010", reducer_id="direct", target_rpm=40.0),
            },
        ),
    )
    r1 = client.post("/api/arm/assemble", json=req_7dof.model_dump())
    assert r1.status_code == 200, r1.text
    d1 = r1.json()

    scene1 = trimesh.load(io.BytesIO(client.get(d1["files"]["glb"]).content), file_type="glb", force="scene")
    nodes1 = set(scene1.graph.nodes_geometry)
    for n in ("cycloidal_j1", "planetary_j2", "upper_arm_roll_j3", "wrist_j4_roll", "wrist_j5_pitch", "wrist_j6_flange"):
        assert n in nodes1, f"Missing v5.0 node '{n}' in {sorted(nodes1)}"

    assert "cnc_dxf" in d1["files"]
    r_dxf = client.get(d1["files"]["cnc_dxf"])
    assert r_dxf.status_code == 200 and "ENTITIES" in r_dxf.text

    # Case B: LeRobot UART Serial Bus STS3215 + lerobot_config.json
    req_lerobot = ModularArmRequest(
        dof=6,
        kinematics_type="articulated",
        payload_kg=0.5,
        material_id="petg_cf",
        auto_fix=True,
        mechatronics=MechatronicsConfig(
            bus_architecture="uart_serial_bus",
            system_voltage_v=7.4,
            joint_actuators={
                "J1": JointActuatorSpec(motor_id="servo_sts3215", reducer_id="direct", target_rpm=35.0),
                "J2": JointActuatorSpec(motor_id="servo_sts3215", reducer_id="direct", target_rpm=35.0),
                "J3": JointActuatorSpec(motor_id="servo_sts3215", reducer_id="direct", target_rpm=45.0),
            },
        ),
    )
    r2 = client.post("/api/arm/assemble", json=req_lerobot.model_dump())
    assert r2.status_code == 200, r2.text
    d2 = r2.json()
    assert "lerobot_json" in d2["files"]
    r_lr = client.get(d2["files"]["lerobot_json"])
    assert r_lr.status_code == 200
    lr_cfg = json.loads(r_lr.text)
    assert isinstance(lr_cfg, dict) and len(lr_cfg) >= 2

    with zipfile.ZipFile(io.BytesIO(client.get(d2["files"]["zip"]).content), "r") as zf:
        assert "lerobot_config.json" in set(zf.namelist())
