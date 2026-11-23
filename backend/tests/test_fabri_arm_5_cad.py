"""Pytest suite for Brazo Robótico Arduino 5.0 (v4.2): Dual-Servo Clevises, Direct-Drive Mode & 3-Part Gripper."""

from __future__ import annotations

import io
import xml.etree.ElementTree as ET
import zipfile

import trimesh

from backend.app.cad.hardware_parts import (
    build_dual_servo_clevis,
    build_gripper_3d,
    build_gripper_parts,
    build_servo_3d,
    build_servo_horn_25t,
)
from backend.app.schemas import JointActuatorSpec, MechatronicsConfig, ModularArmRequest


def test_servo_horn_clevis_and_3part_gripper_solids_valid():
    """TD-8120MG servo, 25T horn, dual-servo clevis, and 3-part gripper must all build valid 3D solids."""
    servo = build_servo_3d("servo_td8120mg")
    assert servo.is_valid and servo.volume > 20_000.0

    horn = build_servo_horn_25t()
    assert horn.is_valid and horn.volume > 800.0

    clevis = build_dual_servo_clevis()
    assert clevis.is_valid and clevis.volume > 15_000.0

    g_base, g_left, g_right = build_gripper_parts()
    assert g_base.is_valid and g_base.volume > 5_000.0
    assert g_left.is_valid and g_left.volume > 1_000.0
    assert g_right.is_valid and g_right.volume > 1_000.0

    gripper_fused = build_gripper_3d()
    assert gripper_fused.is_valid and gripper_fused.volume > 10_000.0


def test_fabri_arm_5_direct_drive_clevises_3part_gripper_and_no_spur_gears():
    """Direct-drive Brazo 5.0 must mount dual-servo clevises, 3-part gripper, red 25T horns, and omit spur gears."""
    from fastapi.testclient import TestClient

    from backend.app.main import app

    client = TestClient(app)

    req = ModularArmRequest(
        dof=6,
        payload_kg=0.45,
        material_id="pla_plus",
        base_height_mm=85.0,
        wrist_mm=65.0,
        auto_fix=True,
        mechatronics=MechatronicsConfig(
            bus_architecture="arduino_pca9685_i2c",
            system_voltage_v=6.5,
            joint_actuators={
                "J1": JointActuatorSpec(
                    motor_id="servo_td8120mg",
                    reducer_id="direct",
                    target_rpm=25.0,
                    parallel_count=2,
                ),
                "J2": JointActuatorSpec(
                    motor_id="servo_td8120mg",
                    reducer_id="direct",
                    target_rpm=30.0,
                    parallel_count=2,
                ),
                "J3": JointActuatorSpec(
                    motor_id="servo_td8120mg",
                    reducer_id="direct",
                    target_rpm=35.0,
                    parallel_count=1,
                ),
            },
        ),
    )

    resp = client.post("/api/arm/assemble", json=req.model_dump())
    assert resp.status_code == 200, resp.text
    data = resp.json()

    # 1. Verify GLB contains dual servos, wrapping clevises, red 25T horns, 3-part gripper, and NO spur gears
    r_glb = client.get(data["files"]["glb"])
    assert r_glb.status_code == 200
    scene = trimesh.load(io.BytesIO(r_glb.content), file_type="glb", force="scene")
    nodes = set(scene.graph.nodes_geometry)

    for expected_node in (
        "motor_j1_shoulder",
        "motor_j2_elbow",
        "motor_j3_wrist",
        "clevis_shoulder",
        "clevis_elbow",
        "horn_25t_j1",
        "horn_25t_j2",
        "horn_25t_j3",
        "gripper_base",
        "gripper_finger_left",
        "gripper_finger_right",
    ):
        assert expected_node in nodes, f"Missing Brazo 5.0 node '{expected_node}' in GLB: {sorted(nodes)}"
        assert expected_node in data["explode_vectors"], f"Missing explode vector for '{expected_node}'"

    # Spur gears MUST be omitted in direct-drive servo mode
    assert "shoulder_gear" not in nodes, "shoulder_gear should be omitted in direct-drive servo mode"
    assert "elbow_gear" not in nodes, "elbow_gear should be omitted in direct-drive servo mode"

    # Dual parallel J1 servo mesh must have roughly 2x the volume of a single servo
    j1_mesh = scene.geometry["motor_j1_shoulder"]
    j3_mesh = scene.geometry["motor_j3_wrist"]
    assert j1_mesh.volume > j3_mesh.volume * 1.7

    # 2. Verify PCA9685 + Arduino UNO SVG schematic
    r_svg = client.get(data["files"]["circuit_svg"])
    assert r_svg.status_code == 200
    assert "PCA9685" in r_svg.text
    assert "arduino" in r_svg.text.lower()
    svg_root = ET.fromstring(r_svg.text)
    assert svg_root.tag.endswith("svg")

    # 3. Verify Master BOM includes TD-8120MG servos & 25T aluminum horns
    mech = data["mechatronics"]
    assert mech is not None
    bom_text = " ".join(f"{b['part_name']} {b['spec']} x{b['qty']}" for b in mech["master_bom"])
    assert "TD-8120MG" in bom_text or "8120" in bom_text
    assert "25T" in bom_text
    assert "PCA9685" in bom_text

    # 4. Verify ZIP pack
    r_zip = client.get(data["files"]["zip"])
    assert r_zip.status_code == 200
    with zipfile.ZipFile(io.BytesIO(r_zip.content), "r") as zf:
        znames = set(zf.namelist())
        assert "arm_circuit_schematic.svg" in znames
        assert "arm_master_bom.csv" in znames
