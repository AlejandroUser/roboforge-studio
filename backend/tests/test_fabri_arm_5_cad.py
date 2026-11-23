"""Pytest suite for Brazo Robótico Arduino 5.0 (FABRI Creator) 3D CAD: Dual Servos, 25T Horns & Gripper."""

from __future__ import annotations

import io
import xml.etree.ElementTree as ET
import zipfile

import trimesh

from backend.app.cad.hardware_parts import (
    build_gripper_3d,
    build_servo_3d,
    build_servo_horn_25t,
)
from backend.app.schemas import JointActuatorSpec, MechatronicsConfig, ModularArmRequest


def test_servo_horn_and_gripper_solids_valid():
    """TD-8120MG servo, 25T aluminum horn, and 2-finger gripper must all build valid 3D solids."""
    servo = build_servo_3d("servo_td8120mg")
    assert servo.is_valid
    assert servo.volume > 20_000.0

    horn = build_servo_horn_25t()
    assert horn.is_valid
    assert horn.volume > 800.0

    gripper = build_gripper_3d()
    assert gripper.is_valid
    assert gripper.volume > 10_000.0


def test_fabri_arm_5_full_assembly_dual_servos_horns_gripper_and_pca9685():
    """Full Brazo Arduino 5.0 assembly must include dual servos, red 25T horns, gripper, PCA9685 SVG, and exact BOM."""
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

    # 1. Verify GLB contains dual servo motors, red 25T horns, and 2-finger gripper
    r_glb = client.get(data["files"]["glb"])
    assert r_glb.status_code == 200
    scene = trimesh.load(io.BytesIO(r_glb.content), file_type="glb", force="scene")
    nodes = set(scene.graph.nodes_geometry)

    for expected_node in (
        "motor_j1_shoulder",
        "motor_j2_elbow",
        "motor_j3_wrist",
        "horn_25t_j1",
        "horn_25t_j2",
        "horn_25t_j3",
        "gripper_assembly",
    ):
        assert expected_node in nodes, f"Missing Brazo 5.0 node '{expected_node}' in GLB: {sorted(nodes)}"
        assert expected_node in data["explode_vectors"], f"Missing explode vector for '{expected_node}'"

    # Dual parallel J1 servo mesh must have roughly 2x the volume of a single servo
    j1_mesh = scene.geometry["motor_j1_shoulder"]
    j3_mesh = scene.geometry["motor_j3_wrist"]
    assert j1_mesh.volume > j3_mesh.volume * 1.7, (
        f"Expected dual J1 servo volume ({j1_mesh.volume:.0f}) > 1.7x single J3 servo ({j3_mesh.volume:.0f})"
    )

    # 2. Verify PCA9685 + Arduino UNO SVG schematic
    r_svg = client.get(data["files"]["circuit_svg"])
    assert r_svg.status_code == 200
    assert "PCA9685" in r_svg.text
    assert "arduino" in r_svg.text.lower()
    svg_root = ET.fromstring(r_svg.text)
    assert svg_root.tag.endswith("svg")

    # 3. Verify Master BOM includes 8× TD-8120MG servos & 8× 25T aluminum horns
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
