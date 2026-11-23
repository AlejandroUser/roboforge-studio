"""Pytest suite for RoboForge Studio Pro v4.0: 3D Hardware CAD, Mechatronics API & Kit ZIP v4.0."""

from __future__ import annotations

import io
import xml.etree.ElementTree as ET
import zipfile

import pytest
import trimesh

from backend.app.cad.hardware_parts import (
    build_ball_bearing,
    build_controller_box,
    build_iso4762_screw,
    build_motor_3d,
)
from backend.app.schemas import JointActuatorSpec, MechatronicsConfig, ModularArmRequest


V4_HARDWARE_NODES = {
    "motor_j1_shoulder",
    "motor_j2_elbow",
    "motor_j3_wrist",
    "bearing_j1",
    "bearing_j2",
    "bearing_j3",
    "fasteners_kit",
    "controller_box",
}


@pytest.mark.parametrize("metric", ["M3", "M4", "M5"])
def test_iso4762_screws_build_validly(metric: str):
    screw = build_iso4762_screw(metric, 14.0)
    assert screw.is_valid
    assert screw.volume > 50.0


@pytest.mark.parametrize("code", ["608", "6002", "6808"])
def test_ball_bearings_build_validly(code: str):
    bearing = build_ball_bearing(code)
    assert bearing.is_valid
    assert bearing.volume > 500.0


@pytest.mark.parametrize(
    "motor_id",
    [
        "nema17_48mm",
        "nema17_mks_servo42d",
        "nema23_76mm",
        "nema23_mks_servo57d",
        "bldc_qdd_5010",
        "bldc_qdd_8308",
    ],
)
def test_motors_3d_and_controller_box_build_validly(motor_id: str):
    motor = build_motor_3d(motor_id)
    assert motor.is_valid
    assert motor.volume > 10_000.0

    box = build_controller_box()
    assert box.is_valid
    assert box.volume > 50_000.0


def test_v4_mechatronics_catalog_and_arm_assemble_zip():
    """Verify GET /api/mechatronics/catalog and POST /api/arm/assemble v4.0 GLB nodes, SVG, YAML, Master BOM & ZIP."""
    from fastapi.testclient import TestClient

    from backend.app.main import app

    client = TestClient(app)

    # 1. Catalog endpoint
    r_cat = client.get("/api/mechatronics/catalog")
    assert r_cat.status_code == 200, r_cat.text
    cat = r_cat.json()
    assert len(cat["actuators"]) >= 6
    assert len(cat["reducers"]) >= 7
    assert len(cat["architectures"]) >= 2

    # 2. Modular Arm assemble with custom v4.0 MechatronicsConfig
    req = ModularArmRequest(
        dof=6,
        payload_kg=1.5,
        material_id="petg_cf",
        auto_fix=True,
        mechatronics=MechatronicsConfig(
            bus_architecture="can_fd_bus",
            system_voltage_v=24.0,
            joint_actuators={
                "J1": JointActuatorSpec(motor_id="bldc_qdd_8308", reducer_id="planetary_10", target_rpm=35.0),
                "J2": JointActuatorSpec(motor_id="nema17_mks_servo42d", reducer_id="planetary_10", target_rpm=40.0),
                "J3": JointActuatorSpec(motor_id="bldc_qdd_5010", reducer_id="direct", target_rpm=50.0),
            },
        ),
    )
    r_asm = client.post("/api/arm/assemble", json=req.model_dump())
    assert r_asm.status_code == 200, r_asm.text
    data = r_asm.json()

    # Check Mechatronics payload
    mech = data["mechatronics"]
    assert mech is not None
    assert mech["bus_architecture"] == "can_fd_bus"
    assert len(mech["joint_reports"]) >= 3
    assert mech["power_budget"]["recommended_psu"]
    assert len(mech["master_bom"]) >= 4

    # Check v4.0 explode_vectors
    for node_name in V4_HARDWARE_NODES:
        assert node_name in data["explode_vectors"], f"Missing explode vector for {node_name}"

    # Check GLB contains all v4.0 hardware semantic nodes
    r_glb = client.get(data["files"]["glb"])
    assert r_glb.status_code == 200
    scene = trimesh.load(io.BytesIO(r_glb.content), file_type="glb", force="scene")
    scene_nodes = set(scene.graph.nodes_geometry)
    missing = V4_HARDWARE_NODES - scene_nodes
    assert not missing, f"Missing v4.0 hardware nodes in GLB: {missing}"

    # Check SVG schematic, WireViz YAML, and Master BOM CSV downloads
    r_svg = client.get(data["files"]["circuit_svg"])
    assert r_svg.status_code == 200 and "svg" in r_svg.headers.get("content-type", "")
    svg_root = ET.fromstring(r_svg.text)
    assert svg_root.tag.endswith("svg")

    r_yml = client.get(data["files"]["harness_yml"])
    assert r_yml.status_code == 200 and len(r_yml.text) > 80

    r_mbom = client.get(data["files"]["master_bom_csv"])
    assert r_mbom.status_code == 200 and len(r_mbom.text.splitlines()) >= 5

    # Check ZIP includes all v4.0 files
    r_zip = client.get(data["files"]["zip"])
    assert r_zip.status_code == 200
    with zipfile.ZipFile(io.BytesIO(r_zip.content), "r") as zf:
        znames = set(zf.namelist())
        for must_have in (
            "arm_circuit_schematic.svg",
            "arm_harness.yml",
            "arm_master_bom.csv",
            "arm_assembly.step",
            "arm_assembly.urdf",
            "arm_mujoco.xml",
        ):
            assert must_have in znames, f"Missing {must_have} in v4.0 ZIP"
