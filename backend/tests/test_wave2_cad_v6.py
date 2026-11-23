"""Pytest suite for RoboForge Studio Pro v6.0 Multi-Topology CAD:
Cartesian Gantry X-Y-Z, Parallel Delta 4-DOF, Surgical Continuum PCC, Capstan/Tendons,
4-Bar Parallelogram, Bevel Differential Wrist, Vacuum Nozzle, and <100 mm Micro-Arm Fix.
"""

from __future__ import annotations

import io
import xml.etree.ElementTree as ET

import trimesh

from backend.app.schemas import (
    LinkDesignRequest,
    ModularArmRequest,
    ModularArmSlots,
)


def _assemble_and_get_nodes(client, req: ModularArmRequest) -> tuple[dict, set[str], str]:
    resp = client.post("/api/arm/assemble", json=req.model_dump())
    assert resp.status_code == 200, resp.text
    data = resp.json()
    r_glb = client.get(data["files"]["glb"])
    assert r_glb.status_code == 200 and len(r_glb.content) > 2000
    scene = trimesh.load(io.BytesIO(r_glb.content), file_type="glb", force="scene")
    nodes = set(scene.graph.nodes_geometry)
    r_urdf = client.get(data["files"]["urdf"])
    assert r_urdf.status_code == 200
    return data, nodes, r_urdf.text


def test_v6_cartesian_gantry_cad_and_3_prismatic_urdf():
    from fastapi.testclient import TestClient

    from backend.app.main import app

    client = TestClient(app)
    req = ModularArmRequest(
        dof=4,
        kinematics_type="cartesian_gantry",
        z_stroke_mm=80.0,
        vacuum_eoat=True,
        payload_kg=0.35,
        auto_fix=True,
    )
    data, nodes, urdf_txt = _assemble_and_get_nodes(client, req)
    for n in (
        "gantry_frame_2020",
        "gantry_y_bridge",
        "gantry_x_carriage",
        "gantry_z_head",
        "vacuum_nozzle",
        "motor_x",
        "motor_y",
        "motor_z",
        "controller_box",
    ):
        assert n in nodes, f"Missing cartesian node '{n}' in {sorted(nodes)}"
        assert n in data["explode_vectors"]

    urdf_root = ET.fromstring(urdf_txt)
    prismatic = [j for j in urdf_root.findall("joint") if j.attrib.get("type") == "prismatic"]
    assert len(prismatic) == 3, f"Expected 3 prismatic joints in Cartesian URDF, got {len(prismatic)}"


def test_v6_delta_parallel_cad_and_urdf():
    from fastapi.testclient import TestClient

    from backend.app.main import app

    client = TestClient(app)
    req = ModularArmRequest(
        dof=4,
        kinematics_type="delta_parallel",
        vacuum_eoat=True,
        payload_kg=1.0,
        auto_fix=True,
    )
    data, nodes, urdf_txt = _assemble_and_get_nodes(client, req)
    for n in (
        "delta_top_frame",
        "delta_motor_1",
        "delta_motor_2",
        "delta_motor_3",
        "delta_bicep_1",
        "delta_bicep_2",
        "delta_bicep_3",
        "delta_forearms_1",
        "delta_forearms_2",
        "delta_forearms_3",
        "delta_platform",
        "delta_z_shaft",
        "vacuum_nozzle",
    ):
        assert n in nodes, f"Missing delta node '{n}' in {sorted(nodes)}"
        assert n in data["explode_vectors"]

    assert ET.fromstring(urdf_txt).tag == "robot"


def test_v6_continuum_pcc_and_micro_arm_90mm_no_500():
    from fastapi.testclient import TestClient

    from backend.app.main import app

    client = TestClient(app)
    # 1. Surgical Continuum PCC with 90 mm upper_arm (verifies <100 mm bug fix!)
    req_pcc = ModularArmRequest(
        dof=4,
        kinematics_type="continuum_pcc",
        payload_kg=0.1,
        base_height_mm=60.0,
        wrist_mm=40.0,
        slots=ModularArmSlots(
            upper_arm=LinkDesignRequest(length_mm=90.0, thickness_mm=12.0),
            forearm=LinkDesignRequest(length_mm=80.0, thickness_mm=10.0),
        ),
        auto_fix=True,
    )
    data, nodes, urdf_txt = _assemble_and_get_nodes(client, req_pcc)
    for n in (
        "continuum_actuator_base",
        "continuum_backbone",
        "continuum_spacer_disks",
        "continuum_tendons",
        "micro_forceps",
    ):
        assert n in nodes, f"Missing continuum node '{n}' in {sorted(nodes)}"
        assert n in data["explode_vectors"]
    assert ET.fromstring(urdf_txt).tag == "robot"

    # 2. Articulated micro-arm 90 mm must also return HTTP 200 with zero crash!
    req_micro = ModularArmRequest(
        dof=5,
        kinematics_type="articulated",
        payload_kg=0.25,
        slots=ModularArmSlots(
            upper_arm=LinkDesignRequest(length_mm=90.0, thickness_mm=14.0),
            forearm=LinkDesignRequest(length_mm=80.0, thickness_mm=12.0),
        ),
        auto_fix=True,
    )
    resp_micro = client.post("/api/arm/assemble", json=req_micro.model_dump())
    assert resp_micro.status_code == 200, resp_micro.text


def test_v6_capstan_four_bar_bevel_and_vacuum_articulated():
    from fastapi.testclient import TestClient

    from backend.app.main import app

    client = TestClient(app)
    req = ModularArmRequest(
        dof=6,
        kinematics_type="articulated",
        remote_base_motors=True,
        four_bar_parallelogram=True,
        bevel_differential_wrist=True,
        vacuum_eoat=True,
        payload_kg=1.5,
        auto_fix=True,
    )
    data, nodes, _ = _assemble_and_get_nodes(client, req)
    for n in (
        "capstan_drum_j1",
        "capstan_drum_j2",
        "tendon_cables",
        "parallelogram_rod",
        "bevel_wrist_diff",
        "vacuum_nozzle",
    ):
        assert n in nodes, f"Missing v6.0 articulated feature node '{n}' in {sorted(nodes)}"
        assert n in data["explode_vectors"]
