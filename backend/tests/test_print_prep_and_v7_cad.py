"""Pytest suite for RoboForge Studio Pro v7.0:
Bimanual 14-DOF, Stewart Hexapod 6-UPS, Soft Pneumatic TPU Bellows,
and 3D Print-Readiness Pack (calibration_fit_test.stl, build_plate_1.3mf, PRINT_GUIDE.md, ZIP).
"""

from __future__ import annotations

import io
import xml.etree.ElementTree as ET
import zipfile

import trimesh

from backend.app.cad.print_prep import build_calibration_fit_test_coupon
from backend.app.schemas import ModularArmRequest


def test_calibration_coupon_solid_valid():
    coupon = build_calibration_fit_test_coupon(ModularArmRequest())
    assert coupon.is_valid
    assert coupon.volume > 5_000.0


def test_v7_bimanual_stewart_bellows_and_print_readiness_pack():
    """Verify Bimanual 14-DOF, Stewart Hexapod, Soft Pneumatic Bellows, and full 3D Print-Readiness Pack."""
    from fastapi.testclient import TestClient

    from backend.app.main import app

    client = TestClient(app)

    # 1. Bimanual 14-DOF + Print-Readiness Pack verification
    req_bi = ModularArmRequest(
        dof=14,
        kinematics_type="bimanual_dual_arm",
        payload_kg=0.8,
        material_id="petg_cf",
        printer_id="bambu_x1c",
        auto_fix=True,
    )
    r_bi = client.post("/api/arm/assemble", json=req_bi.model_dump())
    assert r_bi.status_code == 200, r_bi.text
    d_bi = r_bi.json()

    scene_bi = trimesh.load(io.BytesIO(client.get(d_bi["files"]["glb"]).content), file_type="glb", force="scene")
    nodes_bi = set(scene_bi.graph.nodes_geometry)
    for n in ("bimanual_torso", "left_upper_arm", "left_forearm", "left_gripper", "right_upper_arm", "right_forearm", "right_gripper"):
        assert n in nodes_bi, f"Missing bimanual node '{n}' in {sorted(nodes_bi)}"

    # Verify Print-Readiness Summary & Downloads
    pr = d_bi["print_readiness"]
    assert pr is not None
    assert pr["total_plates"] >= 1
    assert pr["total_filament_g"] > 10.0
    assert pr["estimated_print_hours"] > 0.2
    assert len(pr["plates"]) == pr["total_plates"]

    r_cal = client.get(d_bi["files"]["calibration_stl"])
    assert r_cal.status_code == 200 and len(r_cal.content) > 1000

    r_guide = client.get(d_bi["files"]["print_guide_md"])
    assert r_guide.status_code == 200 and "OrcaSlicer" in r_guide.text and "calibration_fit_test.stl" in r_guide.text

    r_plate1 = client.get(d_bi["files"]["build_plate_1_3mf"])
    assert r_plate1.status_code == 200 and len(r_plate1.content) > 1000
    with zipfile.ZipFile(io.BytesIO(r_plate1.content), "r") as zf_plate:
        assert "3D/3dmodel.model" in set(zf_plate.namelist())

    with zipfile.ZipFile(io.BytesIO(client.get(d_bi["files"]["zip"]).content), "r") as zf:
        znames = set(zf.namelist())
        assert "calibration_fit_test.stl" in znames
        assert "PRINT_GUIDE.md" in znames
        assert "build_plate_1.3mf" in znames

    # 2. Stewart Hexapod 6-UPS
    req_stewart = ModularArmRequest(
        dof=6,
        kinematics_type="stewart_hexapod",
        payload_kg=5.0,
        material_id="al_7075_t6",
        auto_fix=True,
    )
    r_st = client.post("/api/arm/assemble", json=req_stewart.model_dump())
    assert r_st.status_code == 200, r_st.text
    d_st = r_st.json()
    scene_st = trimesh.load(io.BytesIO(client.get(d_st["files"]["glb"]).content), file_type="glb", force="scene")
    nodes_st = set(scene_st.graph.nodes_geometry)
    for n in ("stewart_base", "stewart_platform", "stewart_leg_1", "stewart_leg_2", "stewart_leg_3", "stewart_leg_4", "stewart_leg_5", "stewart_leg_6"):
        assert n in nodes_st, f"Missing stewart node '{n}' in {sorted(nodes_st)}"
    urdf_st = ET.fromstring(client.get(d_st["files"]["urdf"]).text)
    assert len([j for j in urdf_st.findall("joint") if j.attrib.get("type") == "prismatic"]) == 6

    # 3. Soft Pneumatic TPU Bellows
    req_bel = ModularArmRequest(
        dof=6,
        kinematics_type="soft_pneumatic_bellows",
        payload_kg=0.35,
        material_id="tpu_95a",
        auto_fix=True,
    )
    r_bel = client.post("/api/arm/assemble", json=req_bel.model_dump())
    assert r_bel.status_code == 200, r_bel.text
    d_bel = r_bel.json()
    scene_bel = trimesh.load(io.BytesIO(client.get(d_bel["files"]["glb"]).content), file_type="glb", force="scene")
    nodes_bel = set(scene_bel.graph.nodes_geometry)
    for n in ("pneumatic_manifold", "bellows_chamber_1", "bellows_chamber_2", "bellows_chamber_3", "bellows_end_cap"):
        assert n in nodes_bel, f"Missing bellows node '{n}' in {sorted(nodes_bel)}"
