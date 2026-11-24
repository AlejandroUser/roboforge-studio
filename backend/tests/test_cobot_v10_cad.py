"""Sprint v10.0 Cobot 7-DOF (Franka Emika Panda) CAD & Auto-Dovetail Split Tests."""

from __future__ import annotations

import io
import tempfile
import zipfile
from pathlib import Path

import trimesh
from build123d import Box, export_stl
from fastapi.testclient import TestClient

from app.cad.hardware_parts import (
    build_em_brake_disc,
    build_iso9409_flange,
    build_torque_sensor_ring,
)
from app.cad.print_prep import _split_mesh_dovetail_x, _split_solid_dovetail_x
from app.main import app

client = TestClient(app)


def _solid_to_mesh(solid) -> trimesh.Trimesh:
    with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as tmp:
        p = Path(tmp.name)
    try:
        export_stl(solid, str(p), tolerance=0.10, angular_tolerance=0.18)
        m = trimesh.load(str(p), force="mesh")
        return m.dump(concatenate=True) if isinstance(m, trimesh.Scene) else m
    finally:
        if p.exists():
            p.unlink()


def test_cobot_hardware_primitives_watertight() -> None:
    """Torque sensor ring, 24V EM brake disc, and ISO 9409-1-50-4-M6 flange must be watertight single bodies."""
    for name, solid in (
        ("torque_sensor_ring", build_torque_sensor_ring()),
        ("em_brake_disc", build_em_brake_disc()),
        ("iso9409_flange", build_iso9409_flange()),
    ):
        mesh = _solid_to_mesh(solid)
        assert mesh.is_watertight, f"{name} is not watertight"
        assert len(mesh.split(only_watertight=False)) == 1, f"{name} has multiple bodies"
        assert mesh.volume > 1000.0


def test_dovetail_split_solid_and_mesh_watertight() -> None:
    """Splitting a 380 mm oversized link produces two watertight halves that fit a 256 mm Bambu X1C bed."""
    long_beam = Box(380.0, 44.0, 22.0)

    # 1. Exact OpenCASCADE B-Rep dovetail split
    half_a, half_b = _split_solid_dovetail_x(long_beam, clearance_mm=0.20)
    ma, mb = _solid_to_mesh(half_a), _solid_to_mesh(half_b)
    assert ma.is_watertight and mb.is_watertight
    assert len(ma.split(only_watertight=False)) == 1
    assert len(mb.split(only_watertight=False)) == 1
    assert max(ma.extents) <= 220.0 and max(mb.extents) <= 220.0

    # 2. Direct Trimesh split helper
    raw_mesh = _solid_to_mesh(long_beam)
    t_a, t_b = _split_mesh_dovetail_x(raw_mesh, clearance_mm=0.20)
    assert t_a.is_watertight and t_b.is_watertight
    assert max(t_a.extents) <= 200.0 and max(t_b.extents) <= 200.0


def test_franka_panda_7dof_dh_offsets_cobot_nodes_and_auto_dovetail_split_on_x1c() -> None:
    """Franka Panda 7-DOF on Bambu X1C (256 mm) with auto_dovetail_split=True achieves fits_bed==True,
    watertight dovetail halves, DH lateral offset (82.5 mm), torque sensors, EM brakes, and ISO 9409 flange.
    """
    payload = {
        "preset": "franka_panda",
        "kinematics_type": "articulated",
        "dof": 7,
        "payload_kg": 3.0,
        "base_height_mm": 140.0,
        "wrist_mm": 107.0,
        "material_id": "al_7075_t6",
        "printer_id": "bambu_x1c",
        "dh_lateral_offsets_mm": [0.0, 0.0, 0.0, 82.5, -82.5, 0.0, 88.0],
        "joint_torque_sensors": True,
        "electromagnetic_brakes": True,
        "auto_dovetail_split": True,
        "slots": {
            "upper_arm_variant": "hollow_cobot",
            "forearm_variant": "hollow_cobot",
            "upper_arm": {"length_mm": 316.0, "root_width_mm": 54.0, "tip_width_mm": 44.0, "thickness_mm": 28.0},
            "forearm": {"length_mm": 384.0, "root_width_mm": 46.0, "tip_width_mm": 36.0, "thickness_mm": 24.0},
        },
    }
    resp = client.post("/api/arm/assemble", json=payload)
    assert resp.status_code == 200, resp.text
    data = resp.json()

    # 1. Auto-Dovetail Split achieves fits_bed == True on Bambu X1C 256 mm!
    pr = data["print_readiness"]
    assert pr["fits_bed"] is True, f"Expected fits_bed=True after dovetail split, got {pr}"
    assert pr["split_parts_count"] >= 2, f"Expected >=2 split parts, got {pr['split_parts_count']}"

    # 2. Verify every split *_A.stl / *_B.stl in arm_stl_parts_pack.zip is watertight and fits 256 mm bed
    zip_bytes = client.get(data["files"]["stl_parts_zip"]).content
    split_stls = []
    with zipfile.ZipFile(io.BytesIO(zip_bytes)) as zf:
        for name in zf.namelist():
            if name.endswith("_A.stl") or name.endswith("_B.stl"):
                split_stls.append(name)
                m = trimesh.load(io.BytesIO(zf.read(name)), file_type="stl", force="mesh")
                assert isinstance(m, trimesh.Trimesh)
                assert m.is_watertight, f"Split half {name} is not watertight"
                assert max(m.extents) <= 256.0, f"Split half {name} exceeds 256mm bed: {m.extents}"
        guide_text = zf.read("03_PLATOS_3MF_SLICER/PRINT_GUIDE.md").decode("utf-8")
        assert "Cola de Milano" in guide_text
    assert len(split_stls) >= 4, f"Expected >=4 dovetail half STLs, got {split_stls}"

    # 3. Verify GLB contains all v10.0 cobot semantic nodes and real Panda DH lateral offset on forearm_link
    glb_bytes = client.get(data["files"]["glb"]).content
    scene = trimesh.load(io.BytesIO(glb_bytes), file_type="glb")
    assert isinstance(scene, trimesh.Scene)
    geom_names = set(scene.geometry.keys())
    for expected_node in (
        "torque_sensor_j1",
        "torque_sensor_j2",
        "torque_sensor_j3",
        "em_brake_j1",
        "em_brake_j2",
        "em_brake_j3",
        "iso9409_flange",
        "upper_arm_roll_j3",
    ):
        assert expected_node in geom_names, f"Missing v10.0 cobot node {expected_node} in GLB"

    # Forearm centroid Y should reflect the real Panda a4=+82.5 mm lateral offset (clearly > 70 mm)
    forearm_y = float(scene.geometry["forearm_link"].centroid[1])
    assert forearm_y >= 70.0, f"Expected Panda DH lateral offset ~82.5 mm on forearm_link Y, got {forearm_y:.1f} mm"
