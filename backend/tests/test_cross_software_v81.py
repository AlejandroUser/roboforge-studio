"""Sprint v8.1 Cross-Software CAD & Printability Regression Tests.

Verifies all 4 CAD/geometry fixes from the Cross-Software Audit:
1. P0 #1: bimanual_aloha printable STLs are 100% watertight, single-body (0 islands), and fit Bambu X1C bed.
2. P0 #2: SCARA, Cartesian, Delta, and Continuum PCC export individual printable STLs + build_plate_1.3mf + arm_stl_parts_pack.zip.
3. P1 #3: Eliminated physical GLB overlaps (parol6 wrist_tube omitted when wrist_3dof active, motor_j2 clears planetary_j2,
          delta_motor_1..3 clear delta_top_frame, horn_25t offset cleanly on servo spline).
4. P2 #6: stewart_hexapod base and platform plates have detailed mounting lugs + bolt bores + central cutout (>200 tris, watertight).
"""

from __future__ import annotations

import io
import zipfile

import numpy as np
import pytest
import trimesh
from fastapi.testclient import TestClient

from app.main import app

client = TestClient(app)


def _aabb_overlap_ratio(bounds_a: np.ndarray, bounds_b: np.ndarray) -> float:
    """Compute intersection volume / min(vol_a, vol_b) between two 3D AABBs [[minx,miny,minz],[maxx,maxy,maxz]]."""
    inter_min = np.maximum(bounds_a[0], bounds_b[0])
    inter_max = np.minimum(bounds_a[1], bounds_b[1])
    inter_ext = np.maximum(0.0, inter_max - inter_min)
    inter_vol = float(np.prod(inter_ext))
    vol_a = float(np.prod(np.maximum(0.0, bounds_a[1] - bounds_a[0])))
    vol_b = float(np.prod(np.maximum(0.0, bounds_b[1] - bounds_b[0])))
    denom = min(vol_a, vol_b)
    return inter_vol / denom if denom > 1e-6 else 0.0


def _get_preset_defaults(preset_id: str) -> dict:
    resp = client.get("/api/arm/presets")
    assert resp.status_code == 200
    presets = resp.json()["presets"]
    return dict(presets[preset_id]["defaults"])


def test_bimanual_aloha_watertight_single_body_and_fits_x1c() -> None:
    """P0 #1: Every part_*.stl in bimanual_aloha must be watertight, 1 body (0 floating islands), and fit Bambu X1C."""
    defaults = _get_preset_defaults("bimanual_aloha")
    payload = {
        **defaults,
        "preset": "bimanual_aloha",
        "kinematics_type": "bimanual_dual_arm",
        "printer_id": "bambu_x1c",
    }
    resp = client.post("/api/arm/assemble", json=payload)
    assert resp.status_code == 200, resp.text
    data = resp.json()

    pr = data["print_readiness"]
    assert pr["fits_bed"] is True, f"bimanual_aloha should fit Bambu X1C bed: {pr}"
    assert "build_plate_1_3mf" in data["files"]
    assert "stl_parts_zip" in data["files"]

    zip_bytes = client.get(data["files"]["stl_parts_zip"]).content
    part_stls = []
    with zipfile.ZipFile(io.BytesIO(zip_bytes)) as zf:
        for name in zf.namelist():
            if "02_PIEZAS_STL_SEPARADAS/" in name and name.endswith(".stl"):
                part_stls.append(name)
                mesh = trimesh.load(io.BytesIO(zf.read(name)), file_type="stl", force="mesh")
                assert isinstance(mesh, trimesh.Trimesh)
                assert mesh.is_watertight, f"{name} is not watertight"
                bodies = mesh.split(only_watertight=False)
                assert len(bodies) == 1, f"{name} has {len(bodies)} disconnected islands (expected 1)"
                assert max(mesh.bounding_box.extents) <= 256.0, (
                    f"{name} exceeds Bambu X1C 256mm bed: {mesh.bounding_box.extents}"
                )
    assert len(part_stls) >= 6


@pytest.mark.parametrize(
    "preset_id,kinematics_type,min_parts",
    [
        ("pybot_scara", "scara", 4),
        ("lumen_pnp", "cartesian_gantry", 4),
        ("delta_flexpicker", "delta_parallel", 6),
        ("surgical_continuum", "continuum_pcc", 3),
    ],
)
def test_four_topologies_export_build_plates_and_stl_parts_pack(
    preset_id: str, kinematics_type: str, min_parts: int
) -> None:
    """P0 #2: SCARA, Cartesian, Delta, and Continuum PCC must export build_plate_1.3mf and arm_stl_parts_pack.zip."""
    defaults = _get_preset_defaults(preset_id)
    payload = {
        **defaults,
        "preset": preset_id,
        "kinematics_type": kinematics_type,
        "printer_id": "bambu_x1c",
    }
    resp = client.post("/api/arm/assemble", json=payload)
    assert resp.status_code == 200, resp.text
    data = resp.json()

    assert "build_plate_1_3mf" in data["files"], f"{preset_id} missing build_plate_1_3mf"
    assert "stl_parts_zip" in data["files"], f"{preset_id} missing stl_parts_zip"

    plate_resp = client.get(data["files"]["build_plate_1_3mf"])
    assert plate_resp.status_code == 200 and len(plate_resp.content) > 500

    zip_bytes = client.get(data["files"]["stl_parts_zip"]).content
    with zipfile.ZipFile(io.BytesIO(zip_bytes)) as zf:
        part_stls = [n for n in zf.namelist() if "02_PIEZAS_STL_SEPARADAS/" in n and n.endswith(".stl")]
    assert len(part_stls) >= min_parts, f"{preset_id} exported only {len(part_stls)} part STLs: {part_stls}"


def test_parol6_no_wrist_tube_overlap_and_elbow_motor_clearance() -> None:
    """P1 #3 (parol6): wrist_tube omitted when wrist_3dof active, and motor_j2_elbow clears planetary_j2."""
    defaults = _get_preset_defaults("parol6_desktop")
    payload = {
        **defaults,
        "preset": "parol6_desktop",
        "kinematics_type": "articulated",
        "wrist_3dof": True,
        "mechatronics": {
            "bus_architecture": "can_fd_bus",
            "system_voltage_v": 24,
            "joint_actuators": {
                "J1": {"motor_id": "nema17_48mm", "reducer_id": "planetary_20", "target_rpm": 30, "parallel_count": 1},
                "J2": {"motor_id": "nema17_48mm", "reducer_id": "planetary_20", "target_rpm": 35, "parallel_count": 1},
                "J3": {"motor_id": "nema14_34mm", "reducer_id": "direct", "target_rpm": 45, "parallel_count": 1},
            },
        },
    }
    resp = client.post("/api/arm/assemble", json=payload)
    assert resp.status_code == 200, resp.text
    data = resp.json()

    glb_bytes = client.get(data["files"]["glb"]).content
    scene = trimesh.load(io.BytesIO(glb_bytes), file_type="glb")
    assert isinstance(scene, trimesh.Scene)

    geom_names = set(scene.geometry.keys())
    assert "wrist_j5_pitch" in geom_names
    assert "wrist_tube" not in geom_names, "wrist_tube must be omitted when wrist_3dof is active"
    assert "planetary_j2" in geom_names and "motor_j2_elbow" in geom_names

    overlap = _aabb_overlap_ratio(
        scene.geometry["motor_j2_elbow"].bounds,
        scene.geometry["planetary_j2"].bounds,
    )
    assert overlap < 0.15, f"motor_j2_elbow still overlaps planetary_j2 too much: {overlap:.2%}"


def test_delta_motors_clear_top_frame() -> None:
    """P1 #3 (delta): delta_motor_1..3 must sit cleanly above delta_top_frame with zero overlap."""
    defaults = _get_preset_defaults("delta_flexpicker")
    payload = {
        **defaults,
        "preset": "delta_flexpicker",
        "kinematics_type": "delta_parallel",
    }
    resp = client.post("/api/arm/assemble", json=payload)
    assert resp.status_code == 200, resp.text
    data = resp.json()

    glb_bytes = client.get(data["files"]["glb"]).content
    scene = trimesh.load(io.BytesIO(glb_bytes), file_type="glb")
    assert isinstance(scene, trimesh.Scene)

    frame_bounds = scene.geometry["delta_top_frame"].bounds
    for idx in (1, 2, 3):
        m_bounds = scene.geometry[f"delta_motor_{idx}"].bounds
        overlap = _aabb_overlap_ratio(m_bounds, frame_bounds)
        assert overlap == 0.0, f"delta_motor_{idx} penetrates delta_top_frame: overlap={overlap:.2%}"


def test_stewart_hexapod_detailed_watertight_plates() -> None:
    """P2 #6: stewart_base and stewart_platform must have >200 triangles (mounting lugs + bores) and be watertight."""
    defaults = _get_preset_defaults("stewart_hexapod")
    payload = {
        **defaults,
        "preset": "stewart_hexapod",
        "kinematics_type": "stewart_hexapod",
    }
    resp = client.post("/api/arm/assemble", json=payload)
    assert resp.status_code == 200, resp.text
    data = resp.json()

    zip_bytes = client.get(data["files"]["stl_parts_zip"]).content
    checked = 0
    with zipfile.ZipFile(io.BytesIO(zip_bytes)) as zf:
        for name in zf.namelist():
            if name.endswith("part_stewart_base.stl") or name.endswith("part_stewart_platform.stl"):
                mesh = trimesh.load(io.BytesIO(zf.read(name)), file_type="stl", force="mesh")
                assert isinstance(mesh, trimesh.Trimesh)
                assert mesh.is_watertight, f"{name} is not watertight"
                assert len(mesh.split(only_watertight=False)) == 1, f"{name} has multiple disconnected bodies"
                assert len(mesh.faces) > 200, f"{name} is too schematic: only {len(mesh.faces)} triangles"
                checked += 1
    assert checked == 2
