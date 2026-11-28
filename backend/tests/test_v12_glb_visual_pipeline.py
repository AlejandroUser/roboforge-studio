"""Sprint v12.0 Studio Visual GLB Pipeline Tests (Crease Normals, Physical Box UVs, glTF Extras & Cache)."""

from __future__ import annotations

import json
import struct
import tempfile
from pathlib import Path

import numpy as np
import trimesh
from build123d import Cylinder, export_stl
from fastapi.testclient import TestClient

from app.cad.exporter import (
    _STUDIO_MESH_CACHE,
    _stl_to_glb,
    compute_box_projected_uvs,
    part_to_studio_trimesh,
)
from app.families import compute_project_hash, get_family_plugin
from app.main import app
from app.schemas import ProjectGenerateRequest

client = TestClient(app)


def _parse_glb_gltf_json(glb_bytes: bytes) -> dict:
    assert glb_bytes[:4] == b"glTF", "Not a valid binary glTF (.glb) header"
    chunk_len = struct.unpack("<I", glb_bytes[12:16])[0]
    return json.loads(glb_bytes[20 : 20 + chunk_len].decode("utf-8"))


def test_part_to_studio_trimesh_uvs_normals_extras_and_cache() -> None:
    """Studio helper produces crease-split smooth normals, finite physical box UVs, glTF extras, and caches cleanly."""
    cyl = Cylinder(radius=24.0, height=50.0)
    key = "test_v12_cyl_24x50"
    _STUDIO_MESH_CACHE.pop(key, None)

    m1 = part_to_studio_trimesh(
        cyl,
        rgba=[180, 196, 214, 255],
        metallic=0.82,
        roughness=0.26,
        name="test_cyl",
        role="bearing",
        printable=False,
        material_hint="chrome_steel",
        cache_key=key,
    )
    assert key in _STUDIO_MESH_CACHE
    assert m1.visual.uv is not None
    uv = np.asarray(m1.visual.uv, dtype=np.float32)
    assert uv.shape == (len(m1.vertices), 2)
    assert np.isfinite(uv).all()
    assert float(np.ptp(uv)) > 0.2, "Expected physical mm/50 UV span > 0.2"
    assert m1.metadata["role"] == "bearing"
    assert m1.metadata["printable"] is False
    assert m1.metadata["material_hint"] == "chrome_steel"

    # Second call hits cache without temp STL I/O and returns identical geometry
    m2 = part_to_studio_trimesh(
        cyl,
        rgba=[180, 196, 214, 255],
        metallic=0.82,
        roughness=0.26,
        name="test_cyl",
        role="bearing",
        printable=False,
        material_hint="chrome_steel",
        cache_key=key,
    )
    assert len(m2.vertices) == len(m1.vertices)
    assert len(m2.faces) == len(m1.faces)
    assert m2.visual.uv is not None


def test_stl_to_glb_exports_uvs_normals_and_extras() -> None:
    """Single-part _stl_to_glb produces TEXCOORD_0, NORMAL, and semantic extras in glTF JSON."""
    cyl = Cylinder(radius=18.0, height=36.0)
    with tempfile.TemporaryDirectory() as td:
        stl_p = Path(td) / "part.stl"
        glb_p = Path(td) / "part.glb"
        export_stl(cyl, str(stl_p), tolerance=0.09, angular_tolerance=0.14)
        _stl_to_glb(stl_p, glb_p, "organic_ibeam")
        gltf = _parse_glb_gltf_json(glb_p.read_bytes())

    attrs = gltf["meshes"][0]["primitives"][0]["attributes"]
    assert "POSITION" in attrs
    assert "NORMAL" in attrs
    assert "TEXCOORD_0" in attrs
    extras = gltf["meshes"][0].get("extras", {})
    assert extras.get("role") == "structural"
    assert extras.get("printable") is True


def test_modular_arm_glb_has_uvs_normals_and_node_extras() -> None:
    """POST /api/arm/assemble exports a multi-node studio GLB with TEXCOORD_0, NORMAL, and semantic extras."""
    resp = client.post(
        "/api/arm/assemble",
        json={
            "preset": "parol6_desktop",
            "kinematics_type": "articulated",
            "dof": 6,
            "payload_kg": 1.0,
            "base_height_mm": 90.0,
            "material_id": "petg_cf",
            "printer_id": "bambu_x1c",
        },
    )
    assert resp.status_code == 200, resp.text
    glb_bytes = client.get(resp.json()["files"]["glb"]).content
    gltf = _parse_glb_gltf_json(glb_bytes)

    assert len(gltf["meshes"]) >= 10
    for mesh_def in gltf["meshes"]:
        attrs = mesh_def["primitives"][0]["attributes"]
        assert "POSITION" in attrs and "NORMAL" in attrs and "TEXCOORD_0" in attrs
        extras = mesh_def.get("extras", {})
        assert "role" in extras and "printable" in extras and "material_hint" in extras

    # Check node-level extras as well
    nodes_with_extras = [n for n in gltf["nodes"] if "extras" in n and "role" in n["extras"]]
    assert len(nodes_with_extras) >= 10


def test_family_project_glb_has_uvs_normals_and_extras() -> None:
    """Multi-project family GLB (fpv_drone) exports TEXCOORD_0, NORMAL, and printable/role extras."""
    plugin = get_family_plugin("fpv_drone")
    assert plugin is not None
    req = ProjectGenerateRequest(preset_id="cinewhoop_3in", material_id="petg_cf", printer_id="bambu_x1c")
    h = compute_project_hash("fpv_drone", req)
    mat = {"id": "petg_cf", "name": "PETG-CF", "category": "fdm", "density_g_cm3": 1.29}
    printer = {"name": "Bambu Lab X1C", "bed_x_mm": 256.0, "bed_y_mm": 256.0, "bed_z_mm": 256.0, "process_type": "fdm"}

    with tempfile.TemporaryDirectory() as td:
        out_dir = Path(td)
        res = plugin.generate(req, mat, printer, out_dir, h)
        assert res.dfm_audit.status == "pass"
        gltf = _parse_glb_gltf_json((out_dir / "arm_assembly.glb").read_bytes())

    assert len(gltf["meshes"]) >= 4
    for mesh_def in gltf["meshes"]:
        attrs = mesh_def["primitives"][0]["attributes"]
        assert "NORMAL" in attrs and "TEXCOORD_0" in attrs
        extras = mesh_def.get("extras", {})
        assert "role" in extras and "printable" in extras and "material_hint" in extras
