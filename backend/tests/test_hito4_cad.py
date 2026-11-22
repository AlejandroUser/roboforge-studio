"""Pytest suite for Hito 4: Smart .3MF exporter, STEP Ghost Overlay GLB, and Printer/Material CRUD API."""

from __future__ import annotations

import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

from build123d import export_step

from backend.app.cad.exporter import compute_design_hash, export_variants
from backend.app.cad.threemf_exporter import compute_modifier_box_for_mesh, export_smart_3mf
from backend.app.cad.variants import _build_tapered_solid, generate_all_variants
from backend.app.schemas import LinkDesignRequest


def test_smart_3mf_opc_zip_structure(tmp_path: Path):
    """Smart .3MF must be a valid OPC ZIP with 2 distinct <object> meshes in 3D/3dmodel.model."""
    req = LinkDesignRequest(length_mm=140.0)
    design_hash = compute_design_hash(req)
    variants = generate_all_variants(req)
    exported = export_variants(design_hash, variants, exports_root=tmp_path)

    threemf_path = exported["tapered_solid"]["3mf"]
    assert threemf_path.exists() and threemf_path.stat().st_size > 500

    with zipfile.ZipFile(threemf_path, "r") as zf:
        names = set(zf.namelist())
        assert "[Content_Types].xml" in names
        assert "_rels/.rels" in names
        assert "3D/3dmodel.model" in names

        model_xml = zf.read("3D/3dmodel.model")
        root = ET.fromstring(model_xml)
        ns = {"m": "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"}
        objects = root.findall(".//m:resources/m:object", ns)
        assert len(objects) == 2, f"Expected 2 3MF objects (part + modifier), got {len(objects)}"
        assert objects[0].attrib.get("name", "").startswith("RoboForge_")
        assert "Modifier" in objects[1].attrib.get("name", "")
        for obj in objects:
            verts = obj.findall(".//m:vertices/m:vertex", ns)
            tris = obj.findall(".//m:triangles/m:triangle", ns)
            assert len(verts) >= 8
            assert len(tris) >= 12


def test_step_ghost_overlay_and_3mf_api_download(tmp_path: Path):
    """POST /api/analyze-step must return downloadable reference_glb_url, and /api/generate must serve .3mf."""
    from fastapi.testclient import TestClient

    from backend.app.main import app

    client = TestClient(app)

    # 1. Upload .STEP and verify reference_glb_url downloads cleanly
    step_path = tmp_path / "ghost_ref.step"
    export_step(_build_tapered_solid(LinkDesignRequest(length_mm=160.0)), str(step_path))
    with step_path.open("rb") as fh:
        r_step = client.post("/api/analyze-step", files={"file": ("ghost_ref.step", fh, "application/step")})
    assert r_step.status_code == 200, r_step.text
    step_data = r_step.json()
    ref_url = step_data.get("reference_glb_url")
    assert ref_url is not None and ref_url.startswith("/api/files/ref_") and ref_url.endswith("/reference.glb")

    r_ghost = client.get(ref_url)
    assert r_ghost.status_code == 200 and len(r_ghost.content) > 1000

    # 2. Generate and verify threemf_url downloads cleanly
    r_gen = client.post("/api/generate", json=LinkDesignRequest(length_mm=140.0).model_dump())
    assert r_gen.status_code == 200, r_gen.text
    first_var = r_gen.json()["variants"][0]
    threemf_url = first_var.get("threemf_url")
    assert threemf_url is not None and threemf_url.endswith(".3mf")

    r_3mf = client.get(threemf_url)
    assert r_3mf.status_code == 200 and len(r_3mf.content) > 500


def test_printers_and_custom_materials_crud_api():
    """Verify GET/POST/DELETE /api/printers and POST/DELETE /api/materials/custom."""
    from fastapi.testclient import TestClient

    from backend.app.main import app

    client = TestClient(app)

    # 1. Built-in printers exist
    r_list = client.get("/api/printers")
    assert r_list.status_code == 200, r_list.text
    printers = r_list.json()
    assert "bambu_x1c" in printers

    # 2. Create and delete a custom printer
    custom_printer = {
        "id": "test_custom_corexy",
        "name": "Test Custom CoreXY",
        "bed_x_mm": 350.0,
        "bed_y_mm": 350.0,
        "bed_z_mm": 350.0,
        "default_nozzle_mm": 0.6,
        "clearance_mm": 0.14,
        "max_bed_temp_c": 120.0,
        "enclosed": True,
        "notes": "Test printer",
    }
    try:
        r_create = client.post("/api/printers", json=custom_printer)
        assert r_create.status_code == 200, r_create.text
        assert r_create.json()["id"] == "test_custom_corexy"
        assert r_create.json()["is_custom"] is True
        assert "test_custom_corexy" in client.get("/api/printers").json()
    finally:
        r_del = client.delete("/api/printers/test_custom_corexy")
        assert r_del.status_code == 200

    # 3. Create and delete a custom material
    custom_mat = {
        "id": "test_peek_cf",
        "name": "Test PEEK-CF",
        "category": "fdm",
        "density_g_cm3": 1.38,
        "youngs_xy_mpa": 12000.0,
        "youngs_z_mpa": 6500.0,
        "yield_xy_mpa": 115.0,
        "yield_z_mpa": 62.0,
        "max_temp_c": 240.0,
        "cost_per_kg_usd": 380.0,
        "color_hex": "#f59e0b",
        "notes": "Test high-temp polymer",
    }
    try:
        r_mcreate = client.post("/api/materials/custom", json=custom_mat)
        assert r_mcreate.status_code == 200, r_mcreate.text
        assert "test_peek_cf" in client.get("/api/materials").json()
    finally:
        r_mdel = client.delete("/api/materials/custom/test_peek_cf")
        assert r_mdel.status_code == 200
