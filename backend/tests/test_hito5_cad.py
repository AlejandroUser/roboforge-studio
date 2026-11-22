"""Pytest suite for Hito 5: Full Robotic Arm Assembly (CAD + URDF + MuJoCo + BOM) & Copilot API."""

from __future__ import annotations

import xml.etree.ElementTree as ET

from backend.app.schemas import ArmAssemblyRequest


def test_arm_presets_generate_and_all_downloads():
    """Verify GET /api/arm/presets and POST /api/arm/generate with GLB, STEP, STL, URDF, MuJoCo, and BOM CSV."""
    from fastapi.testclient import TestClient

    from backend.app.main import app

    client = TestClient(app)

    # 1. Presets
    r_presets = client.get("/api/arm/presets")
    assert r_presets.status_code == 200, r_presets.text
    presets = r_presets.json()
    assert "presets" in presets
    assert "parol6_desktop" in presets["presets"]

    # 2. Generate full articulated arm assembly
    req = ArmAssemblyRequest(
        preset="parol6_desktop",
        dof=6,
        payload_kg=1.0,
        link_style="hollow_cobot",
    )
    r_gen = client.post("/api/arm/generate", json=req.model_dump())
    assert r_gen.status_code == 200, r_gen.text
    data = r_gen.json()

    assert data["dof"] == 6
    assert data["max_reach_mm"] > 400.0
    assert data["total_arm_mass_kg"] > 0.5
    assert data["printed_mass_g"] > 50.0
    assert len(data["tcp_position_mm"]) == 3
    assert len(data["joints"]) >= 3
    assert len(data["bom"]) >= 3

    # 3. Verify all 6 downloadable files
    for key in ("glb_url", "step_url", "stl_url", "urdf_url", "mujoco_url", "bom_csv_url"):
        url = data[key]
        resp = client.get(url)
        assert resp.status_code == 200, f"Failed downloading {key}: {url}"
        assert len(resp.content) > 100, f"Empty download for {key}"

    # 4. Validate URDF XML structure
    urdf_xml = client.get(data["urdf_url"]).text
    urdf_root = ET.fromstring(urdf_xml)
    assert urdf_root.tag == "robot"
    assert len(urdf_root.findall("link")) >= 4
    assert len(urdf_root.findall("joint")) >= 3

    # 5. Validate MuJoCo MJCF XML structure
    mjcf_xml = client.get(data["mujoco_url"]).text
    mjcf_root = ET.fromstring(mjcf_xml)
    assert mjcf_root.tag == "mujoco"
    assert mjcf_root.find("worldbody") is not None
    assert mjcf_root.find("actuator") is not None

    # 6. Validate BOM CSV content
    bom_csv = client.get(data["bom_csv_url"]).text
    lines = [ln for ln in bom_csv.splitlines() if ln.strip()]
    assert len(lines) >= 4
    assert "Categoria" in lines[0]


def test_copilot_closed_loop_optimize_api():
    """Verify POST /api/copilot/optimize parses Spanish prompt and converges on a valid LinkDesignRequest."""
    from fastapi.testclient import TestClient

    from backend.app.main import app

    client = TestClient(app)

    payload = {
        "prompt": "Brazo articulado de 240mm para 40N en PA12-CF con FS >= 2.5 y deflexión máxima 0.6mm",
    }
    resp = client.post("/api/copilot/optimize", json=payload)
    assert resp.status_code == 200, resp.text
    data = resp.json()

    assert isinstance(data["summary"], str) and len(data["summary"]) > 10
    assert len(data["parsed_constraints"]) >= 2
    assert len(data["iterations"]) >= 1
    opt = data["optimized_request"]
    assert abs(opt["length_mm"] - 240.0) < 1.0
    assert abs(opt["tip_load_n"] - 40.0) < 1.0
    assert opt["material_id"] == "pa12_cf"
