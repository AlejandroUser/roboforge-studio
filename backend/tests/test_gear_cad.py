"""Pytest suite for Family 3: Transmission (Involute Spur Gears & GT2 Pulleys)."""

from __future__ import annotations

from pathlib import Path

import pytest

from backend.app.cad.exporter import (
    compute_gear_design_hash,
    export_gear_variants,
    load_cached_gear_metadata,
)
from backend.app.cad.gear_variants import generate_all_gear_variants
from backend.app.schemas import GearDesignRequest

EXPECTED_GEAR_VARIANTS = ("gear_solid", "gear_webbed", "gear_spoked")


def test_spur_involute_three_variants_and_mass_ordering():
    """Involute spur gear must generate 3 valid solids with webbed and spoked lighter than solid."""
    req = GearDesignRequest(
        gear_kind="spur_involute",
        teeth=28,
        module_mm=2.0,
        face_width_mm=12.0,
        bore_diameter_mm=6.0,
        d_shaft_flat_mm=0.5,
        hub_diameter_mm=16.0,
        hub_height_mm=6.0,
    )
    variants = generate_all_gear_variants(req)

    assert set(variants.keys()) == set(EXPECTED_GEAR_VARIANTS)
    for vid in EXPECTED_GEAR_VARIANTS:
        entry = variants[vid]
        assert entry["is_valid"] is True, f"{vid} is not a valid solid"
        assert entry["volume_mm3"] > 100.0, f"{vid} has invalid volume"

    vol_solid = variants["gear_solid"]["volume_mm3"]
    vol_webbed = variants["gear_webbed"]["volume_mm3"]
    vol_spoked = variants["gear_spoked"]["volume_mm3"]

    assert vol_webbed < vol_solid * 0.90, f"gear_webbed ({vol_webbed}) not lighter than solid ({vol_solid})"
    assert vol_spoked < vol_solid * 0.85, f"gear_spoked ({vol_spoked}) not lighter than solid ({vol_solid})"


@pytest.mark.parametrize("teeth", [20, 40])
def test_gt2_pulley_variants_build_validly(teeth: int):
    """Both compact 20T and larger 40T GT2 pulleys must build validly across all 3 variants."""
    req = GearDesignRequest(
        gear_kind="gt2_pulley",
        teeth=teeth,
        module_mm=1.0,
        face_width_mm=10.0,
        bore_diameter_mm=5.0,
        d_shaft_flat_mm=0.5,
        hub_diameter_mm=12.0 if teeth == 20 else 16.0,
        hub_height_mm=5.0,
    )
    variants = generate_all_gear_variants(req)

    for vid in EXPECTED_GEAR_VARIANTS:
        assert variants[vid]["is_valid"] is True
        assert variants[vid]["volume_mm3"] > 50.0

    if teeth == 40:
        assert variants["gear_spoked"]["volume_mm3"] < variants["gear_solid"]["volume_mm3"] * 0.96


def test_gear_export_step_stl_glb_and_cache(tmp_path: Path):
    """Exported gear .step, .stl, and .glb files must exist, be non-empty, and cache cleanly."""
    req = GearDesignRequest(gear_kind="spur_involute", teeth=24, module_mm=2.0)
    design_hash = compute_gear_design_hash(req)
    assert len(design_hash) == 16
    assert design_hash == compute_gear_design_hash(
        GearDesignRequest(gear_kind="spur_involute", teeth=24, module_mm=2.0)
    )

    variants = generate_all_gear_variants(req)
    exported = export_gear_variants(design_hash, variants, exports_root=tmp_path)

    for vid in EXPECTED_GEAR_VARIANTS:
        for fmt in ("step", "stl", "glb"):
            fp = exported[vid][fmt]
            assert fp.exists(), f"Missing {fmt} for {vid}"
            assert fp.stat().st_size > 100, f"Too small {fmt} for {vid}"

    cached = load_cached_gear_metadata(design_hash, exports_root=tmp_path)
    assert cached is not None
    assert set(cached.keys()) == set(EXPECTED_GEAR_VARIANTS)


def test_gear_api_presets_generate_and_fea():
    """Verify GET /api/gears/presets, POST /api/gears/generate, and POST /api/gears/fea."""
    from fastapi.testclient import TestClient

    from backend.app.main import app

    client = TestClient(app)

    # 1. Presets
    r_presets = client.get("/api/gears/presets")
    assert r_presets.status_code == 200, r_presets.text
    assert "presets" in r_presets.json()

    # 2. Generate 3 variants
    req = GearDesignRequest(gear_kind="spur_involute", teeth=24, module_mm=2.0)
    r_gen = client.post("/api/gears/generate", json=req.model_dump())
    assert r_gen.status_code == 200, r_gen.text
    gen_data = r_gen.json()

    assert len(gen_data["variants"]) == 3
    assert gen_data["pitch_diameter_mm"] == pytest.approx(48.0, abs=0.1)
    assert gen_data["outer_diameter_mm"] == pytest.approx(52.0, abs=0.1)
    assert gen_data["recommended_variant"] in EXPECTED_GEAR_VARIANTS
    design_hash = gen_data["design_hash"]

    r_glb = client.get(gen_data["variants"][2]["glb_url"])
    assert r_glb.status_code == 200 and len(r_glb.content) > 1000

    # 3. FEA on gear_webbed
    fea_payload = {
        "design_hash": design_hash,
        "variant_id": "gear_webbed",
        "request": req.model_dump(),
        "mesh_size_mm": 5.5,
        "load_direction": "y_inplane",
        "deform_scale": 10.0,
    }
    r_fea = client.post("/api/gears/fea", json=fea_payload)
    assert r_fea.status_code == 200, r_fea.text
    fea_data = r_fea.json()

    assert fea_data["variant_id"] == "gear_webbed"
    assert fea_data["node_count"] > 50
    assert fea_data["element_count"] > 100
    assert fea_data["max_von_mises_mpa"] > 0.0
    assert fea_data["max_displacement_mm"] > 0.0
    assert fea_data["fea_safety_factor"] > 0.0

    r_fea_glb = client.get(fea_data["fea_glb_url"])
    assert r_fea_glb.status_code == 200 and len(r_fea_glb.content) > 1000
