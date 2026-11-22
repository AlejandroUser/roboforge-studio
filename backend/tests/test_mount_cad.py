"""Pytest suite for Family 2: Mechatronic Mounts CAD, exports, and FastAPI endpoints."""

from __future__ import annotations

from pathlib import Path

import pytest

from backend.app.cad.exporter import (
    compute_mount_design_hash,
    export_mount_variants,
    load_cached_mount_metadata,
)
from backend.app.cad.mount_variants import generate_all_mount_variants
from backend.app.schemas import MountDesignRequest

EXPECTED_MOUNT_VARIANTS = ("mount_standard", "mount_gusseted", "mount_lightweight")


@pytest.mark.parametrize(
    "actuator_type,base_type,use_heat_inserts",
    [
        ("nema17", "profile_2020", False),
        ("nema23", "profile_2040", True),
        ("servo_mg996r", "flat_plate", True),
        ("profile_2020_gusset", "profile_2020", False),
    ],
)
def test_all_three_mount_variants_across_actuators(
    actuator_type: str,
    base_type: str,
    use_heat_inserts: bool,
):
    """All 3 mount variants must build valid solids with expected volume relationships."""
    req = MountDesignRequest(
        actuator_type=actuator_type,  # type: ignore[arg-type]
        base_type=base_type,  # type: ignore[arg-type]
        use_heat_inserts=use_heat_inserts,
        ventilation_slots=True,
    )
    variants = generate_all_mount_variants(req)

    assert set(variants.keys()) == set(EXPECTED_MOUNT_VARIANTS)
    for vid in EXPECTED_MOUNT_VARIANTS:
        entry = variants[vid]
        assert entry["is_valid"] is True, f"{actuator_type}/{vid} is not a valid solid"
        assert entry["volume_mm3"] > 100.0, f"{actuator_type}/{vid} has invalid volume"
        assert isinstance(entry["name"], str) and len(entry["name"]) > 0
        assert isinstance(entry["tagline"], str) and len(entry["tagline"]) > 0

    vol_std = variants["mount_standard"]["volume_mm3"]
    vol_gus = variants["mount_gusseted"]["volume_mm3"]
    vol_lgt = variants["mount_lightweight"]["volume_mm3"]

    assert vol_gus > vol_std * 1.05, f"Gussets did not add material for {actuator_type}"
    assert vol_lgt < vol_gus * 0.96, f"Lightweight pockets did not reduce mass for {actuator_type}"


def test_mount_export_step_stl_glb_and_cache(tmp_path: Path):
    """Exported mount .step, .stl, and .glb files must exist, be non-empty, and cache cleanly."""
    req = MountDesignRequest(actuator_type="nema17", base_type="profile_2020")
    design_hash = compute_mount_design_hash(req)
    assert len(design_hash) == 16
    assert design_hash == compute_mount_design_hash(
        MountDesignRequest(actuator_type="nema17", base_type="profile_2020")
    )

    variants = generate_all_mount_variants(req)
    exported = export_mount_variants(design_hash, variants, exports_root=tmp_path)

    for vid in EXPECTED_MOUNT_VARIANTS:
        for fmt in ("step", "stl", "glb"):
            fp = exported[vid][fmt]
            assert fp.exists(), f"Missing {fmt} for {vid}"
            assert fp.stat().st_size > 100, f"Too small {fmt} for {vid}"

    cached = load_cached_mount_metadata(design_hash, exports_root=tmp_path)
    assert cached is not None
    assert set(cached.keys()) == set(EXPECTED_MOUNT_VARIANTS)


def test_mount_api_presets_generate_and_fea():
    """Verify GET /api/mounts/presets, POST /api/mounts/generate, and POST /api/mounts/fea."""
    from fastapi.testclient import TestClient

    from backend.app.main import app

    client = TestClient(app)

    # 1. Presets
    r_presets = client.get("/api/mounts/presets")
    assert r_presets.status_code == 200, r_presets.text
    presets_data = r_presets.json()
    assert len(presets_data) >= 1

    # 2. Generate 3 variants
    req = MountDesignRequest(actuator_type="nema17", base_type="profile_2020")
    r_gen = client.post("/api/mounts/generate", json=req.model_dump())
    assert r_gen.status_code == 200, r_gen.text
    gen_data = r_gen.json()

    assert len(gen_data["variants"]) == 3
    assert gen_data["recommended_variant"] in EXPECTED_MOUNT_VARIANTS
    design_hash = gen_data["design_hash"]

    # Verify GLB serving
    glb_url = gen_data["variants"][1]["glb_url"]
    r_glb = client.get(glb_url)
    assert r_glb.status_code == 200 and len(r_glb.content) > 1000

    # 3. FEA on mount_gusseted
    fea_payload = {
        "design_hash": design_hash,
        "variant_id": "mount_gusseted",
        "request": req.model_dump(),
        "mesh_size_mm": 6.5,
        "load_direction": "y_inplane",
        "deform_scale": 10.0,
    }
    r_fea = client.post("/api/mounts/fea", json=fea_payload)
    assert r_fea.status_code == 200, r_fea.text
    fea_data = r_fea.json()

    assert fea_data["variant_id"] == "mount_gusseted"
    assert fea_data["node_count"] > 50
    assert fea_data["element_count"] > 100
    assert fea_data["max_von_mises_mpa"] > 0.0
    assert fea_data["max_displacement_mm"] > 0.0
    assert fea_data["fea_safety_factor"] > 0.0

    r_fea_glb = client.get(fea_data["fea_glb_url"])
    assert r_fea_glb.status_code == 200 and len(r_fea_glb.content) > 1000
