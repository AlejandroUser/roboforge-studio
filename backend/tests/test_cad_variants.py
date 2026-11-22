"""Pytest suite for RoboForge Studio parametric CAD variants, STEP analyzer, and file exporters."""

from __future__ import annotations

from pathlib import Path

import pytest
from build123d import export_step

from backend.app.cad.exporter import compute_design_hash, export_variants, load_cached_metadata
from backend.app.cad.step_analyzer import analyze_step_file
from backend.app.cad.variants import _build_tapered_solid, generate_all_variants
from backend.app.schemas import EndInterfaceParams, LinkDesignRequest


EXPECTED_VARIANTS = (
    "tapered_solid",
    "organic_ibeam",
    "rounded_truss",
    "hollow_cobot",
    "organic_topopt",
)


def test_all_five_variants_build_validly_and_mass_ordering():
    """All 5 variants must build valid solids with positive volume and genuine mass differences."""
    req = LinkDesignRequest()
    variants = generate_all_variants(req)

    assert set(variants.keys()) == set(EXPECTED_VARIANTS)

    for vid in EXPECTED_VARIANTS:
        entry = variants[vid]
        assert entry["is_valid"] is True, f"{vid} is not a valid solid"
        assert entry["volume_mm3"] > 0.0, f"{vid} has non-positive volume"
        assert isinstance(entry["name"], str) and len(entry["name"]) > 0
        assert isinstance(entry["tagline"], str) and len(entry["tagline"]) > 0

    vol_solid = variants["tapered_solid"]["volume_mm3"]
    vol_ibeam = variants["organic_ibeam"]["volume_mm3"]
    vol_truss = variants["rounded_truss"]["volume_mm3"]
    vol_cobot = variants["hollow_cobot"]["volume_mm3"]
    vol_topopt = variants["organic_topopt"]["volume_mm3"]

    # Lightened variants must genuinely remove material compared to the solid baseline
    assert vol_ibeam < vol_solid * 0.92, f"organic_ibeam ({vol_ibeam}) not lighter than solid ({vol_solid})"
    assert vol_truss < vol_solid * 0.90, f"rounded_truss ({vol_truss}) not lighter than solid ({vol_solid})"
    assert vol_cobot < vol_solid * 0.95, f"hollow_cobot ({vol_cobot}) not lighter than solid ({vol_solid})"
    assert vol_topopt < vol_solid * 0.92, f"organic_topopt ({vol_topopt}) not lighter than solid ({vol_solid})"


def test_export_step_stl_glb_and_cache(tmp_path: Path):
    """Exported .step, .stl, and .glb files must exist, be non-empty, and cache deterministically."""
    req = LinkDesignRequest(length_mm=150.0, organic_taper=0.6)
    design_hash = compute_design_hash(req)
    assert len(design_hash) == 16
    assert design_hash == compute_design_hash(LinkDesignRequest(length_mm=150.0, organic_taper=0.6))

    variants = generate_all_variants(req)
    exported = export_variants(design_hash, variants, exports_root=tmp_path)

    for vid in EXPECTED_VARIANTS:
        paths = exported[vid]
        for fmt in ("step", "stl", "glb"):
            fp = paths[fmt]
            assert fp.exists(), f"Missing {fmt} for {vid}"
            assert fp.stat().st_size > 100, f"Empty/too small {fmt} for {vid}: {fp.stat().st_size} bytes"

    cached = load_cached_metadata(design_hash, exports_root=tmp_path)
    assert cached is not None
    assert set(cached.keys()) == set(EXPECTED_VARIANTS)


def test_step_analyzer_roundtrip(tmp_path: Path):
    """Export a known link to .STEP, analyze it, and verify envelope, volume, bores, and suggested request."""
    req = LinkDesignRequest(length_mm=180.0, thickness_mm=20.0)
    part = _build_tapered_solid(req)
    expected_vol_cm3 = part.volume / 1000.0

    step_path = tmp_path / "reference_arm.step"
    export_step(part, str(step_path))

    analysis = analyze_step_file(step_path, "reference_arm.step")

    assert analysis.filename == "reference_arm.step"
    assert abs(analysis.volume_cm3 - expected_vol_cm3) < 0.5
    assert analysis.bbox_length_mm > 200.0
    assert abs(analysis.bbox_thickness_mm - 20.0) < 0.5
    assert len(analysis.detected_holes) >= 2
    assert abs(analysis.suggested_request.length_mm - 180.0) < 1.5
    assert analysis.suggested_request.reference_volume_cm3 == pytest.approx(analysis.volume_cm3, abs=0.05)
    assert len(analysis.notes) >= 2


@pytest.mark.parametrize(
    "kwargs",
    [
        {"organic_taper": 0.0, "cable_channel": False},
        {
            "length_mm": 90.0,
            "root_width_mm": 26.0,
            "tip_width_mm": 18.0,
            "thickness_mm": 12.0,
            "root_end": EndInterfaceParams(
                preset="bearing_608",
                outer_diameter_mm=30.0,
                bore_diameter_mm=22.0,
                bolt_count=0,
                bolt_pcd_mm=25.0,
                bolt_size="M3",
                use_heat_inserts=False,
            ),
            "tip_end": EndInterfaceParams(
                preset="bearing_608",
                outer_diameter_mm=28.0,
                bore_diameter_mm=22.0,
                bolt_count=0,
                bolt_pcd_mm=24.0,
                bolt_size="M3",
                use_heat_inserts=False,
            ),
        },
        {
            "length_mm": 280.0,
            "root_width_mm": 64.0,
            "tip_width_mm": 36.0,
            "thickness_mm": 32.0,
            "organic_taper": 1.0,
            "topopt_volume_fraction": 0.45,
            "root_end": EndInterfaceParams(
                preset="nema23",
                outer_diameter_mm=68.0,
                bore_diameter_mm=38.1,
                bolt_count=4,
                bolt_pcd_mm=56.0,
                bolt_size="M5",
                use_heat_inserts=True,
            ),
        },
    ],
)
def test_edge_case_geometries_build_validly(kwargs):
    """Short links, straight tapers, and large NEMA23 links must all build validly across 5 variants."""
    req = LinkDesignRequest(**kwargs)
    variants = generate_all_variants(req)
    for vid in EXPECTED_VARIANTS:
        assert variants[vid]["is_valid"] is True
        assert variants[vid]["volume_mm3"] > 0.0
