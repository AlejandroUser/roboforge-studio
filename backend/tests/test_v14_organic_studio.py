"""Pytest suite for RoboForge v14.0 Unified Organic Sculpting Studio (all 6 domains)."""

from __future__ import annotations

from pathlib import Path

import pytest

from backend.app.cad.organic_studio import generate_organic_studio_model
from backend.app.physics.mechanics import load_materials
from backend.app.physics.profiles import load_printers


@pytest.mark.parametrize(
    "domain,params",
    [
        ("link", {"length_mm": 180.0, "payload_kg": 2.0, "volume_fraction": 0.38}),
        ("mount", {"actuator_type": "nema17", "torque_nm": 2.5}),
        ("transmission", {"teeth": 28, "module_mm": 2.0, "face_width_mm": 12.0, "torque_nm": 4.0}),
        ("full_arm", {"upper_length_mm": 160.0, "forearm_length_mm": 140.0, "payload_kg": 1.2}),
        ("propeller", {"target_thrust_g": 600.0, "diameter_inch": 5.0, "rpm": 18000.0, "priority": "max_efficiency"}),
        ("fpv_drone", {"prop_inch": 5.0, "wheelbase_mm": 215.0, "rpm": 20000.0, "priority": "ultra_quiet"}),
    ],
)
def test_organic_studio_all_six_domains_pass_dfm_and_export_glb(
    domain: str, params: dict, tmp_path: Path
):
    """Every domain in Organic Sculpting Studio must export a valid GLB, STL/3MF, and pass DFM."""
    mats = load_materials()
    printers = load_printers()
    mat = mats.get("pa12_cf") or mats["petg_cf"]
    printer = printers.get("bambu_x1c") or next(iter(printers.values()))

    out_dir = tmp_path / domain
    res = generate_organic_studio_model(
        domain=domain,
        params=params,
        material=mat,
        printer=printer,
        out_dir=out_dir,
        design_hash=f"deadbeef000000{len(domain):02d}",
    )

    assert res["domain"] == domain
    assert res["dfm_audit"]["status"] == "pass", f"{domain} failed DFM: {res['dfm_audit']}"
    assert res["glb_url"].endswith(".glb")
    glb_name = Path(res["glb_url"]).name
    assert (out_dir / glb_name).exists() and (out_dir / glb_name).stat().st_size > 2000
    assert res["stl_url"] and (out_dir / Path(res["stl_url"]).name).exists()
    assert res["threemf_url"] and (out_dir / Path(res["threemf_url"]).name).exists()
