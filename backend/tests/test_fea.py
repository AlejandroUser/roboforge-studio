from __future__ import annotations

from pathlib import Path

import pytest
from build123d import export_step

from app.cad.variants import generate_all_variants
from app.physics.fea_solver import run_step_fea
from app.physics.mechanics import load_materials
from app.schemas import LinkDesignRequest


def test_run_step_fea_on_tapered_solid_step(tmp_path: Path):
    req = LinkDesignRequest(
        material_id="petg_cf",
        length_mm=160.0,
        root_width_mm=36.0,
        tip_width_mm=24.0,
        thickness_mm=18.0,
        tip_load_n=25.0,
    )
    material = load_materials()[req.material_id]

    variants = generate_all_variants(req)
    solid_part = variants["tapered_solid"]["part"]
    step_path = tmp_path / "tapered_solid.step"
    export_step(solid_part, str(step_path))
    assert step_path.exists() and step_path.stat().st_size > 0

    glb_path = tmp_path / "tapered_solid_fea_y_inplane.glb"
    res = run_step_fea(
        step_path=step_path,
        output_glb_path=glb_path,
        req=req,
        design_hash="test_hash_fea",
        variant_id="tapered_solid",
        material=material,
        mesh_size_mm=8.0,
        load_direction="y_inplane",
    )

    assert res.design_hash == "test_hash_fea"
    assert res.variant_id == "tapered_solid"
    assert res.node_count > 50
    assert res.element_count > 100
    assert res.solve_time_s > 0.0
    assert res.max_von_mises_mpa >= res.p95_von_mises_mpa > 0.0
    assert res.max_displacement_mm > 0.0
    assert res.effective_yield_mpa == pytest.approx(material["yield_xy_mpa"])
    assert res.fea_safety_factor == pytest.approx(
        res.effective_yield_mpa / res.p95_von_mises_mpa, rel=0.05
    )
    assert res.fea_glb_url == "/api/files/test_hash_fea/tapered_solid_fea_y_inplane.glb"
    assert len(res.notes) >= 2

    # Verify GLB file exists, is non-empty, and includes COLOR_0 vertex color attribute
    assert glb_path.exists()
    glb_bytes = glb_path.read_bytes()
    assert len(glb_bytes) > 1000
    assert b"COLOR_0" in glb_bytes

    # Verify linear elasticity scaling when doubling tip load
    req_2x = req.model_copy(update={"tip_load_n": 50.0})
    glb_2x = tmp_path / "tapered_solid_fea_2x.glb"
    res_2x = run_step_fea(
        step_path=step_path,
        output_glb_path=glb_2x,
        req=req_2x,
        design_hash="test_hash_fea",
        variant_id="tapered_solid",
        material=material,
        mesh_size_mm=8.0,
        load_direction="y_inplane",
    )
    assert res_2x.max_displacement_mm / res.max_displacement_mm == pytest.approx(2.0, rel=0.05)
    assert res_2x.p95_von_mises_mpa / res.p95_von_mises_mpa == pytest.approx(2.0, rel=0.05)

    # Verify out-of-plane Z load direction also succeeds
    glb_z = tmp_path / "tapered_solid_fea_z_outofplane.glb"
    res_z = run_step_fea(
        step_path=step_path,
        output_glb_path=glb_z,
        req=req,
        design_hash="test_hash_fea",
        variant_id="tapered_solid",
        material=material,
        mesh_size_mm=8.0,
        load_direction="z_outofplane",
    )
    assert res_z.max_displacement_mm > 0.0
    assert res_z.p95_von_mises_mpa > 0.0
    assert glb_z.exists() and glb_z.stat().st_size > 1000
