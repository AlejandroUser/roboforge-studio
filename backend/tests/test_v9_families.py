"""Tests for RoboForge v9.0 Multi-Project Families (FPV Drone, Rover 4WD, Camera Gimbal)."""

from __future__ import annotations

import tempfile
from pathlib import Path

import pytest
import trimesh

from app.families import compute_project_hash, get_family_plugin, list_family_infos
from app.schemas import ProjectGenerateRequest

BAMBU_X1C = {
    "name": "Bambu Lab X1C",
    "bed_x_mm": 256.0,
    "bed_y_mm": 256.0,
    "bed_z_mm": 256.0,
    "process_type": "fdm",
}
MARS_5_ULTRA = {
    "name": "Elegoo Mars 5 Ultra",
    "bed_x_mm": 153.36,
    "bed_y_mm": 77.76,
    "bed_z_mm": 165.0,
    "process_type": "msla_resin",
}
PETG_CF = {"id": "petg_cf", "name": "PETG-CF", "category": "fdm", "density_g_cm3": 1.29}
RESIN_ABS = {"id": "resin_abs_like", "name": "Resina ABS-Like", "category": "msla", "density_g_cm3": 1.18}


def test_family_registry_lists_three_families() -> None:
    infos = list_family_infos()
    ids = {f["id"] for f in infos}
    assert ids == {"fpv_drone", "rover_4wd", "camera_gimbal"}
    for info in infos:
        assert len(info["presets"]) >= 2


@pytest.mark.parametrize(
    "family_id,preset_id,printer,material,expect_dxf",
    [
        ("fpv_drone", "cinewhoop_3in", BAMBU_X1C, PETG_CF, True),
        ("fpv_drone", "freestyle_5in", BAMBU_X1C, PETG_CF, True),
        ("rover_4wd", "mini_rover_tt", BAMBU_X1C, PETG_CF, False),
        ("rover_4wd", "explorer_lidar", BAMBU_X1C, PETG_CF, False),
        ("camera_gimbal", "gimbal_2axis_mars5", MARS_5_ULTRA, RESIN_ABS, False),
        ("camera_gimbal", "gimbal_3axis_gopro", BAMBU_X1C, PETG_CF, False),
    ],
)
def test_generate_family_projects_pass_goldfish_dfm(
    family_id: str,
    preset_id: str,
    printer: dict,
    material: dict,
    expect_dxf: bool,
) -> None:
    plugin = get_family_plugin(family_id)
    assert plugin is not None

    req = ProjectGenerateRequest(
        preset_id=preset_id,
        material_id=material["id"],
        printer_id="elegoo_mars_5_ultra" if printer is MARS_5_ULTRA else "bambu_x1c",
    )
    design_hash = compute_project_hash(family_id, req)
    assert design_hash.startswith("prj_") and len(design_hash) == 20

    with tempfile.TemporaryDirectory() as td:
        out_dir = Path(td)
        res = plugin.generate(req, material, printer, out_dir, design_hash)

        assert res.family_id == family_id
        assert res.preset_id == preset_id
        assert res.design_hash == design_hash
        assert res.dfm_audit.status == "pass", f"{family_id}/{preset_id} DFM failed: {res.dfm_audit}"
        assert res.total_bom_cost_usd > 0
        assert len(res.bom) >= 5
        assert len(res.explode_vectors) >= 3

        for key in ("glb", "step", "stl_parts_zip", "zip", "print_guide_md"):
            assert key in res.files, f"{family_id}/{preset_id} missing {key}"
        if expect_dxf:
            assert "dxf" in res.files

        # Verify generated GLB loads as a valid multi-node Scene
        glb_path = out_dir / "arm_assembly.glb"
        assert glb_path.exists() and glb_path.stat().st_size > 1000
        scene = trimesh.load(str(glb_path), file_type="glb")
        assert isinstance(scene, trimesh.Scene)
        assert len(scene.geometry) >= 3

        # Family-specific physics sanity checks
        if family_id == "fpv_drone":
            assert res.metrics["twr"] > 2.0
            assert res.metrics["flight_time_min"] > 2.0
        elif family_id == "rover_4wd":
            assert res.metrics["max_climb_deg"] > 10.0
            assert res.metrics["total_torque_nm"] > 0.5
        elif family_id == "camera_gimbal":
            assert res.metrics["torque_margin"] > 2.0
