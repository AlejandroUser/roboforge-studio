"""Sprint v13.0 Phase 3 & 4 Family Tests (Rover Rocker-Bogie/Ackerman + Gimbal CoM Balancing + SDK)."""

from __future__ import annotations

import tempfile
from pathlib import Path

import pytest

from app.families import compute_project_hash, get_family_plugin
from app.schemas import ProjectGenerateRequest

BAMBU_X1C = {"name": "Bambu Lab X1C", "bed_x_mm": 256.0, "bed_y_mm": 256.0, "bed_z_mm": 256.0, "process_type": "fdm"}
MARS_5_ULTRA = {"name": "Elegoo Mars 5 Ultra", "bed_x_mm": 153.36, "bed_y_mm": 77.76, "bed_z_mm": 165.0, "process_type": "msla_resin"}
PETG_CF = {"id": "petg_cf", "name": "PETG-CF", "category": "fdm", "density_g_cm3": 1.29}
RESIN_ABS = {"id": "resin_abs_like", "name": "Resina ABS-Like", "category": "msla", "density_g_cm3": 1.18}


@pytest.mark.parametrize("suspension_mode,expected_node", [
    ("rigid", None),
    ("rocker_bogie", "suspension_rocker_bogie"),
    ("ackerman", "steering_ackerman"),
])
def test_rover_4wd_suspension_modes_and_slope_physics_pass_dfm(
    suspension_mode: str,
    expected_node: str | None,
) -> None:
    """Rover 4WD generates real Rocker-Bogie & Ackerman 3D mechanisms + slope physics and passes Goldfish DFM 5/5."""
    plugin = get_family_plugin("rover_4wd")
    assert plugin is not None
    req = ProjectGenerateRequest(
        preset_id="explorer_lidar",
        material_id="petg_cf",
        printer_id="bambu_x1c",
        params={"suspension_mode": suspension_mode},
    )
    h = compute_project_hash("rover_4wd", req)

    with tempfile.TemporaryDirectory() as td:
        res = plugin.generate(req, PETG_CF, BAMBU_X1C, Path(td), h)
        assert res.dfm_audit.status == "pass", f"Rover ({suspension_mode}) DFM failed: {res.dfm_audit}"
        assert res.metrics["suspension_mode"] == suspension_mode
        assert res.metrics["max_slope_deg"] > 10.0
        assert res.metrics["climb_torque_nm"] > 0.1
        if expected_node:
            assert expected_node in res.explode_vectors


@pytest.mark.parametrize("preset_id,printer,material", [
    ("gimbal_2axis_mars5", MARS_5_ULTRA, RESIN_ABS),
    ("gimbal_3axis_gopro", BAMBU_X1C, PETG_CF),
])
def test_camera_gimbal_com_balancing_passes_dfm(preset_id: str, printer: dict, material: dict) -> None:
    """Camera Gimbal computes Pitch/Roll CoM offsets + counterweight mass and passes Goldfish DFM 5/5."""
    plugin = get_family_plugin("camera_gimbal")
    assert plugin is not None
    req = ProjectGenerateRequest(
        preset_id=preset_id,
        material_id=material["id"],
        printer_id="elegoo_mars_5_ultra" if printer is MARS_5_ULTRA else "bambu_x1c",
    )
    h = compute_project_hash("camera_gimbal", req)

    with tempfile.TemporaryDirectory() as td:
        res = plugin.generate(req, material, printer, Path(td), h)
        assert res.dfm_audit.status == "pass", f"Gimbal ({preset_id}) DFM failed: {res.dfm_audit}"
        assert res.metrics["pitch_com_offset_mm"] > 0.0
        assert res.metrics["roll_com_offset_mm"] > 0.0
        assert res.metrics["recommended_counterweight_g"] > 0.0
        assert res.metrics["idle_torque_reduction_pct"] >= 60.0
        assert res.metrics["com_balanced"] is True
        assert any("CoM" in n for n in res.notes)


def test_plugin_sdk_docs_exists() -> None:
    sdk_path = Path(__file__).resolve().parents[2] / "docs" / "PLUGIN_SDK.md"
    assert sdk_path.exists()
    text = sdk_path.read_text(encoding="utf-8")
    assert "ProjectFamilyPlugin" in text
    assert "FAMILY_REGISTRY" in text
    assert "Goldfish DFM" in text
