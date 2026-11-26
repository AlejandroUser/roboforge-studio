"""Pytest suite for v11.0 BEMT Aerodynamic Solver, Parametric Propeller CAD (Conventional & Toroidal),
and FPV Drone / Standalone Propeller Family Integration."""

from __future__ import annotations

from pathlib import Path

import pytest
import trimesh

from backend.app.cad.propeller_gen import generate_propeller_part
from backend.app.families import FAMILY_REGISTRY, get_family_plugin
from backend.app.physics.mechanics import load_materials
from backend.app.physics.propeller_bemt import (
    AIRFOIL_LIBRARY,
    sample_airfoil_coords,
    solve_propeller_bemt,
)
from backend.app.schemas import ProjectGenerateRequest


def _default_printer() -> dict:
    return {
        "id": "bambu_x1c",
        "name": "Bambu Lab X1C (256×256×256)",
        "process": "fdm",
        "bed_x_mm": 256.0,
        "bed_y_mm": 256.0,
        "bed_z_mm": 256.0,
        "nozzle_mm": 0.4,
    }


def test_bemt_aerodynamic_scaling_laws_and_airfoils():
    """Verify BEMT follows T ~ RPM^2, P ~ RPM^3, and works across all embedded airfoils."""
    r_low = solve_propeller_bemt(5.0, 4.3, 3, rpm=12000.0, airfoil_id="naca4412")
    r_high = solve_propeller_bemt(5.0, 4.3, 3, rpm=24000.0, airfoil_id="naca4412")

    # Doubling RPM -> ~4x thrust, ~8x power
    t_ratio = r_high.thrust_n / r_low.thrust_n
    p_ratio = r_high.power_w / r_low.power_w
    assert 3.7 <= t_ratio <= 4.3, f"Expected T~RPM^2 (~4.0), got {t_ratio:.2f}"
    assert 7.4 <= p_ratio <= 8.6, f"Expected P~RPM^3 (~8.0), got {p_ratio:.2f}"

    # All 3 airfoils valid and produce realistic Figure of Merit
    for af_id in AIRFOIL_LIBRARY:
        coords = sample_airfoil_coords(af_id)
        assert len(coords) >= 20
        res = solve_propeller_bemt(7.0, 3.5, 2, rpm=11000.0, airfoil_id=af_id)
        assert res.thrust_g > 200.0
        assert 0.25 <= res.figure_of_merit <= 0.90
        assert res.efficiency_g_per_w > 3.0


def test_bemt_toroidal_acoustic_and_structural_warnings():
    """Toroidal mode reduces dBA & root stress; extreme RPM triggers Mach/FS warnings."""
    conv = solve_propeller_bemt(5.0, 4.3, 3, rpm=18000.0, prop_style="conventional")
    toro = solve_propeller_bemt(5.0, 4.3, 3, rpm=18000.0, prop_style="toroidal")
    assert toro.estimated_noise_dba < conv.estimated_noise_dba - 5.0
    assert toro.root_safety_factor > conv.root_safety_factor

    extreme = solve_propeller_bemt(7.0, 4.5, 3, rpm=32000.0, material_yield_z_mpa=25.0)
    assert extreme.tip_mach > 0.72
    assert any("Mach" in w for w in extreme.warnings)
    assert any("raíz" in w or "FS" in w for w in extreme.warnings)


def test_propeller_cad_conventional_ccw_cw_and_toroidal():
    """Conventional CCW/CW and toroidal B-Rep solids must have positive volume and accurate diameter."""
    p_ccw = generate_propeller_part(5.0, 4.3, 3, "naca4412", "conventional", "ccw")
    p_cw = generate_propeller_part(5.0, 4.3, 3, "naca4412", "conventional", "cw")
    p_toro = generate_propeller_part(5.0, 4.0, 3, "clark_y", "toroidal", "ccw")

    assert p_ccw.volume > 1500.0
    assert abs(p_ccw.volume - p_cw.volume) / p_ccw.volume < 0.03
    assert p_toro.volume > 1800.0

    # Tip radius check (~5 inch = 63.5 mm radius; 3-blade XY bbox is sqrt(3)*R in Y)
    bb = p_ccw.bounding_box()
    tip_r = max(abs(bb.max.X), abs(bb.min.X), abs(bb.max.Y), abs(bb.min.Y))
    assert 60.0 <= tip_r <= 65.0, f"Unexpected 5in prop tip radius: {tip_r:.1f} mm"


def test_propeller_standalone_and_fpv_drone_integration(tmp_path: Path):
    """PropellerPlugin is registered and FpvDronePlugin mounts live BEMT 3D propellers."""
    assert "propeller" in FAMILY_REGISTRY
    mats = load_materials()
    mat = mats["petg_cf"]
    printer = _default_printer()

    # 1. Standalone Toroidal Propeller family
    prop_plugin = get_family_plugin("propeller")
    assert prop_plugin is not None
    req_prop = ProjectGenerateRequest(
        family_id="propeller",
        preset_id="toroidal_5040_quiet",
        material_id="petg_cf",
    )
    res_prop = prop_plugin.generate(req_prop, mat, printer, tmp_path / "prop_toro", "test_toro_hash")
    assert res_prop.metrics["prop_style"] == "toroidal"
    assert res_prop.metrics["thrust_g"] > 350.0
    assert res_prop.metrics["cad_volume_mm3"] > 1800.0
    assert res_prop.dfm_audit.status == "pass", f"Toroidal DFM audit failed: {res_prop.dfm_audit}"

    # 2. FPV Drone with mounted BEMT propellers
    fpv_plugin = get_family_plugin("fpv_drone")
    assert fpv_plugin is not None
    req_fpv = ProjectGenerateRequest(
        family_id="fpv_drone",
        preset_id="freestyle_5in",
        material_id="petg_cf",
    )
    res_fpv = fpv_plugin.generate(req_fpv, mat, printer, tmp_path / "fpv_5in", "test_fpv_hash")
    assert res_fpv.metrics["bemt_thrust_per_motor_g"] > 500.0
    assert res_fpv.metrics["twr"] > 2.5
    assert res_fpv.dfm_audit.status == "pass", f"FPV DFM audit failed: {res_fpv.dfm_audit}"
    glb_path = tmp_path / "fpv_5in" / "arm_assembly.glb"
    assert glb_path.exists()
    scene = trimesh.load(str(glb_path), force="scene")
    nodes = set(scene.graph.nodes_geometry)
    assert "propellers_ccw" in nodes and "propellers_cw" in nodes
