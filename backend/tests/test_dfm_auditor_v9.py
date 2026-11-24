"""Unit tests for Goldfish Pre-Print DFM Auditor and Mars-Mini v9.0 CAD fixes."""

from __future__ import annotations

import tempfile
from pathlib import Path

import trimesh
from build123d import Box, Compound, Location, export_stl

from app.cad.arm_assembly import _build_base_pedestal, build_modular_arm_assembly
from app.cad.dfm_auditor import audit_assembly_dfm
from app.cad.hardware_parts import build_dual_servo_clevis, build_gripper_parts
from app.cad.print_prep import build_calibration_fit_test_coupon
from app.schemas import ModularArmRequest


def _to_trimesh(solid) -> trimesh.Trimesh:
    with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as tmp:
        p = Path(tmp.name)
    try:
        export_stl(solid, str(p), tolerance=0.10, angular_tolerance=0.18)
        m = trimesh.load(str(p), force="mesh")
        return m.dump(concatenate=True) if isinstance(m, trimesh.Scene) else m
    finally:
        if p.exists():
            p.unlink()


def test_dfm_auditor_pass_clean_assembly() -> None:
    part_a = Box(40.0, 30.0, 12.0)
    part_b = Location((60.0, 0.0, 0.0)) * Box(30.0, 25.0, 10.0)
    meshes = {"part_a": _to_trimesh(part_a), "part_b": _to_trimesh(part_b)}
    printer = {"name": "Bambu Lab X1C", "bed_x_mm": 256.0, "bed_y_mm": 256.0, "bed_z_mm": 256.0}

    report = audit_assembly_dfm(
        printable_parts={"part_a": part_a, "part_b": part_b},
        placed_meshes=meshes,
        printer_profile=printer,
        bom_part_names=["part_a", "part_b"],
    )
    assert report.status == "pass", report
    assert len(report.checks) == 5
    assert all(c.status == "pass" for c in report.checks)


def test_dfm_auditor_detects_islands_oversize_and_collision() -> None:
    island_part = Compound(children=[Box(20.0, 20.0, 10.0), Location((50.0, 0.0, 0.0)) * Box(20.0, 20.0, 10.0)])
    huge_part = Box(320.0, 120.0, 15.0)
    overlap_a = Box(30.0, 30.0, 30.0)
    overlap_b = Location((2.0, 0.0, 0.0)) * Box(30.0, 30.0, 30.0)
    printer = {"name": "Elegoo Mars 5 Ultra", "bed_x_mm": 153.36, "bed_y_mm": 77.76, "bed_z_mm": 165.0}

    report = audit_assembly_dfm(
        printable_parts={"island_part": island_part, "huge_part": huge_part},
        placed_meshes={"overlap_a": _to_trimesh(overlap_a), "overlap_b": _to_trimesh(overlap_b)},
        printer_profile=printer,
        bom_part_names=["only_one"],
    )
    assert report.status == "fail"
    by_id = {c.id: c for c in report.checks}
    assert by_id["floating_islands"].status == "fail"
    assert by_id["fits_bed"].status == "fail"
    assert by_id["aabb_collisions"].status == "fail"
    assert by_id["bom_parity"].status == "warn"


def test_mars_mini_cad_fixes_v9() -> None:
    # 1. Base pedestal compact MSLA has real 608 bearing seat (Ø22.2 x 7.2 mm) & stays Ø64 mm watertight
    base = _build_base_pedestal(62.0, 0.9, compact_msla=True)
    m_base = _to_trimesh(base)
    assert m_base.is_watertight
    assert len(m_base.split(only_watertight=False)) == 1
    assert max(m_base.extents[0], m_base.extents[1]) <= 64.5

    # 2. Compact gripper base has >= 28 x 18 x 20 mm actuator block with MG90S pocket
    g_base, g_left, g_right = build_gripper_parts(compact=True)
    m_gb = _to_trimesh(g_base)
    assert m_gb.is_watertight
    assert m_gb.extents[0] >= 28.0 and m_gb.extents[1] >= 20.0 and m_gb.extents[2] >= 20.0

    # 3. Clevis supports M3 (3.25 mm) bolts for MSLA
    clevis_m3 = build_dual_servo_clevis(width_mm=44.0, depth_mm=38.0, height_mm=40.0, bolt_d_mm=3.25)
    m_cl = _to_trimesh(clevis_m3)
    assert m_cl.is_watertight and len(m_cl.split(only_watertight=False)) == 1

    # 4. Calibration coupon keeps 608 bearing bore + M3 hex pocket in MSLA mode
    class _MslaReq:
        printer_id = "elegoo_mars_5_ultra"
        material_id = "resin_abs_like"
        mechatronics = None

    coupon = build_calibration_fit_test_coupon(_MslaReq())
    m_c = _to_trimesh(coupon)
    assert m_c.is_watertight and len(m_c.split(only_watertight=False)) == 1

    # 5. parallelogram_rod is included in printable_parts when four_bar_parallelogram=True
    req = ModularArmRequest(
        preset="mars_mini",
        kinematics_type="articulated",
        dof=4,
        payload_kg=0.25,
        base_height_mm=62.0,
        material_id="resin_abs_like",
        printer_id="elegoo_mars_5_ultra",
        four_bar_parallelogram=True,
    )
    with tempfile.TemporaryDirectory() as td:
        res = build_modular_arm_assembly(req, {"color_hex": "#3eb8cc"}, Path(td), "test_mars9")
        assert "parallelogram_rod" in res["printable_parts"]
