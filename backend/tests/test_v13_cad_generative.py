"""Sprint v13.0 Generative Physics CAD Tests (Organic SDF Link Builder + Evolved Propeller Loft)."""

from __future__ import annotations

import json
import math
import struct
import tempfile
from pathlib import Path

import numpy as np
import trimesh
from build123d import export_stl

from app.cad.organic_sdf_builder import build_organic_sdf_link, export_organic_sdf_link_files
from app.cad.propeller_gen import generate_propeller_part
from app.families import compute_project_hash, get_family_plugin
from app.schemas import ProjectGenerateRequest


def _synthetic_truss_rho_3d(nx: int = 30, ny: int = 10, nz: int = 10) -> np.ndarray:
    """Create a realistic synthetic 3D SIMP truss density field (top/bottom chords + diagonal struts)."""
    rho = np.zeros((nx, ny, nz), dtype=np.float32)
    # Solid root/tip caps + outer top/bottom chords
    rho[:3, :, :] = 1.0
    rho[-3:, :, :] = 1.0
    rho[:, :2, :] = 0.92
    rho[:, -2:, :] = 0.92
    # Diagonal X-bracing inside the span
    for i in range(nx):
        u = i / max(nx - 1, 1)
        y1 = int(round((ny - 1) * (0.2 + 0.6 * abs((u * 3.0) % 2.0 - 1.0))))
        y2 = (ny - 1) - y1
        for yc in (y1, y2):
            y_lo, y_hi = max(0, yc - 1), min(ny, yc + 2)
            rho[i, y_lo:y_hi, 2 : nz - 2] = 0.88
    return rho


def _sample_evolved_stations(radius_mm: float = 63.5) -> list[dict]:
    """Non-linear scimitar-swept blade stations resembling an NSGA-II Pareto winner."""
    stations = []
    for k in range(10):
        r_norm = 0.18 + (0.98 - 0.18) * (k / 9.0)
        r_mm = r_norm * radius_mm
        # Non-linear paddle chord peaking near r/R=0.58, tapering sharply at tip
        chord_mm = 9.0 + 9.5 * math.sin(math.pi * (r_norm ** 0.85))
        # Adkins-Liebeck style hyperbolic twist
        twist_deg = max(9.0, min(42.0, math.degrees(math.atan2(105.0, 2.0 * math.pi * r_mm)) + 3.0))
        thickness_mm = max(0.95, chord_mm * (0.14 - 0.05 * r_norm))
        # Progressive scimitar sweep + subtle forward rake
        sweep_mm = 7.5 * ((r_norm - 0.25) ** 2) if r_norm > 0.25 else 0.0
        rake_mm = 2.2 * (r_norm ** 1.5)
        stations.append(
            {
                "r_mm": round(r_mm, 3),
                "r_norm": round(r_norm, 4),
                "chord_mm": round(chord_mm, 3),
                "twist_deg": round(twist_deg, 3),
                "thickness_mm": round(thickness_mm, 3),
                "sweep_mm": round(sweep_mm, 3),
                "rake_mm": round(rake_mm, 3),
            }
        )
    return stations


def test_build_organic_sdf_link_watertight_single_body_and_exports() -> None:
    """Organic SDF builder turns a 3D SIMP density field + Preserve/Obstacle interfaces into a
    watertight, single-body mesh and exports .stl, .3mf, and studio .glb with UVs & extras.
    """
    rho = _synthetic_truss_rho_3d(30, 10, 10)
    mesh, stats = build_organic_sdf_link(
        rho_3d=rho,
        length_mm=180.0,
        root_width_mm=40.0,
        tip_width_mm=30.0,
        thickness_mm=20.0,
        root_od_mm=38.0,
        tip_od_mm=28.0,
        root_bore_mm=12.0,
        tip_bore_mm=10.0,
        root_bolt_pcd_mm=26.0,
        root_bolt_count=4,
        root_bolt_d_mm=3.2,
        tip_bolt_pcd_mm=20.0,
        tip_bolt_count=4,
        tip_bolt_d_mm=3.2,
    )

    assert stats["is_watertight"] is True
    assert stats["bodies"] == 1
    assert float(mesh.extents[0]) >= 180.0 + 0.35 * (38.0 + 28.0), f"Root-to-tip connectivity broken: extents={mesh.extents}"
    assert stats["volume_mm3"] > 10000.0
    assert stats["elapsed_s"] < 6.0

    with tempfile.TemporaryDirectory() as td:
        files = export_organic_sdf_link_files(
            mesh=mesh,
            material_rgba=[62, 184, 204, 255],
            metallic=0.55,
            roughness=0.32,
            out_dir=Path(td),
            stem="test_organic_link",
        )
        assert files["stl"].exists() and files["stl"].stat().st_size > 2000
        assert files["3mf"].exists() and files["3mf"].stat().st_size > 1000
        assert files["glb"].exists() and files["glb"].stat().st_size > 2000

        glb_bytes = files["glb"].read_bytes()
        chunk_len = struct.unpack("<I", glb_bytes[12:16])[0]
        gltf = json.loads(glb_bytes[20 : 20 + chunk_len].decode("utf-8"))
        attrs = gltf["meshes"][0]["primitives"][0]["attributes"]
        assert "POSITION" in attrs and "NORMAL" in attrs and "TEXCOORD_0" in attrs
        assert gltf["meshes"][0]["extras"]["printable"] is True


def test_evolved_propeller_custom_stations_ccw_cw_watertight() -> None:
    """generate_propeller_part lofts non-linear evolved custom_stations into watertight 1-body CCW and CW solids."""
    stations = _sample_evolved_stations(63.5)
    for direction in ("ccw", "cw"):
        solid = generate_propeller_part(
            diameter_inch=5.0,
            pitch_inch=4.3,
            blade_count=3,
            airfoil_id="eppler_e63",
            prop_style="conventional",
            direction=direction,
            shaft_diam_mm=5.0,
            custom_stations=stations,
        )
        with tempfile.TemporaryDirectory() as td:
            stl_p = Path(td) / f"prop_{direction}.stl"
            export_stl(solid, str(stl_p), tolerance=0.10, angular_tolerance=0.18)
            m = trimesh.load(str(stl_p), force="mesh")
            assert isinstance(m, trimesh.Trimesh)
            assert m.is_watertight, f"Evolved {direction} propeller is not watertight"
            assert len(m.split(only_watertight=False)) == 1, f"Evolved {direction} propeller has multiple bodies"


def test_propeller_family_evolved_stations_passes_goldfish_dfm() -> None:
    """PropellerPlugin accepts custom_stations + evolved_label and passes Goldfish DFM 5/5."""
    plugin = get_family_plugin("propeller")
    assert plugin is not None
    stations = _sample_evolved_stations(63.5)
    req = ProjectGenerateRequest(
        preset_id="fpv_5043_tri",
        material_id="petg_cf",
        printer_id="bambu_x1c",
        params={
            "diameter_inch": 5.0,
            "blade_count": 3,
            "airfoil_id": "eppler_e63",
            "custom_stations": stations,
            "evolved_label": "Ultra-Silenciosa (Pareto)",
        },
    )
    h = compute_project_hash("propeller", req)
    mat = {"id": "petg_cf", "name": "PETG-CF", "category": "fdm", "density_g_cm3": 1.29, "yield_z_mpa": 42.0}
    printer = {"name": "Bambu Lab X1C", "bed_x_mm": 256.0, "bed_y_mm": 256.0, "bed_z_mm": 256.0, "process_type": "fdm"}

    with tempfile.TemporaryDirectory() as td:
        res = plugin.generate(req, mat, printer, Path(td), h)
        assert res.dfm_audit.status == "pass", f"Evolved propeller DFM failed: {res.dfm_audit}"
        assert res.metrics["evolved"] is True
        assert "Ultra-Silenciosa (Pareto)" in res.name
        assert any("NSGA-II" in n for n in res.notes)
