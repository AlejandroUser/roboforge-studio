"""
Field-Driven Implicit Modeling (SDF + TPMS Gyroid) engine inspired by LEAP 71 PicoGK & nTop.

Evaluates a 3D Signed Distance Field f(x, y, z) <= 0 of a tapered bionic robotic link,
blends solid mounting bosses + protective outer rim with a stress-graded TPMS Gyroid lattice:
    G(x, y, z) = sin(kx)*cos(ky) + sin(ky)*cos(kz) + sin(kz)*cos(kx)
where local wall thickness t(x) is graded continuously from dense near the high-stress root
to ultra-light open lattice near the low-stress tip, then extracts a watertight 3D mesh via
Marching Cubes and exports GLB, STL, and Smart 3MF.
"""

from __future__ import annotations

import hashlib
import json
import math
import time
from pathlib import Path
from typing import Any

import numpy as np
import trimesh
from skimage.measure import marching_cubes

from ..schemas import GyroidRequest, GyroidResult
from .threemf_exporter import compute_modifier_box_for_mesh, export_smart_3mf


def _smooth_min(a: np.ndarray, b: np.ndarray, k: float = 3.5) -> np.ndarray:
    """Polynomial smooth minimum for organic vascular/bionic SDF unions."""
    h = np.maximum(k - np.abs(a - b), 0.0) / k
    return np.minimum(a, b) - h * h * k * 0.25


def _compute_gyroid_hash(req: GyroidRequest) -> str:
    canonical = json.dumps(req.model_dump(mode="json"), sort_keys=True, separators=(",", ":"))
    return "gyr_" + hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:12]


def generate_field_driven_gyroid(
    req: GyroidRequest,
    material: dict[str, Any],
    out_dir: Path,
) -> GyroidResult:
    """Generate a field-driven TPMS Gyroid bionic robotic link via 3D SDF + Marching Cubes."""
    t0 = time.perf_counter()
    link = req.request
    design_hash = _compute_gyroid_hash(req)
    out_dir.mkdir(parents=True, exist_ok=True)

    L = float(link.length_mm)
    r_a = max(float(link.root_end.outer_diameter_mm) * 0.5, float(link.root_width_mm) * 0.5)
    r_b = max(float(link.tip_end.outer_diameter_mm) * 0.5, float(link.tip_width_mm) * 0.5)
    bore_a = (float(link.root_end.bore_diameter_mm) + float(link.printer_clearance_mm)) * 0.5
    bore_b = (float(link.tip_end.bore_diameter_mm) + float(link.printer_clearance_mm)) * 0.5
    thick = float(link.thickness_mm)
    half_z = thick * 0.5
    taper = float(link.organic_taper)

    # Adaptive voxel pitch so computation stays fast (< 1.2s) even on 400mm links
    pad = 3.5
    x_min, x_max = -r_a - pad, L + r_b + pad
    y_max = max(r_a, r_b) + pad
    y_min = -y_max
    z_min, z_max = -half_z - pad, half_z + pad

    dx = max(float(req.voxel_size_mm), (x_max - x_min) / 150.0)
    nx = int(math.ceil((x_max - x_min) / dx)) + 1
    ny = int(math.ceil((y_max - y_min) / dx)) + 1
    nz = int(math.ceil((z_max - z_min) / dx)) + 1

    xs = np.linspace(x_min, x_max, nx, dtype=np.float32)
    ys = np.linspace(y_min, y_max, ny, dtype=np.float32)
    zs = np.linspace(z_min, z_max, nz, dtype=np.float32)
    X, Y, Z = np.meshgrid(xs, ys, zs, indexing="ij")

    # 1. Exact 2D bionic tapered envelope SDF in XY
    u = np.clip(X / max(L, 1.0), 0.0, 1.0)
    lin_w = r_a * (1.0 - u) + r_b * u
    # Parabolic bionic waist pinch controlled by organic_taper
    waist = 1.0 - 0.18 * taper * np.sin(np.pi * u)
    half_w = np.maximum(lin_w * waist, min(r_a, r_b) * 0.55)

    # Rounded span body SDF (clamped to x in [0, L])
    dx_span = np.maximum(np.maximum(-X, X - L), 0.0)
    dy_span = np.abs(Y) - half_w
    sdf_span_2d = np.where(
        dx_span > 0.0,
        np.hypot(dx_span, np.maximum(dy_span, 0.0)),
        dy_span,
    )

    # Root and tip circular boss SDFs
    dist_a = np.hypot(X, Y)
    dist_b = np.hypot(X - L, Y)
    sdf_boss_a = dist_a - r_a
    sdf_boss_b = dist_b - r_b

    # Organic smooth union of bosses + tapered body
    sdf_2d = _smooth_min(_smooth_min(sdf_span_2d, sdf_boss_a, k=4.0), sdf_boss_b, k=4.0)

    # Extrude to 3D with rounded Z fillets (radius ~1.8 mm)
    dz = np.abs(Z) - (half_z - 1.5)
    sdf_outer = np.hypot(np.maximum(sdf_2d + 1.5, 0.0), np.maximum(dz, 0.0)) + np.minimum(
        np.maximum(sdf_2d + 1.5, dz), 0.0
    ) - 1.5

    # 2. Field-Driven TPMS Gyroid core inside the span (keeping solid mounting bosses & perimeter rim)
    shell = float(req.shell_thickness_mm)
    k_freq = (2.0 * np.pi) / max(float(req.cell_size_mm), 4.0)
    gyroid = (
        np.sin(k_freq * X) * np.cos(k_freq * Y)
        + np.sin(k_freq * Y) * np.cos(k_freq * Z)
        + np.sin(k_freq * Z) * np.cos(k_freq * X)
    )

    # Stress-graded density field: dense near root (u=0, max bending moment), open near tip (u=1)
    stress_field = np.clip((1.0 - u) ** 1.15, 0.0, 1.0)
    local_density = float(req.min_density) + (float(req.max_density) - float(req.min_density)) * stress_field
    # Gyroid sheet threshold (|G| <= t_g gives solid wall)
    t_g = 0.28 + 0.85 * local_density
    sdf_gyroid = (np.abs(gyroid) - t_g) / k_freq

    # Solid regions that must stay 100% dense: mounting bosses + outer perimeter frame
    keep_solid_bosses = np.minimum(dist_a - (r_a + 1.5), dist_b - (r_b + 1.5))
    keep_outer_rim = np.maximum(sdf_2d, -(sdf_2d + shell))
    solid_preserve = np.minimum(keep_solid_bosses, keep_outer_rim)

    # Inside the link: solid where (solid_preserve <= 0) OR (sdf_gyroid <= 0)
    sdf_lattice_link = np.maximum(sdf_outer, np.minimum(solid_preserve, sdf_gyroid))

    # 3. Subtract central mounting bores (always clean & open)
    sdf_bores = np.minimum(dist_a - bore_a, dist_b - bore_b)
    sdf_final = np.maximum(sdf_lattice_link, -sdf_bores)

    # Ensure strictly positive outer boundary for closed Marching Cubes surface
    sdf_final[0, :, :] = 5.0
    sdf_final[-1, :, :] = 5.0
    sdf_final[:, 0, :] = 5.0
    sdf_final[:, -1, :] = 5.0
    sdf_final[:, :, 0] = 5.0
    sdf_final[:, :, -1] = 5.0

    verts_idx, faces, _, _ = marching_cubes(sdf_final, level=0.0, spacing=(dx, dx, dx))
    verts = verts_idx + np.array([x_min, y_min, z_min], dtype=np.float64)

    mesh = trimesh.Trimesh(vertices=verts, faces=faces, process=True)
    mesh.fix_normals()
    if mesh.volume < 0:
        mesh.invert()

    # Export STL, PBR GLB (bionic iridescent emerald-teal), and Smart 3MF
    stl_path = out_dir / "gyroid_sdf.stl"
    glb_path = out_dir / "gyroid_sdf.glb"
    threemf_path = out_dir / "gyroid_sdf.3mf"

    mesh.export(str(stl_path), file_type="stl")

    # Apply PBR finish for GLB
    pbr_mat = trimesh.visual.material.PBRMaterial(
        name="gyroid_sdf_bionic",
        baseColorFactor=[34, 211, 168, 255],
        metallicFactor=0.45,
        roughnessFactor=0.35,
    )
    mesh.visual = trimesh.visual.TextureVisuals(material=pbr_mat)
    mesh.export(str(glb_path), file_type="glb")

    mod_center, mod_size = compute_modifier_box_for_mesh(stl_path, family="links")
    export_smart_3mf(
        stl_path=stl_path,
        output_3mf_path=threemf_path,
        part_name="RoboForge_Gyroid_SDF",
        modifier_box_center=mod_center,
        modifier_box_size=mod_size,
    )

    vol_cm3 = abs(float(mesh.volume)) / 1000.0
    density_g_cm3 = float(material.get("density_g_cm3", 1.29))
    mass_g = vol_cm3 * density_g_cm3
    solve_time_s = time.perf_counter() - t0

    notes = [
        f"Campo implícito SDF + TPMS Gyroid (celda {req.cell_size_mm:.1f} mm, piel {req.shell_thickness_mm:.1f} mm).",
        f"Gradiente guiado por esfuerzo: densidad {req.max_density*100:.0f}% en raíz → {req.min_density*100:.0f}% en punta.",
        f"Malla Marching Cubes ({nx}×{ny}×{nz} vóxeles, {len(faces):,} triángulos) lista para impresión 3D.",
    ]

    return GyroidResult(
        design_hash=design_hash,
        cell_size_mm=req.cell_size_mm,
        voxel_grid=[nx, ny, nz],
        triangle_count=int(len(faces)),
        volume_cm3=round(vol_cm3, 2),
        mass_g=round(mass_g, 1),
        solve_time_s=round(solve_time_s, 3),
        glb_url=f"/api/files/{design_hash}/gyroid_sdf.glb",
        stl_url=f"/api/files/{design_hash}/gyroid_sdf.stl",
        threemf_url=f"/api/files/{design_hash}/gyroid_sdf.3mf",
        notes=notes,
    )
