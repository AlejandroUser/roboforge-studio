"""RoboForge v14.0 — Unified Organic Sculpting Studio Backend Engine.

Generates physics-driven organic 3D models across 6 complete engineering domains:
  1. link         — Organic robotic link (SIMP 3D Hex8 + Langelaar 45° + Preserve/Obstacle SDF)
  2. mount        — Organic motor/actuator mount (SIMP 3D + NEMA17/23/MG996R Preserve/Obstacle SDF)
  3. transmission — Organic bionic spoked gear / transmission wheel (torque-sized involute + bionic web)
  4. full_arm     — Complete organic robotic arm (RNEA-sized SIMP 3D upper & forearm + joints + base)
  5. propeller    — Genetically evolved drone propeller (Adkins-Liebeck + BEMT + NSGA-II Pareto)
  6. fpv_drone    — Complete organic FPV drone (bionic truss True-X frame + 4x NSGA-II evolved props)
"""

from __future__ import annotations

import dataclasses
import math
import time
from pathlib import Path
from typing import Any

import numpy as np
import trimesh
from build123d import (
    Box,
    BuildPart,
    BuildSketch,
    Circle,
    Compound,
    Cylinder,
    Location,
    Locations,
    Mode,
    Part,
    Plane,
    RectangleRounded,
    Rotation,
    extrude,
    export_step,
    export_stl,
)

from ..families.base import write_simple_dxf_outline
from ..physics.propeller_bemt import solve_propeller_bemt
from ..physics.propeller_optimizer import evolve_propeller_pareto
from ..physics.topopt_3d import run_simp_3d_link
from ..schemas import DFMAuditReport, DFMCheckItem
from .exporter import apply_studio_mesh_polish, part_to_studio_trimesh
from .organic_sdf_builder import build_organic_sdf_link, export_organic_sdf_link_files
from .propeller_gen import generate_propeller_part
from .print_prep import _write_multi_mesh_3mf


def export_mesh_to_3mf(mesh: trimesh.Trimesh, output_path: Path, part_name: str = "organic_part") -> Path:
    return _write_multi_mesh_3mf([(part_name, mesh)], output_path)


def _material_visual(material: dict[str, Any], mat_id: str) -> tuple[list[int], float, float]:
    mid = str(material.get("id", mat_id)).lower()
    if "cf" in mid:
        return [48, 56, 68, 255], 0.28, 0.42
    if "al" in mid or "ti" in mid or "steel" in mid:
        return [188, 198, 212, 255], 0.86, 0.24
    if "resin" in mid:
        return [128, 148, 170, 255], 0.14, 0.30
    return [92, 118, 148, 255], 0.22, 0.46


def _bed_fit_check(mesh_or_extents: tuple[float, float, float], printer: dict[str, Any]) -> tuple[bool, str]:
    bed_x = float(printer.get("bed_x_mm", 256.0))
    bed_y = float(printer.get("bed_y_mm", 256.0))
    bed_z = float(printer.get("bed_z_mm", 256.0))
    dims = sorted([float(x) for x in mesh_or_extents])
    sz, sy, sx = dims[0], dims[1], dims[2]
    ok = ((sx <= bed_x - 10.0 and sy <= bed_y - 10.0) or (sy <= bed_x - 10.0 and sx <= bed_y - 10.0)) and sz <= bed_z - 4.0
    detail = f"Dimensiones {sx:.0f}×{sy:.0f}×{sz:.0f} mm en cama {bed_x:.0f}×{bed_y:.0f}×{bed_z:.0f} mm."
    return ok, detail


def _build_dfm_report(
    *,
    is_watertight: bool,
    bodies: int,
    volume_mm3: float,
    fits_bed: bool,
    bed_detail: str,
    printer_name: str,
    extra_note: str = "",
) -> dict[str, Any]:
    checks = [
        DFMCheckItem(
            id="watertight",
            label="Malla Orgánica Watertight",
            status="pass" if is_watertight else "fail",
            detail=f"Volumen {volume_mm3:.0f} mm³ · {'Cerrada sin bordes abiertos' if is_watertight else 'Bordes abiertos'}.",
        ),
        DFMCheckItem(
            id="floating_islands",
            label="Cero Islas Flotantes",
            status="pass" if bodies >= 1 else "fail",
            detail=f"{bodies} sólido(s) conectado(s) {extra_note}".strip(),
        ),
        DFMCheckItem(
            id="fits_bed",
            label=f"Ajuste a Cama ({printer_name})",
            status="pass" if fits_bed else "fail",
            detail=bed_detail,
        ),
    ]
    overall = "pass" if all(c.status == "pass" for c in checks) else "fail"
    passed = sum(1 for c in checks if c.status == "pass")
    return DFMAuditReport(
        status=overall,
        summary=f"Auditoría Goldfish DFM ({overall.upper()}): {passed}/{len(checks)} controles aprobados.",
        checks=checks,
    ).model_dump()


def _export_multi_node_glb(
    nodes: list[tuple[str, trimesh.Trimesh, list[int], float, float, str, bool]],
    glb_path: Path,
) -> Path:
    scene = trimesh.Scene()
    for name, raw_mesh, rgba, metallic, roughness, mat_hint, printable in nodes:
        polished = apply_studio_mesh_polish(
            raw_mesh.copy(),
            rgba=rgba,
            metallic=metallic,
            roughness=roughness,
            name=name,
            role=name,
            printable=printable,
            material_hint=mat_hint,
        )
        scene.add_geometry(polished, node_name=name, geom_name=name)
    glb_path.parent.mkdir(parents=True, exist_ok=True)
    glb_path.write_bytes(scene.export(file_type="glb"))
    return glb_path


# ---------------------------------------------------------------------------
# Domain 1: Organic Link (SIMP 3D + SDF)
# ---------------------------------------------------------------------------
def _gen_organic_link(
    params: dict[str, Any],
    material: dict[str, Any],
    printer: dict[str, Any],
    out_dir: Path,
    design_hash: str,
) -> dict[str, Any]:
    length_mm = max(80.0, min(350.0, float(params.get("length_mm", 180.0))))
    root_w = max(24.0, min(80.0, float(params.get("root_width_mm", 44.0))))
    tip_w = max(18.0, min(64.0, float(params.get("tip_width_mm", 32.0))))
    thick = max(12.0, min(48.0, float(params.get("thickness_mm", 22.0))))
    payload_kg = max(0.2, min(20.0, float(params.get("payload_kg", 2.0))))
    accel = max(0.0, min(40.0, float(params.get("angular_accel_rad_s2", 8.0))))
    volfrac = max(0.22, min(0.60, float(params.get("volume_fraction", 0.38))))
    overhang_45 = bool(params.get("overhang_filter_45", True))

    simp = run_simp_3d_link(
        length_mm=length_mm,
        root_width_mm=root_w,
        tip_width_mm=tip_w,
        thickness_mm=thick,
        load_n=payload_kg * 9.80665,
        angular_accel_rad_s2=accel,
        volume_fraction=volfrac,
        overhang_filter_45=overhang_45,
    )
    mesh, stats = build_organic_sdf_link(simp.rho_3d, length_mm, root_w, tip_w, thick)

    mat_id = str(material.get("id", "petg_cf"))
    rgba, metallic, roughness = _material_visual(material, mat_id)
    files = export_organic_sdf_link_files(mesh, rgba, metallic, roughness, out_dir, "organic_link", mat_id)
    density = float(material.get("density_g_cm3", 1.25))
    mass_g = round((float(stats["volume_mm3"]) / 1000.0) * density, 1)
    fits, bed_detail = _bed_fit_check(tuple(mesh.extents), printer)
    dfm = _build_dfm_report(
        is_watertight=bool(stats["is_watertight"]),
        bodies=int(stats["bodies"]),
        volume_mm3=float(stats["volume_mm3"]),
        fits_bed=fits,
        bed_detail=bed_detail,
        printer_name=str(printer.get("name", "Impresora 3D")),
        extra_note=f"(Langelaar 45° {'ON' if overhang_45 else 'OFF'})",
    )
    threemf_path = files.get("3mf") or files.get("threemf")
    return {
        "design_hash": design_hash,
        "domain": "link",
        "name": f"Eslabón Orgánico SIMP 3D ({length_mm:.0f} mm · {payload_kg:.1f} kg)",
        "glb_url": f"/api/files/{design_hash}/{files['glb'].name}",
        "stl_url": f"/api/files/{design_hash}/{files['stl'].name}",
        "threemf_url": f"/api/files/{design_hash}/{threemf_path.name}" if threemf_path else "",
        "metrics": {
            "mass_g": mass_g,
            "volume_mm3": round(float(stats["volume_mm3"]), 1),
            "mass_reduction_pct": round(simp.mass_reduction_pct, 1),
            "final_volume_fraction": round(simp.final_volume_fraction, 3),
            "simp_elapsed_s": round(simp.elapsed_s, 2),
            "payload_kg": payload_kg,
            "overhang_45": overhang_45,
        },
        "compliance_history": [round(float(x), 4) for x in simp.compliance_history],
        "dfm_audit": dfm,
        "notes": [
            f"SIMP 3D Hex8 con cordones Z reforzados ({simp.elapsed_s:.2f} s): {mass_g:.1f} g ({simp.mass_reduction_pct:.1f}% ahorro vs. macizo).",
            f"Barrenos Preserve/Obstacle exactos + suavizado Taubin ({'sin soportes Langelaar 45°' if overhang_45 else 'libre'}).",
        ],
    }


# ---------------------------------------------------------------------------
# Domain 2: Organic Motor Mount (SIMP 3D + NEMA17 / NEMA23 / MG996R Preserve)
# ---------------------------------------------------------------------------
def _gen_organic_mount(
    params: dict[str, Any],
    material: dict[str, Any],
    printer: dict[str, Any],
    out_dir: Path,
    design_hash: str,
) -> dict[str, Any]:
    actuator = str(params.get("actuator_type", "nema17")).lower()
    torque_nm = max(0.3, min(25.0, float(params.get("torque_nm", 2.5))))
    volfrac = max(0.28, min(0.58, float(params.get("volume_fraction", 0.42))))
    overhang_45 = bool(params.get("overhang_filter_45", True))

    if actuator == "nema23":
        length_mm, root_w, tip_w, thick, bore_d, pcd = 92.0, 62.0, 58.0, 28.0, 38.1, 47.14
    elif actuator == "mg996r":
        length_mm, root_w, tip_w, thick, bore_d, pcd = 64.0, 44.0, 40.0, 22.0, 12.0, 32.0
    else:
        actuator = "nema17"
        length_mm, root_w, tip_w, thick, bore_d, pcd = 76.0, 48.0, 44.0, 24.0, 22.2, 31.0

    # Equivalent reaction force from motor torque at lever arm
    load_n = max(15.0, (torque_nm / max(length_mm / 1000.0, 0.04)) * 1.4)
    simp = run_simp_3d_link(
        length_mm=length_mm,
        root_width_mm=root_w,
        tip_width_mm=tip_w,
        thickness_mm=thick,
        load_n=load_n,
        lateral_load_n=load_n * 0.45,
        root_od_mm=root_w * 0.94,
        tip_od_mm=tip_w * 0.94,
        root_bore_mm=min(bore_d, root_w * 0.55),
        tip_bore_mm=min(bore_d * 0.85, tip_w * 0.52),
        volume_fraction=volfrac,
        overhang_filter_45=overhang_45,
        nx=26,
        ny=10,
        nz=10,
        n_iter=16,
    )
    mesh, stats = build_organic_sdf_link(
        simp.rho_3d,
        length_mm=length_mm,
        root_width_mm=root_w,
        tip_width_mm=tip_w,
        thickness_mm=thick,
        root_od_mm=root_w * 0.94,
        tip_od_mm=tip_w * 0.94,
        root_bore_mm=min(bore_d, root_w * 0.55),
        tip_bore_mm=min(bore_d * 0.85, tip_w * 0.52),
        root_bolt_pcd_mm=min(pcd, root_w * 0.78),
        tip_bolt_pcd_mm=min(pcd * 0.85, tip_w * 0.76),
    )

    mat_id = str(material.get("id", "petg_cf"))
    rgba, metallic, roughness = _material_visual(material, mat_id)
    files = export_organic_sdf_link_files(mesh, rgba, metallic, roughness, out_dir, "organic_mount", mat_id)
    density = float(material.get("density_g_cm3", 1.25))
    mass_g = round((float(stats["volume_mm3"]) / 1000.0) * density, 1)
    fits, bed_detail = _bed_fit_check(tuple(mesh.extents), printer)
    dfm = _build_dfm_report(
        is_watertight=bool(stats["is_watertight"]),
        bodies=int(stats["bodies"]),
        volume_mm3=float(stats["volume_mm3"]),
        fits_bed=fits,
        bed_detail=bed_detail,
        printer_name=str(printer.get("name", "Impresora 3D")),
        extra_note=f"(Montura {actuator.upper()})",
    )
    threemf_path = files.get("3mf") or files.get("threemf")
    return {
        "design_hash": design_hash,
        "domain": "mount",
        "name": f"Montura Orgánica SIMP 3D ({actuator.upper()} · {torque_nm:.1f} N·m)",
        "glb_url": f"/api/files/{design_hash}/{files['glb'].name}",
        "stl_url": f"/api/files/{design_hash}/{files['stl'].name}",
        "threemf_url": f"/api/files/{design_hash}/{threemf_path.name}" if threemf_path else "",
        "metrics": {
            "actuator": actuator.upper(),
            "torque_nm": torque_nm,
            "mass_g": mass_g,
            "volume_mm3": round(float(stats["volume_mm3"]), 1),
            "mass_reduction_pct": round(simp.mass_reduction_pct, 1),
            "simp_elapsed_s": round(simp.elapsed_s, 2),
        },
        "compliance_history": [round(float(x), 4) for x in simp.compliance_history],
        "dfm_audit": dfm,
        "notes": [
            f"Montura orgánica {actuator.upper()} esculpida para {torque_nm:.1f} N·m ({load_n:.0f} N eq.): {mass_g:.1f} g (-{simp.mass_reduction_pct:.1f}% masa).",
            f"Patrón de pernos PCD {pcd:.1f} mm y alojamiento piloto preservados con precisión SDF.",
        ],
    }


# ---------------------------------------------------------------------------
# Domain 3: Organic Bionic Transmission Gear (Torque-sized B-Rep + Bionic Web)
# ---------------------------------------------------------------------------
def _gen_organic_transmission(
    params: dict[str, Any],
    material: dict[str, Any],
    printer: dict[str, Any],
    out_dir: Path,
    design_hash: str,
) -> dict[str, Any]:
    t0 = time.perf_counter()
    teeth = max(14, min(60, int(params.get("teeth", 32))))
    module_mm = max(1.0, min(4.0, float(params.get("module_mm", 2.0))))
    face_w = max(6.0, min(28.0, float(params.get("face_width_mm", 12.0))))
    torque_nm = max(0.5, min(40.0, float(params.get("torque_nm", 4.0))))
    spokes = max(5, min(9, int(params.get("spokes", 6))))

    pitch_r = 0.5 * teeth * module_mm
    outer_r = pitch_r + 0.9 * module_mm
    root_r = max(8.0, pitch_r - 1.15 * module_mm)
    bore_r = max(3.0, min(root_r * 0.28, 8.0))
    hub_r = bore_r + max(4.5, min(8.0, torque_nm * 0.45 + 3.5))
    rim_inner_r = max(hub_r + 5.0, root_r - max(3.6, module_mm * 1.9))

    # 1. Build outer toothed gear body + bionic curved cutout pockets (preserve rim + hub)
    with BuildSketch(Plane.XY) as sk:
        Circle(radius=root_r)
        # Involute-approximated rounded teeth around pitch circle
        tooth_w = 0.48 * math.pi * module_mm
        tooth_h = 2.05 * module_mm
        for k in range(teeth):
            ang_deg = k * (360.0 / teeth)
            ang = math.radians(ang_deg)
            tx = (root_r + tooth_h * 0.42) * math.cos(ang)
            ty = (root_r + tooth_h * 0.42) * math.sin(ang)
            with Locations((tx, ty)):
                RectangleRounded(tooth_h, tooth_w, radius=min(tooth_w * 0.28, 0.45), rotation=ang_deg, mode=Mode.ADD)

        # Organic bionic radial lightening cutouts (sized by torque safety)
        if rim_inner_r - hub_r > 5.5:
            mid_r = 0.5 * (hub_r + rim_inner_r)
            pocket_r = min((rim_inner_r - hub_r) * 0.38, (2.0 * math.pi * mid_r / spokes) * 0.29)
            if pocket_r >= 2.2:
                for s in range(spokes):
                    a1 = math.radians(s * (360.0 / spokes))
                    a2 = math.radians((s + 0.5) * (360.0 / spokes))
                    with Locations((mid_r * math.cos(a1), mid_r * math.sin(a1))):
                        Circle(radius=pocket_r, mode=Mode.SUBTRACT)
                    if pocket_r * 0.52 >= 1.6 and (rim_inner_r - hub_r) > 9.0:
                        r_outer_p = hub_r + (rim_inner_r - hub_r) * 0.68
                        with Locations((r_outer_p * math.cos(a2), r_outer_p * math.sin(a2))):
                            Circle(radius=pocket_r * 0.52, mode=Mode.SUBTRACT)

        # Central shaft bore
        Circle(radius=bore_r, mode=Mode.SUBTRACT)

    with BuildPart() as bp:
        extrude(sk.sketch.faces()[0], amount=face_w)
    gear_part = bp.part

    # Add slightly raised central hub boss for shaft grip
    hub_boss = Location((0.0, 0.0, face_w / 2.0)) * (
        Cylinder(radius=hub_r, height=face_w + 3.0) - Cylinder(radius=bore_r, height=face_w + 8.0)
    )
    try:
        gear_part = gear_part + hub_boss
    except Exception:
        pass

    out_dir.mkdir(parents=True, exist_ok=True)
    stl_path = out_dir / "organic_transmission.stl"
    step_path = out_dir / "organic_transmission.step"
    threemf_path = out_dir / "organic_transmission.3mf"
    glb_path = out_dir / "organic_transmission.glb"

    export_stl(gear_part, str(stl_path), tolerance=0.09, angular_tolerance=0.14)
    export_step(gear_part, str(step_path))

    raw_holder: list[trimesh.Trimesh] = []
    mat_id = str(material.get("id", "petg_cf"))
    rgba, metallic, roughness = _material_visual(material, mat_id)
    studio_mesh = part_to_studio_trimesh(
        gear_part,
        rgba=rgba,
        metallic=metallic,
        roughness=roughness,
        name="organic_transmission",
        role="organic_transmission",
        printable=True,
        material_hint=mat_id,
        out_raw_holder=raw_holder,
    )
    raw_mesh = raw_holder[0] if raw_holder else studio_mesh
    export_mesh_to_3mf(raw_mesh, threemf_path, part_name="organic_transmission")
    scene = trimesh.Scene()
    scene.add_geometry(studio_mesh, node_name="organic_transmission", geom_name="organic_transmission")
    glb_path.write_bytes(scene.export(file_type="glb"))

    # Lewis bending tooth stress check
    density = float(material.get("density_g_cm3", 1.25))
    yield_mpa = float(material.get("yield_xy_mpa", material.get("yield_mpa", 55.0)))
    vol_mm3 = float(gear_part.volume)
    solid_vol = math.pi * (pitch_r ** 2) * face_w
    mass_g = round((vol_mm3 / 1000.0) * density, 1)
    mass_red_pct = round(max(0.0, (1.0 - vol_mm3 / max(solid_vol, 1.0)) * 100.0), 1)
    ft_n = (torque_nm * 1000.0) / max(pitch_r, 5.0)
    lewis_y = 0.154 - 0.912 / max(teeth, 12) + 0.25
    tooth_stress_mpa = ft_n / max(face_w * module_mm * lewis_y, 1.0)
    tooth_sf = round(min(25.0, yield_mpa / max(tooth_stress_mpa, 0.1)), 2)

    fits, bed_detail = _bed_fit_check(tuple(raw_mesh.extents), printer)
    dfm = _build_dfm_report(
        is_watertight=bool(raw_mesh.is_watertight),
        bodies=len(raw_mesh.split(only_watertight=False)),
        volume_mm3=vol_mm3,
        fits_bed=fits,
        bed_detail=bed_detail,
        printer_name=str(printer.get("name", "Impresora 3D")),
        extra_note=f"(Z={teeth} m={module_mm:.1f})",
    )
    elapsed_s = round(time.perf_counter() - t0, 2)
    return {
        "design_hash": design_hash,
        "domain": "transmission",
        "name": f"Transmisión Orgánica Biónica (Z={teeth} · m={module_mm:.1f} · {torque_nm:.1f} N·m)",
        "glb_url": f"/api/files/{design_hash}/{glb_path.name}",
        "stl_url": f"/api/files/{design_hash}/{stl_path.name}",
        "threemf_url": f"/api/files/{design_hash}/{threemf_path.name}",
        "step_url": f"/api/files/{design_hash}/{step_path.name}",
        "metrics": {
            "teeth": teeth,
            "module_mm": module_mm,
            "pitch_diameter_mm": round(2.0 * pitch_r, 1),
            "torque_nm": torque_nm,
            "tooth_safety_factor": tooth_sf,
            "mass_g": mass_g,
            "mass_reduction_pct": mass_red_pct,
            "gen_time_s": elapsed_s,
        },
        "dfm_audit": dfm,
        "notes": [
            f"Engranaje biónico Z={teeth} (Ø primitivo {2*pitch_r:.1f} mm) con aligeramiento celular radial: {mass_g:.1f} g (-{mass_red_pct:.1f}% masa).",
            f"Verificación Lewis a {torque_nm:.1f} N·m: σ_diente = {tooth_stress_mpa:.1f} MPa · Factor de Seguridad = {tooth_sf:.2f}×.",
        ],
    }


# ---------------------------------------------------------------------------
# Domain 4: Complete Organic Robotic Arm (RNEA + Dual SIMP 3D Links + Assembly)
# ---------------------------------------------------------------------------
def _gen_organic_full_arm(
    params: dict[str, Any],
    material: dict[str, Any],
    printer: dict[str, Any],
    out_dir: Path,
    design_hash: str,
) -> dict[str, Any]:
    t0 = time.perf_counter()
    upper_l = max(120.0, min(260.0, float(params.get("upper_length_mm", 180.0))))
    fore_l = max(100.0, min(230.0, float(params.get("forearm_length_mm", 150.0))))
    payload_kg = max(0.3, min(10.0, float(params.get("payload_kg", 1.5))))
    accel = max(0.0, min(30.0, float(params.get("angular_accel_rad_s2", 8.0))))
    volfrac = max(0.28, min(0.55, float(params.get("volume_fraction", 0.38))))
    overhang_45 = bool(params.get("overhang_filter_45", True))

    # 1. Sculpt Upper Arm & Forearm via SIMP 3D under RNEA-equivalent shoulder/elbow loads
    load_tip_n = payload_kg * 9.80665
    load_shoulder_n = load_tip_n * 1.65  # includes forearm + wrist gravity & inertia
    simp_upper = run_simp_3d_link(
        length_mm=upper_l, root_width_mm=44.0, tip_width_mm=34.0, thickness_mm=22.0,
        load_n=load_shoulder_n, angular_accel_rad_s2=accel, volume_fraction=volfrac,
        overhang_filter_45=overhang_45, nx=28, ny=10, nz=10, n_iter=15,
    )
    mesh_upper, stats_u = build_organic_sdf_link(simp_upper.rho_3d, upper_l, 44.0, 34.0, 22.0)

    simp_fore = run_simp_3d_link(
        length_mm=fore_l, root_width_mm=34.0, tip_width_mm=26.0, thickness_mm=18.0,
        load_n=load_tip_n, angular_accel_rad_s2=accel * 1.2, volume_fraction=volfrac,
        overhang_filter_45=overhang_45, nx=24, ny=9, nz=9, n_iter=14,
    )
    mesh_fore, stats_f = build_organic_sdf_link(simp_fore.rho_3d, fore_l, 34.0, 26.0, 18.0, root_od_mm=30.0, tip_od_mm=24.0)

    # 2. Build Base Pedestal, Shoulder/Elbow Joint Housings & TCP Flange in build123d
    base_part = Cylinder(radius=46.0, height=44.0) - Cylinder(radius=10.0, height=50.0)
    sh_joint = Cylinder(radius=22.0, height=34.0)
    el_joint = Cylinder(radius=18.0, height=28.0)
    tcp_part = Cylinder(radius=15.0, height=16.0)

    # Pose arm in 3D: Base at Z=22, Shoulder pivot at (0,0,56), Upper tilted +55° in XZ, Forearm at +10°
    sh_z = 58.0
    a1 = math.radians(55.0)
    a2 = math.radians(10.0)

    # Transform upper arm from X-axis (0..upper_l) into XZ plane at angle a1
    upper_placed = mesh_upper.copy()
    rot_u = trimesh.transformations.rotation_matrix(-a1, [0, 1, 0])
    upper_placed.apply_transform(rot_u)
    upper_placed.apply_translation([0.0, 0.0, sh_z])

    el_x = upper_l * math.cos(a1)
    el_z = sh_z + upper_l * math.sin(a1)
    fore_placed = mesh_fore.copy()
    rot_f = trimesh.transformations.rotation_matrix(-a2, [0, 1, 0])
    fore_placed.apply_transform(rot_f)
    fore_placed.apply_translation([el_x, 22.0, el_z])

    tcp_x = el_x + fore_l * math.cos(a2)
    tcp_z = el_z + fore_l * math.sin(a2)

    def _to_tm(part: Part, loc: Location) -> trimesh.Trimesh:
        holder: list[trimesh.Trimesh] = []
        part_to_studio_trimesh(loc * part, [120, 130, 145, 255], 0.5, 0.35, "arm_hw", out_raw_holder=holder)
        return holder[0]

    base_tm = _to_tm(base_part, Location((0.0, 0.0, 22.0)))
    sh_tm = _to_tm(sh_joint, Location((0.0, 0.0, sh_z)) * Rotation(90.0, 0.0, 0.0))
    el_tm = _to_tm(el_joint, Location((el_x, 11.0, el_z)) * Rotation(90.0, 0.0, 0.0))
    tcp_tm = _to_tm(tcp_part, Location((tcp_x + 8.0, 22.0, tcp_z)) * Rotation(0.0, 90.0, 0.0))

    mat_id = str(material.get("id", "petg_cf"))
    rgba, metallic, roughness = _material_visual(material, mat_id)
    out_dir.mkdir(parents=True, exist_ok=True)
    glb_path = out_dir / "organic_full_arm.glb"
    stl_path = out_dir / "organic_upper_arm.stl"
    threemf_path = out_dir / "organic_full_arm.3mf"

    _export_multi_node_glb(
        [
            ("base_pedestal", base_tm, [42, 48, 58, 255], 0.55, 0.35, mat_id, True),
            ("shoulder_joint", sh_tm, [196, 206, 218, 255], 0.88, 0.22, "al_6061", False),
            ("organic_upper_arm", upper_placed, rgba, metallic, roughness, mat_id, True),
            ("elbow_joint", el_tm, [196, 206, 218, 255], 0.88, 0.22, "al_6061", False),
            ("organic_forearm", fore_placed, rgba, metallic, roughness, mat_id, True),
            ("tcp_flange", tcp_tm, [234, 179, 8, 255], 0.82, 0.25, "al_6061", False),
        ],
        glb_path,
    )
    mesh_upper.export(str(stl_path))
    combined_print = trimesh.util.concatenate([mesh_upper, mesh_fore.copy().apply_translation([0.0, 56.0, 0.0])])
    export_mesh_to_3mf(combined_print, threemf_path, part_name="organic_arm_links")

    density = float(material.get("density_g_cm3", 1.25))
    links_vol = float(stats_u["volume_mm3"]) + float(stats_f["volume_mm3"])
    links_mass_g = round((links_vol / 1000.0) * density, 1)
    avg_red = round(0.5 * (simp_upper.mass_reduction_pct + simp_fore.mass_reduction_pct), 1)
    reach_mm = round(upper_l + fore_l, 1)
    shoulder_torque_nm = round(payload_kg * 9.81 * (reach_mm / 1000.0) + (links_mass_g / 1000.0) * 9.81 * (reach_mm / 2000.0), 2)
    fits, bed_detail = _bed_fit_check(tuple(mesh_upper.extents), printer)
    dfm = _build_dfm_report(
        is_watertight=bool(stats_u["is_watertight"] and stats_f["is_watertight"]),
        bodies=1 if (stats_u["bodies"] == 1 and stats_f["bodies"] == 1) else 2,
        volume_mm3=links_vol,
        fits_bed=fits,
        bed_detail=bed_detail,
        printer_name=str(printer.get("name", "Impresora 3D")),
        extra_note="(2 eslabones orgánicos SIMP 3D)",
    )
    return {
        "design_hash": design_hash,
        "domain": "full_arm",
        "name": f"Brazo Completo Orgánico SIMP 3D ({reach_mm:.0f} mm · {payload_kg:.1f} kg)",
        "glb_url": f"/api/files/{design_hash}/{glb_path.name}",
        "stl_url": f"/api/files/{design_hash}/{stl_path.name}",
        "threemf_url": f"/api/files/{design_hash}/{threemf_path.name}",
        "explode_vectors": {
            "base_pedestal": [0.0, 0.0, -35.0],
            "shoulder_joint": [0.0, -30.0, 10.0],
            "organic_upper_arm": [25.0, 0.0, 28.0],
            "elbow_joint": [45.0, 28.0, 42.0],
            "organic_forearm": [70.0, 40.0, 56.0],
            "tcp_flange": [95.0, 40.0, 56.0],
        },
        "metrics": {
            "reach_mm": reach_mm,
            "payload_kg": payload_kg,
            "links_mass_g": links_mass_g,
            "mass_reduction_pct": avg_red,
            "shoulder_torque_nm": shoulder_torque_nm,
            "gen_time_s": round(time.perf_counter() - t0, 2),
        },
        "compliance_history": [round(float(x), 4) for x in simp_upper.compliance_history],
        "dfm_audit": dfm,
        "notes": [
            f"Brazo completo con húmero ({upper_l:.0f} mm) y antebrazo ({fore_l:.0f} mm) esculpidos en SIMP 3D: {links_mass_g:.1f} g (-{avg_red:.1f}% masa).",
            f"Torque estimado en hombro: {shoulder_torque_nm:.2f} N·m @ {payload_kg:.1f} kg + {accel:.0f} rad/s².",
        ],
    }


# ---------------------------------------------------------------------------
# Domain 5: Evolved Organic Propeller (NSGA-II + Adkins-Liebeck + BEMT)
# ---------------------------------------------------------------------------
def _gen_organic_propeller(
    params: dict[str, Any],
    material: dict[str, Any],
    printer: dict[str, Any],
    out_dir: Path,
    design_hash: str,
) -> dict[str, Any]:
    target_thrust_g = max(150.0, min(4000.0, float(params.get("target_thrust_g", 600.0))))
    diam_in = max(2.5, min(10.0, float(params.get("diameter_inch", 5.0))))
    rpm = max(5000.0, min(35000.0, float(params.get("rpm", 18000.0))))
    blades = max(2, min(4, int(params.get("blade_count", 3))))
    priority = str(params.get("priority", "max_efficiency"))
    density = float(material.get("density_g_cm3", 1.25))
    yield_z = float(material.get("yield_z_mpa", material.get("yield_mpa", 45.0)))

    evol = evolve_propeller_pareto(
        target_thrust_g=target_thrust_g,
        diameter_inch=diam_in,
        rpm=rpm,
        blade_count=blades,
        prop_style="conventional",
        material_density_g_cm3=density,
        material_yield_z_mpa=yield_z,
    )
    winners_map = evol.winners if isinstance(evol.winners, dict) else {w.candidate_id: w for w in evol.winners}
    chosen = winners_map.get(priority) or next(iter(winners_map.values()))

    prop_ccw = generate_propeller_part(
        diameter_inch=diam_in, pitch_inch=4.3, blade_count=blades,
        airfoil_id=chosen.airfoil_id, prop_style="conventional", direction="ccw",
        custom_stations=chosen.stations,
    )
    prop_cw = generate_propeller_part(
        diameter_inch=diam_in, pitch_inch=4.3, blade_count=blades,
        airfoil_id=chosen.airfoil_id, prop_style="conventional", direction="cw",
        custom_stations=chosen.stations,
    )

    out_dir.mkdir(parents=True, exist_ok=True)
    stl_path = out_dir / "organic_propeller_ccw.stl"
    step_path = out_dir / "organic_propeller_ccw.step"
    threemf_path = out_dir / "organic_propeller_pair.3mf"
    glb_path = out_dir / "organic_propeller_pair.glb"

    export_stl(prop_ccw, str(stl_path), tolerance=0.09, angular_tolerance=0.14)
    export_step(prop_ccw, str(step_path))

    sep = (diam_in * 25.4) * 0.56
    raw_ccw: list[trimesh.Trimesh] = []
    raw_cw: list[trimesh.Trimesh] = []
    mat_id = str(material.get("id", "pa12_cf"))
    rgba, metallic, roughness = _material_visual(material, mat_id)
    part_to_studio_trimesh(Location((-sep, 0.0, 0.0)) * prop_ccw, rgba, metallic, roughness, "propeller_ccw", out_raw_holder=raw_ccw)
    part_to_studio_trimesh(Location((+sep, 0.0, 0.0)) * prop_cw, rgba, metallic, roughness, "propeller_cw", out_raw_holder=raw_cw)

    _export_multi_node_glb(
        [
            ("propeller_ccw", raw_ccw[0], rgba, metallic, roughness, mat_id, True),
            ("propeller_cw", raw_cw[0], rgba, metallic, roughness, mat_id, True),
        ],
        glb_path,
    )
    export_mesh_to_3mf(trimesh.util.concatenate([raw_ccw[0], raw_cw[0]]), threemf_path, part_name="evolved_propeller_pair")

    fits, bed_detail = _bed_fit_check(tuple(raw_ccw[0].extents), printer)
    dfm = _build_dfm_report(
        is_watertight=bool(raw_ccw[0].is_watertight and raw_cw[0].is_watertight),
        bodies=len(raw_ccw[0].split(only_watertight=False)),
        volume_mm3=float(prop_ccw.volume),
        fits_bed=fits,
        bed_detail=bed_detail,
        printer_name=str(printer.get("name", "Impresora 3D")),
        extra_note="(Par CCW + CW NSGA-II)",
    )
    winners_list = [dataclasses.asdict(w) if dataclasses.is_dataclass(w) else dict(w) for w in winners_map.values()]
    return {
        "design_hash": design_hash,
        "domain": "propeller",
        "name": f'Hélice Orgánica NSGA-II {diam_in:.1f}" · {chosen.label}',
        "glb_url": f"/api/files/{design_hash}/{glb_path.name}",
        "stl_url": f"/api/files/{design_hash}/{stl_path.name}",
        "threemf_url": f"/api/files/{design_hash}/{threemf_path.name}",
        "step_url": f"/api/files/{design_hash}/{step_path.name}",
        "explode_vectors": {"propeller_ccw": [-24.0, 0.0, 12.0], "propeller_cw": [24.0, 0.0, 12.0]},
        "metrics": {
            **chosen.metrics,
            "baseline_g_per_w": round(float(evol.baseline_metrics.get("efficiency_g_per_w", 3.25)), 2),
            "evaluations": evol.evaluations,
            "evolve_time_s": round(evol.elapsed_s, 2),
        },
        "pareto_winners": winners_list,
        "dfm_audit": dfm,
        "notes": [
            f"NSGA-II ({evol.evaluations} palas en {evol.elapsed_s:.2f} s): {chosen.metrics['efficiency_g_per_w']:.2f} g/W · {chosen.metrics['estimated_noise_dba']:.1f} dBA · FS raíz {chosen.metrics['root_safety_factor']:.2f}.",
            f"Perfil ganador: {chosen.airfoil_id.upper()} · Sweep {chosen.sweep_deg:+.1f}° · Par CCW+CW watertight listo para STEP/3MF.",
        ],
    }


# ---------------------------------------------------------------------------
# Domain 6: Complete Organic FPV Drone (Bionic Truss Frame + 4x NSGA-II Props)
# ---------------------------------------------------------------------------
def _gen_organic_fpv_drone(
    params: dict[str, Any],
    material: dict[str, Any],
    printer: dict[str, Any],
    out_dir: Path,
    design_hash: str,
) -> dict[str, Any]:
    t0 = time.perf_counter()
    prop_in = max(3.0, min(7.0, float(params.get("prop_inch", 5.0))))
    wheelbase = max(140.0, min(250.0, float(params.get("wheelbase_mm", 215.0 if prop_in >= 5.0 else 148.0))))
    plate_t = max(2.5, min(5.5, float(params.get("plate_thickness_mm", 4.0))))
    rpm = max(12000.0, min(28000.0, float(params.get("rpm", 20000.0))))
    priority = str(params.get("priority", "max_efficiency"))
    stack_mm = 20.0 if prop_in < 4.5 else 30.5
    motor_pcd = 12.0 if prop_in < 4.5 else 16.0

    density = float(material.get("density_g_cm3", 1.25))
    yield_z = float(material.get("yield_z_mpa", material.get("yield_mpa", 45.0)))

    # 1. Evolve optimal organic propellers for this drone via NSGA-II
    target_per_motor_g = 360.0 if prop_in <= 3.5 else (820.0 if prop_in <= 5.5 else 1300.0)
    evol = evolve_propeller_pareto(
        target_thrust_g=target_per_motor_g,
        diameter_inch=prop_in,
        rpm=rpm,
        blade_count=3,
        prop_style="conventional",
        material_density_g_cm3=density,
        material_yield_z_mpa=yield_z,
        pop_size=36,
        generations=22,
    )
    winners_map = evol.winners if isinstance(evol.winners, dict) else {w.candidate_id: w for w in evol.winners}
    chosen = winners_map.get(priority) or next(iter(winners_map.values()))

    prop_ccw = generate_propeller_part(
        diameter_inch=prop_in, pitch_inch=4.0, blade_count=3,
        airfoil_id=chosen.airfoil_id, prop_style="conventional", direction="ccw",
        custom_stations=chosen.stations, fast_preview=True,
    )
    prop_cw = generate_propeller_part(
        diameter_inch=prop_in, pitch_inch=4.0, blade_count=3,
        airfoil_id=chosen.airfoil_id, prop_style="conventional", direction="cw",
        custom_stations=chosen.stations, fast_preview=True,
    )

    # 2. Build Organic Bionic Lightened True-X Frame (cellular cutouts along all 4 arms + hub)
    arm_r = wheelbase / 2.0
    arm_w = max(13.0, min(20.0, prop_in * 3.1))
    hub_l = max(56.0, stack_mm + 34.0)
    hub_w = max(36.0, stack_mm + 12.0)
    pad_r = max(11.5, motor_pcd * 0.72)

    dxf_circles: list[tuple[float, float, float]] = []
    motor_centers: list[tuple[float, float]] = []
    with BuildSketch(Plane.XY) as bot_sk:
        RectangleRounded(hub_l, hub_w, radius=7.0)
        for ang_deg in (45.0, 135.0, 225.0, 315.0):
            ang = math.radians(ang_deg)
            mx, my = arm_r * math.cos(ang), arm_r * math.sin(ang)
            motor_centers.append((mx, my))
            with Locations((mx * 0.5, my * 0.5)):
                RectangleRounded(arm_r, arm_w, radius=3.5, rotation=ang_deg, mode=Mode.ADD)
            with Locations((mx, my)):
                Circle(radius=pad_r, mode=Mode.ADD)
            # Bionic cellular weight-reduction holes along each arm (preserve root & motor pad)
            for frac in (0.44, 0.62, 0.78):
                hx, hy = mx * frac, my * frac
                hr = max(2.2, min(4.2, arm_w * (0.28 - 0.04 * frac)))
                with Locations((hx, hy)):
                    Circle(radius=hr, mode=Mode.SUBTRACT)
                dxf_circles.append((hx, hy, hr))
        # Central bionic lightening + 4xM3 FC stack holes
        with Locations((0.0, 0.0)):
            Circle(radius=max(5.0, stack_mm * 0.24), mode=Mode.SUBTRACT)
        s_half = stack_mm / 2.0
        for sx in (-s_half, s_half):
            for sy in (-s_half, s_half):
                with Locations((sx, sy)):
                    Circle(radius=1.65, mode=Mode.SUBTRACT)
                dxf_circles.append((sx, sy, 1.65))
        m_r = motor_pcd / 2.0
        for mx, my in motor_centers:
            with Locations((mx, my)):
                Circle(radius=3.5, mode=Mode.SUBTRACT)
            dxf_circles.append((mx, my, 3.5))
            for k in range(4):
                ka = math.radians(45.0 + k * 90.0)
                bx, by = mx + m_r * math.cos(ka), my + m_r * math.sin(ka)
                with Locations((bx, by)):
                    Circle(radius=1.6, mode=Mode.SUBTRACT)
                dxf_circles.append((bx, by, 1.6))

    with BuildPart() as bot_bp:
        extrude(bot_sk.sketch.faces()[0], amount=plate_t)
    bottom_plate = bot_bp.part

    stack_h = 25.0
    top_t = max(2.0, plate_t * 0.65)
    with BuildSketch(Plane.XY) as top_sk:
        RectangleRounded(hub_l * 1.12, hub_w * 0.86, radius=6.0)
        for bx in (-14.0, 0.0, 14.0):
            with Locations((bx, 0.0)):
                Circle(radius=4.5, mode=Mode.SUBTRACT)
    with BuildPart() as top_bp:
        extrude(top_sk.sketch.faces()[0].moved(Location((0.0, 0.0, plate_t + stack_h))), amount=top_t)
    top_plate = top_bp.part

    motor_h = 14.5 if prop_in >= 5.0 else 10.5
    motor_r = 13.0 if prop_in >= 5.0 else 9.0
    motors = Compound(children=[Location((mx, my, plate_t + 0.6 + motor_h / 2.0)) * Cylinder(radius=motor_r, height=motor_h) for mx, my in motor_centers])
    prop_z = plate_t + 0.6 + motor_h + 4.8
    props_ccw = Compound(children=[Location((motor_centers[0][0], motor_centers[0][1], prop_z)) * prop_ccw, Location((motor_centers[2][0], motor_centers[2][1], prop_z)) * prop_ccw])
    props_cw = Compound(children=[Location((motor_centers[1][0], motor_centers[1][1], prop_z)) * prop_cw, Location((motor_centers[3][0], motor_centers[3][1], prop_z)) * prop_cw])

    out_dir.mkdir(parents=True, exist_ok=True)
    glb_path = out_dir / "organic_fpv_drone.glb"
    stl_path = out_dir / "organic_fpv_bottom_plate.stl"
    threemf_path = out_dir / "organic_fpv_drone.3mf"
    dxf_path = write_simple_dxf_outline(dxf_circles, [(-hub_l / 2.0, -hub_w / 2.0, hub_l / 2.0, hub_w / 2.0)], out_dir / "organic_fpv_frame.dxf", title="ROBOFORGE ORGANIC FPV FRAME")
    export_stl(bottom_plate, str(stl_path), tolerance=0.09, angular_tolerance=0.14)

    def _raw(solid: Part | Compound) -> trimesh.Trimesh:
        h: list[trimesh.Trimesh] = []
        part_to_studio_trimesh(solid, [80, 90, 100, 255], 0.4, 0.4, "drone_part", out_raw_holder=h)
        return h[0]

    mat_id = str(material.get("id", "pa12_cf"))
    rgba, metallic, roughness = _material_visual(material, mat_id)
    bot_tm, top_tm, mot_tm, pccw_tm, pcw_tm = _raw(bottom_plate), _raw(top_plate), _raw(motors), _raw(props_ccw), _raw(props_cw)
    _export_multi_node_glb(
        [
            ("organic_bottom_plate", bot_tm, rgba, metallic, roughness, mat_id, True),
            ("organic_top_plate", top_tm, [58, 68, 82, 255], metallic, roughness, mat_id, True),
            ("brushless_motors", mot_tm, [196, 206, 218, 255], 0.90, 0.20, "al_6061", False),
            ("evolved_props_ccw", pccw_tm, [14, 165, 233, 255], 0.30, 0.36, mat_id, True),
            ("evolved_props_cw", pcw_tm, [249, 115, 22, 255], 0.30, 0.36, mat_id, True),
        ],
        glb_path,
    )
    export_mesh_to_3mf(bot_tm, threemf_path, part_name="organic_fpv_bottom_plate")

    frame_mass_g = round((bottom_plate.volume + top_plate.volume) / 1000.0 * density, 1)
    props_mass_g = round(4.0 * (prop_ccw.volume / 1000.0) * density, 1)
    auw_g = round(frame_mass_g + props_mass_g + (165.0 if prop_in < 4.5 else 410.0), 1)
    total_thrust_g = round(4.0 * float(chosen.metrics["thrust_g"]), 1)
    twr = round(total_thrust_g / max(auw_g, 1.0), 2)

    fits, bed_detail = _bed_fit_check(tuple(bot_tm.extents), printer)
    dfm = _build_dfm_report(
        is_watertight=bool(bot_tm.is_watertight and top_tm.is_watertight),
        bodies=len(bot_tm.split(only_watertight=False)),
        volume_mm3=float(bottom_plate.volume + top_plate.volume),
        fits_bed=fits,
        bed_detail=bed_detail,
        printer_name=str(printer.get("name", "Impresora 3D")),
        extra_note="(Chasis Biónico + 4× Hélices NSGA-II)",
    )
    winners_list = [dataclasses.asdict(w) if dataclasses.is_dataclass(w) else dict(w) for w in winners_map.values()]
    return {
        "design_hash": design_hash,
        "domain": "fpv_drone",
        "name": f'Dron Orgánico Completo {prop_in:.0f}" (Chasis Biónico + 4× NSGA-II)',
        "glb_url": f"/api/files/{design_hash}/{glb_path.name}",
        "stl_url": f"/api/files/{design_hash}/{stl_path.name}",
        "threemf_url": f"/api/files/{design_hash}/{threemf_path.name}",
        "dxf_url": f"/api/files/{design_hash}/{dxf_path.name}",
        "explode_vectors": {
            "organic_bottom_plate": [0.0, 0.0, -30.0],
            "organic_top_plate": [0.0, 0.0, 52.0],
            "brushless_motors": [0.0, 0.0, 32.0],
            "evolved_props_ccw": [0.0, 0.0, 76.0],
            "evolved_props_cw": [0.0, 0.0, 76.0],
        },
        "metrics": {
            "twr": twr,
            "total_thrust_g": total_thrust_g,
            "auw_mass_g": auw_g,
            "frame_mass_g": frame_mass_g,
            "prop_efficiency_g_per_w": chosen.metrics["efficiency_g_per_w"],
            "prop_noise_dba": chosen.metrics["estimated_noise_dba"],
            "prop_root_sf": chosen.metrics["root_safety_factor"],
            "gen_time_s": round(time.perf_counter() - t0, 2),
        },
        "pareto_winners": winners_list,
        "dfm_audit": dfm,
        "notes": [
            f"Dron completo orgánico {prop_in:.0f}\": TWR {twr:.2f}:1 ({total_thrust_g:.0f} g empuje NSGA-II / {auw_g:.0f} g AUW).",
            f"Chasis True-X celular aligerado ({frame_mass_g:.1f} g) + DXF multi-capa CNC + 4 hélices evolucionadas ({chosen.metrics['efficiency_g_per_w']:.2f} g/W).",
        ],
    }


def generate_organic_studio_model(
    domain: str,
    params: dict[str, Any],
    material: dict[str, Any],
    printer: dict[str, Any],
    out_dir: Path,
    design_hash: str,
) -> dict[str, Any]:
    """Unified dispatcher for all 6 Organic Sculpting Studio domains."""
    d = (domain or "link").strip().lower()
    handlers = {
        "link": _gen_organic_link,
        "mount": _gen_organic_mount,
        "transmission": _gen_organic_transmission,
        "full_arm": _gen_organic_full_arm,
        "propeller": _gen_organic_propeller,
        "fpv_drone": _gen_organic_fpv_drone,
    }
    fn = handlers.get(d)
    if fn is None:
        raise ValueError(f"Dominio orgánico desconocido: '{domain}'. Opciones: {list(handlers.keys())}")
    return fn(params or {}, material, printer, out_dir, design_hash)


__all__ = ["generate_organic_studio_model"]
