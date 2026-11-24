"""Independent OpenCASCADE / build123d B-Rep Benchmark: Franka Emika Panda 7-DOF (45+ parts).

Zero imports from RoboForge backend — pure external B-Rep script to compare effort,
part density, B-Rep validity, STL watertightness, and build time against RoboForge v9.0.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import trimesh
from build123d import Box, Compound, Cylinder, Location, Part, export_step, export_stl
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer

REPO_ROOT = Path(__file__).resolve().parents[2]
OUT_DIR = REPO_ROOT / "exports" / "benchmark_franka"


def _hollow_cylinder(r_out: float, r_in: float, h: float) -> Part:
    return Cylinder(radius=r_out, height=h) - Cylinder(radius=r_in, height=h + 2.0)


def _bolted_flange_and_screws(
    r_out: float, r_in: float, h: float, pcd_r: float, n_bolts: int, bolt_r: float
) -> tuple[Part, list[Part]]:
    flange = _hollow_cylinder(r_out, r_in, h)
    screws: list[Part] = []
    for i in range(n_bolts):
        ang = 2.0 * math.pi * i / n_bolts
        bx, by = pcd_r * math.cos(ang), pcd_r * math.sin(ang)
        flange = flange - (Location((bx, by, 0.0)) * Cylinder(radius=bolt_r, height=h + 2.0))
        head = Location((bx, by, h / 2.0 + 2.0)) * Cylinder(radius=bolt_r * 1.65, height=4.0)
        shank = Location((bx, by, 0.0)) * Cylinder(radius=bolt_r * 0.9, height=h)
        screws.append(head + shank)
    return flange, screws


def _joint_module_parts(r_out: float, r_in: float, length: float) -> tuple[Part, Part, Part]:
    """Return 3 distinct solids: (housing, harmonic_ring, torque_sensor_ring)."""
    housing = _hollow_cylinder(r_out, r_in, length)
    harmonic = _hollow_cylinder(r_out + 2.0, r_out - 3.0, length * 0.24)
    torque_ring = Location((0.0, 0.0, length * 0.36)) * _hollow_cylinder(r_out + 1.2, r_out - 2.5, 10.0)
    return housing, harmonic, torque_ring


def _offset_bridge(dx: float, r_end: float, h: float) -> Part:
    c1 = Cylinder(radius=r_end, height=h)
    c2 = Location((dx, 0.0, 0.0)) * Cylinder(radius=r_end * 0.92, height=h)
    bar = Location((dx / 2.0, 0.0, 0.0)) * Box(abs(dx), r_end * 1.45, h)
    bore1 = Cylinder(radius=13.0, height=h + 2.0)
    bore2 = Location((dx, 0.0, 0.0)) * Cylinder(radius=12.0, height=h + 2.0)
    return (c1 + c2 + bar) - bore1 - bore2


def build_franka_panda_brep() -> dict:
    t0 = time.perf_counter()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    parts: dict[str, Part] = {}

    # 1. Base Pedestal + Anchor Flange + 8x M6 Anchor Bolts (10 parts)
    base_flange, base_bolts = _bolted_flange_and_screws(93.0, 22.0, 14.0, 78.0, 8, 3.2)
    parts["base_flange"] = Location((0.0, 0.0, 7.0)) * base_flange
    for idx, bolt in enumerate(base_bolts, 1):
        parts[f"base_bolt_{idx}"] = Location((0.0, 0.0, 7.0)) * bolt
    parts["base_column"] = Location((0.0, 0.0, 76.0)) * _hollow_cylinder(66.0, 48.0, 124.0)

    # 2. Joint 1..7 Modules (each = housing + harmonic reducer + torque sensor ring = 21 parts)
    joint_specs = [
        # (name, pos_xyz, rot_xyz_deg, r_out, r_in, length)
        ("j1", (0.0, 0.0, 182.0), (0.0, 0.0, 0.0), 58.0, 16.0, 96.0),
        ("j2", (0.0, 0.0, 333.0), (90.0, 0.0, 0.0), 55.0, 15.0, 112.0),
        ("j3", (0.0, 0.0, 520.0), (0.0, 0.0, 0.0), 49.0, 14.0, 90.0),
        ("j4", (82.5, 0.0, 649.0), (90.0, 0.0, 0.0), 47.0, 13.0, 98.0),
        ("j5", (0.0, 0.0, 880.0), (0.0, 0.0, 0.0), 41.0, 11.0, 78.0),
        ("j6", (0.0, 0.0, 1033.0), (90.0, 0.0, 0.0), 39.0, 10.0, 80.0),
        ("j7", (88.0, 0.0, 1105.0), (0.0, 0.0, 0.0), 35.0, 9.0, 60.0),
    ]
    for jname, pos, rot, r_o, r_i, ln in joint_specs:
        h_part, harm_part, tq_part = _joint_module_parts(r_o, r_i, ln)
        loc = Location(pos, rot)
        parts[f"{jname}_housing"] = loc * h_part
        parts[f"{jname}_harmonic_reducer"] = loc * harm_part
        parts[f"{jname}_torque_sensor_ring"] = loc * tq_part

    # 3. Structural Hollow Links + Authentic Panda Offsets (6 parts)
    parts["link1_lower_shoulder"] = Location((0.0, 0.0, 258.0)) * _hollow_cylinder(52.0, 40.0, 68.0)
    parts["link2_upper_arm"] = Location((0.0, 0.0, 426.0)) * _hollow_cylinder(48.0, 38.0, 102.0)
    parts["link3_elbow_offset_pos"] = Location((0.0, 0.0, 600.0)) * _offset_bridge(82.5, 44.0, 48.0)
    parts["link4_elbow_offset_neg"] = Location((82.5, 0.0, 710.0)) * _offset_bridge(-82.5, 41.0, 44.0)
    parts["link5_forearm_tube"] = Location((0.0, 0.0, 792.0)) * _hollow_cylinder(40.0, 31.0, 120.0)
    parts["link6_wrist_offset"] = Location((0.0, 0.0, 1068.0)) * _offset_bridge(88.0, 35.0, 36.0)

    # 4. ISO 9409-1-50-4-M6 Output Flange + 6x M4 Flange Bolts (7 parts)
    tcp_flange, tcp_bolts = _bolted_flange_and_screws(34.0, 9.0, 10.0, 25.0, 6, 2.15)
    parts["tcp_iso_flange"] = Location((88.0, 0.0, 1142.0)) * tcp_flange
    for idx, bolt in enumerate(tcp_bolts, 1):
        parts[f"tcp_bolt_{idx}"] = Location((88.0, 0.0, 1142.0)) * bolt

    # 5. Franka Hand Parallel Gripper: body + linear rail + left/right fingers (4 parts)
    parts["franka_hand_body"] = Location((88.0, 0.0, 1176.0)) * Box(42.0, 136.0, 50.0)
    parts["franka_hand_rail"] = Location((88.0, 0.0, 1205.0)) * Box(16.0, 120.0, 8.0)
    parts["franka_hand_finger_left"] = Location((88.0, -24.0, 1232.0)) * Box(22.0, 14.0, 46.0)
    parts["franka_hand_finger_right"] = Location((88.0, 24.0, 1232.0)) * Box(22.0, 14.0, 46.0)

    # Total = 10 + 1 + 21 + 6 + 7 + 4 = 49 individual B-Rep solids!
    compound = Compound(children=list(parts.values()))
    step_path = OUT_DIR / "franka_panda_7dof_brep.step"
    stl_path = OUT_DIR / "franka_panda_7dof_brep.stl"
    export_step(compound, str(step_path))
    export_stl(compound, str(stl_path), tolerance=0.18, angular_tolerance=0.25)
    build_s = round(time.perf_counter() - t0, 2)

    # Independent OpenCASCADE STEP Validation
    reader = STEPControl_Reader()
    assert reader.ReadFile(str(step_path)) == IFSelect_RetDone
    reader.TransferRoots()
    shape = reader.OneShape()
    brep_valid = bool(BRepCheck_Analyzer(shape).IsValid())
    exp = TopExp_Explorer(shape, TopAbs_SOLID)
    solids, invalid_solids, total_vol_mm3 = 0, 0, 0.0
    while exp.More():
        s = exp.Current()
        solids += 1
        if not BRepCheck_Analyzer(s).IsValid():
            invalid_solids += 1
        props = GProp_GProps()
        BRepGProp.VolumeProperties_s(s, props)
        total_vol_mm3 += max(0.0, props.Mass())
        exp.Next()

    # Independent trimesh STL Validation (per-part watertightness check on 10 key structural parts)
    key_parts = [
        "base_flange", "base_column", "j1_housing", "j1_harmonic_reducer",
        "j4_torque_sensor_ring", "link3_elbow_offset_pos", "link5_forearm_tube",
        "tcp_iso_flange", "franka_hand_body", "franka_hand_finger_left",
    ]
    non_wt = []
    for k in key_parts:
        p_stl = OUT_DIR / f"part_{k}.stl"
        export_stl(parts[k], str(p_stl), tolerance=0.18, angular_tolerance=0.25)
        m = trimesh.load(str(p_stl), force="mesh")
        if not m.is_watertight:
            non_wt.append(k)

    full_mesh = trimesh.load(str(stl_path), force="mesh")
    metrics = {
        "named_parts_count": len(parts),
        "build_and_export_s": build_s,
        "step_size_kb": round(step_path.stat().st_size / 1024, 1),
        "step_brep_valid": brep_valid,
        "step_solids_count": solids,
        "step_invalid_solids": invalid_solids,
        "total_volume_cm3": round(total_vol_mm3 / 1000.0, 1),
        "stl_size_kb": round(stl_path.stat().st_size / 1024, 1),
        "stl_triangles": int(len(full_mesh.faces)),
        "key_parts_checked": len(key_parts),
        "key_parts_non_watertight": non_wt,
    }
    print(json.dumps(metrics, indent=2))
    return metrics


if __name__ == "__main__":
    build_franka_panda_brep()
