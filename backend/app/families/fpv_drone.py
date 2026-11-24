"""FPV Drone Multi-Project Family Plugin (Cinewhoop 3", Freestyle 5", Long-Range 7")."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

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
    extrude,
)

from ..schemas import MasterBomItem, ProjectGenerateRequest, ProjectGenerateResponse
from .base import ProjectFamilyPlugin, package_project_exports, write_simple_dxf_outline

FPV_PRESETS: dict[str, dict[str, Any]] = {
    "cinewhoop_3in": {
        "id": "cinewhoop_3in",
        "name": 'Cinewhoop 3" Ductless (20×20)',
        "subtitle": "Compacto 148 mm · Motores 1404 · Batería 4S 850 mAh",
        "defaults": {
            "prop_inch": 3.0,
            "wheelbase_mm": 148.0,
            "stack_mm": 20.0,
            "motor_pcd_mm": 12.0,
            "plate_thickness_mm": 3.0,
            "battery_mah": 850.0,
            "motor_thrust_g": 340.0,
        },
    },
    "freestyle_5in": {
        "id": "freestyle_5in",
        "name": 'Freestyle 5" True-X (30.5×30.5)',
        "subtitle": "Estándar 215 mm · Motores 2207 · Batería 6S 1300 mAh",
        "defaults": {
            "prop_inch": 5.0,
            "wheelbase_mm": 215.0,
            "stack_mm": 30.5,
            "motor_pcd_mm": 16.0,
            "plate_thickness_mm": 4.0,
            "battery_mah": 1300.0,
            "motor_thrust_g": 1150.0,
        },
    },
    "longrange_7in": {
        "id": "longrange_7in",
        "name": 'Long-Range 7" Cruiser (30.5×30.5)',
        "subtitle": "Eficiencia 240 mm · Motores 2806.5 · Li-Ion 4000 mAh",
        "defaults": {
            "prop_inch": 7.0,
            "wheelbase_mm": 240.0,
            "stack_mm": 30.5,
            "motor_pcd_mm": 19.0,
            "plate_thickness_mm": 5.0,
            "battery_mah": 4000.0,
            "motor_thrust_g": 1650.0,
        },
    },
}


class FpvDronePlugin(ProjectFamilyPlugin):
    family_id = "fpv_drone"
    name = "Drones FPV & Cinewhoop"
    subtitle = "Chasis True-X paramétricos · CNC Carbono (.DXF) + Impresión 3D"
    icon = "drone"

    def get_presets(self) -> list[dict[str, Any]]:
        return list(FPV_PRESETS.values())

    def generate(
        self,
        req: ProjectGenerateRequest,
        material: dict[str, Any],
        printer_profile: dict[str, Any],
        output_dir: Path,
        design_hash: str,
    ) -> ProjectGenerateResponse:
        preset = FPV_PRESETS.get(req.preset_id, FPV_PRESETS["freestyle_5in"])
        cfg = {**preset["defaults"], **(req.params or {})}

        prop_inch = float(cfg.get("prop_inch", 5.0))
        wheelbase = max(120.0, min(255.0, float(cfg.get("wheelbase_mm", 215.0))))
        stack_mm = float(cfg.get("stack_mm", 30.5))
        motor_pcd = float(cfg.get("motor_pcd_mm", 16.0))
        plate_t = max(2.5, min(6.0, float(cfg.get("plate_thickness_mm", 4.0))))
        batt_mah = float(cfg.get("battery_mah", 1300.0))
        motor_thrust_g = float(cfg.get("motor_thrust_g", 1150.0))

        arm_r = wheelbase / 2.0
        arm_w = max(12.0, min(20.0, prop_inch * 3.0))
        hub_l = max(56.0, stack_mm + 34.0)
        hub_w = max(36.0, stack_mm + 12.0)
        stack_h = 26.0
        top_t = max(2.0, plate_t * 0.65)
        pad_r = max(11.5, motor_pcd * 0.72)

        # 1. Bottom Frame Plate with 4 True-X arms, motor pads, motor bolt pattern, and FC stack holes
        dxf_circles: list[tuple[float, float, float]] = []
        motor_centers: list[tuple[float, float]] = []
        with BuildSketch(Plane.XY) as bot_sk:
            RectangleRounded(hub_l, hub_w, radius=6.0)
            for ang_deg in (45.0, 135.0, 225.0, 315.0):
                ang = math.radians(ang_deg)
                mx = arm_r * math.cos(ang)
                my = arm_r * math.sin(ang)
                motor_centers.append((mx, my))
                with Locations((mx * 0.5, my * 0.5, 0.0)):
                    RectangleRounded(arm_r, arm_w, radius=3.0, rotation=ang_deg, mode=Mode.ADD)
                with Locations((mx, my)):
                    Circle(radius=pad_r, mode=Mode.ADD)
            # Subtract central FC stack 4xM3 holes
            s_half = stack_mm / 2.0
            for sx in (-s_half, s_half):
                for sy in (-s_half, s_half):
                    with Locations((sx, sy)):
                        Circle(radius=1.65, mode=Mode.SUBTRACT)
                    dxf_circles.append((sx, sy, 1.65))
            # Subtract 4x motor mounting holes + center shaft relief on each motor pad
            m_r = motor_pcd / 2.0
            for mx, my in motor_centers:
                with Locations((mx, my)):
                    Circle(radius=3.5, mode=Mode.SUBTRACT)
                dxf_circles.append((mx, my, 3.5))
                for k in range(4):
                    ka = math.radians(45.0 + k * 90.0)
                    hx = mx + m_r * math.cos(ka)
                    hy = my + m_r * math.sin(ka)
                    with Locations((hx, hy)):
                        Circle(radius=1.6, mode=Mode.SUBTRACT)
                    dxf_circles.append((hx, hy, 1.6))

        with BuildPart() as bot_bp:
            extrude(bot_sk.sketch.faces()[0], amount=plate_t)
        bottom_plate = bot_bp.part

        # 2. Top Battery Plate (cleanly above standoffs)
        so_x = hub_l * 0.40
        so_y = hub_w * 0.36
        standoff_pts = [(-so_x, -so_y), (-so_x, so_y), (so_x, -so_y), (so_x, so_y)]
        with BuildSketch(Plane.XY) as top_sk:
            RectangleRounded(hub_l * 1.15, hub_w * 0.88, radius=5.0)
            with Locations(*standoff_pts):
                Circle(radius=1.65, mode=Mode.SUBTRACT)
            for bx in (-14.0, 14.0):
                with Locations((bx, 0.0)):
                    RectangleRounded(4.5, hub_w * 0.56, radius=1.5, mode=Mode.SUBTRACT)
        with BuildPart() as top_bp:
            extrude(top_sk.sketch.faces()[0].moved(Location((0.0, 0.0, plate_t + stack_h))), amount=top_t)
        top_plate = top_bp.part

        # 3. 4x Aluminum Standoffs (non-overlapping Z = plate_t + 0.3 .. plate_t + stack_h - 0.3)
        standoff_solids = [
            Location((sx, sy, plate_t + stack_h / 2.0)) * Cylinder(radius=2.6, height=stack_h - 0.6)
            for sx, sy in standoff_pts
        ]
        standoffs = Compound(children=standoff_solids)

        # 4. Front FPV Camera Mount Cradle (mounted cleanly forward of top/bottom hub)
        cam_x = hub_l * 0.5 + 11.0
        cam_z = plate_t + stack_h * 0.5
        cam_base = Location((cam_x, 0.0, cam_z - 9.0)) * Box(16.0, 26.0, 4.0)
        cam_l = Location((cam_x, -11.5, cam_z)) * Box(16.0, 3.0, 20.0)
        cam_r = Location((cam_x, +11.5, cam_z)) * Box(16.0, 3.0, 20.0)
        camera_mount = cam_base + cam_l + cam_r

        # 5. Visual Brushless Motors seated cleanly on top of motor pads
        motor_h = 15.0 if prop_inch >= 5.0 else 10.5
        motor_r = 13.5 if prop_inch >= 5.0 else 9.0
        motor_solids = [
            Location((mx, my, plate_t + 0.6 + motor_h / 2.0)) * Cylinder(radius=motor_r, height=motor_h)
            for mx, my in motor_centers
        ]
        brushless_motors = Compound(children=motor_solids)

        # CNC DXF export
        dxf_rects = [(-hub_l / 2.0, -hub_w / 2.0, hub_l / 2.0, hub_w / 2.0)]
        dxf_path = write_simple_dxf_outline(dxf_circles, dxf_rects, output_dir / "fpv_bottom_plate.dxf")

        # Real FPV Flight Physics
        density = float(material.get("density_g_cm3", 1.35))
        frame_mass_g = max(25.0, (bottom_plate.volume + top_plate.volume + camera_mount.volume) / 1000.0 * density)
        electronics_g = 48.0 if prop_inch <= 3.5 else (95.0 if prop_inch <= 5.5 else 130.0)
        motors_g = 4.0 * (12.0 if prop_inch <= 3.5 else (33.0 if prop_inch <= 5.5 else 52.0))
        batt_g = batt_mah * (0.11 if prop_inch <= 3.5 else 0.165)
        auw_g = round(frame_mass_g + electronics_g + motors_g + batt_g, 1)
        total_thrust_g = round(4.0 * motor_thrust_g, 1)
        twr = round(total_thrust_g / max(auw_g, 1.0), 2)
        hover_throttle_pct = round(min(95.0, (auw_g / max(total_thrust_g, 1.0)) * 100.0), 1)
        # Approximate cruise draw (A) -> flight time (min)
        cruise_a = max(4.0, (auw_g / 1000.0) * 18.0)
        flight_time_min = round((batt_mah / 1000.0) * 0.80 / cruise_a * 60.0, 1)

        metrics = {
            "auw_mass_g": auw_g,
            "frame_mass_g": round(frame_mass_g, 1),
            "total_thrust_g": total_thrust_g,
            "twr": twr,
            "hover_throttle_pct": hover_throttle_pct,
            "flight_time_min": flight_time_min,
            "wheelbase_mm": round(wheelbase, 1),
            "prop_inch": prop_inch,
        }

        bom = [
            MasterBomItem(
                category="printed_cnc",
                category_label="Chasis CNC / 3D",
                part_code="FPV-BOT-01",
                part_name="bottom_plate",
                spec=f"Placa inferior True-X {wheelbase:.0f}mm · {plate_t:.1f}mm ({material.get('name', req.material_id)})",
                qty=1,
                unit_cost_usd=14.50,
                total_cost_usd=14.50,
            ),
            MasterBomItem(
                category="printed_cnc",
                category_label="Chasis CNC / 3D",
                part_code="FPV-TOP-02",
                part_name="top_plate",
                spec=f"Placa superior batería {top_t:.1f}mm",
                qty=1,
                unit_cost_usd=6.50,
                total_cost_usd=6.50,
            ),
            MasterBomItem(
                category="printed_cnc",
                category_label="Chasis CNC / 3D",
                part_code="FPV-CAM-03",
                part_name="camera_mount",
                spec="Soporte cámara FPV 19mm (TPU/PETG-CF)",
                qty=1,
                unit_cost_usd=3.20,
                total_cost_usd=3.20,
            ),
            MasterBomItem(
                category="actuators_reducers",
                category_label="Propulsión Brushless",
                part_code="FPV-MOT-04",
                part_name="Motores Brushless FPV (x4)",
                spec=f"Patrón {motor_pcd:.0f}×{motor_pcd:.0f} mm · {motor_thrust_g:.0f}g empuje c/u",
                qty=4,
                unit_cost_usd=18.00,
                total_cost_usd=72.00,
            ),
            MasterBomItem(
                category="electronics_wiring",
                category_label="Electrónica FPV",
                part_code="FPV-STK-05",
                part_name="Stack FC F7 + ESC 4-en-1",
                spec=f"Montaje {stack_mm:.1f}×{stack_mm:.1f} mm · 45A BLHeli32",
                qty=1,
                unit_cost_usd=68.00,
                total_cost_usd=68.00,
            ),
            MasterBomItem(
                category="fasteners_hardware",
                category_label="Tornillería & Standoffs",
                part_code="FPV-SO-06",
                part_name="Kit Standoffs M3×26mm + Tornillos M3",
                spec="4× Standoff Alu 7075 + 24× M3×8mm",
                qty=1,
                unit_cost_usd=6.80,
                total_cost_usd=6.80,
            ),
        ]

        node_specs = [
            ("bottom_plate", bottom_plate, [42, 48, 56, 255], 0.45, 0.36),
            ("top_plate", top_plate, [52, 60, 72, 255], 0.45, 0.36),
            ("standoffs", standoffs, [220, 48, 56, 255], 0.88, 0.24),
            ("camera_mount", camera_mount, [14, 165, 233, 255], 0.28, 0.52),
            ("brushless_motors", brushless_motors, [196, 206, 218, 255], 0.90, 0.20),
        ]
        printable_parts = {
            "bottom_plate": bottom_plate,
            "top_plate": top_plate,
            "camera_mount": camera_mount,
        }
        explode_vectors = {
            "bottom_plate": [0.0, 0.0, -32.0],
            "top_plate": [0.0, 0.0, 56.0],
            "standoffs": [0.0, 0.0, 22.0],
            "camera_mount": [48.0, 0.0, 12.0],
            "brushless_motors": [0.0, 0.0, 34.0],
        }

        return package_project_exports(
            family_id=self.family_id,
            preset_id=preset["id"],
            name=preset["name"],
            design_hash=design_hash,
            req=req,
            material=material,
            printer_profile=printer_profile,
            output_dir=output_dir,
            node_specs=node_specs,
            printable_parts=printable_parts,
            explode_vectors=explode_vectors,
            metrics=metrics,
            bom=bom,
            notes=[
                f"Empuje/Peso (TWR): {twr:.2f}:1 ({total_thrust_g:.0f} g empuje / {auw_g:.0f} g AUW).",
                f"Autonomía estimada crucero: {flight_time_min:.1f} min con batería {batt_mah:.0f} mAh.",
            ],
            dxf_path=dxf_path,
        )
