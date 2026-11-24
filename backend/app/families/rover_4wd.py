"""4WD / Mecanum Autonomous Rover Multi-Project Family Plugin."""

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
from .base import ProjectFamilyPlugin, package_project_exports

ROVER_PRESETS: dict[str, dict[str, Any]] = {
    "mini_rover_tt": {
        "id": "mini_rover_tt",
        "name": "Mini Rover 4WD (Motorreductores TT + ESP32)",
        "subtitle": "Chasis 165×115 mm · Ruedas Ø65 mm · Imprimible 100%",
        "defaults": {
            "length_mm": 165.0,
            "width_mm": 115.0,
            "wheel_d_mm": 65.0,
            "motor_torque_nm": 0.18,
            "motor_rpm": 140.0,
            "battery_wh": 22.0,
        },
    },
    "mecanum_nema17": {
        "id": "mecanum_nema17",
        "name": "Rover Omnidireccional Mecanum (NEMA 17)",
        "subtitle": "Chasis 215×155 mm · Ruedas Mecanum Ø80 mm · ROS 2",
        "defaults": {
            "length_mm": 215.0,
            "width_mm": 155.0,
            "wheel_d_mm": 80.0,
            "motor_torque_nm": 0.45,
            "motor_rpm": 180.0,
            "battery_wh": 44.0,
        },
    },
    "explorer_lidar": {
        "id": "explorer_lidar",
        "name": "Explorer SLAM 4WD (25GA Metal + Torreta LiDAR)",
        "subtitle": "Chasis 235×170 mm · RPLiDAR A1 · Jetson/RPi 5",
        "defaults": {
            "length_mm": 235.0,
            "width_mm": 170.0,
            "wheel_d_mm": 86.0,
            "motor_torque_nm": 0.65,
            "motor_rpm": 160.0,
            "battery_wh": 66.0,
        },
    },
}


class Rover4WDPlugin(ProjectFamilyPlugin):
    family_id = "rover_4wd"
    name = "Rovers 4WD & Mecanum"
    subtitle = "Chasis autónomos 2 cubiertas · Tracción 4WD + Torreta LiDAR SLAM"
    icon = "rover"

    def get_presets(self) -> list[dict[str, Any]]:
        return list(ROVER_PRESETS.values())

    def generate(
        self,
        req: ProjectGenerateRequest,
        material: dict[str, Any],
        printer_profile: dict[str, Any],
        output_dir: Path,
        design_hash: str,
    ) -> ProjectGenerateResponse:
        preset = ROVER_PRESETS.get(req.preset_id, ROVER_PRESETS["explorer_lidar"])
        cfg = {**preset["defaults"], **(req.params or {})}

        length = max(130.0, min(244.0, float(cfg.get("length_mm", 215.0))))
        width = max(95.0, min(195.0, float(cfg.get("width_mm", 155.0))))
        wheel_d = max(52.0, min(100.0, float(cfg.get("wheel_d_mm", 80.0))))
        wheel_r = wheel_d / 2.0
        wheel_w = 24.0 if wheel_d >= 75.0 else 18.0
        motor_torque_nm = float(cfg.get("motor_torque_nm", 0.45))
        motor_rpm = float(cfg.get("motor_rpm", 160.0))
        battery_wh = float(cfg.get("battery_wh", 44.0))

        deck_t = 5.0
        z_low = max(24.0, wheel_r - 12.0)
        deck_gap = 38.0
        z_up = z_low + deck_t + deck_gap

        so_x = length * 0.40
        so_y = width * 0.40
        standoff_pts = [(-so_x, -so_y), (-so_x, so_y), (so_x, -so_y), (so_x, so_y)]

        # 1. Lower Chassis Deck with 4 integrated motor mount walls & weight-relief cutouts
        with BuildSketch(Plane.XY) as low_sk:
            RectangleRounded(length, width, radius=10.0)
            with Locations(*standoff_pts):
                Circle(radius=1.7, mode=Mode.SUBTRACT)
            for cx in (-length * 0.22, length * 0.22):
                with Locations((cx, 0.0)):
                    RectangleRounded(length * 0.26, width * 0.42, radius=5.0, mode=Mode.SUBTRACT)
        with BuildPart() as low_bp:
            extrude(low_sk.sketch.faces()[0].moved(Location((0.0, 0.0, z_low))), amount=deck_t)
        chassis_lower = low_bp.part

        # Add 4 motor mount side ribs underneath lower deck (still single connected body!)
        axle_x = length * 0.30
        for sx in (-axle_x, axle_x):
            for sy in (-width / 2.0 + 6.0, width / 2.0 - 6.0):
                rib = Location((sx, sy, z_low - 8.0)) * Box(24.0, 8.0, 18.0)
                chassis_lower = chassis_lower + rib

        # 2. Upper Electronics Deck (ESP32 / Raspberry Pi / Battery bay)
        with BuildSketch(Plane.XY) as up_sk:
            RectangleRounded(length * 0.94, width * 0.92, radius=10.0)
            with Locations(*standoff_pts):
                Circle(radius=1.7, mode=Mode.SUBTRACT)
            with Locations((0.0, 0.0)):
                Circle(radius=16.0, mode=Mode.SUBTRACT)
        with BuildPart() as up_bp:
            extrude(up_sk.sketch.faces()[0].moved(Location((0.0, 0.0, z_up))), amount=deck_t)
        chassis_upper = up_bp.part

        # 3. 4x Deck Standoffs (non-overlapping Z between lower and upper decks)
        standoffs = Compound(
            children=[
                Location((sx, sy, z_low + deck_t + deck_gap / 2.0)) * Cylinder(radius=3.5, height=deck_gap - 0.8)
                for sx, sy in standoff_pts
            ]
        )

        # 4. LiDAR / Camera Turret Mount cleanly atop upper deck (z > z_up + deck_t + 0.5)
        turret_z = z_up + deck_t + 0.6
        turret_base = Location((length * 0.14, 0.0, turret_z + 10.0)) * Cylinder(radius=28.0, height=20.0)
        turret_ring = Location((length * 0.14, 0.0, turret_z + 22.0)) * (
            Cylinder(radius=31.0, height=6.0) - Cylinder(radius=21.0, height=10.0)
        )
        lidar_turret = turret_base + turret_ring

        # 5. Printable Wheel Hub (1 spare template for STL pack + 4 placed wheels cleanly outside width/2)
        with BuildSketch(Plane.XY) as w_sk:
            Circle(radius=wheel_r)
            Circle(radius=4.0, mode=Mode.SUBTRACT)
            for k in range(6):
                ka = math.radians(k * 60.0)
                with Locations((wheel_r * 0.55 * math.cos(ka), wheel_r * 0.55 * math.sin(ka))):
                    Circle(radius=wheel_r * 0.16, mode=Mode.SUBTRACT)
        with BuildPart() as w_bp:
            extrude(w_sk.sketch.faces()[0], amount=wheel_w)
        rover_wheel_raw = w_bp.part

        rot_wheel = Location((0.0, 0.0, 0.0), (90.0, 0.0, 0.0))
        wheel_y_off = width / 2.0 + wheel_w / 2.0 + 4.0
        wheel_solids = [
            Location((sx, sy, wheel_r)) * (rot_wheel * (Location((0.0, 0.0, -wheel_w / 2.0)) * rover_wheel_raw))
            for sx in (-axle_x, axle_x)
            for sy in (-wheel_y_off, wheel_y_off)
        ]
        wheels_4wd = Compound(children=wheel_solids)

        # Real 4WD Traction Physics
        density = float(material.get("density_g_cm3", 1.29))
        printed_g = (chassis_lower.volume + chassis_upper.volume + lidar_turret.volume + 4.0 * rover_wheel_raw.volume) / 1000.0 * density
        hardware_g = 420.0 if preset["id"] == "mini_rover_tt" else (1150.0 if preset["id"] == "mecanum_nema17" else 1480.0)
        total_mass_g = round(printed_g + hardware_g, 1)
        total_mass_kg = total_mass_g / 1000.0
        total_torque_nm = round(4.0 * motor_torque_nm, 2)
        tractive_force_n = total_torque_nm / max(wheel_r / 1000.0, 0.01)
        sin_slope = min(0.85, (tractive_force_n * 0.72) / max(total_mass_kg * 9.81, 0.1))
        max_climb_deg = round(math.degrees(math.asin(sin_slope)), 1)
        max_speed_m_s = round((motor_rpm / 60.0) * math.pi * (wheel_d / 1000.0), 2)
        runtime_min = round((battery_wh / max(8.0, total_mass_kg * 12.0)) * 60.0, 1)

        metrics = {
            "total_mass_g": total_mass_g,
            "total_torque_nm": total_torque_nm,
            "max_climb_deg": max_climb_deg,
            "max_speed_m_s": max_speed_m_s,
            "runtime_min": runtime_min,
            "wheelbase_mm": round(axle_x * 2.0, 1),
        }

        bom = [
            MasterBomItem(
                category="printed_cnc",
                category_label="Chasis Impreso 3D",
                part_code="ROV-LOW-01",
                part_name="chassis_lower",
                spec=f"Cubierta inferior {length:.0f}×{width:.0f} mm ({material.get('name', req.material_id)})",
                qty=1,
                unit_cost_usd=6.80,
                total_cost_usd=6.80,
            ),
            MasterBomItem(
                category="printed_cnc",
                category_label="Chasis Impreso 3D",
                part_code="ROV-TOP-02",
                part_name="chassis_upper",
                spec=f"Cubierta electrónica superior {length * 0.94:.0f}×{width * 0.92:.0f} mm",
                qty=1,
                unit_cost_usd=5.40,
                total_cost_usd=5.40,
            ),
            MasterBomItem(
                category="printed_cnc",
                category_label="Chasis Impreso 3D",
                part_code="ROV-LID-03",
                part_name="lidar_turret",
                spec="Torreta LiDAR SLAM / Cámara Ø62 mm",
                qty=1,
                unit_cost_usd=3.10,
                total_cost_usd=3.10,
            ),
            MasterBomItem(
                category="printed_cnc",
                category_label="Chasis Impreso 3D",
                part_code="ROV-WHL-04",
                part_name="rover_wheel",
                spec=f"Rueda imprimible Ø{wheel_d:.0f} mm × {wheel_w:.0f} mm (x4)",
                qty=4,
                unit_cost_usd=2.50,
                total_cost_usd=10.00,
            ),
            MasterBomItem(
                category="actuators_reducers",
                category_label="Tracción 4WD",
                part_code="ROV-MOT-05",
                part_name="Motores Tracción 4WD (x4)",
                spec=f"{motor_torque_nm:.2f} N·m · {motor_rpm:.0f} RPM c/u",
                qty=4,
                unit_cost_usd=14.00,
                total_cost_usd=56.00,
            ),
            MasterBomItem(
                category="electronics_wiring",
                category_label="Control & Navegación",
                part_code="ROV-MCU-06",
                part_name="Controlador ESP32 / RPi + Dual H-Bridge + Batería",
                spec=f"Pack Li-Ion {battery_wh:.0f} Wh + Driver 4 canales",
                qty=1,
                unit_cost_usd=58.00,
                total_cost_usd=58.00,
            ),
            MasterBomItem(
                category="fasteners_hardware",
                category_label="Tornillería",
                part_code="ROV-HW-07",
                part_name="Separadores M3×38mm + Tornillería M3",
                spec="4× Standoff latón M3 + 20× M3×10mm",
                qty=1,
                unit_cost_usd=5.20,
                total_cost_usd=5.20,
            ),
        ]

        node_specs = [
            ("chassis_lower", chassis_lower, [56, 72, 94, 255], 0.55, 0.34),
            ("chassis_upper", chassis_upper, [64, 156, 198, 255], 0.52, 0.34),
            ("standoffs", standoffs, [218, 174, 58, 255], 0.88, 0.24),
            ("lidar_turret", lidar_turret, [34, 40, 50, 255], 0.65, 0.32),
            ("wheels_4wd", wheels_4wd, [28, 32, 38, 255], 0.18, 0.78),
        ]
        printable_parts = {
            "chassis_lower": chassis_lower,
            "chassis_upper": chassis_upper,
            "lidar_turret": lidar_turret,
            "rover_wheel": rover_wheel_raw,
        }
        explode_vectors = {
            "chassis_lower": [0.0, 0.0, -28.0],
            "chassis_upper": [0.0, 0.0, 44.0],
            "standoffs": [0.0, 0.0, 18.0],
            "lidar_turret": [0.0, 0.0, 82.0],
            "wheels_4wd": [0.0, 0.0, -48.0],
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
                f"Tracción 4WD total: {total_torque_nm:.2f} N·m · Pendiente máxima estimada: {max_climb_deg:.1f}°.",
                f"Velocidad punta: {max_speed_m_s:.2f} m/s · Autonomía: {runtime_min:.0f} min.",
            ],
        )
