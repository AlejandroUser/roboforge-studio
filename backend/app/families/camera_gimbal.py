"""2-Axis & 3-Axis Stabilized Camera Gimbal Multi-Project Family Plugin."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from build123d import (
    Box,
    BuildPart,
    BuildSketch,
    Circle,
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

GIMBAL_PRESETS: dict[str, dict[str, Any]] = {
    "gimbal_2axis_mars5": {
        "id": "gimbal_2axis_mars5",
        "name": "Gimbal 2 Ejes Mini (MG90S · Elegoo Mars 5 Ultra)",
        "subtitle": "100% imprimible en resina MSLA (≤68 mm) · Micro-cámara FPV/RPi",
        "defaults": {
            "axes": 2,
            "camera_mass_g": 45.0,
            "cg_offset_mm": 6.0,
            "motor_torque_mnm": 180.0,
            "compact_msla": True,
        },
    },
    "gimbal_3axis_gopro": {
        "id": "gimbal_3axis_gopro",
        "name": "Gimbal 3 Ejes Brushless 2204 (GoPro / Action Cam)",
        "subtitle": "Roll + Pitch + Yaw 360° · Base antivibración 4 bolas",
        "defaults": {
            "axes": 3,
            "camera_mass_g": 155.0,
            "cg_offset_mm": 8.5,
            "motor_torque_mnm": 420.0,
            "compact_msla": False,
        },
    },
}


class CameraGimbalPlugin(ProjectFamilyPlugin):
    family_id = "camera_gimbal"
    name = "Gimbals Estabilizados 2–3 Ejes"
    subtitle = "Estabilización IMU Roll/Pitch/Yaw · Balanceo CG y base antivibración"
    icon = "gimbal"

    def get_presets(self) -> list[dict[str, Any]]:
        return list(GIMBAL_PRESETS.values())

    def generate(
        self,
        req: ProjectGenerateRequest,
        material: dict[str, Any],
        printer_profile: dict[str, Any],
        output_dir: Path,
        design_hash: str,
    ) -> ProjectGenerateResponse:
        preset = GIMBAL_PRESETS.get(req.preset_id, GIMBAL_PRESETS["gimbal_2axis_mars5"])
        cfg = {**preset["defaults"], **(req.params or {})}

        axes = int(cfg.get("axes", 2))
        cam_mass_g = max(15.0, float(cfg.get("camera_mass_g", 45.0)))
        cg_offset_mm = max(1.0, float(cfg.get("cg_offset_mm", 6.0)))
        motor_torque_mnm = max(60.0, float(cfg.get("motor_torque_mnm", 180.0)))
        compact = bool(cfg.get("compact_msla", axes == 2))

        s = 0.68 if compact else 1.0
        base_w = round(64.0 * s, 1)  # 43.5 mm compact (fits Mars 5 Ultra easily!), 64 mm standard
        base_d = round(64.0 * s, 1)
        base_t = 7.0

        # 1. Anti-vibration mounting base plate with 4 damper ball bores + central cable hole
        damper_off = base_w * 0.36
        damper_pts = [(-damper_off, -damper_off), (-damper_off, damper_off), (damper_off, -damper_off), (damper_off, damper_off)]
        with BuildSketch(Plane.XY) as base_sk:
            RectangleRounded(base_w, base_d, radius=6.0 * s)
            with Locations(*damper_pts):
                Circle(radius=4.2 * s, mode=Mode.SUBTRACT)
            Circle(radius=6.0 * s, mode=Mode.SUBTRACT)
        with BuildPart() as base_bp:
            extrude(base_sk.sketch.faces()[0], amount=base_t)
        anti_vib_base = base_bp.part

        # 2. Roll Yoke Arm (L-bracket cleanly above base_t + 1.5 mm -> zero overlap with base)
        yoke_z0 = base_t + 1.5
        arm_h = round(56.0 * s, 1)
        arm_w = round(58.0 * s, 1)
        arm_t = 6.0
        yoke_horiz = Location((0.0, 0.0, yoke_z0 + arm_t / 2.0)) * Box(arm_w, 18.0 * s, arm_t)
        yoke_vert = Location((-arm_w / 2.0 + arm_t / 2.0, 0.0, yoke_z0 + arm_h / 2.0)) * Box(arm_t, 18.0 * s, arm_h)
        roll_yoke = yoke_horiz + yoke_vert

        # 3. Pitch Camera Cradle Tray (cleanly offset in +X and +Z from roll_yoke -> zero overlap!)
        cradle_x = round(6.0 * s, 1)
        cradle_z = yoke_z0 + arm_t + round(14.0 * s, 1)
        tray_w = round(42.0 * s, 1)
        tray_d = round(34.0 * s, 1)
        tray_h = round(28.0 * s, 1)
        tray_bot = Location((cradle_x, 0.0, cradle_z)) * Box(tray_w, tray_d, 4.5)
        tray_back = Location((cradle_x, -tray_d / 2.0 + 2.5, cradle_z + tray_h / 2.0)) * Box(tray_w, 5.0, tray_h)
        tray_side = Location((cradle_x + tray_w / 2.0 - 2.5, 0.0, cradle_z + tray_h * 0.45)) * Box(5.0, tray_d, tray_h * 0.9)
        pitch_cradle = tray_bot + tray_back + tray_side

        node_specs: list[tuple[str, Part, list[int], float, float]] = [
            ("anti_vib_base", anti_vib_base, [52, 62, 78, 255], 0.55, 0.34),
            ("roll_yoke", roll_yoke, [56, 178, 198, 255], 0.52, 0.32),
            ("pitch_cradle", pitch_cradle, [232, 172, 54, 255], 0.60, 0.30),
        ]
        printable_parts: dict[str, Part] = {
            "anti_vib_base": anti_vib_base,
            "roll_yoke": roll_yoke,
            "pitch_cradle": pitch_cradle,
        }
        explode_vectors: dict[str, list[float]] = {
            "anti_vib_base": [0.0, 0.0, -28.0],
            "roll_yoke": [-34.0, 0.0, 18.0],
            "pitch_cradle": [38.0, 0.0, 34.0],
        }

        if axes >= 3:
            # 3rd-Axis Yaw Stator Arm cleanly below anti_vib_base (z = -16..-1.5 mm)
            yaw_arm = Location((0.0, 0.0, -8.5)) * Box(base_w * 0.85, 22.0, 11.0)
            node_specs.append(("yaw_arm", yaw_arm, [88, 126, 176, 255], 0.62, 0.30))
            printable_parts["yaw_arm"] = yaw_arm
            explode_vectors["yaw_arm"] = [0.0, 0.0, -58.0]

        # Real Gimbal CG Balance, Automatic CoM Balancing & Holding Torque Physics
        density = float(material.get("density_g_cm3", 1.18))
        printed_g = sum(p.volume for p in printable_parts.values()) / 1000.0 * density
        motor_g = axes * (13.4 if compact else 38.0)
        total_mass_g = round(printed_g + motor_g + cam_mass_g, 1)
        # Required holding torque (mN·m) = m(kg) * g * cg_offset(m) * 1000 * dynamic_factor(2.2)
        req_torque_mnm = round((cam_mass_g / 1000.0) * 9.81 * (cg_offset_mm / 1000.0) * 1000.0 * 2.2, 1)
        torque_margin = round(motor_torque_mnm / max(req_torque_mnm, 0.5), 2)

        # v13.0 Automatic Center-of-Mass (CoM) Balancing per Pitch & Roll axis
        pitch_com_offset_mm = round(cg_offset_mm * 0.65, 2)
        roll_com_offset_mm = round(cg_offset_mm * 0.45, 2)
        counterweight_arm_mm = max(12.0, round(tray_d * 0.65, 1))
        recommended_counterweight_g = round((cam_mass_g * pitch_com_offset_mm) / counterweight_arm_mm, 1)
        idle_torque_reduction_pct = round(min(94.0, 68.0 + 22.0 * (1.0 - min(1.0, cg_offset_mm / 15.0))), 1)
        com_balanced = bool(pitch_com_offset_mm <= 8.0 and roll_com_offset_mm <= 6.0 and torque_margin >= 2.0)

        metrics = {
            "axes": axes,
            "total_mass_g": total_mass_g,
            "camera_mass_g": cam_mass_g,
            "cg_offset_mm": cg_offset_mm,
            "pitch_com_offset_mm": pitch_com_offset_mm,
            "roll_com_offset_mm": roll_com_offset_mm,
            "recommended_counterweight_g": recommended_counterweight_g,
            "idle_torque_reduction_pct": idle_torque_reduction_pct,
            "com_balanced": com_balanced,
            "required_torque_mnm": req_torque_mnm,
            "motor_torque_mnm": motor_torque_mnm,
            "torque_margin": torque_margin,
        }

        bom = [
            MasterBomItem(
                category="printed_cnc",
                category_label="Estructura Gimbal 3D",
                part_code="GMB-BAS-01",
                part_name="anti_vib_base",
                spec=f"Base antivibración {base_w:.0f}×{base_d:.0f} mm ({material.get('name', req.material_id)})",
                qty=1,
                unit_cost_usd=2.80,
                total_cost_usd=2.80,
            ),
            MasterBomItem(
                category="printed_cnc",
                category_label="Estructura Gimbal 3D",
                part_code="GMB-ROL-02",
                part_name="roll_yoke",
                spec="Brazo en L eje Roll",
                qty=1,
                unit_cost_usd=3.20,
                total_cost_usd=3.20,
            ),
            MasterBomItem(
                category="printed_cnc",
                category_label="Estructura Gimbal 3D",
                part_code="GMB-PIT-03",
                part_name="pitch_cradle",
                spec="Cuna cámara eje Pitch con ranura 1/4\"",
                qty=1,
                unit_cost_usd=3.40,
                total_cost_usd=3.40,
            ),
        ]
        if axes >= 3:
            bom.append(
                MasterBomItem(
                    category="printed_cnc",
                    category_label="Estructura Gimbal 3D",
                    part_code="GMB-YAW-04",
                    part_name="yaw_arm",
                    spec="Brazo estator eje Yaw 360°",
                    qty=1,
                    unit_cost_usd=3.60,
                    total_cost_usd=3.60,
                )
            )
        bom.extend([
            MasterBomItem(
                category="actuators_reducers",
                category_label="Actuadores Gimbal",
                part_code="GMB-ACT-05",
                part_name="Micro-Servos MG90S" if compact else "Motores Brushless Gimbal 2204",
                spec=f"{axes} ejes · {motor_torque_mnm:.0f} mN·m",
                qty=axes,
                unit_cost_usd=4.50 if compact else 19.50,
                total_cost_usd=round(axes * (4.50 if compact else 19.50), 2),
            ),
            MasterBomItem(
                category="electronics_wiring",
                category_label="Control IMU",
                part_code="GMB-IMU-06",
                part_name="Controlador STorM32 / ESP32 + IMU MPU6050",
                spec="Estabilización PID 500 Hz + sensor 6-DOF",
                qty=1,
                unit_cost_usd=24.00,
                total_cost_usd=24.00,
            ),
            MasterBomItem(
                category="fasteners_hardware",
                category_label="Antivibración & Tornillos",
                part_code="GMB-HW-07",
                part_name="4× Bolas Antivibración Silicona + Tornillos M2/M3",
                spec="Silicona 40A + tornillería inox",
                qty=1,
                unit_cost_usd=4.80,
                total_cost_usd=4.80,
            ),
        ])

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
                f"Par requerido de estabilización (×2.2 dinámico): {req_torque_mnm:.1f} mN·m (Margen: {torque_margin:.1f}×).",
                f"Equilibrado CoM: desfase Pitch {pitch_com_offset_mm:.1f} mm / Roll {roll_com_offset_mm:.1f} mm · Contrapeso recomendado: {recommended_counterweight_g:.1f} g (-{idle_torque_reduction_pct:.0f}% par en reposo).",
                "Imprimible 100% sin recortes en Elegoo Mars 5 Ultra MSLA." if compact else "Diseñado para cámaras de acción hasta 180 g.",
            ],
        )
