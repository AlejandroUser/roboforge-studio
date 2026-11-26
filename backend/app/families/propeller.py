"""Parametric Drone Propeller Family Plugin (BEMT + Conventional & Toroidal NURBS CAD)."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from build123d import Location

from ..cad.propeller_gen import generate_propeller_part
from ..physics.propeller_bemt import solve_propeller_bemt
from ..schemas import MasterBomItem, ProjectGenerateRequest, ProjectGenerateResponse
from .base import ProjectFamilyPlugin, package_project_exports

PROPELLER_PRESETS: dict[str, dict[str, Any]] = {
    "fpv_5043_tri": {
        "id": "fpv_5043_tri",
        "name": 'FPV Freestyle 5043 × 3 Palas (NACA 4412)',
        "subtitle": '5.0" · Paso 4.3" · 18,000 RPM · Par CW + CCW',
        "defaults": {
            "diameter_inch": 5.0,
            "pitch_inch": 4.3,
            "blade_count": 3,
            "rpm": 18000.0,
            "airfoil_id": "naca4412",
            "prop_style": "conventional",
            "shaft_diam_mm": 5.0,
            "chord_scale": 1.0,
            "sweep_deg": 4.0,
            "rake_deg": 0.0,
        },
    },
    "toroidal_5040_quiet": {
        "id": "toroidal_5040_quiet",
        "name": 'Toroidal Silenciosa 5040 × 3 Lóbulos (Clark-Y)',
        "subtitle": '5.0" · Paso 4.0" · Lazo cerrado MIT (-6.5 dBA) · Par CW + CCW',
        "defaults": {
            "diameter_inch": 5.0,
            "pitch_inch": 4.0,
            "blade_count": 3,
            "rpm": 15000.0,
            "airfoil_id": "clark_y",
            "prop_style": "toroidal",
            "shaft_diam_mm": 5.0,
            "chord_scale": 1.0,
            "sweep_deg": 0.0,
            "rake_deg": 0.0,
        },
    },
    "longrange_7035_bi": {
        "id": "longrange_7035_bi",
        "name": 'Long-Range 7035 × 2 Palas (Eppler E63)',
        "subtitle": '7.0" · Paso 3.5" · Alta eficiencia g/W · 11,000 RPM',
        "defaults": {
            "diameter_inch": 7.0,
            "pitch_inch": 3.5,
            "blade_count": 2,
            "rpm": 11000.0,
            "airfoil_id": "eppler_e63",
            "prop_style": "conventional",
            "shaft_diam_mm": 5.0,
            "chord_scale": 0.95,
            "sweep_deg": 6.0,
            "rake_deg": 2.0,
        },
    },
    "cinewhoop_3028_quad": {
        "id": "cinewhoop_3028_quad",
        "name": 'Cinewhoop 3028 × 4 Palas (Clark-Y)',
        "subtitle": '3.0" · Paso 2.8" · Empuje compacto · 24,000 RPM',
        "defaults": {
            "diameter_inch": 3.0,
            "pitch_inch": 2.8,
            "blade_count": 4,
            "rpm": 24000.0,
            "airfoil_id": "clark_y",
            "prop_style": "conventional",
            "shaft_diam_mm": 5.0,
            "chord_scale": 1.05,
            "sweep_deg": 0.0,
            "rake_deg": 0.0,
        },
    },
}


class PropellerPlugin(ProjectFamilyPlugin):
    family_id = "propeller"
    name = "Hélices de Dron (BEMT + Toroidal)"
    subtitle = "Perfiles NACA/Clark-Y/Eppler · Loft NURBS · Pares CW/CCW (.STEP/.3MF)"
    icon = "fan"

    def get_presets(self) -> list[dict[str, Any]]:
        return list(PROPELLER_PRESETS.values())

    def generate(
        self,
        req: ProjectGenerateRequest,
        material: dict[str, Any],
        printer_profile: dict[str, Any],
        output_dir: Path,
        design_hash: str,
    ) -> ProjectGenerateResponse:
        preset = PROPELLER_PRESETS.get(req.preset_id, PROPELLER_PRESETS["fpv_5043_tri"])
        cfg = {**preset["defaults"], **(req.params or {})}

        diam_in = max(2.0, min(12.0, float(cfg.get("diameter_inch", 5.0))))
        pitch_in = max(1.5, min(9.0, float(cfg.get("pitch_inch", 4.3))))
        blades = max(2, min(4, int(cfg.get("blade_count", 3))))
        rpm = max(3000.0, min(40000.0, float(cfg.get("rpm", 18000.0))))
        airfoil_id = str(cfg.get("airfoil_id", "naca4412"))
        prop_style = str(cfg.get("prop_style", "conventional"))
        shaft_d = max(1.5, min(8.0, float(cfg.get("shaft_diam_mm", 5.0))))
        chord_scale = max(0.7, min(1.4, float(cfg.get("chord_scale", 1.0))))
        sweep_deg = max(-15.0, min(20.0, float(cfg.get("sweep_deg", 0.0))))
        rake_deg = max(-10.0, min(12.0, float(cfg.get("rake_deg", 0.0))))

        density = float(material.get("density_g_cm3", 1.25))
        yield_z = float(material.get("yield_z_mpa", material.get("yield_mpa", 38.0)))

        # 1. Solve real BEMT aerodynamics & root structural stress
        bemt = solve_propeller_bemt(
            diameter_inch=diam_in,
            pitch_inch=pitch_in,
            blade_count=blades,
            rpm=rpm,
            airfoil_id=airfoil_id,
            prop_style=prop_style,
            material_density_g_cm3=density,
            material_yield_z_mpa=yield_z,
        )

        # 2. Generate matched CCW & CW B-Rep propeller solids
        prop_ccw_raw = generate_propeller_part(
            diameter_inch=diam_in,
            pitch_inch=pitch_in,
            blade_count=blades,
            airfoil_id=airfoil_id,
            prop_style=prop_style,
            direction="ccw",
            shaft_diam_mm=shaft_d,
            chord_scale=chord_scale,
            sweep_deg=sweep_deg,
            rake_deg=rake_deg,
        )
        prop_cw_raw = generate_propeller_part(
            diameter_inch=diam_in,
            pitch_inch=pitch_in,
            blade_count=blades,
            airfoil_id=airfoil_id,
            prop_style=prop_style,
            direction="cw",
            shaft_diam_mm=shaft_d,
            chord_scale=chord_scale,
            sweep_deg=sweep_deg,
            rake_deg=rake_deg,
        )

        # Side-by-side presentation offset (no overlap in 3D viewport)
        sep_x = (diam_in * 25.4) * 0.56
        prop_ccw_placed = Location((-sep_x, 0.0, 0.0)) * prop_ccw_raw
        prop_cw_placed = Location((+sep_x, 0.0, 0.0)) * prop_cw_raw

        real_mass_g = round((prop_ccw_raw.volume / 1000.0) * density, 2)
        metrics = bemt.to_metrics_dict()
        metrics["cad_volume_mm3"] = round(float(prop_ccw_raw.volume), 1)
        metrics["cad_mass_g"] = real_mass_g

        style_label = "Toroidal (Lazo Cerrado)" if bemt.prop_style == "toroidal" else "Convencional NURBS"
        bom = [
            MasterBomItem(
                category="printed_cnc",
                category_label="Hélices Impresas / CNC",
                part_code="PROP-CCW-01",
                part_name="propeller_ccw",
                spec=f'Hélice CCW {diam_in:.1f}×{pitch_in:.1f}" · {blades}p · {style_label} ({airfoil_id.upper()})',
                qty=2,
                unit_cost_usd=0.85,
                total_cost_usd=1.70,
            ),
            MasterBomItem(
                category="printed_cnc",
                category_label="Hélices Impresas / CNC",
                part_code="PROP-CW-02",
                part_name="propeller_cw",
                spec=f'Hélice CW {diam_in:.1f}×{pitch_in:.1f}" · {blades}p · {style_label} ({airfoil_id.upper()})',
                qty=2,
                unit_cost_usd=0.85,
                total_cost_usd=1.70,
            ),
            MasterBomItem(
                category="fasteners_hardware",
                category_label="Fijación Eje Motor",
                part_code="PROP-NUT-03",
                part_name="Tuercas Autoblocantes M5 Nylon (x4)",
                spec=f"Eje Ø{shaft_d:.1f} mm · 2× rosca normal + 2× brida",
                qty=4,
                unit_cost_usd=0.35,
                total_cost_usd=1.40,
            ),
        ]

        node_specs = [
            ("propeller_ccw", prop_ccw_placed, [14, 165, 233, 255], 0.35, 0.38),
            ("propeller_cw", prop_cw_placed, [249, 115, 22, 255], 0.35, 0.38),
        ]
        printable_parts = {
            "propeller_ccw": prop_ccw_raw,
            "propeller_cw": prop_cw_raw,
        }
        explode_vectors = {
            "propeller_ccw": [-24.0, 0.0, 12.0],
            "propeller_cw": [24.0, 0.0, 12.0],
        }

        notes = [
            f"BEMT ({style_label} · {airfoil_id.upper()}): {bemt.thrust_g:.0f} g empuje @ {rpm:.0f} RPM ({bemt.power_w:.0f} W · {bemt.efficiency_g_per_w:.2f} g/W · FM {bemt.figure_of_merit:.2f}).",
            f"Estructural raíz: F_centrífuga = {bemt.centrifugal_force_n:.1f} N · σ = {bemt.root_stress_mpa:.1f} MPa · FS = {bemt.root_safety_factor:.2f} · Ruido est. {bemt.estimated_noise_dba:.1f} dBA.",
            *bemt.warnings,
        ]

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
            notes=notes,
        )
