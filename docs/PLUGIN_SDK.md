# RoboForge Studio Pro — SDK de Plugins de Familias 3D (`v13.0`)

Esta guía explica cómo crear, validar y registrar una **nueva familia de proyectos robóticos 3D** en RoboForge Studio Pro usando **un único archivo Python**, con soporte automático de renderizado PBR de estudio (`.GLB` con normales *smooth_shade* y UVs físicas), exportación `.STEP`, empaquetado de platos `.3MF`, lista de materiales (BOM) y auditoría **Goldfish DFM 5/5**.

---

## 1. Arquitectura de un Plugin (`ProjectFamilyPlugin`)

Todos los plugins viven en `backend/app/families/` y heredan de `ProjectFamilyPlugin` (`backend/app/families/base.py`):

| Método / Atributo | Propósito |
| :--- | :--- |
| `family_id: str` | Identificador único en minúsculas (ej. `"hexapod_walker"`, `"pan_tilt_turret"`). |
| `name: str`, `subtitle: str`, `icon: str` | Metadatos mostrados en `GET /api/projects/families` y en `ProjectsWorkspace.tsx`. |
| `get_presets() -> list[dict]` | Lista de presets en 1 clic con `id`, `name`, `subtitle` y `defaults`. |
| `generate(req, material, printer_profile, output_dir, design_hash) -> ProjectGenerateResponse` | Construye sólidos `build123d`, calcula física, genera BOM y delega el empaquetado a `package_project_exports(...)`. |

---

## 2. Plantilla Mínima en 1 Archivo (`backend/app/families/mi_familia.py`)

```python
from pathlib import Path
from typing import Any
from build123d import Box, Cylinder, Location

from ..schemas import MasterBomItem, ProjectGenerateRequest, ProjectGenerateResponse
from .base import ProjectFamilyPlugin, package_project_exports

MI_PRESETS = {
    "demo_v1": {
        "id": "demo_v1",
        "name": "Torreta Pan-Tilt 2-DOF",
        "subtitle": "Base + horquilla · Imprimible en 1 plato",
        "defaults": {"base_d_mm": 60.0, "height_mm": 45.0},
    }
}

class MiFamiliaPlugin(ProjectFamilyPlugin):
    family_id = "pan_tilt_turret"
    name = "Torreta Pan-Tilt"
    subtitle = "Mecanismo 2 ejes con servomotores"
    icon = "gimbal"

    def get_presets(self) -> list[dict[str, Any]]:
        return list(MI_PRESETS.values())

    def generate(
        self,
        req: ProjectGenerateRequest,
        material: dict[str, Any],
        printer_profile: dict[str, Any],
        output_dir: Path,
        design_hash: str,
    ) -> ProjectGenerateResponse:
        preset = MI_PRESETS.get(req.preset_id, MI_PRESETS["demo_v1"])
        cfg = {**preset["defaults"], **(req.params or {})}
        d = float(cfg["base_d_mm"])
        h = float(cfg["height_mm"])

        # 1. Sólidos build123d (separados sin penetración AABB)
        base_part = Location((0.0, 0.0, 5.0)) * Cylinder(radius=d / 2.0, height=10.0)
        yoke_part = Location((0.0, 0.0, 12.0 + h / 2.0)) * Box(d * 0.7, 16.0, h)

        # 2. BOM con paridad 1:1 para cada pieza imprimible (category="printed_cnc")
        bom = [
            MasterBomItem(
                category="printed_cnc", category_label="Piezas 3D",
                part_code="PT-01", part_name="turret_base", spec=f"Base Ø{d:.0f} mm",
                qty=1, unit_cost_usd=2.20, total_cost_usd=2.20,
            ),
            MasterBomItem(
                category="printed_cnc", category_label="Piezas 3D",
                part_code="PT-02", part_name="turret_yoke", spec=f"Horquilla {h:.0f} mm",
                qty=1, unit_cost_usd=2.80, total_cost_usd=2.80,
            ),
        ]

        # 3. Empaquetado automático: GLB PBR + STEP + STL + .3MF + Goldfish DFM 5/5 + ZIP
        return package_project_exports(
            family_id=self.family_id,
            preset_id=preset["id"],
            name=preset["name"],
            design_hash=design_hash,
            req=req,
            material=material,
            printer_profile=printer_profile,
            output_dir=output_dir,
            node_specs=[
                ("turret_base", base_part, [64, 78, 96, 255], 0.55, 0.34),
                ("turret_yoke", yoke_part, [56, 178, 198, 255], 0.52, 0.32),
            ],
            printable_parts={"turret_base": base_part, "turret_yoke": yoke_part},
            explode_vectors={"turret_base": [0.0, 0.0, -25.0], "turret_yoke": [0.0, 0.0, 35.0]},
            metrics={"total_mass_g": round((base_part.volume + yoke_part.volume) / 1000.0 * 1.25, 1)},
            bom=bom,
            notes=["Ejemplo mínimo compatible con Goldfish DFM 5/5."],
        )
```

---

## 3. Registro en `FAMILY_REGISTRY`

En `backend/app/families/__init__.py`, importa tu clase y añádela al diccionario `FAMILY_REGISTRY`:

```python
from .mi_familia import MiFamiliaPlugin

FAMILY_REGISTRY["pan_tilt_turret"] = MiFamiliaPlugin()
```

El router `backend/app/routers/projects.py` (`GET /api/projects/families` y `POST /api/projects/{family}/generate`) expondrá automáticamente la nueva familia sin tocar `main.py`.

---

## 4. Contrato Obligatorio Goldfish DFM 5/5

`package_project_exports()` ejecuta `audit_assembly_dfm()` sobre tus piezas. Para obtener **`dfm_audit.status == "pass"`**:
1. **Watertight:** todas las piezas en `printable_parts` deben ser sólidos cerrados sin bordes abiertos.
2. **Cero Islas Flotantes (`bodies == 1`):** cada entrada en `printable_parts` debe ser 1 solo sólido conectado (usa `Compound` solo en `node_specs` visuales para conjuntos de tornillos/ruedas).
3. **Ajuste a Cama (`fits_bed`):** cada pieza orientada plana sobre `Z = 0` debe caber en `(bed_x - 5 mm, bed_y - 5 mm, bed_z - 4 mm)`.
4. **Cero Colisiones AABB:** deja al menos `0.5–1.5 mm` de holgura entre sólidos adyacentes en `node_specs`.
5. **Paridad BOM ↔ STL:** incluye al menos 1 línea `MasterBomItem(category="printed_cnc", ...)` por cada pieza en `printable_parts`.

---

## 5. Prueba Automatizada Recomendada (`pytest`)

```python
from pathlib import Path
from app.families import compute_project_hash, get_family_plugin
from app.schemas import ProjectGenerateRequest

def test_mi_familia_passes_goldfish_dfm(tmp_path: Path):
    plugin = get_family_plugin("pan_tilt_turret")
    req = ProjectGenerateRequest(preset_id="demo_v1", material_id="petg_cf", printer_id="bambu_x1c")
    res = plugin.generate(
        req,
        {"id": "petg_cf", "name": "PETG-CF", "category": "fdm", "density_g_cm3": 1.29},
        {"name": "Bambu Lab X1C", "bed_x_mm": 256.0, "bed_y_mm": 256.0, "bed_z_mm": 256.0, "process_type": "fdm"},
        tmp_path,
        compute_project_hash("pan_tilt_turret", req),
    )
    assert res.dfm_audit.status == "pass"
```
