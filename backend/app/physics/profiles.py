from __future__ import annotations

import copy
import json
import threading
from functools import lru_cache
from pathlib import Path

from ..schemas import (
    AlertItem,
    CustomMaterialCreate,
    PrinterProfile,
    PrinterProfileCreate,
)

DATA_DIR = Path(__file__).resolve().parent.parent / "data"
USER_PROFILES_PATH = DATA_DIR / "user_profiles.json"

_PROFILE_LOCK = threading.Lock()
_ENCLOSURE_REQUIRED_IDS = {"abs_asa", "pa12_cf"}


@lru_cache(maxsize=1)
def _read_builtin_printers_raw() -> dict[str, dict]:
    path = DATA_DIR / "printers.json"
    with path.open("r", encoding="utf-8") as fh:
        entries = json.load(fh)
    result: dict[str, dict] = {}
    for entry in entries:
        item = dict(entry)
        item["is_custom"] = False
        result[item["id"]] = item
    return result


@lru_cache(maxsize=1)
def _read_builtin_material_ids() -> set[str]:
    path = DATA_DIR / "materials.json"
    with path.open("r", encoding="utf-8") as fh:
        entries = json.load(fh)
    return {entry["id"] for entry in entries}


def _read_user_profiles() -> dict[str, dict[str, dict]]:
    if not USER_PROFILES_PATH.exists():
        return {"printers": {}, "materials": {}}
    try:
        with USER_PROFILES_PATH.open("r", encoding="utf-8") as fh:
            data = json.load(fh)
        if not isinstance(data, dict):
            return {"printers": {}, "materials": {}}
        return {
            "printers": dict(data.get("printers", {})),
            "materials": dict(data.get("materials", {})),
        }
    except Exception:
        return {"printers": {}, "materials": {}}


def _write_user_profiles(data: dict[str, dict[str, dict]]) -> None:
    USER_PROFILES_PATH.parent.mkdir(parents=True, exist_ok=True)
    with USER_PROFILES_PATH.open("w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=2, ensure_ascii=False)


def load_printers() -> dict[str, dict]:
    """Load built-in printers merged with user-defined custom printers."""
    merged = copy.deepcopy(_read_builtin_printers_raw())
    with _PROFILE_LOCK:
        user_data = _read_user_profiles()
    for pid, pdata in user_data["printers"].items():
        item = dict(pdata)
        item["is_custom"] = True
        merged[pid] = item
    return merged


def save_custom_printer(profile: PrinterProfileCreate) -> PrinterProfile:
    """Persist a custom 3D printer profile in user_profiles.json."""
    if profile.id in _read_builtin_printers_raw():
        raise ValueError(
            f"El identificador '{profile.id}' pertenece a una impresora predefinida; usa otro ID."
        )
    full = PrinterProfile(**profile.model_dump(), is_custom=True)
    with _PROFILE_LOCK:
        user_data = _read_user_profiles()
        user_data["printers"][full.id] = full.model_dump()
        _write_user_profiles(user_data)
    return full


def delete_custom_printer(printer_id: str) -> bool:
    """Delete a custom printer profile. Returns False if built-in or not found."""
    if printer_id in _read_builtin_printers_raw():
        return False
    with _PROFILE_LOCK:
        user_data = _read_user_profiles()
        if printer_id not in user_data["printers"]:
            return False
        del user_data["printers"][printer_id]
        _write_user_profiles(user_data)
    return True


def load_custom_materials() -> dict[str, dict]:
    """Load only user-defined custom materials from user_profiles.json."""
    with _PROFILE_LOCK:
        user_data = _read_user_profiles()
    out: dict[str, dict] = {}
    for mid, mdata in user_data["materials"].items():
        item = dict(mdata)
        item["is_custom"] = True
        out[mid] = item
    return out


def save_custom_material(mat: CustomMaterialCreate) -> dict:
    """Persist a custom engineering material in user_profiles.json."""
    if mat.id in _read_builtin_material_ids():
        raise ValueError(
            f"El identificador '{mat.id}' pertenece a un material predefinido; usa otro ID."
        )
    item = mat.model_dump()
    item["is_custom"] = True
    with _PROFILE_LOCK:
        user_data = _read_user_profiles()
        user_data["materials"][mat.id] = item
        _write_user_profiles(user_data)
    return item


def delete_custom_material(material_id: str) -> bool:
    """Delete a custom material. Returns False if built-in or not found."""
    if material_id in _read_builtin_material_ids():
        return False
    with _PROFILE_LOCK:
        user_data = _read_user_profiles()
        if material_id not in user_data["materials"]:
            return False
        del user_data["materials"][material_id]
        _write_user_profiles(user_data)
    return True


def evaluate_printer_dfm_alerts(
    printer_id: str | None,
    material: dict,
    part_x_mm: float,
    part_y_mm: float,
    part_z_mm: float,
) -> list[AlertItem]:
    """
    Evaluate build-volume and enclosure compatibility alerts for a selected 3D printer.
    """
    if not printer_id:
        return []
    printers = load_printers()
    printer = printers.get(printer_id)
    if not printer:
        return []

    alerts: list[AlertItem] = []
    bed_x = float(printer["bed_x_mm"])
    bed_y = float(printer["bed_y_mm"])
    bed_z = float(printer["bed_z_mm"])
    printer_name = str(printer["name"])

    fits_xy = (part_x_mm <= bed_x and part_y_mm <= bed_y) or (
        part_x_mm <= bed_y and part_y_mm <= bed_x
    )
    fits_z = part_z_mm <= bed_z

    if not (fits_xy and fits_z):
        alerts.append(
            AlertItem(
                level="critical",
                code="PRINTER_BED_EXCEEDED",
                message=(
                    f"La pieza ({part_x_mm:.0f}×{part_y_mm:.0f}×{part_z_mm:.0f} mm) excede el "
                    f"volumen de impresión de {printer_name} ({bed_x:.0f}×{bed_y:.0f}×{bed_z:.0f} mm). "
                    "Reduce dimensiones o selecciona una impresora de mayor cama (ej. Voron 2.4)."
                ),
            )
        )

    category = str(material.get("category", "fdm"))
    mat_id = str(material.get("id", ""))
    mat_name = str(material.get("name", mat_id))
    max_temp_c = float(material.get("max_temp_c", 0.0))
    needs_enclosure = category == "fdm" and (
        mat_id in _ENCLOSURE_REQUIRED_IDS or max_temp_c >= 90.0
    )

    if needs_enclosure and not bool(printer.get("enclosed", False)):
        alerts.append(
            AlertItem(
                level="warning",
                code="PRINTER_ENCLOSURE_REQUIRED",
                message=(
                    f"{mat_name} requiere cámara cerrada para evitar warping y delaminación, "
                    f"pero {printer_name} es una impresora abierta. Usa PETG-CF o una impresora cerrada."
                ),
            )
        )

    return alerts
