from __future__ import annotations

from pathlib import Path

import pytest

from app.physics import profiles as profiles_mod
from app.physics.dfm_rules import evaluate_global_alerts
from app.physics.gear_mechanics import evaluate_gear_global_alerts
from app.physics.mechanics import load_materials
from app.physics.mount_mechanics import evaluate_mount_global_alerts
from app.physics.profiles import (
    delete_custom_material,
    delete_custom_printer,
    evaluate_printer_dfm_alerts,
    load_printers,
    save_custom_material,
    save_custom_printer,
)
from app.schemas import (
    CustomMaterialCreate,
    GearDesignRequest,
    LinkDesignRequest,
    MountDesignRequest,
    PrinterProfile,
    PrinterProfileCreate,
)


@pytest.fixture
def isolated_user_profiles(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    path = tmp_path / "user_profiles.json"
    monkeypatch.setattr(profiles_mod, "USER_PROFILES_PATH", path)
    return path


def test_load_builtin_printers(isolated_user_profiles):
    printers = load_printers()
    expected = {"bambu_x1c", "prusa_mk4s", "voron_24", "creality_k1c", "ender3_v3"}
    assert expected.issubset(printers.keys())
    for pid in expected:
        p = PrinterProfile(**printers[pid])
        assert p.is_custom is False
    assert printers["bambu_x1c"]["enclosed"] is True
    assert printers["prusa_mk4s"]["enclosed"] is False
    assert printers["ender3_v3"]["clearance_mm"] == pytest.approx(0.22)


def test_custom_printer_crud(isolated_user_profiles):
    new_p = PrinterProfileCreate(
        id="my_ratrig_400",
        name="RatRig V-Core 400",
        bed_x_mm=400.0,
        bed_y_mm=400.0,
        bed_z_mm=400.0,
        default_nozzle_mm=0.6,
        clearance_mm=0.15,
        max_bed_temp_c=120.0,
        enclosed=True,
        notes="Custom large-format CoreXY",
    )
    saved = save_custom_printer(new_p)
    assert saved.id == "my_ratrig_400"
    assert saved.is_custom is True

    printers = load_printers()
    assert "my_ratrig_400" in printers
    assert printers["my_ratrig_400"]["bed_x_mm"] == 400.0

    # Built-in printer cannot be overwritten or deleted
    with pytest.raises(ValueError):
        save_custom_printer(PrinterProfileCreate(id="bambu_x1c", name="Duplicate"))
    assert delete_custom_printer("bambu_x1c") is False

    # Delete custom printer
    assert delete_custom_printer("my_ratrig_400") is True
    assert "my_ratrig_400" not in load_printers()
    assert delete_custom_printer("my_ratrig_400") is False


def test_custom_material_crud_and_load_materials_merge(isolated_user_profiles):
    mat = CustomMaterialCreate(
        id="pc_cf_custom",
        name="PC-CF Blend",
        category="fdm",
        density_g_cm3=1.25,
        youngs_xy_mpa=5800.0,
        youngs_z_mpa=3100.0,
        yield_xy_mpa=72.0,
        yield_z_mpa=34.0,
        max_temp_c=125.0,
        cost_per_kg_usd=85.0,
        color_hex="#6366f1",
        notes="Custom polycarbonate carbon fiber",
    )
    saved = save_custom_material(mat)
    assert saved["id"] == "pc_cf_custom"
    assert saved["is_custom"] is True

    all_mats = load_materials()
    assert "pc_cf_custom" in all_mats
    assert all_mats["pc_cf_custom"]["youngs_xy_mpa"] == 5800.0
    assert all_mats["petg_cf"]["is_custom"] is False

    # Cannot overwrite or delete built-in material
    with pytest.raises(ValueError):
        save_custom_material(mat.model_copy(update={"id": "petg_cf"}))
    assert delete_custom_material("petg_cf") is False

    assert delete_custom_material("pc_cf_custom") is True
    assert "pc_cf_custom" not in load_materials()


def test_printer_dfm_bed_exceeded_and_enclosure_alerts(isolated_user_profiles):
    materials = load_materials()
    pa12_cf = materials["pa12_cf"]
    petg_cf = materials["petg_cf"]

    # Part 280mm long exceeds Ender-3 V3 220x220 bed AND PA12-CF needs enclosure
    alerts = evaluate_printer_dfm_alerts("ender3_v3", pa12_cf, 280.0, 45.0, 20.0)
    codes = {a.code for a in alerts}
    assert "PRINTER_BED_EXCEEDED" in codes
    assert "PRINTER_ENCLOSURE_REQUIRED" in codes
    assert any(a.code == "PRINTER_BED_EXCEEDED" and a.level == "critical" for a in alerts)
    assert any(a.code == "PRINTER_ENCLOSURE_REQUIRED" and a.level == "warning" for a in alerts)

    # Same part fits on Voron 2.4 (300^3, enclosed) with zero printer alerts
    alerts_voron = evaluate_printer_dfm_alerts("voron_24", pa12_cf, 280.0, 45.0, 20.0)
    assert alerts_voron == []

    # PETG-CF on open Prusa MK4S (250x210) with small part -> zero printer alerts
    alerts_prusa = evaluate_printer_dfm_alerts("prusa_mk4s", petg_cf, 180.0, 40.0, 20.0)
    assert alerts_prusa == []


def test_printer_alerts_integrated_across_all_three_families(isolated_user_profiles):
    materials = load_materials()
    pa12_cf = materials["pa12_cf"]

    # 1. Link family
    link_req = LinkDesignRequest(
        length_mm=300.0, material_id="pa12_cf", printer_id="ender3_v3"
    )
    link_codes = {a.code for a in evaluate_global_alerts(link_req, pa12_cf)}
    assert "PRINTER_BED_EXCEEDED" in link_codes
    assert "PRINTER_ENCLOSURE_REQUIRED" in link_codes

    # 2. Mount family
    mount_req = MountDesignRequest(material_id="pa12_cf", printer_id="prusa_mk4s")
    mount_codes = {a.code for a in evaluate_mount_global_alerts(mount_req, pa12_cf)}
    assert "PRINTER_ENCLOSURE_REQUIRED" in mount_codes

    # 3. Gear family
    gear_req = GearDesignRequest(
        teeth=60, module_mm=4.5, material_id="pa12_cf", printer_id="ender3_v3"
    )
    gear_codes = {a.code for a in evaluate_gear_global_alerts(gear_req, pa12_cf)}
    assert "PRINTER_BED_EXCEEDED" in gear_codes
    assert "PRINTER_ENCLOSURE_REQUIRED" in gear_codes
