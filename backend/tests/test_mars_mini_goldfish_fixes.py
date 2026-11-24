"""Tests for Sprint v9.0 Mars-Mini Goldfish fixes, v9.0 schemas, and projects router."""

from __future__ import annotations

from pathlib import Path

from app.main import HASH_PATTERN, app
from app.physics.arm_compatibility import evaluate_modular_arm
from app.physics.firmware_gen import generate_esp32_wifi_pca9685_ino
from app.physics.mechatronics import (
    evaluate_mechatronics,
    generate_circuit_schematic_svg,
    generate_wireviz_yaml,
)
from app.schemas import (
    DFMAuditReport,
    DFMCheckItem,
    MasterBomItem,
    MechatronicsConfig,
    ModularArmRequest,
    ProjectGenerateRequest,
    ProjectGenerateResponse,
)

REPO_ROOT = Path(__file__).resolve().parents[2]


def _mars_mini_request() -> ModularArmRequest:
    return ModularArmRequest(
        name="RoboForge Mars-Mini",
        dof=4,
        kinematics_type="articulated",
        four_bar_parallelogram=True,
        payload_kg=0.15,
        base_height_mm=72.0,
        wrist_mm=45.0,
        material_id="resin_abs_like",
        printer_id="elegoo_mars_5_ultra",
        mechatronics=MechatronicsConfig(
            bus_architecture="esp32_pca9685_5v",
            system_voltage_v=5.0,
        ),
    )


def test_mars_mini_bom_fasteners_svg_and_wireviz_esp32_branch() -> None:
    req = _mars_mini_request()
    eval_res = evaluate_modular_arm(req, real_volumes_mm3={})
    mech = evaluate_mechatronics(req, eval_res)

    assert mech.bus_architecture == "esp32_pca9685_5v"
    assert mech.total_bom_cost_usd < 45.0

    codes = {item.part_code for item in mech.master_bom}
    assert "ISO4762-M2x8" in codes
    assert "ISO4762-M3x10" in codes
    assert "ISO4762-M3x16" in codes
    assert "ISO4762-M3x25" in codes
    assert "ISO4032-M3-NUT" in codes
    assert "BRG-608-2RS" in codes
    assert "SRV-MG996R" in codes
    assert "SRV-MG90S" in codes

    svg = generate_circuit_schematic_svg(mech, req.dof)
    assert "ESP32 DEVKIT" in svg
    assert "PCA9685" in svg and "0x40" in svg
    assert "5V 5A" in svg
    for ch in ("CH0", "CH1", "CH2", "CH3"):
        assert ch in svg
    assert "MG996R" in svg and "MG90S" in svg
    assert "GPIO21" in svg and "GPIO22" in svg
    assert "E-STOP + FUSIBLE" not in svg
    assert "JST-XH 4P" not in svg

    wv = generate_wireviz_yaml(mech, req.dof)
    assert "X1_PSU_5V5A:" in wv
    assert "X2_ESP32_DEVKIT:" in wv
    assert "X3_PCA9685_0x40:" in wv
    assert "S_CH0_J0_Base_Yaw:" in wv
    assert "S_CH1_J1_Shoulder:" in wv
    assert "S_CH2_J2_Elbow_4Bar:" in wv
    assert "S_CH3_Gripper:" in wv
    assert "MG996R" in wv and "MG90S" in wv
    assert "STM32F407" not in wv
    assert "XT60-M" not in wv


def test_esp32_firmware_initial_pose_matches_html_sliders() -> None:
    ino = generate_esp32_wifi_pca9685_ino("RoboForge Mars-Mini", poses=[], speed_ms=14)
    assert "const Pose POSE_INICIAL = {90, 120, 60, 80};" in ino
    assert "Pose poseActual = POSE_INICIAL;" in ino
    assert "poseActual = POSE_INICIAL;\n  aplicarPose(poseActual);" in ino
    assert 'id="s0" type="range" min="0" max="180" value="90"' in ino
    assert 'id="s1" type="range" min="20" max="165" value="120"' in ino
    assert 'id="s2" type="range" min="20" max="165" value="60"' in ino
    assert 'id="s3" type="range" min="25" max="140" value="80"' in ino


def test_mars_mini_build_guide_alignment() -> None:
    guide_path = REPO_ROOT / "docs" / "MARS_MINI_BUILD_GUIDE.md"
    text = guide_path.read_text(encoding="utf-8")

    for stl_name in (
        "part_base_pedestal.stl",
        "part_upper_arm_link.stl",
        "part_forearm_link.stl",
        "part_parallelogram_rod.stl",
        "part_gripper_base.stl",
        "part_gripper_finger_left.stl",
        "part_gripper_finger_right.stl",
    ):
        assert stl_name in text

    for fastener in ("M2×8", "M3×10", "M3×16", "M3×25"):
        assert fastener in text

    assert "80°" in text
    assert "pre-centrado en `45°`" not in text


def test_v9_schemas_hash_pattern_and_router_mounted() -> None:
    req = ProjectGenerateRequest(
        preset_id="freestyle_5in",
        material_id="petg_cf",
        printer_id="bambu_x1c",
        params={"prop_inch": 5.0},
    )
    audit = DFMAuditReport(
        status="pass",
        summary="5/5 checks passed",
        checks=[
            DFMCheckItem(
                id="watertight",
                label="Mallas Watertight",
                status="pass",
                detail="Todas las piezas son herméticas.",
            )
        ],
    )
    resp = ProjectGenerateResponse(
        family_id="fpv_drone",
        preset_id=req.preset_id,
        design_hash="prj_0123456789abcdef",
        name="Freestyle 5\"",
        metrics={"mass_g": 420.0, "twr": 4.8},
        bom=[
            MasterBomItem(
                category="printed_cnc",
                category_label="Piezas Impresas / CNC",
                part_code="FPV-FRAME-5",
                part_name="Chasis 5 pulgadas",
                spec="PETG-CF",
                qty=1,
                unit_cost_usd=6.5,
                total_cost_usd=6.5,
            )
        ],
        total_bom_cost_usd=6.5,
        dfm_audit=audit,
        explode_vectors={"top_plate": [0.0, 0.0, 45.0]},
        files={"glb": "/api/exports/prj_0123456789abcdef/project.glb"},
    )
    assert resp.design_hash == "prj_0123456789abcdef"
    assert HASH_PATTERN.match(resp.design_hash)
    assert HASH_PATTERN.match("prj_abcdef012345")
    assert HASH_PATTERN.match("ref_abcdef012345")
    assert HASH_PATTERN.match("gyr_abcdef012345")
    assert not HASH_PATTERN.match("../bad_hash")

    openapi_paths = set(app.openapi()["paths"].keys())
    assert "/api/projects/families" in openapi_paths
    assert "/api/projects/{family}/generate" in openapi_paths
