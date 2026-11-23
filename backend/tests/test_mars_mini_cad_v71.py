"""Pytest suite for RoboForge Mars-Mini (v7.1): Elegoo Mars 5 Ultra MSLA, MG90S/MG996R, fits_bed==True, BOM<$45, ESP32 Wi-Fi .INO."""

from __future__ import annotations

import io
import zipfile

from backend.app.cad.hardware_parts import build_gripper_parts, build_servo_mg90s_3d
from backend.app.cad.print_prep import build_calibration_fit_test_coupon
from backend.app.schemas import (
    JointActuatorSpec,
    LinkDesignRequest,
    MechatronicsConfig,
    ModularArmRequest,
    ModularArmSlots,
)


def test_mg90s_compact_gripper_and_msla_calibration_coupon():
    """MG90S micro-servo, compact gripper (<=52 mm), and MSLA calibration coupon must build validly."""
    mg90s = build_servo_mg90s_3d()
    assert mg90s.is_valid and mg90s.volume > 3_000.0

    gb, gl, gr = build_gripper_parts(compact=True)
    assert gb.is_valid and gl.is_valid and gr.is_valid
    assert gl.bounding_box().size.X <= 52.0

    req_msla = ModularArmRequest(
        printer_id="elegoo_mars_5_ultra",
        material_id="resin_abs_like",
        mechatronics=MechatronicsConfig(bus_architecture="esp32_pca9685_5v", system_voltage_v=5.0),
    )
    coupon = build_calibration_fit_test_coupon(req_msla)
    assert coupon.is_valid and coupon.volume > 4_000.0


def test_mars_mini_elegoo_mars_5_ultra_fits_bed_bom_and_esp32_ino():
    """Mars-Mini on Elegoo Mars 5 Ultra must have fits_bed==True (0 bed warnings), MSLA guide, BOM<$45, ESP32 .INO & STL pack."""
    from fastapi.testclient import TestClient

    from backend.app.main import app

    client = TestClient(app)

    req = ModularArmRequest(
        dof=4,
        kinematics_type="articulated",
        payload_kg=0.15,
        base_height_mm=62.0,
        wrist_mm=45.0,
        four_bar_parallelogram=True,
        wrist_3dof=False,
        material_id="resin_abs_like",
        printer_id="elegoo_mars_5_ultra",
        auto_fix=True,
        slots=ModularArmSlots(
            upper_arm=LinkDesignRequest(length_mm=105.0, root_width_mm=24.0, tip_width_mm=18.0, thickness_mm=12.0),
            upper_arm_variant="organic_ibeam",
            forearm=LinkDesignRequest(length_mm=100.0, root_width_mm=20.0, tip_width_mm=15.0, thickness_mm=11.0),
            forearm_variant="organic_ibeam",
        ),
        mechatronics=MechatronicsConfig(
            bus_architecture="esp32_pca9685_5v",
            system_voltage_v=5.0,
            joint_actuators={
                "J1": JointActuatorSpec(motor_id="servo_mg996r", reducer_id="direct", target_rpm=30.0),
                "J2": JointActuatorSpec(motor_id="servo_mg996r", reducer_id="direct", target_rpm=30.0),
                "J3": JointActuatorSpec(motor_id="servo_mg90s", reducer_id="direct", target_rpm=45.0),
            },
        ),
    )

    resp = client.post("/api/arm/assemble", json=req.model_dump())
    assert resp.status_code == 200, resp.text
    data = resp.json()

    # 1. Strict Elegoo Mars 5 Ultra bed fit (0 pieces out of bed!)
    pr = data["print_readiness"]
    assert pr is not None
    assert pr["process_type"] == "msla_resin"
    assert pr["fits_bed"] is True, f"Parts exceeded Mars 5 Ultra bed: {pr['warnings']}"
    assert len(pr["warnings"]) == 0, f"Unexpected bed warnings: {pr['warnings']}"
    assert pr["layer_height_mm"] == 0.05
    assert pr["exposure_s"] is not None and 2.0 <= pr["exposure_s"] <= 3.0
    assert pr["recommended_tilt_deg"] == 35

    # 2. MSLA SatelLite/Chitubox PRINT_GUIDE.md (no FDM nozzle/hotbed jargon)
    r_guide = client.get(data["files"]["print_guide_md"])
    assert r_guide.status_code == 200
    assert "SatelLite" in r_guide.text and "Chitubox" in r_guide.text and "IPA" in r_guide.text
    assert "Temperatura boquilla" not in r_guide.text

    # 3. Ultra-economic BOM < $45 USD
    mech = data["mechatronics"]
    assert mech is not None
    assert mech["total_bom_cost_usd"] < 45.0, f"Mars-Mini BOM too expensive: ${mech['total_bom_cost_usd']}"

    # 4. ESP32 Wi-Fi .INO in files, ZIP, and dedicated endpoint
    assert "firmware_esp32_ino" in data["files"]
    r_ino = client.get(data["files"]["firmware_esp32_ino"])
    assert r_ino.status_code == 200 and "WiFi.h" in r_ino.text and "RoboForge-MarsMini" in r_ino.text

    r_ep = client.post(
        "/api/arm/firmware-esp32-ino",
        json={
            "arm_name": "Mars-Mini Test",
            "poses": [{"j0_base": 0, "j1_shoulder": 30, "j2_elbow": -30, "j3_wrist": 0, "gripper_pct": 50}],
            "speed_ms": 18,
        },
    )
    assert r_ep.status_code == 200 and "WebServer.h" in r_ep.text

    # 5. arm_stl_parts_pack.zip and arm_manufacturing_pack.zip both downloadable
    r_stl_zip = client.get(data["files"]["stl_parts_zip"])
    assert r_stl_zip.status_code == 200 and len(r_stl_zip.content) > 2000
    with zipfile.ZipFile(io.BytesIO(r_stl_zip.content), "r") as zf_stl:
        assert any(name.startswith("02_PIEZAS_STL_SEPARADAS/") for name in zf_stl.namelist())

    with zipfile.ZipFile(io.BytesIO(client.get(data["files"]["zip"]).content), "r") as zf:
        assert "mars_mini_esp32_wifi.ino" in set(zf.namelist())
