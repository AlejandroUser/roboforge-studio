from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest

from app.physics.arm_compatibility import evaluate_modular_arm
from app.physics.arm_kinematics import load_arm_presets
from app.physics.firmware_gen import generate_esp32_wifi_pca9685_ino
from app.physics.mechanics import load_materials
from app.physics.mechatronics import (
    ACTUATOR_CATALOG,
    evaluate_mechatronics,
    generate_circuit_schematic_svg,
)
from app.physics.profiles import load_printers
from app.schemas import (
    ArmAssemblyRequest,
    JointActuatorSpec,
    LinkDesignRequest,
    MechatronicsConfig,
    ModularArmRequest,
    ModularArmSlots,
    PrinterProfile,
    PrintReadinessSummary,
)


def test_resin_abs_like_and_elegoo_mars_5_ultra_and_mars_mini_preset():
    mats = load_materials()
    assert "resin_abs_like" in mats
    resin = mats["resin_abs_like"]
    assert resin["category"] == "msla"
    assert resin["youngs_xy_mpa"] == resin["youngs_z_mpa"] == 2000.0
    assert resin["yield_xy_mpa"] == resin["yield_z_mpa"] == 45.0
    assert resin["density_g_cm3"] == pytest.approx(1.15)
    assert resin["cost_per_kg_usd"] == pytest.approx(28.0)

    printers = load_printers()
    assert "elegoo_mars_5_ultra" in printers
    mars = PrinterProfile(**printers["elegoo_mars_5_ultra"])
    assert mars.bed_x_mm == pytest.approx(153.36)
    assert mars.bed_y_mm == pytest.approx(77.76)
    assert mars.bed_z_mm == pytest.approx(165.0)
    assert mars.process_type == "msla_resin"
    assert mars.xy_resolution_um == pytest.approx(18.0)

    presets = load_arm_presets()["presets"]
    assert "mars_mini" in presets
    req = ArmAssemblyRequest(**presets["mars_mini"]["defaults"])
    assert req.preset == "mars_mini"
    assert req.upper_arm_mm == 105.0
    assert req.forearm_mm == 100.0
    assert req.wrist_mm == 45.0
    assert req.base_height_mm == 62.0
    assert req.material_id == "resin_abs_like"
    assert req.printer_id == "elegoo_mars_5_ultra"


def test_mars_mini_mechatronics_budget_under_45_usd_and_svg():
    assert "servo_mg996r" in ACTUATOR_CATALOG
    assert "servo_mg90s" in ACTUATOR_CATALOG
    assert ACTUATOR_CATALOG["servo_mg996r"]["unit_cost_usd"] == pytest.approx(3.5)
    assert ACTUATOR_CATALOG["servo_mg90s"]["unit_cost_usd"] == pytest.approx(2.0)

    req = ModularArmRequest(
        dof=4,
        kinematics_type="articulated",
        four_bar_parallelogram=True,
        payload_kg=0.15,
        base_height_mm=62.0,
        wrist_mm=45.0,
        material_id="resin_abs_like",
        printer_id="elegoo_mars_5_ultra",
        slots=ModularArmSlots(
            upper_arm=LinkDesignRequest(length_mm=105.0, root_width_mm=24.0, tip_width_mm=18.0, thickness_mm=12.0),
            forearm=LinkDesignRequest(length_mm=100.0, root_width_mm=22.0, tip_width_mm=16.0, thickness_mm=12.0),
        ),
        mechatronics=MechatronicsConfig(
            bus_architecture="esp32_pca9685_5v",
            system_voltage_v=5.0,
            joint_actuators={
                "J1": JointActuatorSpec(motor_id="servo_mg996r", reducer_id="direct", target_rpm=30.0),
                "J2": JointActuatorSpec(motor_id="servo_mg996r", reducer_id="direct", target_rpm=30.0),
                "J3": JointActuatorSpec(motor_id="servo_mg996r", reducer_id="direct", target_rpm=32.0),
                "J4": JointActuatorSpec(motor_id="servo_mg90s", reducer_id="direct", target_rpm=45.0),
            },
        ),
    )

    eval_res = evaluate_modular_arm(req, {})
    mech = evaluate_mechatronics(req, eval_res)

    assert mech.bus_architecture == "esp32_pca9685_5v"
    assert mech.power_budget.system_voltage_v == pytest.approx(5.0)
    assert mech.power_budget.peak_power_w <= 25.0
    assert mech.total_bom_cost_usd < 45.0

    codes = {i.part_code: i for i in mech.master_bom}
    assert codes["SRV-MG996R"].qty == 3
    assert codes["SRV-MG90S"].qty == 1
    assert codes["EL-ESP32-DEV"].qty == 1
    assert codes["EL-PCA9685-16CH"].qty == 1
    assert codes["ISO4032-M3-NUT"].qty == 24
    assert codes["BRG-608-2RS"].qty == 1

    svg = generate_circuit_schematic_svg(mech, req.dof)
    assert "ESP32 WI-FI + PCA9685" in svg
    assert ET.fromstring(svg).tag.endswith("svg")

    # MSLA PrintReadinessSummary schema check
    pr = PrintReadinessSummary(
        fits_bed=True,
        printer_name="Elegoo Mars 5 Ultra (Resina 9K)",
        bed_mm=[153.36, 77.76, 165.0],
        total_plates=2,
        total_filament_g=175.0,
        estimated_print_hours=3.4,
        layer_height_mm=0.05,
        wall_loops=0,
        infill_pct=100,
        infill_pattern="Resina Sólida MSLA",
        nozzle_temp_c=0,
        bed_temp_c=0,
        process_type="msla_resin",
        exposure_s=2.3,
        bottom_exposure_s=28.0,
        recommended_tilt_deg=35,
    )
    assert pr.process_type == "msla_resin"
    assert pr.exposure_s == pytest.approx(2.3)


def test_esp32_wifi_pca9685_firmware_generation():
    ino = generate_esp32_wifi_pca9685_ino(
        "RoboForge Mars-Mini",
        poses=[
            {"j0_base": 0, "j1_shoulder": 30, "j2_elbow": -45, "j3_wrist": 15, "gripper_pct": 90},
            {"j0_base": 40, "j1_shoulder": 15, "j2_elbow": -30, "j3_wrist": 15, "gripper_pct": 10},
        ],
        speed_ms=16,
    )
    assert "#include <WiFi.h>" in ino
    assert "#include <WebServer.h>" in ino
    assert "#include <Adafruit_PWMServoDriver.h>" in ino
    assert 'AP_SSID = "RoboForge-MarsMini"' in ino
    assert "handlePlay" in ino
    assert "SECUENCIA[]" in ino
    assert "SPEED_MS = 16;" in ino
