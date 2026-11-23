from __future__ import annotations

import io
import zipfile

from fastapi.testclient import TestClient

from app.main import app
from app.physics.firmware_gen import generate_arduino_pca9685_ino

client = TestClient(app)


def test_generate_arduino_pca9685_ino_structure_and_inversion():
    poses = [
        {"j0_base": 0, "j1_shoulder": 30, "j2_elbow": -40, "j3_wrist": 10, "gripper_pct": 100},
        {"j0_base": -90, "j1_shoulder": 0, "j2_elbow": 0, "j3_wrist": 0, "gripper_pct": 0},
    ]
    ino = generate_arduino_pca9685_ino("Mi Brazo 5.0", poses=poses, speed_ms=15)

    assert "#include <Wire.h>" in ino
    assert "#include <Adafruit_PWMServoDriver.h>" in ino
    assert "Adafruit_PWMServoDriver(0x40)" in ino
    assert "pwm.setPWMFreq(50);" in ino
    assert "CH0_BASE_YAW" in ino
    assert "CH2_SHOULDER_B" in ino
    assert "180 - p.hombro" in ino
    assert "180 - p.codo" in ino
    assert "CH7_GRIPPER" in ino
    assert "const Pose SECUENCIA[]" in ino
    assert "SPEED_MS = 15;" in ino
    assert "void setup()" in ino
    assert "void loop()" in ino


def test_api_firmware_ino_and_arm_database():
    resp = client.post(
        "/api/arm/firmware-ino",
        json={
            "arm_name": "Brazo 5.0 Test",
            "poses": [
                {"j0_base": 20, "j1_shoulder": 40, "j2_elbow": -50, "j3_wrist": 15, "gripper_pct": 75},
                {"j0_base": -30, "j1_shoulder": 10, "j2_elbow": -20, "j3_wrist": 5, "gripper_pct": 10},
            ],
            "speed_ms": 20,
        },
    )
    assert resp.status_code == 200
    assert "brazo_5_0_secuencia.ino" in resp.headers.get("content-disposition", "")
    assert "Adafruit_PWMServoDriver(0x40)" in resp.text
    assert "180 - p.hombro" in resp.text

    db_resp = client.get("/api/arm/database")
    assert db_resp.status_code == 200
    data = db_resp.json()
    assert isinstance(data, dict)


def test_api_arm_assemble_pca9685_includes_firmware_ino_and_zip():
    resp = client.post(
        "/api/arm/assemble",
        json={
            "dof": 6,
            "payload_kg": 0.35,
            "material_id": "pla_plus",
            "mechatronics": {
                "bus_architecture": "arduino_pca9685_i2c",
                "system_voltage_v": 6.5,
                "joint_actuators": {
                    "J1": {"motor_id": "servo_td8120mg", "reducer_id": "direct", "target_rpm": 30, "parallel_count": 2},
                    "J2": {"motor_id": "servo_td8120mg", "reducer_id": "direct", "target_rpm": 30, "parallel_count": 2},
                    "J3": {"motor_id": "servo_td8120mg", "reducer_id": "direct", "target_rpm": 35, "parallel_count": 1},
                },
            },
        },
    )
    assert resp.status_code == 200
    body = resp.json()
    files = body["files"]
    assert "firmware_ino" in files
    assert files["firmware_ino"].endswith("/brazo_5_0_secuencia.ino")

    ino_dl = client.get(files["firmware_ino"])
    assert ino_dl.status_code == 200
    assert "Adafruit_PWMServoDriver(0x40)" in ino_dl.text

    zip_dl = client.get(files["zip"])
    assert zip_dl.status_code == 200
    with zipfile.ZipFile(io.BytesIO(zip_dl.content), "r") as zf:
        names = set(zf.namelist())
        assert "brazo_5_0_secuencia.ino" in names
        assert "arm_circuit_schematic.svg" in names
        assert "arm_master_bom.csv" in names
