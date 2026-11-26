"""Comprehensive electrical, WireViz, SVG, firmware, LeRobot, and serial pool safety tests."""

from __future__ import annotations

import json
import re
import sys
import types
import xml.etree.ElementTree as ET

import yaml

import app.main as main_mod
from app.physics.arm_compatibility import evaluate_modular_arm
from app.physics.firmware_gen import (
    generate_arduino_pca9685_ino,
    generate_esp32_wifi_pca9685_ino,
)
from app.physics.mechatronics import (
    ARDUINO_PCA9685_CHANNELS,
    evaluate_mechatronics,
    generate_circuit_schematic_svg,
    generate_lerobot_config_json,
    generate_wireviz_yaml,
)
from app.schemas import MechatronicsConfig, ModularArmRequest


def _eval_arch(arch: str, **overrides):
    base_cfg = {
        "esp32_pca9685_5v": dict(dof=4, payload_kg=0.15, material_id="resin_abs_like", v=5.0),
        "arduino_pca9685_i2c": dict(dof=6, payload_kg=0.5, v=6.5),
        "uart_serial_bus": dict(dof=5, payload_kg=0.4, v=7.4),
        "stm32_step_dir": dict(dof=6, payload_kg=1.0, v=24.0),
        "teensy_dm542": dict(dof=6, payload_kg=2.0, v=24.0),
        "can_fd_bus": dict(dof=6, payload_kg=3.5, v=48.0),
    }[arch]
    v = base_cfg.pop("v")
    base_cfg.update(overrides)
    req = ModularArmRequest(
        **base_cfg,
        mechatronics=MechatronicsConfig(bus_architecture=arch, system_voltage_v=v),
    )
    ev = evaluate_modular_arm(req, real_volumes_mm3={})
    mech = evaluate_mechatronics(req, ev)
    svg = generate_circuit_schematic_svg(mech, req.dof)
    wv_raw = generate_wireviz_yaml(mech, req.dof)
    ET.fromstring(svg)  # Must be strictly valid XML
    wv = yaml.safe_load(wv_raw)  # Must be strictly valid YAML
    return req, mech, svg, wv_raw, wv


def _extract_psu_amps(psu_name: str) -> float:
    m = re.search(r"(\d+(?:\.\d+)?)\s*A\b", psu_name)
    assert m is not None, f"Could not parse PSU amp rating from: {psu_name}"
    return float(m.group(1))


def test_all_six_architectures_xml_yaml_and_psu_ampacity() -> None:
    for arch in (
        "esp32_pca9685_5v",
        "arduino_pca9685_i2c",
        "uart_serial_bus",
        "stm32_step_dir",
        "teensy_dm542",
        "can_fd_bus",
    ):
        _, mech, _, _, wv = _eval_arch(arch)
        pb = mech.power_budget
        psu_a = _extract_psu_amps(pb.recommended_psu)
        assert psu_a >= pb.peak_current_a, (
            f"{arch}: PSU {psu_a}A < peak {pb.peak_current_a}A ({pb.recommended_psu})"
        )
        assert isinstance(wv, dict)
        assert "connectors" in wv and "cables" in wv and "connections" in wv


def test_arduino_pca9685_no_6v5_to_5v_short_and_channel_parity() -> None:
    _, mech, svg, _, wv = _eval_arch("arduino_pca9685_i2c")
    pca_pins = wv["connectors"]["X3_PCA9685"]["pins"]
    assert "V+" in pca_pins and "VCC_5V" in pca_pins
    vplus_idx = pca_pins.index("V+") + 1
    vcc5_idx = pca_pins.index("VCC_5V") + 1
    assert vplus_idx != vcc5_idx

    # Verify PSU connects to V+ (pin 1) and Arduino 5V connects to VCC_5V (pin 3), never same pin
    conns = wv["connections"]
    psu_pca = [c for c in conns if "X1_PSU_6V5" in c[0]][0]
    ard_pca = [c for c in conns if "X2_ARDUINO_UNO" in c[0]][0]
    assert psu_pca[2]["X3_PCA9685"][0] == vplus_idx
    assert ard_pca[2]["X3_PCA9685"][0] == vcc5_idx

    # High peak (>8A) must not use barrel DC jack
    if mech.power_budget.peak_current_a > 8.0:
        assert "DC-Jack" not in wv["connectors"]["X1_PSU_6V5"]["subtype"]

    # 1:1 PCA9685 channel parity between mechatronics.py (SVG + WireViz) and firmware_gen.py
    ino = generate_arduino_pca9685_ino("Brazo 5.0", poses=[])
    expected_fw_consts = [
        "CH0_BASE_YAW     = 0;",
        "CH1_SHOULDER_A   = 1;",
        "CH2_SHOULDER_B   = 2;",
        "CH3_ELBOW_A      = 3;",
        "CH4_ELBOW_B      = 4;",
        "CH5_WRIST_PITCH  = 5;",
        "CH6_WRIST_ROLL   = 6;",
        "CH7_GRIPPER      = 7;",
    ]
    for const_line in expected_fw_consts:
        assert const_line in ino

    for ch, ch_id, label, _ in ARDUINO_PCA9685_CHANNELS:
        assert f"{ch} · {label}" in svg
        assert f"S_{ch_id}" in wv["connectors"]


def test_can_fd_separates_power_xt30_from_jst_gh_signal() -> None:
    _, _, svg, _, wv = _eval_arch("can_fd_bus")
    assert "XT30 (Pwr) + JST-GH (CAN)" in svg
    for cname, cdata in wv["connectors"].items():
        if cdata.get("subtype") == "JST-GH 4P":
            assert "VBUS" not in cdata["pins"], f"High-power VBUS found on JST-GH connector {cname}!"
    assert "M_PWR_J1" in wv["connectors"]
    assert wv["connectors"]["M_PWR_J1"]["subtype"] == "XT30-M"
    assert "M_CAN_J1" in wv["connectors"]
    assert wv["connectors"]["M_CAN_J1"]["subtype"] == "JST-GH 4P"
    assert "14 AWG" in wv["cables"]["W2_PWR_TRUNK"]["gauge"] or "16 AWG" in wv["cables"]["W2_PWR_TRUNK"]["gauge"]
    assert "24 AWG" in wv["cables"]["W3_CAN_FD_BUS"]["gauge"]


def test_stm32_step_dir_uses_18awg_for_motor_phases_not_26awg() -> None:
    _, _, _, _, wv = _eval_arch("stm32_step_dir")
    assert "X3_MCU_STM32" in wv["connectors"]
    assert "X4_TMC2209_SHIELD" in wv["connectors"]
    assert "18 AWG" in wv["cables"]["W3_MOTOR_PHASE"]["gauge"]
    # Every motor connection must use W3_MOTOR_PHASE (18 AWG), never 26 AWG signal cable
    motor_conns = [c for c in wv["connections"] if any(k.startswith("M_J") for k in c[2])]
    assert len(motor_conns) == 6
    for c in motor_conns:
        assert "W3_MOTOR_PHASE" in c[1]
        assert "W2_STEP_DIR_SIG" not in c[1]


def test_uart_serial_bus_and_teensy_dm542_have_dedicated_wireviz_and_svg() -> None:
    _, _, svg_u, wv_u_raw, wv_u = _eval_arch("uart_serial_bus")
    assert "Molex 5264 3P (TTL)" in svg_u
    assert "X2_WAVESHARE_TTL" in wv_u["connectors"]
    assert "S_J1" in wv_u["connectors"]
    assert wv_u["connectors"]["S_J1"]["subtype"] == "Molex-5264-3P"
    assert wv_u["connectors"]["S_J1"]["pins"] == ["VCC", "GND", "DATA"]
    assert "STM32F407" not in wv_u_raw

    _, _, svg_t, wv_t_raw, wv_t = _eval_arch("teensy_dm542")
    assert "Bornera / GX16-4" in svg_t
    assert "X3_TEENSY41" in wv_t["connectors"]
    assert "D_DM542T_J1" in wv_t["connectors"]
    assert "18 AWG" in wv_t["cables"]["W3_MOTOR_PHASE"]["gauge"]
    assert wv_t["connectors"]["M_J1"]["subtype"] == "GX16-4 / Bornera"
    assert "STM32F407" not in wv_t_raw


def test_firmware_live_serial_parser_sanitization_and_nonblocking_esp32() -> None:
    evil_name = "Robot */ int hacked = 1; /* Test"
    ard = generate_arduino_pca9685_ino(evil_name, poses=[])
    esp = generate_esp32_wifi_pca9685_ino(evil_name, poses=[])

    for ino in (ard, esp):
        assert "void leerSerialLive()" in ino
        assert "serialBuf[0] == 'P' && serialBuf[1] == ':'" in ino
        assert 'sscanf(serialBuf + 2, "%d,%d,%d,%d,%d"' in ino
        assert 'Serial.println(F("OK"));' in ino
        # Comment close injection must be neutralized
        header = ino.split("*/", 1)[0]
        assert "int hacked" in header  # text preserved safely inside comment
        assert "Robot * /" in header

    assert "void esperarNoBloqueante(unsigned long ms)" in esp
    assert "server.handleClient();" in esp
    assert "esperarNoBloqueante(450);" in esp


def test_lerobot_12bit_sts3215_schema() -> None:
    req = ModularArmRequest(
        dof=5,
        mechatronics=MechatronicsConfig(bus_architecture="uart_serial_bus", system_voltage_v=7.4),
    )
    cfg = json.loads(generate_lerobot_config_json(req))
    assert cfg["baudrate"] == 1000000
    assert len(cfg["motors"]) == 6
    for m_name, m_cfg in cfg["motors"].items():
        assert m_cfg["drive_mode"] == 0, m_name
        assert m_cfg["homing_offset"] == 0, m_name
        assert m_cfg["range_min"] == 0, m_name
        assert m_cfg["range_max"] == 4095, m_name


def test_persistent_serial_pool_reuses_port_and_cleans_up_on_error(monkeypatch) -> None:
    opened_calls: list[tuple[str, int]] = []
    sleep_calls: list[float] = []
    written_frames: list[bytes] = []
    fail_next_write = {"flag": False}

    class FakeSerial:
        def __init__(self, port: str, baudrate: int, timeout: float = 0.5) -> None:
            self.port = port
            self.baudrate = baudrate
            self.timeout = timeout
            self.is_open = True
            opened_calls.append((port, baudrate))

        def write(self, data: bytes) -> int:
            if fail_next_write["flag"]:
                raise OSError("USB cable unplugged")
            written_frames.append(data)
            return len(data)

        def flush(self) -> None:
            pass

        def close(self) -> None:
            self.is_open = False

    fake_mod = types.SimpleNamespace(Serial=FakeSerial)
    monkeypatch.setitem(sys.modules, "serial", fake_mod)
    monkeypatch.setattr(main_mod.time, "sleep", lambda s: sleep_calls.append(s))
    main_mod._SERIAL_POOL.clear()

    pose = main_mod.HardwarePoseRequest(
        port="COM99",
        baudrate=115200,
        j0_base=10.0,
        j1_shoulder=30.0,
        j2_elbow=-45.0,
        j3_wrist=15.0,
        gripper_pct=70.0,
    )
    r1 = main_mod.send_hardware_pose(pose)
    r2 = main_mod.send_hardware_pose(pose)
    r3 = main_mod.send_hardware_pose(pose)

    assert r1["ok"] and r2["ok"] and r3["ok"]
    assert r1["frame"] == "P:10,30,-45,15,70\n"
    # Opened ONCE, booted ONCE, written 3 times
    assert len(opened_calls) == 1
    assert len(sleep_calls) == 1 and sleep_calls[0] == main_mod._SERIAL_BOOT_WAIT_S
    assert len(written_frames) == 3

    # Write failure must evict broken port from pool
    fail_next_write["flag"] = True
    r_err = main_mod.send_hardware_pose(pose)
    assert r_err["ok"] is False
    assert r_err["mode"] == "serial_error"
    assert "COM99" not in main_mod._SERIAL_POOL

    # Re-open and close explicitly via endpoint helper
    fail_next_write["flag"] = False
    r4 = main_mod.send_hardware_pose(pose)
    assert r4["ok"] is True
    assert "COM99" in main_mod._SERIAL_POOL
    c_res = main_mod.close_hardware_port(main_mod.HardwareClosePortRequest(port="COM99"))
    assert c_res["closed"] is True
    assert "COM99" not in main_mod._SERIAL_POOL
