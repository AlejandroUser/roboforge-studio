from __future__ import annotations

import pytest
from fastapi.testclient import TestClient

from app.main import app
from app.physics.ik_solver import compute_caliper_compensation, solve_arm_ik

client = TestClient(app)


def test_articulated_ik_reachable_and_out_of_reach_clamping():
    # Reachable 3D point for standard arm (L1=200, L2=170, L3=75, H=95)
    res_ok = solve_arm_ik(
        target_x_mm=240.0,
        target_y_mm=80.0,
        target_z_mm=160.0,
        l1_mm=200.0,
        l2_mm=170.0,
        l3_mm=75.0,
        base_height_mm=95.0,
        kinematics_type="articulated",
        elbow_up=True,
    )
    assert res_ok["reachable"] is True
    assert res_ok["reach_error_mm"] < 1.0
    assert res_ok["tcp_mm"][0] == pytest.approx(240.0, abs=1.0)
    assert res_ok["tcp_mm"][1] == pytest.approx(80.0, abs=1.0)
    assert res_ok["tcp_mm"][2] == pytest.approx(160.0, abs=1.0)

    # Unreachable far point -> smooth clamping, reachable == False
    res_far = solve_arm_ik(
        target_x_mm=800.0,
        target_y_mm=0.0,
        target_z_mm=300.0,
        l1_mm=200.0,
        l2_mm=170.0,
        l3_mm=75.0,
        base_height_mm=95.0,
    )
    assert res_far["reachable"] is False
    assert res_far["reach_error_mm"] > 100.0


def test_scara_ik_reachable_and_clamped():
    res = solve_arm_ik(
        target_x_mm=160.0,
        target_y_mm=90.0,
        target_z_mm=80.0,
        l1_mm=150.0,
        l2_mm=140.0,
        l3_mm=0.0,
        base_height_mm=160.0,
        kinematics_type="scara",
    )
    assert res["reachable"] is True
    assert res["reach_error_mm"] < 0.5
    assert res["z_mm"] == pytest.approx(80.0)


def test_caliper_compensation_tight_perfect_loose():
    # Tight (undersized holes in MSLA resin -> reduce exposure, positive XY hole comp)
    tight = compute_caliper_compensation(3.0, 22.0, 3.2, 22.2, process_type="msla_resin")
    assert tight["status"] == "tight"
    assert tight["xy_hole_compensation_mm"] > 0.0
    assert tight["exposure_delta_s"] < 0.0

    # Perfect
    perf = compute_caliper_compensation(3.22, 22.18, 3.2, 22.2, process_type="msla_resin")
    assert perf["status"] == "perfect"
    assert perf["exposure_delta_s"] == 0.0

    # Loose in FDM
    loose = compute_caliper_compensation(3.4, 22.4, 3.2, 22.2, process_type="fdm")
    assert loose["status"] == "loose"
    assert loose["xy_hole_compensation_mm"] < 0.0
    assert loose["exposure_delta_s"] == 0.0


def test_api_v8_ik_calibrate_and_virtual_usb_hardware():
    r_ik = client.post(
        "/api/arm/ik",
        json={
            "target_x_mm": 180.0,
            "target_y_mm": 40.0,
            "target_z_mm": 130.0,
            "l1_mm": 105.0,
            "l2_mm": 100.0,
            "l3_mm": 45.0,
            "base_height_mm": 62.0,
            "kinematics_type": "articulated",
        },
    )
    assert r_ik.status_code == 200
    assert r_ik.json()["reachable"] is True

    r_cal = client.post(
        "/api/print/calibrate",
        json={
            "measured_m3_mm": 3.05,
            "measured_bore_mm": 22.05,
            "process_type": "msla_resin",
        },
    )
    assert r_cal.status_code == 200
    assert r_cal.json()["status"] == "tight"

    r_ports = client.get("/api/hardware/ports")
    assert r_ports.status_code == 200
    ports = r_ports.json()["ports"]
    assert any(p["port"] == "VIRTUAL_SIM" for p in ports)

    r_send = client.post(
        "/api/hardware/send-pose",
        json={
            "port": "VIRTUAL_SIM",
            "baudrate": 115200,
            "j0_base": 15.0,
            "j1_shoulder": 35.0,
            "j2_elbow": -50.0,
            "j3_wrist": 15.0,
            "gripper_pct": 80.0,
        },
    )
    assert r_send.status_code == 200
    body = r_send.json()
    assert body["ok"] is True
    assert body["mode"] == "virtual_sim"
    assert body["frame"] == "P:15,35,-50,15,80\n"
