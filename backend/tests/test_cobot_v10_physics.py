"""Tests for Sprint v10.0 Industrial Cobot physics, BOM, PowerBudget, and Franka Panda preset."""

from __future__ import annotations

import pytest

from app.physics.arm_compatibility import evaluate_modular_arm
from app.physics.arm_kinematics import load_arm_presets
from app.physics.mechatronics import evaluate_mechatronics
from app.schemas import (
    ArmAssemblyRequest,
    MechatronicsConfig,
    ModularArmRequest,
    PrintReadinessSummary,
)


def test_v10_schemas_and_franka_panda_preset() -> None:
    presets = load_arm_presets()["presets"]
    assert "franka_panda" in presets
    fp = presets["franka_panda"]
    d = fp["defaults"]
    assert d["dof"] == 7
    assert d["upper_arm_mm"] == 316.0
    assert d["forearm_mm"] == 384.0
    assert d["wrist_mm"] == 107.0
    assert d["base_height_mm"] == 140.0
    assert d["payload_kg"] == 3.0
    assert d["material_id"] == "al_7075_t6"
    assert d["printer_id"] == "voron_24_350"

    arm_req = ArmAssemblyRequest(preset="franka_panda", dof=7)
    assert arm_req.preset == "franka_panda"

    mod_req = ModularArmRequest(
        dof=7,
        payload_kg=3.0,
        material_id="al_7075_t6",
        dh_lateral_offsets_mm=[0.0, 0.0, 0.0, 82.5, -82.5, 0.0, 88.0],
        joint_torque_sensors=True,
        electromagnetic_brakes=True,
        auto_dovetail_split=True,
    )
    assert mod_req.dh_lateral_offsets_mm == [0.0, 0.0, 0.0, 82.5, -82.5, 0.0, 88.0]
    assert mod_req.joint_torque_sensors is True
    assert mod_req.electromagnetic_brakes is True
    assert mod_req.auto_dovetail_split is True

    prs = PrintReadinessSummary(
        fits_bed=True,
        printer_name="Bambu Lab X1-Carbon",
        bed_mm=[256.0, 256.0, 256.0],
        total_plates=2,
        total_filament_g=420.0,
        estimated_print_hours=8.5,
        layer_height_mm=0.2,
        wall_loops=4,
        infill_pct=35,
        infill_pattern="gyroid",
        nozzle_temp_c=265,
        bed_temp_c=85,
        split_parts_count=2,
    )
    assert prs.split_parts_count == 2


def test_cobot_v10_torque_sensors_and_em_brakes_bom_and_power() -> None:
    base_req = ModularArmRequest(
        dof=7,
        payload_kg=3.0,
        base_height_mm=140.0,
        wrist_mm=107.0,
        material_id="al_7075_t6",
        mechatronics=MechatronicsConfig(bus_architecture="can_fd_bus", system_voltage_v=24.0),
        joint_torque_sensors=False,
        electromagnetic_brakes=False,
    )
    cobot_req = base_req.model_copy(
        update={
            "dh_lateral_offsets_mm": [0.0, 0.0, 0.0, 82.5, -82.5, 0.0, 88.0],
            "joint_torque_sensors": True,
            "electromagnetic_brakes": True,
            "auto_dovetail_split": True,
        }
    )

    eval_base = evaluate_modular_arm(base_req, real_volumes_mm3={})
    eval_cobot = evaluate_modular_arm(cobot_req, real_volumes_mm3={})

    mech_base = evaluate_mechatronics(base_req, eval_base)
    mech_cobot = evaluate_mechatronics(cobot_req, eval_cobot)

    bom_by_code = {item.part_code: item for item in mech_cobot.master_bom}
    assert "SNS-TRQ-SG16" in bom_by_code
    sns = bom_by_code["SNS-TRQ-SG16"]
    assert sns.category == "electronics_wiring"
    assert sns.qty == 7
    assert sns.unit_cost_usd == 65.0
    assert sns.total_cost_usd == 455.0
    assert "42 g" in sns.spec

    assert "BRK-EM24V-HOLD" in bom_by_code
    brk = bom_by_code["BRK-EM24V-HOLD"]
    assert brk.qty == 7
    assert brk.unit_cost_usd == 42.0
    assert brk.total_cost_usd == 294.0
    assert "95 g" in brk.spec

    # Holding current delta: 7 brakes * 0.25 A = +1.75 A on continuous and peak
    expected_delta_a = 7 * 0.25
    assert mech_cobot.power_budget.continuous_current_a == pytest.approx(
        mech_base.power_budget.continuous_current_a + expected_delta_a, abs=0.02
    )
    assert mech_cobot.power_budget.peak_current_a == pytest.approx(
        mech_base.power_budget.peak_current_a + expected_delta_a, abs=0.02
    )
    assert mech_cobot.total_bom_cost_usd >= mech_base.total_bom_cost_usd + 455.0 + 294.0
