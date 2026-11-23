from __future__ import annotations

import pytest

from app.physics.arm_compatibility import (
    auto_match_interfaces,
    evaluate_modular_arm,
    validate_arm_compatibility,
)
from app.schemas import (
    EndInterfaceParams,
    GearDesignRequest,
    LinkDesignRequest,
    ModularArmRequest,
    ModularArmSlots,
    MountDesignRequest,
)


def test_validate_arm_compatibility_detects_four_real_failures():
    # 1. Shaft mismatch (NEMA 23 = 6.35mm vs gear bore = 5.0mm)
    # 2. Bolt PCD/size mismatch (NEMA 23 = 4xM5@66.7mm vs upper_arm = 4xM3@31mm)
    # 3. Under-strength gear & thin standard mount under 4 kg payload
    # 4. Printer bed exceeded (320mm link on 220mm Ender-3 V3) + open enclosure for PA12-CF
    req = ModularArmRequest(
        payload_kg=4.0,
        material_id="pa12_cf",
        printer_id="ender3_v3",
        slots=ModularArmSlots(
            shoulder_mount=MountDesignRequest(
                actuator_type="nema23",
                wall_thickness_mm=2.5,
                material_id="pa12_cf",
            ),
            shoulder_mount_variant="mount_standard",
            shoulder_gear=GearDesignRequest(
                teeth=18,
                module_mm=0.9,
                face_width_mm=4.0,
                bore_diameter_mm=5.0,
                material_id="pa12_cf",
            ),
            shoulder_gear_variant="gear_spoked",
            upper_arm=LinkDesignRequest(
                length_mm=320.0,
                root_width_mm=38.0,
                root_end=EndInterfaceParams(
                    preset="nema17",
                    outer_diameter_mm=46.0,
                    bore_diameter_mm=22.0,
                    bolt_count=4,
                    bolt_pcd_mm=31.0,
                    bolt_size="M3",
                ),
                material_id="pa12_cf",
            ),
        ),
    )

    issues = validate_arm_compatibility(req)
    critical_pairs = {(i.slot, i.parameter) for i in issues if i.severity == "critical"}
    all_pairs = {(i.slot, i.parameter) for i in issues}

    assert ("shoulder_gear", "bore_diameter_mm") in critical_pairs
    assert ("upper_arm", "root_end.bolt_pcd_mm") in critical_pairs
    assert ("shoulder_gear", "face_width_mm") in critical_pairs
    assert ("shoulder_mount", "wall_thickness_mm") in critical_pairs
    assert ("upper_arm", "length_mm") in critical_pairs
    assert ("upper_arm", "printer_id") in all_pairs


def test_auto_match_interfaces_fixes_mismatches():
    req = ModularArmRequest(
        payload_kg=1.0,
        material_id="petg_cf",
        printer_id="bambu_x1c",
        slots=ModularArmSlots(
            shoulder_mount=MountDesignRequest(actuator_type="nema23", wall_thickness_mm=6.5),
            shoulder_mount_variant="mount_gusseted",
            shoulder_gear=GearDesignRequest(teeth=30, module_mm=2.0, face_width_mm=12.0, bore_diameter_mm=5.0),
            upper_arm=LinkDesignRequest(
                length_mm=180.0,
                root_end=EndInterfaceParams(
                    preset="nema17",
                    outer_diameter_mm=46.0,
                    bore_diameter_mm=22.0,
                    bolt_count=4,
                    bolt_pcd_mm=31.0,
                    bolt_size="M3",
                ),
            ),
            elbow_gear=GearDesignRequest(teeth=24, module_mm=2.0, face_width_mm=10.0, bore_diameter_mm=5.0),
        ),
    )

    issues_before = validate_arm_compatibility(req)
    assert any(i.severity == "critical" for i in issues_before)

    fixed_req, notes = auto_match_interfaces(req)
    assert len(notes) >= 3
    assert fixed_req.slots.shoulder_gear.bore_diameter_mm == pytest.approx(6.35)
    assert fixed_req.slots.upper_arm.root_end.bolt_size == "M5"
    assert fixed_req.slots.upper_arm.root_end.bolt_pcd_mm == pytest.approx(66.7)
    assert fixed_req.slots.elbow_gear.bore_diameter_mm == pytest.approx(8.0)

    issues_after = validate_arm_compatibility(fixed_req)
    assert not any(i.severity == "critical" for i in issues_after)


def test_evaluate_modular_arm_organic_topopt_reduces_shoulder_torque_vs_solid():
    base_req = ModularArmRequest(
        payload_kg=0.8,
        material_id="petg_cf",
        printer_id="bambu_x1c",
        slots=ModularArmSlots(
            upper_arm_variant="tapered_solid",
            forearm_variant="tapered_solid",
        ),
    )
    vols_solid = {
        "base_pedestal": 120_000.0,
        "shoulder_mount": 22_000.0,
        "shoulder_gear": 14_000.0,
        "upper_arm_link": 140_000.0,
        "elbow_gear": 9_000.0,
        "forearm_link": 100_000.0,
        "wrist_tube": 26_000.0,
    }
    res_solid = evaluate_modular_arm(base_req, vols_solid)

    topopt_req = base_req.model_copy(
        update={
            "slots": base_req.slots.model_copy(
                update={
                    "upper_arm_variant": "organic_topopt",
                    "forearm_variant": "organic_topopt",
                }
            )
        }
    )
    # Realistic 38% volume reduction on bionic SIMP links
    vols_topopt = dict(vols_solid)
    vols_topopt["upper_arm_link"] = vols_solid["upper_arm_link"] * 0.62
    vols_topopt["forearm_link"] = vols_solid["forearm_link"] * 0.62
    res_topopt = evaluate_modular_arm(topopt_req, vols_topopt)

    assert res_topopt["total_mass_g"] < res_solid["total_mass_g"]
    assert res_topopt["shoulder_torque_nm"] < res_solid["shoulder_torque_nm"]
    assert res_solid["compatible"] is True
    assert res_topopt["compatible"] is True
    assert len(res_topopt["per_part_metrics"]) == 5
    assert res_topopt["min_safety_factor"] > 1.5
    assert len(res_topopt["joints"]) == 3
    assert len(res_topopt["bom"]) >= 5
