from __future__ import annotations

import math

from ..schemas import (
    BOMItem,
    CompatibilityIssue,
    EndInterfaceParams,
    GearDesignRequest,
    JointTorqueSummary,
    LinkDesignRequest,
    ModularArmRequest,
    ModularArmSlots,
    MountDesignRequest,
    PartPhysicsMetric,
)
from .arm_kinematics import GEARBOX_EFFICIENCY, GRAVITY_M_S2, _recommend_actuator
from .gear_mechanics import compute_gear_diameters, evaluate_gear_metrics
from .mechanics import evaluate_variant_metrics, load_materials
from .mount_mechanics import PLATE_WIDTH_MM, evaluate_mount_metrics
from .profiles import evaluate_printer_dfm_alerts

# Canonical mechanical interface specifications per actuator type
ACTUATOR_INTERFACE_SPECS: dict[str, dict] = {
    "nema17": {
        "shaft_diameter_mm": 5.0,
        "pilot_diameter_mm": 22.0,
        "outer_diameter_mm": 46.0,
        "bolt_count": 4,
        "bolt_pcd_mm": 31.0,
        "bolt_size": "M3",
        "end_preset": "nema17",
        "motor_mass_g": 320.0,
    },
    "nema23": {
        "shaft_diameter_mm": 6.35,
        "pilot_diameter_mm": 38.1,
        "outer_diameter_mm": 78.0,
        "bolt_count": 4,
        "bolt_pcd_mm": 66.7,
        "bolt_size": "M5",
        "end_preset": "nema23",
        "motor_mass_g": 680.0,
    },
    "servo_mg996r": {
        "shaft_diameter_mm": 6.0,
        "pilot_diameter_mm": 22.0,
        "outer_diameter_mm": 36.0,
        "bolt_count": 4,
        "bolt_pcd_mm": 26.0,
        "bolt_size": "M3",
        "end_preset": "custom",
        "motor_mass_g": 65.0,
    },
    "profile_2020_gusset": {
        "shaft_diameter_mm": 5.0,
        "pilot_diameter_mm": 22.0,
        "outer_diameter_mm": 38.0,
        "bolt_count": 2,
        "bolt_pcd_mm": 28.0,
        "bolt_size": "M5",
        "end_preset": "custom",
        "motor_mass_g": 280.0,
    },
}

SHAFT_TOLERANCE_MM = 0.20


def _expected_bearing_shaft_mm(end: EndInterfaceParams) -> float:
    """Return expected mating shaft diameter (mm) for a link end interface."""
    if end.preset == "bearing_608":
        return 8.0
    if end.preset == "bearing_6805":
        return 25.0
    if end.preset == "nema17":
        return 5.0
    if end.preset == "nema23":
        return 6.35
    return float(end.bore_diameter_mm)


def resolve_default_slots(req: ModularArmRequest) -> ModularArmSlots:
    """
    Fill any unspecified slots in ModularArmRequest.slots with mechanically
    consistent defaults inheriting req.material_id and req.printer_id.
    """
    s = req.slots
    mat_id = req.material_id
    printer_id = req.printer_id

    mech_bus = getattr(getattr(req, "mechatronics", None), "bus_architecture", "") or ""
    is_compact_msla = (
        printer_id == "elegoo_mars_5_ultra"
        or mech_bus == "esp32_pca9685_5v"
        or float(getattr(req, "base_height_mm", 95.0)) <= 66.0
    )

    shoulder_mount = s.shoulder_mount or MountDesignRequest(
        actuator_type="servo_mg996r" if is_compact_msla else "nema17",
        base_type="profile_2020",
        wall_thickness_mm=4.5 if is_compact_msla else 6.0,
        base_depth_mm=34.0 if is_compact_msla else 44.0,
        shaft_height_mm=26.0 if is_compact_msla else 34.0,
        gusset_thickness_mm=3.5 if is_compact_msla else 4.5,
        radial_load_n=min(500.0, max(25.0, req.payload_kg * 35.0)),
        motor_torque_nm=min(20.0, max(0.6, req.payload_kg * 0.8)),
        material_id=mat_id,
        printer_id=printer_id,
    )
    act_spec = ACTUATOR_INTERFACE_SPECS.get(
        shoulder_mount.actuator_type, ACTUATOR_INTERFACE_SPECS["nema17"]
    )

    shoulder_gear = s.shoulder_gear or GearDesignRequest(
        preset="spur_nema17",
        gear_kind="spur_involute",
        teeth=30,
        module_mm=2.0,
        face_width_mm=12.0,
        bore_diameter_mm=act_spec["shaft_diameter_mm"],
        hub_diameter_mm=max(16.0, act_spec["shaft_diameter_mm"] + 10.0),
        hub_height_mm=5.0,
        torque_nm=min(50.0, max(1.2, req.payload_kg * 1.5)),
        material_id=mat_id,
        printer_id=printer_id,
    )

    upper_arm = s.upper_arm or LinkDesignRequest(
        architecture="articulated_6dof",
        length_mm=100.0 if is_compact_msla else 200.0,
        root_width_mm=26.0 if is_compact_msla else 42.0,
        tip_width_mm=20.0 if is_compact_msla else 28.0,
        thickness_mm=14.0 if is_compact_msla else 22.0,
        root_end=EndInterfaceParams(
            preset="custom" if is_compact_msla else act_spec["end_preset"],
            outer_diameter_mm=32.0 if is_compact_msla else act_spec["outer_diameter_mm"],
            bore_diameter_mm=12.0 if is_compact_msla else act_spec["pilot_diameter_mm"],
            bolt_count=act_spec["bolt_count"],
            bolt_pcd_mm=act_spec["bolt_pcd_mm"],
            bolt_size=act_spec["bolt_size"],
            use_heat_inserts=not is_compact_msla,
        ),
        tip_end=EndInterfaceParams(
            preset="custom" if is_compact_msla else "bearing_608",
            outer_diameter_mm=22.0 if is_compact_msla else 32.0,
            bore_diameter_mm=10.0 if is_compact_msla else 22.0,
            bolt_count=0,
            bolt_pcd_mm=16.0 if is_compact_msla else 26.0,
            bolt_size="M3",
            use_heat_inserts=False,
        ),
        tip_load_n=min(500.0, max(20.0, req.payload_kg * 22.0)),
        material_id=mat_id,
        printer_id=printer_id,
    )

    elbow_shaft = _expected_bearing_shaft_mm(upper_arm.tip_end)
    elbow_gear = s.elbow_gear or GearDesignRequest(
        preset="spur_nema17",
        gear_kind="spur_involute",
        teeth=24,
        module_mm=2.0,
        face_width_mm=10.0,
        bore_diameter_mm=elbow_shaft,
        hub_diameter_mm=max(16.0, elbow_shaft + 8.0),
        hub_height_mm=5.0,
        torque_nm=min(50.0, max(0.8, req.payload_kg * 0.9)),
        material_id=mat_id,
        printer_id=printer_id,
    )

    forearm = s.forearm or LinkDesignRequest(
        architecture="articulated_6dof",
        length_mm=95.0 if is_compact_msla else 170.0,
        root_width_mm=22.0 if is_compact_msla else 34.0,
        tip_width_mm=16.0 if is_compact_msla else 22.0,
        thickness_mm=12.0 if is_compact_msla else 18.0,
        root_end=EndInterfaceParams(
            preset=upper_arm.tip_end.preset,
            outer_diameter_mm=upper_arm.tip_end.outer_diameter_mm,
            bore_diameter_mm=upper_arm.tip_end.bore_diameter_mm,
            bolt_count=0,
            bolt_pcd_mm=upper_arm.tip_end.bolt_pcd_mm,
            bolt_size=upper_arm.tip_end.bolt_size,
            use_heat_inserts=False,
        ),
        tip_end=EndInterfaceParams(
            preset="bearing_608",
            outer_diameter_mm=30.0,
            bore_diameter_mm=22.0,
            bolt_count=0,
            bolt_pcd_mm=25.0,
            bolt_size="M3",
            use_heat_inserts=False,
        ),
        tip_load_n=min(500.0, max(12.0, req.payload_kg * 12.0)),
        material_id=mat_id,
        printer_id=printer_id,
    )

    return ModularArmSlots(
        shoulder_mount=shoulder_mount,
        shoulder_mount_variant=s.shoulder_mount_variant,
        shoulder_gear=shoulder_gear,
        shoulder_gear_variant=s.shoulder_gear_variant,
        upper_arm=upper_arm,
        upper_arm_variant=s.upper_arm_variant,
        elbow_gear=elbow_gear,
        elbow_gear_variant=s.elbow_gear_variant,
        forearm=forearm,
        forearm_variant=s.forearm_variant,
    )


def _estimate_slot_volumes_mm3(slots: ModularArmSlots) -> dict[str, float]:
    """Analytical volume fallback (mm^3) when CAD volumes are not yet provided."""
    ua = slots.upper_arm
    fa = slots.forearm
    sm = slots.shoulder_mount
    sg = slots.shoulder_gear
    eg = slots.elbow_gear
    assert ua and fa and sm and sg and eg

    link_fill = {
        "tapered_solid": 0.92,
        "hollow_cobot": 0.62,
        "organic_ibeam": 0.58,
        "rounded_truss": 0.54,
        "organic_topopt": 0.52,
    }
    ua_vol = (
        ua.length_mm
        * (0.5 * (ua.root_width_mm + ua.tip_width_mm))
        * ua.thickness_mm
        * link_fill.get(slots.upper_arm_variant, 0.65)
    )
    fa_vol = (
        fa.length_mm
        * (0.5 * (fa.root_width_mm + fa.tip_width_mm))
        * fa.thickness_mm
        * link_fill.get(slots.forearm_variant, 0.65)
    )
    w_m = PLATE_WIDTH_MM[sm.actuator_type]
    mount_vol = w_m * (sm.base_depth_mm + sm.shaft_height_mm + w_m * 0.4) * sm.wall_thickness_mm
    dp_s, _ = compute_gear_diameters(sg)
    dp_e, _ = compute_gear_diameters(eg)
    sg_vol = math.pi * ((dp_s * 0.5) ** 2 - (sg.bore_diameter_mm * 0.5) ** 2) * sg.face_width_mm * 0.72
    eg_vol = math.pi * ((dp_e * 0.5) ** 2 - (eg.bore_diameter_mm * 0.5) ** 2) * eg.face_width_mm * 0.72

    return {
        "base_pedestal": 125_000.0,
        "shoulder_mount": max(mount_vol, 8_000.0),
        "shoulder_gear": max(sg_vol, 4_000.0),
        "upper_arm_link": max(ua_vol, 15_000.0),
        "elbow_gear": max(eg_vol, 3_000.0),
        "forearm_link": max(fa_vol, 12_000.0),
        "wrist_tube": 28_000.0,
    }


def _lookup_vol(real_volumes_mm3: dict[str, float], keys: tuple[str, ...], fallback: float) -> float:
    for k in keys:
        if k in real_volumes_mm3 and real_volumes_mm3[k] > 0:
            return float(real_volumes_mm3[k])
    return fallback


def _effective_gravity_and_drag(
    req: ModularArmRequest, density_g_cm3: float, reach_m: float
) -> tuple[float, float]:
    """
    Compute (g_eff_m_s2, morison_drag_torque_nm) for earth, space_zero_g, or underwater_rov.
    """
    if req.environment_mode == "space_zero_g":
        return 0.0, 0.0
    if req.environment_mode == "underwater_rov":
        rho_water = 1025.0
        rho_mat = max(rho_water + 100.0, density_g_cm3 * 1000.0)
        buoyancy_factor = max(0.08, 1.0 - (rho_water / rho_mat))
        g_eff = float(req.gravity_m_s2) * buoyancy_factor
        # Morison hydrodynamic drag torque at ~0.5 m/s tip sweep speed (Cd=1.1, proj area ~0.04*L)
        v_sweep = 0.5
        drag_force_n = 0.5 * rho_water * 1.1 * (0.04 * max(reach_m, 0.2)) * (v_sweep**2)
        drag_torque_nm = drag_force_n * (reach_m * 0.6)
        return g_eff, drag_torque_nm
    return float(req.gravity_m_s2), 0.0


def _compute_cascaded_torques(
    req: ModularArmRequest,
    slots: ModularArmSlots,
    masses_g: dict[str, float],
    density_g_cm3: float = 1.29,
) -> dict[str, float]:
    """
    Compute worst-case joint torques & effective structural loads for all 8
    kinematic topologies and 3 physical environments (earth, space_zero_g, underwater_rov).
    """
    ua = slots.upper_arm
    fa = slots.forearm
    sm = slots.shoulder_mount
    assert ua and fa and sm

    l1_m = float(ua.length_mm) / 1000.0
    l2_m = float(fa.length_mm) / 1000.0
    l3_m = float(req.wrist_mm) / 1000.0
    payload_kg = float(req.payload_kg)
    alpha = float(req.max_accel_rad_s2)
    g_eff, drag_nm = _effective_gravity_and_drag(req, density_g_cm3, l1_m + l2_m + l3_m)

    m_upper_kg = masses_g["upper_arm_link"] / 1000.0
    m_fore_kg = masses_g["forearm_link"] / 1000.0
    m_wrist_kg = masses_g["wrist_tube"] / 1000.0
    m_elbow_gear_kg = masses_g["elbow_gear"] / 1000.0

    act_spec = ACTUATOR_INTERFACE_SPECS.get(sm.actuator_type, ACTUATOR_INTERFACE_SPECS["nema17"])
    m_mot_shoulder_kg = max(float(act_spec["motor_mass_g"]) / 1000.0, 0.32 + 0.18 * min(payload_kg, 25.0))
    m_mot_elbow_kg = max(0.28, m_mot_shoulder_kg * 0.68)
    m_mot_wrist_kg = max(0.16, m_mot_shoulder_kg * 0.38)

    ktype = req.kinematics_type

    # 1. Stewart Hexapod 6-UPS (6 parallel linear actuators)
    if ktype == "stewart_hexapod":
        a_lin = min(15.0, max(1.0, alpha * 0.20))
        m_moving_kg = payload_kg + 1.2  # moving top platform + legs share
        f_leg_n = (m_moving_kg * (g_eff + a_lin)) / 6.0
        f_leg_static_n = (m_moving_kg * g_eff) / 6.0
        # Leadscrew pitch ~8mm -> torque = F * p / (2*pi*eta)
        tau_s1 = max(0.05, f_leg_static_n * 0.0022)
        tau_d1 = max(0.08, f_leg_n * 0.0022)
        return {
            "tau_s1": tau_s1,
            "tau_d1": tau_d1,
            "tau_s2": tau_s1,
            "tau_d2": tau_d1,
            "tau_s3": tau_s1,
            "tau_d3": tau_d1,
            "f_upper_tip_n": max(1.0, min(80.0, f_leg_n * 0.4)),
            "f_forearm_tip_n": max(1.0, min(60.0, f_leg_n * 0.3)),
            "m_mot_shoulder_g": 280.0,
            "m_mot_elbow_g": 280.0,
            "m_mot_wrist_g": 280.0,
        }

    # 2. Soft Pneumatic Bellows (Festo BHA: 3 TPU chambers @ 120 deg, 0.6-1.5 bar)
    if ktype == "soft_pneumatic_bellows":
        pressure_mpa = 0.12  # 1.2 bar nominal pneumatic pressure
        area_mm2 = math.pi * (18.0**2) / 4.0  # Ø18 mm effective bellows chamber
        f_bellows_n = pressure_mpa * area_mm2  # ~30.5 N
        tau_bend_nm = max(0.15, (f_bellows_n * 0.022) + payload_kg * g_eff * (l1_m * 0.5))
        return {
            "tau_s1": round(tau_bend_nm * 0.8, 3),
            "tau_d1": round(tau_bend_nm, 3),
            "tau_s2": round(tau_bend_nm * 0.55, 3),
            "tau_d2": round(tau_bend_nm * 0.7, 3),
            "tau_s3": 0.12,
            "tau_d3": 0.18,
            "f_upper_tip_n": max(1.0, min(25.0, f_bellows_n * 0.25)),
            "f_forearm_tip_n": max(1.0, min(18.0, f_bellows_n * 0.18)),
            "m_mot_shoulder_g": 85.0,
            "m_mot_elbow_g": 85.0,
            "m_mot_wrist_g": 65.0,
        }

    # 3. Delta Parallel 3-Arm Kinematics (ABB FlexPicker)
    if ktype == "delta_parallel":
        l_bicep_m = max(0.08, min(l1_m, 0.26))
        a_lin = min(25.0, max(2.0, alpha * 0.22))
        m_moving_kg = payload_kg + 0.24
        f_arm_n = (m_moving_kg * (g_eff + a_lin)) / 3.0
        f_arm_static_n = (m_moving_kg * g_eff) / 3.0
        tau_s1 = f_arm_static_n * l_bicep_m
        tau_d1 = f_arm_n * l_bicep_m
        tau_s2 = tau_s1 * 0.35
        tau_d2 = tau_d1 * 0.35
        tau_s3 = max(0.08, payload_kg * g_eff * 0.025)
        tau_d3 = tau_s3 * 1.4
        return {
            "tau_s1": tau_s1,
            "tau_d1": tau_d1,
            "tau_s2": tau_s2,
            "tau_d2": tau_d2,
            "tau_s3": tau_s3,
            "tau_d3": tau_d3,
            "f_upper_tip_n": max(1.0, min(120.0, f_arm_n * 0.55)),
            "f_forearm_tip_n": max(1.0, min(80.0, f_arm_n * 0.35)),
            "m_mot_shoulder_g": m_mot_shoulder_kg * 1000.0,
            "m_mot_elbow_g": m_mot_shoulder_kg * 1000.0,
            "m_mot_wrist_g": m_mot_shoulder_kg * 1000.0,
        }

    # 4. Cartesian Gantry P-P-P-R (LumenPnP / OpenPnP)
    if ktype == "cartesian_gantry":
        mu_rail = 0.04
        a_lin = min(15.0, max(1.5, alpha * 0.25))
        m_head_kg = payload_kg + 0.35
        m_bridge_kg = m_head_kg + 0.65
        f_x_n = m_head_kg * a_lin + mu_rail * m_head_kg * g_eff
        f_y_n = m_bridge_kg * a_lin + mu_rail * m_bridge_kg * g_eff
        f_z_n = m_head_kg * (g_eff + a_lin * 0.5)
        r_pulley_m = 0.00637
        tau_s1 = max(0.05, mu_rail * m_bridge_kg * g_eff * r_pulley_m)
        tau_d1 = max(0.12, f_y_n * r_pulley_m)
        tau_s2 = max(0.04, mu_rail * m_head_kg * g_eff * r_pulley_m)
        tau_d2 = max(0.08, f_x_n * r_pulley_m)
        tau_s3 = max(0.04, m_head_kg * g_eff * 0.004)
        tau_d3 = max(0.06, f_z_n * 0.004)
        return {
            "tau_s1": tau_s1,
            "tau_d1": tau_d1,
            "tau_s2": tau_s2,
            "tau_d2": tau_d2,
            "tau_s3": tau_s3,
            "tau_d3": tau_d3,
            "f_upper_tip_n": max(1.0, min(60.0, f_y_n)),
            "f_forearm_tip_n": max(1.0, min(40.0, f_z_n)),
            "m_mot_shoulder_g": m_mot_shoulder_kg * 1000.0,
            "m_mot_elbow_g": m_mot_elbow_kg * 1000.0,
            "m_mot_wrist_g": m_mot_wrist_kg * 1000.0,
        }

    # 5. Continuum PCC (Nitinol flexible backbone + 3 tendons @ 120 deg)
    if ktype == "continuum_pcc":
        e_nitinol_mpa = 75_000.0
        d_mm = 2.0
        i_backbone_mm4 = (math.pi * (d_mm**4)) / 64.0
        l_backbone_mm = max(40.0, float(ua.length_mm))
        bend_rad = math.radians(max(20.0, abs(float(req.joint_angles_deg[0])) + 35.0))
        kappa_inv_mm = bend_rad / l_backbone_mm
        m_flex_n_mm = e_nitinol_mpa * i_backbone_mm4 * kappa_inv_mm
        m_flex_nm = m_flex_n_mm / 1000.0
        r_tendon_mm = 4.5
        f_tendon_n = m_flex_n_mm / r_tendon_mm
        tau_d1 = max(0.08, m_flex_nm + payload_kg * g_eff * (l_backbone_mm / 1000.0))
        return {
            "tau_s1": max(0.05, m_flex_nm),
            "tau_d1": tau_d1,
            "tau_s2": max(0.04, m_flex_nm * 0.7),
            "tau_d2": max(0.06, tau_d1 * 0.7),
            "tau_s3": 0.03,
            "tau_d3": 0.05,
            "f_upper_tip_n": max(1.0, min(25.0, f_tendon_n * 0.15)),
            "f_forearm_tip_n": max(1.0, min(15.0, f_tendon_n * 0.10)),
            "m_mot_shoulder_g": 120.0,
            "m_mot_elbow_g": 120.0,
            "m_mot_wrist_g": 80.0,
        }

    # 6. Serial Articulated / SCARA / Bimanual Dual-Arm
    arm_payload_kg = payload_kg * 0.5 if ktype == "bimanual_dual_arm" else payload_kg
    if req.remote_base_motors:
        m_mot_elbow_moving = 0.0
        m_mot_wrist_moving = 0.0
        m_elbow_gear_moving = 0.0
    else:
        m_mot_elbow_moving = m_mot_elbow_kg
        m_mot_wrist_moving = m_mot_wrist_kg
        m_elbow_gear_moving = m_elbow_gear_kg

    # J3 Wrist
    tau_s3 = g_eff * (m_wrist_kg * (l3_m * 0.5) + arm_payload_kg * l3_m)
    i_rot3 = m_wrist_kg * (l3_m**2) / 3.0 + arm_payload_kg * (l3_m**2)
    tau_d3 = tau_s3 + i_rot3 * alpha + drag_nm * 0.15

    # J2 Elbow
    tau_s2 = g_eff * (
        m_fore_kg * (l2_m * 0.5)
        + m_mot_wrist_moving * l2_m
        + m_wrist_kg * (l2_m + l3_m * 0.5)
        + arm_payload_kg * (l2_m + l3_m)
    )
    i_rot2 = (
        m_fore_kg * (l2_m**2) / 3.0
        + m_mot_wrist_moving * (l2_m**2)
        + m_wrist_kg * ((l2_m + l3_m * 0.5) ** 2)
        + arm_payload_kg * ((l2_m + l3_m) ** 2)
    )
    tau_d2 = tau_s2 + i_rot2 * alpha + drag_nm * 0.40

    # J1 Shoulder
    tau_s1 = g_eff * (
        m_upper_kg * (l1_m * 0.5)
        + (m_mot_elbow_moving + m_elbow_gear_moving) * l1_m
        + m_fore_kg * (l1_m + l2_m * 0.5)
        + m_mot_wrist_moving * (l1_m + l2_m)
        + m_wrist_kg * (l1_m + l2_m + l3_m * 0.5)
        + arm_payload_kg * (l1_m + l2_m + l3_m)
    )
    # Hydropneumatic J2 counterbalance (KUKA KR 210) offsets 65% of static gravity torque
    if req.counterbalance_j2:
        tau_s1 *= 0.35
        tau_s2 *= 0.35

    i_rot1 = (
        m_upper_kg * (l1_m**2) / 3.0
        + (m_mot_elbow_moving + m_elbow_gear_moving) * (l1_m**2)
        + m_fore_kg * ((l1_m + l2_m * 0.5) ** 2)
        + m_mot_wrist_moving * ((l1_m + l2_m) ** 2)
        + m_wrist_kg * ((l1_m + l2_m + l3_m * 0.5) ** 2)
        + arm_payload_kg * ((l1_m + l2_m + l3_m) ** 2)
    )
    tau_d1 = tau_s1 + i_rot1 * alpha + drag_nm

    # Equivalent tip forces (N) acting on upper arm and forearm roots
    f_forearm_tip_n = g_eff * (arm_payload_kg + m_wrist_kg + m_mot_wrist_moving) + (
        (i_rot2 * alpha) / max(l2_m, 0.05)
    ) * 0.35
    f_upper_tip_n = g_eff * (
        arm_payload_kg + m_wrist_kg + m_mot_wrist_moving + m_fore_kg + m_mot_elbow_moving + m_elbow_gear_moving
    ) + ((i_rot1 * alpha) / max(l1_m, 0.05)) * 0.35

    if req.counterbalance_j2:
        f_upper_tip_n *= 0.35
    if req.remote_base_motors:
        f_upper_tip_n *= 0.55
        f_forearm_tip_n *= 0.65

    return {
        "tau_s1": tau_s1,
        "tau_d1": tau_d1,
        "tau_s2": tau_s2,
        "tau_d2": tau_d2,
        "tau_s3": tau_s3,
        "tau_d3": tau_d3,
        "f_upper_tip_n": max(1.0, min(500.0, f_upper_tip_n)),
        "f_forearm_tip_n": max(1.0, min(500.0, f_forearm_tip_n)),
        "m_mot_shoulder_g": m_mot_shoulder_kg * 1000.0,
        "m_mot_elbow_g": m_mot_elbow_kg * 1000.0,
        "m_mot_wrist_g": m_mot_wrist_kg * 1000.0,
    }


def validate_arm_compatibility(
    req: ModularArmRequest,
    real_volumes_mm3: dict[str, float] | None = None,
) -> list[CompatibilityIssue]:
    """
    Validate mechanical compatibility across all modular arm slots:
    1. Actuator shaft vs shoulder gear bore (±0.2 mm).
    2. Elbow gear bore vs link bearing shaft & upper/forearm joint bore match.
    3. Bolt pattern (count, PCD, M3/M4/M5) between shoulder mount & upper arm root.
    4. Cascaded torque vs individual component strength (Lewis gear, mount corner, links).
    5. Printer build volume & enclosure compatibility for every printable slot.
    """
    slots = resolve_default_slots(req)
    sm = slots.shoulder_mount
    sg = slots.shoulder_gear
    ua = slots.upper_arm
    eg = slots.elbow_gear
    fa = slots.forearm
    assert sm and sg and ua and eg and fa

    materials = load_materials()
    material = materials.get(req.material_id) or materials["petg_cf"]
    density = float(material["density_g_cm3"])

    est_vols = _estimate_slot_volumes_mm3(slots)
    vols = {
        k: _lookup_vol(real_volumes_mm3 or {}, (k,), est_vols[k])
        for k in est_vols
    }
    masses_g = {k: (v / 1000.0) * density for k, v in vols.items()}
    torques = _compute_cascaded_torques(req, slots, masses_g, density_g_cm3=density)

    issues: list[CompatibilityIssue] = []
    act_spec = ACTUATOR_INTERFACE_SPECS.get(sm.actuator_type, ACTUATOR_INTERFACE_SPECS["nema17"])

    # Non-serial topologies validate their own specialized mechanics cleanly
    if req.kinematics_type in (
        "continuum_pcc",
        "cartesian_gantry",
        "delta_parallel",
        "stewart_hexapod",
        "soft_pneumatic_bellows",
    ):
        return issues

    # 1. Shoulder motor shaft vs shoulder gear bore (±0.2 mm)
    expected_shaft = float(act_spec["shaft_diameter_mm"])
    if abs(sg.bore_diameter_mm - expected_shaft) > SHAFT_TOLERANCE_MM:
        issues.append(
            CompatibilityIssue(
                severity="critical",
                slot="shoulder_gear",
                parameter="bore_diameter_mm",
                current_value=f"{sg.bore_diameter_mm:.2f} mm",
                suggested_value=f"{expected_shaft:.2f} mm",
                message=(
                    f"El agujero del engranaje de hombro ({sg.bore_diameter_mm:.2f} mm) no coincide "
                    f"con el eje de {sm.actuator_type.upper()} ({expected_shaft:.2f} mm, tol ±0.20 mm)."
                ),
            )
        )

    # 2. Gear hub vs upper arm root outer boss & elbow shaft compatibility
    if sg.hub_diameter_mm > ua.root_end.outer_diameter_mm:
        issues.append(
            CompatibilityIssue(
                severity="warning",
                slot="shoulder_gear",
                parameter="hub_diameter_mm",
                current_value=f"{sg.hub_diameter_mm:.1f} mm",
                suggested_value=f"{ua.root_end.outer_diameter_mm - 4.0:.1f} mm",
                message=(
                    f"El cubo del engranaje de hombro ({sg.hub_diameter_mm:.1f} mm) excede el jefe "
                    f"raíz del brazo superior ({ua.root_end.outer_diameter_mm:.1f} mm)."
                ),
            )
        )

    expected_elbow_shaft = _expected_bearing_shaft_mm(ua.tip_end)
    if abs(eg.bore_diameter_mm - expected_elbow_shaft) > SHAFT_TOLERANCE_MM:
        issues.append(
            CompatibilityIssue(
                severity="critical",
                slot="elbow_gear",
                parameter="bore_diameter_mm",
                current_value=f"{eg.bore_diameter_mm:.2f} mm",
                suggested_value=f"{expected_elbow_shaft:.2f} mm",
                message=(
                    f"El eje del engranaje de codo ({eg.bore_diameter_mm:.2f} mm) no coincide con "
                    f"el eje del rodamiento/interfaz de punta del brazo superior ({expected_elbow_shaft:.2f} mm)."
                ),
            )
        )

    if abs(ua.tip_end.bore_diameter_mm - fa.root_end.bore_diameter_mm) > SHAFT_TOLERANCE_MM:
        issues.append(
            CompatibilityIssue(
                severity="warning",
                slot="forearm",
                parameter="root_end.bore_diameter_mm",
                current_value=f"{fa.root_end.bore_diameter_mm:.1f} mm",
                suggested_value=f"{ua.tip_end.bore_diameter_mm:.1f} mm",
                message=(
                    f"El alojamiento raíz del antebrazo ({fa.root_end.bore_diameter_mm:.1f} mm) "
                    f"difiere de la punta del brazo superior ({ua.tip_end.bore_diameter_mm:.1f} mm)."
                ),
            )
        )

    # 3. Bolt pattern PCD / count / size between shoulder mount and upper arm root
    if ua.root_end.bolt_count > 0:
        pcd_diff = abs(ua.root_end.bolt_pcd_mm - float(act_spec["bolt_pcd_mm"]))
        size_mismatch = ua.root_end.bolt_size != act_spec["bolt_size"]
        count_mismatch = ua.root_end.bolt_count != int(act_spec["bolt_count"])
        if pcd_diff > 0.5 or size_mismatch or count_mismatch:
            issues.append(
                CompatibilityIssue(
                    severity="critical",
                    slot="upper_arm",
                    parameter="root_end.bolt_pcd_mm",
                    current_value=f"{ua.root_end.bolt_count}×{ua.root_end.bolt_size} @ Ø{ua.root_end.bolt_pcd_mm:.1f} mm",
                    suggested_value=(
                        f"{act_spec['bolt_count']}×{act_spec['bolt_size']} @ Ø{act_spec['bolt_pcd_mm']:.1f} mm"
                    ),
                    message=(
                        f"El patrón de pernos de la raíz del brazo superior no coincide con la brida "
                        f"{sm.actuator_type.upper()} de la montura de hombro."
                    ),
                )
            )

    # 4. Cascaded torque vs component structural strength
    sg_loaded = sg.model_copy(
        update={
            "torque_nm": max(0.05, min(50.0, torques["tau_d1"] / max(1.0, sg.teeth / 12.0))),
            "material_id": req.material_id,
        }
    )
    sg_metrics, _ = evaluate_gear_metrics(
        sg_loaded, slots.shoulder_gear_variant, vols["shoulder_gear"], None, material
    )
    if sg_metrics.safety_factor < 1.5:
        issues.append(
            CompatibilityIssue(
                severity="critical",
                slot="shoulder_gear",
                parameter="face_width_mm",
                current_value=f"FS {sg_metrics.safety_factor:.2f} (b={sg.face_width_mm:.0f} mm)",
                suggested_value=f"b ≥ {min(35.0, sg.face_width_mm * 1.5):.0f} mm / m ≥ {min(6.0, sg.module_mm + 0.5):.1f}",
                message=(
                    f"El engranaje de hombro tiene FS={sg_metrics.safety_factor:.2f} (< 1.5) bajo el "
                    f"torque en cascada ({torques['tau_d1']:.2f} N·m). Aumenta ancho de cara o módulo."
                ),
            )
        )

    sm_loaded = sm.model_copy(
        update={
            "radial_load_n": torques["f_upper_tip_n"],
            "motor_torque_nm": min(20.0, torques["tau_d1"] * 0.25),
            "material_id": req.material_id,
        }
    )
    sm_metrics, _ = evaluate_mount_metrics(
        sm_loaded, slots.shoulder_mount_variant, vols["shoulder_mount"], None, material
    )
    if sm_metrics.safety_factor < 1.5:
        issues.append(
            CompatibilityIssue(
                severity="critical",
                slot="shoulder_mount",
                parameter="wall_thickness_mm",
                current_value=f"FS {sm_metrics.safety_factor:.2f} (t={sm.wall_thickness_mm:.1f} mm)",
                suggested_value=f"t ≥ {min(16.0, sm.wall_thickness_mm + 2.5):.1f} mm (mount_gusseted)",
                message=(
                    f"La montura de hombro tiene FS={sm_metrics.safety_factor:.2f} (< 1.5) bajo la "
                    f"carga real del brazo ({torques['f_upper_tip_n']:.1f} N). Usa cartelas o mayor espesor."
                ),
            )
        )

    ua_loaded = ua.model_copy(
        update={"tip_load_n": torques["f_upper_tip_n"], "material_id": req.material_id}
    )
    ua_metrics, _ = evaluate_variant_metrics(
        ua_loaded, slots.upper_arm_variant, vols["upper_arm_link"], None, material
    )
    if ua_metrics.safety_factor < 1.5:
        issues.append(
            CompatibilityIssue(
                severity="critical",
                slot="upper_arm",
                parameter="root_width_mm",
                current_value=f"FS {ua_metrics.safety_factor:.2f} (h={ua.root_width_mm:.0f} mm)",
                suggested_value=f"h ≥ {min(110.0, ua.root_width_mm * 1.25):.0f} mm",
                message=(
                    f"El brazo superior tiene FS={ua_metrics.safety_factor:.2f} (< 1.5) bajo la "
                    f"carga en cascada ({torques['f_upper_tip_n']:.1f} N)."
                ),
            )
        )

    # 5. Printer bed & enclosure checks (FDM only — CNC aerospace/steel parts are not 3D printed)
    if req.printer_id and material.get("category") == "fdm":
        longest_x = max(
            ua.length_mm + 0.5 * (ua.root_end.outer_diameter_mm + ua.tip_end.outer_diameter_mm),
            fa.length_mm + 0.5 * (fa.root_end.outer_diameter_mm + fa.tip_end.outer_diameter_mm),
        )
        max_y = max(ua.root_width_mm, ua.root_end.outer_diameter_mm, fa.root_width_mm)
        max_z = max(ua.thickness_mm, fa.thickness_mm)
        for alert in evaluate_printer_dfm_alerts(req.printer_id, material, longest_x, max_y, max_z):
            issues.append(
                CompatibilityIssue(
                    severity="critical" if alert.level == "critical" else "warning",
                    slot="upper_arm",
                    parameter="printer_id" if alert.code == "PRINTER_ENCLOSURE_REQUIRED" else "length_mm",
                    current_value=f"{longest_x:.0f} mm / {req.printer_id}",
                    suggested_value="voron_24 o reducir longitud",
                    message=alert.message,
                )
            )

    return issues


def auto_match_interfaces(req: ModularArmRequest) -> tuple[ModularArmRequest, list[str]]:
    """
    Automatically align shaft bores, bolt PCD patterns, material/printer IDs,
    and under-strength gear/mount dimensions across all slots in one click.
    """
    slots = resolve_default_slots(req)
    sm = slots.shoulder_mount
    sg = slots.shoulder_gear
    ua = slots.upper_arm
    eg = slots.elbow_gear
    fa = slots.forearm
    assert sm and sg and ua and eg and fa

    notes: list[str] = []
    act_spec = ACTUATOR_INTERFACE_SPECS.get(sm.actuator_type, ACTUATOR_INTERFACE_SPECS["nema17"])

    # 1. Unify material & printer across all slots
    sm = sm.model_copy(update={"material_id": req.material_id, "printer_id": req.printer_id})
    sg = sg.model_copy(update={"material_id": req.material_id, "printer_id": req.printer_id})
    ua = ua.model_copy(update={"material_id": req.material_id, "printer_id": req.printer_id})
    eg = eg.model_copy(update={"material_id": req.material_id, "printer_id": req.printer_id})
    fa = fa.model_copy(update={"material_id": req.material_id, "printer_id": req.printer_id})

    # 2. Match shoulder gear bore to shoulder actuator shaft
    expected_shaft = float(act_spec["shaft_diameter_mm"])
    if abs(sg.bore_diameter_mm - expected_shaft) > SHAFT_TOLERANCE_MM:
        old_b = sg.bore_diameter_mm
        sg = sg.model_copy(
            update={
                "bore_diameter_mm": expected_shaft,
                "hub_diameter_mm": max(sg.hub_diameter_mm, expected_shaft + 8.0),
            }
        )
        notes.append(
            f"Engranaje hombro: bore_diameter_mm ajustado {old_b:.2f} → {expected_shaft:.2f} mm ({sm.actuator_type.upper()})."
        )

    # 3. Match upper arm root bolt pattern & bore to shoulder actuator flange
    new_root_end = ua.root_end.model_copy(
        update={
            "preset": act_spec["end_preset"],
            "outer_diameter_mm": max(ua.root_end.outer_diameter_mm, float(act_spec["outer_diameter_mm"])),
            "bore_diameter_mm": float(act_spec["pilot_diameter_mm"]),
            "bolt_count": int(act_spec["bolt_count"]),
            "bolt_pcd_mm": float(act_spec["bolt_pcd_mm"]),
            "bolt_size": act_spec["bolt_size"],
        }
    )
    if new_root_end != ua.root_end:
        ua = ua.model_copy(update={"root_end": new_root_end})
        notes.append(
            f"Brazo superior: patrón raíz alineado a {act_spec['bolt_count']}×{act_spec['bolt_size']} @ Ø{act_spec['bolt_pcd_mm']:.1f} mm."
        )

    # 4. Ensure shoulder gear hub fits within upper arm root boss
    if sg.hub_diameter_mm > ua.root_end.outer_diameter_mm:
        new_hub = max(10.0, ua.root_end.outer_diameter_mm - 4.0)
        sg = sg.model_copy(update={"hub_diameter_mm": new_hub})
        notes.append(f"Engranaje hombro: hub_diameter_mm ajustado a {new_hub:.1f} mm.")

    # 5. Match forearm root bore & elbow gear bore to upper arm tip interface
    if abs(fa.root_end.bore_diameter_mm - ua.tip_end.bore_diameter_mm) > SHAFT_TOLERANCE_MM:
        fa = fa.model_copy(
            update={
                "root_end": fa.root_end.model_copy(
                    update={
                        "preset": ua.tip_end.preset,
                        "bore_diameter_mm": ua.tip_end.bore_diameter_mm,
                        "outer_diameter_mm": max(
                            fa.root_end.outer_diameter_mm, ua.tip_end.outer_diameter_mm
                        ),
                    }
                )
            }
        )
        notes.append(
            f"Antebrazo: alojamiento raíz alineado a Ø{ua.tip_end.bore_diameter_mm:.1f} mm."
        )

    expected_elbow_shaft = _expected_bearing_shaft_mm(ua.tip_end)
    if abs(eg.bore_diameter_mm - expected_elbow_shaft) > SHAFT_TOLERANCE_MM:
        old_eb = eg.bore_diameter_mm
        eg = eg.model_copy(
            update={
                "bore_diameter_mm": expected_elbow_shaft,
                "hub_diameter_mm": max(eg.hub_diameter_mm, expected_elbow_shaft + 8.0),
            }
        )
        notes.append(
            f"Engranaje codo: bore_diameter_mm ajustado {old_eb:.2f} → {expected_elbow_shaft:.2f} mm."
        )

    # 6. Auto-reinforce under-strength mount/gear if needed
    mount_var = slots.shoulder_mount_variant
    updated_slots = ModularArmSlots(
        shoulder_mount=sm,
        shoulder_mount_variant=mount_var,
        shoulder_gear=sg,
        shoulder_gear_variant=slots.shoulder_gear_variant,
        upper_arm=ua,
        upper_arm_variant=slots.upper_arm_variant,
        elbow_gear=eg,
        elbow_gear_variant=slots.elbow_gear_variant,
        forearm=fa,
        forearm_variant=slots.forearm_variant,
    )
    probe_req = req.model_copy(update={"slots": updated_slots})
    post_issues = validate_arm_compatibility(probe_req)
    for iss in post_issues:
        if iss.severity != "critical":
            continue
        if iss.slot == "shoulder_mount" and iss.parameter == "wall_thickness_mm":
            sm = sm.model_copy(update={"wall_thickness_mm": min(16.0, sm.wall_thickness_mm + 2.5)})
            mount_var = "mount_gusseted"
            notes.append(
                f"Montura hombro reforzada: espesor {sm.wall_thickness_mm:.1f} mm con cartelas."
            )
        elif iss.slot == "shoulder_gear" and iss.parameter == "face_width_mm":
            sg = sg.model_copy(
                update={
                    "face_width_mm": min(35.0, round(sg.face_width_mm * 1.5, 1)),
                    "module_mm": min(6.0, round(sg.module_mm + 0.5, 2)),
                }
            )
            notes.append(
                f"Engranaje hombro reforzado: b={sg.face_width_mm:.1f} mm, m={sg.module_mm:.1f} mm."
            )
        elif iss.slot == "upper_arm" and iss.parameter == "root_width_mm":
            ua = ua.model_copy(
                update={
                    "root_width_mm": min(110.0, round(ua.root_width_mm * 1.25, 1)),
                    "thickness_mm": min(60.0, round(ua.thickness_mm * 1.15, 1)),
                }
            )
            notes.append(
                f"Brazo superior reforzado: raíz {ua.root_width_mm:.1f} mm × espesor {ua.thickness_mm:.1f} mm."
            )

    final_slots = ModularArmSlots(
        shoulder_mount=sm,
        shoulder_mount_variant=mount_var,
        shoulder_gear=sg,
        shoulder_gear_variant=slots.shoulder_gear_variant,
        upper_arm=ua,
        upper_arm_variant=slots.upper_arm_variant,
        elbow_gear=eg,
        elbow_gear_variant=slots.elbow_gear_variant,
        forearm=fa,
        forearm_variant=slots.forearm_variant,
    )
    if not notes:
        notes.append("Todas las interfaces mecánicas ya son compatibles.")
    return req.model_copy(update={"slots": final_slots}), notes


def evaluate_modular_arm(
    req: ModularArmRequest,
    real_volumes_mm3: dict[str, float],
) -> dict:
    """
    Propagate REAL CAD part volumes (mm^3 -> mass_g = vol * density / 1000,
    without double-counting actuator masses) in cascade from wrist to shoulder,
    and evaluate per-part safety factors, joint torques, compatibility issues,
    FK TCP position, and BOM.
    """
    slots = resolve_default_slots(req)
    sm = slots.shoulder_mount
    sg = slots.shoulder_gear
    ua = slots.upper_arm
    eg = slots.elbow_gear
    fa = slots.forearm
    assert sm and sg and ua and eg and fa

    materials = load_materials()
    material = materials.get(req.material_id) or materials["petg_cf"]
    density = float(material["density_g_cm3"])

    est_vols = _estimate_slot_volumes_mm3(slots)
    vols = {
        "base_pedestal": _lookup_vol(real_volumes_mm3, ("base_pedestal", "base"), est_vols["base_pedestal"]),
        "shoulder_mount": _lookup_vol(real_volumes_mm3, ("shoulder_mount",), est_vols["shoulder_mount"]),
        "shoulder_gear": _lookup_vol(real_volumes_mm3, ("shoulder_gear",), est_vols["shoulder_gear"]),
        "upper_arm_link": _lookup_vol(real_volumes_mm3, ("upper_arm_link", "upper_arm"), est_vols["upper_arm_link"]),
        "elbow_gear": _lookup_vol(real_volumes_mm3, ("elbow_gear",), est_vols["elbow_gear"]),
        "forearm_link": _lookup_vol(real_volumes_mm3, ("forearm_link", "forearm"), est_vols["forearm_link"]),
        "wrist_tube": _lookup_vol(real_volumes_mm3, ("wrist_tube", "wrist"), est_vols["wrist_tube"]),
    }
    masses_g = {k: (v / 1000.0) * density for k, v in vols.items()}

    torques = _compute_cascaded_torques(req, slots, masses_g, density_g_cm3=density)
    issues = validate_arm_compatibility(req, real_volumes_mm3=vols)

    # Evaluate individual real parts under actual cascaded loads
    sm_metrics, _ = evaluate_mount_metrics(
        sm.model_copy(
            update={
                "radial_load_n": torques["f_upper_tip_n"],
                "motor_torque_nm": min(20.0, torques["tau_d1"] * 0.25),
                "material_id": req.material_id,
            }
        ),
        slots.shoulder_mount_variant,
        vols["shoulder_mount"],
        None,
        material,
    )
    sg_metrics, _ = evaluate_gear_metrics(
        sg.model_copy(
            update={
                "torque_nm": max(0.05, min(50.0, torques["tau_d1"] / max(1.0, sg.teeth / 12.0))),
                "material_id": req.material_id,
            }
        ),
        slots.shoulder_gear_variant,
        vols["shoulder_gear"],
        None,
        material,
    )
    ua_metrics, _ = evaluate_variant_metrics(
        ua.model_copy(update={"tip_load_n": torques["f_upper_tip_n"], "material_id": req.material_id}),
        slots.upper_arm_variant,
        vols["upper_arm_link"],
        None,
        material,
    )
    eg_metrics, _ = evaluate_gear_metrics(
        eg.model_copy(
            update={
                "torque_nm": max(0.05, min(50.0, torques["tau_d2"] / max(1.0, eg.teeth / 12.0))),
                "material_id": req.material_id,
            }
        ),
        slots.elbow_gear_variant,
        vols["elbow_gear"],
        None,
        material,
    )
    fa_metrics, _ = evaluate_variant_metrics(
        fa.model_copy(update={"tip_load_n": torques["f_forearm_tip_n"], "material_id": req.material_id}),
        slots.forearm_variant,
        vols["forearm_link"],
        None,
        material,
    )

    def _status(sf: float) -> str:
        if sf < 1.5:
            return "critical"
        if sf < 2.0:
            return "warning"
        return "ok"

    per_part_metrics = [
        PartPhysicsMetric(
            slot="shoulder_mount",
            part_name=f"Montura Hombro ({sm.actuator_type.upper()})",
            variant=slots.shoulder_mount_variant,
            mass_g=round(masses_g["shoulder_mount"], 1),
            volume_mm3=round(vols["shoulder_mount"], 1),
            safety_factor=sm_metrics.safety_factor,
            torque_nm=round(torques["tau_d1"], 2),
            status=_status(sm_metrics.safety_factor),
        ),
        PartPhysicsMetric(
            slot="shoulder_gear",
            part_name=f"Transmisión Hombro ({sg.teeth}T)",
            variant=slots.shoulder_gear_variant,
            mass_g=round(masses_g["shoulder_gear"], 1),
            volume_mm3=round(vols["shoulder_gear"], 1),
            safety_factor=sg_metrics.safety_factor,
            torque_nm=round(torques["tau_d1"], 2),
            status=_status(sg_metrics.safety_factor),
        ),
        PartPhysicsMetric(
            slot="upper_arm_link",
            part_name=f"Brazo Superior ({ua.length_mm:.0f} mm)",
            variant=slots.upper_arm_variant,
            mass_g=round(masses_g["upper_arm_link"], 1),
            volume_mm3=round(vols["upper_arm_link"], 1),
            safety_factor=ua_metrics.safety_factor,
            torque_nm=round(torques["tau_d1"], 2),
            status=_status(ua_metrics.safety_factor),
        ),
        PartPhysicsMetric(
            slot="elbow_gear",
            part_name=f"Transmisión Codo ({eg.teeth}T)",
            variant=slots.elbow_gear_variant,
            mass_g=round(masses_g["elbow_gear"], 1),
            volume_mm3=round(vols["elbow_gear"], 1),
            safety_factor=eg_metrics.safety_factor,
            torque_nm=round(torques["tau_d2"], 2),
            status=_status(eg_metrics.safety_factor),
        ),
        PartPhysicsMetric(
            slot="forearm_link",
            part_name=f"Antebrazo ({fa.length_mm:.0f} mm)",
            variant=slots.forearm_variant,
            mass_g=round(masses_g["forearm_link"], 1),
            volume_mm3=round(vols["forearm_link"], 1),
            safety_factor=fa_metrics.safety_factor,
            torque_nm=round(torques["tau_d2"], 2),
            status=_status(fa_metrics.safety_factor),
        ),
    ]

    # Forward Kinematics TCP position (mm)
    j1_deg, j2_deg, j3_deg = (
        float(req.joint_angles_deg[0]),
        float(req.joint_angles_deg[1]),
        float(req.joint_angles_deg[2]),
    )
    a1 = math.radians(j1_deg)
    a2 = math.radians(j1_deg + j2_deg)
    a3 = math.radians(j1_deg + j2_deg + j3_deg)
    l1, l2, l3 = float(ua.length_mm), float(fa.length_mm), float(req.wrist_mm)
    tcp_x = l1 * math.cos(a1) + l2 * math.cos(a2) + l3 * math.cos(a3)
    tcp_z = float(req.base_height_mm) + l1 * math.sin(a1) + l2 * math.sin(a2) + l3 * math.sin(a3)
    tcp_position_mm = [round(tcp_x, 1), 0.0, round(tcp_z, 1)]
    reach_mm = round(l1 + l2 + l3, 1)

    # Joint summaries (J1, J2, J3)
    gr1 = max(4.0, round(sg.teeth / 12.0 * 4.0, 1))
    gr2 = max(4.0, round(eg.teeth / 12.0 * 3.0, 1))
    gr3 = 6.0
    req_m1 = torques["tau_d1"] / (gr1 * GEARBOX_EFFICIENCY)
    req_m2 = torques["tau_d2"] / (gr2 * GEARBOX_EFFICIENCY)
    req_m3 = torques["tau_d3"] / (gr3 * GEARBOX_EFFICIENCY)
    act1, cost1, _ = _recommend_actuator(req_m1, torques["tau_d1"])
    act2, cost2, _ = _recommend_actuator(req_m2, torques["tau_d2"])
    act3, cost3, _ = _recommend_actuator(req_m3, torques["tau_d3"])

    joints = [
        JointTorqueSummary(
            joint_id="J1",
            joint_name="J1 Hombro (Modular)",
            segment_length_mm=round(l1, 1),
            segment_mass_g=round(masses_g["upper_arm_link"], 1),
            motor_mass_g=round(torques["m_mot_shoulder_g"], 1),
            static_torque_nm=round(torques["tau_s1"], 2),
            dynamic_torque_nm=round(torques["tau_d1"], 2),
            gear_ratio=gr1,
            required_motor_torque_nm=round(req_m1, 3),
            recommended_actuator=act1,
            safety_factor=min(sm_metrics.safety_factor, sg_metrics.safety_factor, ua_metrics.safety_factor),
        ),
        JointTorqueSummary(
            joint_id="J2",
            joint_name="J2 Codo (Modular)",
            segment_length_mm=round(l2, 1),
            segment_mass_g=round(masses_g["forearm_link"], 1),
            motor_mass_g=round(torques["m_mot_elbow_g"], 1),
            static_torque_nm=round(torques["tau_s2"], 2),
            dynamic_torque_nm=round(torques["tau_d2"], 2),
            gear_ratio=gr2,
            required_motor_torque_nm=round(req_m2, 3),
            recommended_actuator=act2,
            safety_factor=min(eg_metrics.safety_factor, fa_metrics.safety_factor),
        ),
        JointTorqueSummary(
            joint_id="J3",
            joint_name="J3 Muñeca (Modular)",
            segment_length_mm=round(l3, 1),
            segment_mass_g=round(masses_g["wrist_tube"], 1),
            motor_mass_g=round(torques["m_mot_wrist_g"], 1),
            static_torque_nm=round(torques["tau_s3"], 2),
            dynamic_torque_nm=round(torques["tau_d3"], 2),
            gear_ratio=gr3,
            required_motor_torque_nm=round(req_m3, 3),
            recommended_actuator=act3,
            safety_factor=fa_metrics.safety_factor,
        ),
    ]

    printed_total_g = sum(masses_g.values())
    total_mass_g = round(
        printed_total_g
        + torques["m_mot_shoulder_g"]
        + torques["m_mot_elbow_g"]
        + torques["m_mot_wrist_g"],
        1,
    )
    min_sf = min(m.safety_factor for m in per_part_metrics)
    compatible = not any(i.severity == "critical" for i in issues)

    printed_cost = round((printed_total_g / 1000.0) * float(material["cost_per_kg_usd"]) * 1.2 + 4.0, 2)
    act_spec = ACTUATOR_INTERFACE_SPECS.get(sm.actuator_type, ACTUATOR_INTERFACE_SPECS["nema17"])
    bom = [
        BOMItem(
            category="Impresión 3D",
            part_name="Piezas modulares impresas (Brazo + Montura + Engranajes)",
            spec=f"{material['name']} — {printed_total_g:.0f} g reales",
            qty=1,
            unit_cost_usd=printed_cost,
            total_cost_usd=printed_cost,
        ),
        BOMItem(
            category="Actuadores",
            part_name=f"Actuador Hombro ({act1})",
            spec=f"Brida {sm.actuator_type.upper()} — {req_m1:.2f} N·m",
            qty=2,
            unit_cost_usd=cost1,
            total_cost_usd=round(cost1 * 2, 2),
        ),
        BOMItem(
            category="Actuadores",
            part_name=f"Actuador Codo ({act2})",
            spec=f"{req_m2:.2f} N·m",
            qty=1,
            unit_cost_usd=cost2,
            total_cost_usd=round(cost2, 2),
        ),
        BOMItem(
            category="Actuadores",
            part_name=f"Actuador Muñeca ({act3})",
            spec=f"{req_m3:.2f} N·m",
            qty=max(1, req.dof - 3),
            unit_cost_usd=cost3,
            total_cost_usd=round(cost3 * max(1, req.dof - 3), 2),
        ),
        BOMItem(
            category="Rodamientos",
            part_name="Rodamientos de juntas (6805 / 608)",
            spec="Sellados 2RS",
            qty=6,
            unit_cost_usd=2.5,
            total_cost_usd=15.0,
        ),
        BOMItem(
            category="Tornillería",
            part_name="Kit Pernos + Insertos Térmicos",
            spec=f"{act_spec['bolt_size']} / M3 / M4 acero 12.9",
            qty=1,
            unit_cost_usd=14.0,
            total_cost_usd=14.0,
        ),
    ]

    return {
        "compatible": compatible,
        "compatibility_issues": issues,
        "per_part_metrics": per_part_metrics,
        "total_mass_g": total_mass_g,
        "printed_mass_g": round(printed_total_g, 1),
        "reach_mm": reach_mm,
        "shoulder_torque_nm": round(torques["tau_d1"], 2),
        "min_safety_factor": round(min_sf, 2),
        "tcp_position_mm": tcp_position_mm,
        "slots_used": slots,
        "joints": joints,
        "bom": bom,
    }
