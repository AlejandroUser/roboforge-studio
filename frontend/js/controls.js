/**
 * controls.js — Gestión del panel izquierdo de parámetros de RoboForge Studio
 */

const ARCH_META = {
  articulated_6dof: {
    label: "Articulado 6-DOF",
    subtitle: "Brazo industrial multi-eje",
    icon: `<svg viewBox="0 0 24 24" fill="none"><circle cx="6" cy="18" r="3" stroke="currentColor" stroke-width="1.8"/><circle cx="13" cy="10" r="2.5" stroke="currentColor" stroke-width="1.8"/><circle cx="19" cy="6" r="2" stroke="currentColor" stroke-width="1.8"/><path d="M8 15.8L11.3 12M15 8.6L17.2 7.2" stroke="currentColor" stroke-width="1.8" stroke-linecap="round"/></svg>`,
  },
  scara: {
    label: "SCARA",
    subtitle: "Alta velocidad en plano XY",
    icon: `<svg viewBox="0 0 24 24" fill="none"><circle cx="6" cy="12" r="3.2" stroke="currentColor" stroke-width="1.8"/><circle cx="18" cy="12" r="2.4" stroke="currentColor" stroke-width="1.8"/><path d="M9.2 10.5L15.8 10.8M9.2 13.5L15.8 13.2" stroke="currentColor" stroke-width="1.7" stroke-linecap="round"/></svg>`,
  },
  delta_rod: {
    label: "Biela Delta",
    subtitle: "Ultra-ligera, baja inercia",
    icon: `<svg viewBox="0 0 24 24" fill="none"><circle cx="5" cy="12" r="2.2" stroke="currentColor" stroke-width="1.8"/><circle cx="19" cy="12" r="2.2" stroke="currentColor" stroke-width="1.8"/><path d="M7.2 12H16.8" stroke="currentColor" stroke-width="2" stroke-linecap="round"/><path d="M10 9.5L14 14.5M14 9.5L10 14.5" stroke="currentColor" stroke-width="1.2" stroke-opacity="0.6"/></svg>`,
  },
  robot_leg: {
    label: "Pata Robótica",
    subtitle: "Cuadrúpedo / alto impacto",
    icon: `<svg viewBox="0 0 24 24" fill="none"><circle cx="7" cy="6" r="2.6" stroke="currentColor" stroke-width="1.8"/><circle cx="15" cy="13" r="2.2" stroke="currentColor" stroke-width="1.8"/><path d="M8.8 8L13.5 11.6M14.2 15.1L11.5 20.5" stroke="currentColor" stroke-width="1.9" stroke-linecap="round"/></svg>`,
  },
};

const MOUNT_META = {
  nema17: {
    label: "NEMA 17 + 2020",
    subtitle: "Motor paso a paso 42mm",
    icon: `<svg viewBox="0 0 24 24" fill="none"><rect x="4" y="4" width="16" height="16" rx="2.5" stroke="currentColor" stroke-width="1.7"/><circle cx="12" cy="12" r="3.5" stroke="currentColor" stroke-width="1.7"/><circle cx="7.5" cy="7.5" r="1" fill="currentColor"/><circle cx="16.5" cy="7.5" r="1" fill="currentColor"/><circle cx="7.5" cy="16.5" r="1" fill="currentColor"/><circle cx="16.5" cy="16.5" r="1" fill="currentColor"/></svg>`,
  },
  nema23: {
    label: "NEMA 23 Heavy",
    subtitle: "Alto torque 57mm + 2040",
    icon: `<svg viewBox="0 0 24 24" fill="none"><rect x="3" y="3" width="18" height="18" rx="3" stroke="currentColor" stroke-width="1.9"/><circle cx="12" cy="12" r="4.2" stroke="currentColor" stroke-width="1.9"/><circle cx="6.8" cy="6.8" r="1.2" fill="currentColor"/><circle cx="17.2" cy="6.8" r="1.2" fill="currentColor"/><circle cx="6.8" cy="17.2" r="1.2" fill="currentColor"/><circle cx="17.2" cy="17.2" r="1.2" fill="currentColor"/></svg>`,
  },
  servo_mg996r: {
    label: "Servo MG996R",
    subtitle: "Servo estándar 40×20mm",
    icon: `<svg viewBox="0 0 24 24" fill="none"><rect x="3" y="7" width="18" height="10" rx="2" stroke="currentColor" stroke-width="1.7"/><circle cx="16" cy="12" r="2.2" stroke="currentColor" stroke-width="1.6"/><path d="M6 12H10" stroke="currentColor" stroke-width="1.6" stroke-linecap="round"/></svg>`,
  },
  profile_2020_gusset: {
    label: "Escuadra 2020",
    subtitle: "Unión estructural 90°",
    icon: `<svg viewBox="0 0 24 24" fill="none"><path d="M5 4V19H20" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"/><path d="M5 10L14 19" stroke="currentColor" stroke-width="1.6" stroke-dasharray="2.5 2"/></svg>`,
  },
};

const FALLBACK_MOUNT_DEFAULTS = {
  nema17: {
    actuator_type: "nema17",
    base_type: "profile_2020",
    wall_thickness_mm: 5.0,
    base_depth_mm: 42.0,
    shaft_height_mm: 32.0,
    gusset_thickness_mm: 4.0,
    use_heat_inserts: false,
    ventilation_slots: true,
    radial_load_n: 35.0,
    motor_torque_nm: 0.6,
    target_safety_factor: 2.0,
    max_deflection_mm: 0.3,
    motor_temp_c: 58.0,
    material_id: "petg_cf",
    print_orientation: "xy_flat",
    nozzle_diameter_mm: 0.4,
    printer_clearance_mm: 0.15,
  },
  nema23: {
    actuator_type: "nema23",
    base_type: "profile_2040",
    wall_thickness_mm: 7.0,
    base_depth_mm: 58.0,
    shaft_height_mm: 42.0,
    gusset_thickness_mm: 6.0,
    use_heat_inserts: true,
    ventilation_slots: true,
    radial_load_n: 90.0,
    motor_torque_nm: 2.2,
    target_safety_factor: 2.2,
    max_deflection_mm: 0.25,
    motor_temp_c: 65.0,
    material_id: "pa12_cf",
    print_orientation: "xy_flat",
    nozzle_diameter_mm: 0.6,
    printer_clearance_mm: 0.15,
  },
  servo_mg996r: {
    actuator_type: "servo_mg996r",
    base_type: "flat_plate",
    wall_thickness_mm: 4.5,
    base_depth_mm: 36.0,
    shaft_height_mm: 28.0,
    gusset_thickness_mm: 3.5,
    use_heat_inserts: true,
    ventilation_slots: false,
    radial_load_n: 20.0,
    motor_torque_nm: 1.1,
    target_safety_factor: 2.0,
    max_deflection_mm: 0.35,
    motor_temp_c: 48.0,
    material_id: "petg_cf",
    print_orientation: "xy_flat",
    nozzle_diameter_mm: 0.4,
    printer_clearance_mm: 0.15,
  },
  profile_2020_gusset: {
    actuator_type: "profile_2020_gusset",
    base_type: "profile_2020",
    wall_thickness_mm: 5.0,
    base_depth_mm: 36.0,
    shaft_height_mm: 30.0,
    gusset_thickness_mm: 4.5,
    use_heat_inserts: false,
    ventilation_slots: false,
    radial_load_n: 60.0,
    motor_torque_nm: 0.0,
    target_safety_factor: 2.5,
    max_deflection_mm: 0.2,
    motor_temp_c: 35.0,
    material_id: "petg_cf",
    print_orientation: "xy_flat",
    nozzle_diameter_mm: 0.4,
    printer_clearance_mm: 0.15,
  },
};

const GEAR_META = {
  spur_nema17: {
    label: "Piñón NEMA 17",
    subtitle: "Involuta 24T · m=2.0 · Ø5 D-shaft",
    icon: `<svg viewBox="0 0 24 24" fill="none"><circle cx="12" cy="12" r="6.5" stroke="currentColor" stroke-width="1.7" stroke-dasharray="2.5 1.8"/><circle cx="12" cy="12" r="2.6" stroke="currentColor" stroke-width="1.7"/></svg>`,
  },
  spur_heavy: {
    label: "Engranaje Alto Torque",
    subtitle: "Involuta 36T · m=2.5 · Ø8 mm",
    icon: `<svg viewBox="0 0 24 24" fill="none"><circle cx="12" cy="12" r="8" stroke="currentColor" stroke-width="2" stroke-dasharray="3 2"/><circle cx="12" cy="12" r="4.2" stroke="currentColor" stroke-width="1.3"/><circle cx="12" cy="12" r="2" stroke="currentColor" stroke-width="1.6"/></svg>`,
  },
  gt2_pulley_20t: {
    label: "Polea GT2 20T",
    subtitle: "Correa 2mm · Baja inercia",
    icon: `<svg viewBox="0 0 24 24" fill="none"><circle cx="12" cy="12" r="7" stroke="currentColor" stroke-width="1.6"/><circle cx="12" cy="12" r="4.8" stroke="currentColor" stroke-width="1.2" stroke-dasharray="1.8 1.5"/><circle cx="12" cy="12" r="2.2" stroke="currentColor" stroke-width="1.6"/></svg>`,
  },
  gt2_pulley_40t: {
    label: "Polea GT2 40T",
    subtitle: "Reducción 2:1 · Radios ligeros",
    icon: `<svg viewBox="0 0 24 24" fill="none"><circle cx="12" cy="12" r="8.2" stroke="currentColor" stroke-width="1.7"/><circle cx="12" cy="12" r="2.4" stroke="currentColor" stroke-width="1.6"/><path d="M12 4V9.6M12 14.4V20M4 12H9.6M14.4 12H20" stroke="currentColor" stroke-width="1.3"/></svg>`,
  },
};

const FALLBACK_GEAR_DEFAULTS = {
  spur_nema17: {
    preset: "spur_nema17",
    gear_kind: "spur_involute",
    teeth: 24,
    module_mm: 2.0,
    pressure_angle_deg: 20.0,
    face_width_mm: 10.0,
    bore_diameter_mm: 5.0,
    d_shaft_flat_mm: 0.5,
    hub_diameter_mm: 16.0,
    hub_height_mm: 5.0,
    backlash_mm: 0.15,
    torque_nm: 1.2,
    rpm: 120.0,
    target_safety_factor: 2.0,
    operating_temp_c: 45.0,
    material_id: "petg_cf",
    print_orientation: "xy_flat",
    nozzle_diameter_mm: 0.4,
    printer_clearance_mm: 0.15,
  },
  spur_heavy: {
    preset: "spur_heavy",
    gear_kind: "spur_involute",
    teeth: 36,
    module_mm: 2.5,
    pressure_angle_deg: 20.0,
    face_width_mm: 16.0,
    bore_diameter_mm: 8.0,
    d_shaft_flat_mm: 0.8,
    hub_diameter_mm: 24.0,
    hub_height_mm: 8.0,
    backlash_mm: 0.18,
    torque_nm: 5.5,
    rpm: 90.0,
    target_safety_factor: 2.2,
    operating_temp_c: 50.0,
    material_id: "pa12_cf",
    print_orientation: "xy_flat",
    nozzle_diameter_mm: 0.4,
    printer_clearance_mm: 0.15,
  },
  gt2_pulley_20t: {
    preset: "gt2_pulley_20t",
    gear_kind: "gt2_pulley",
    teeth: 20,
    module_mm: 2.0,
    pressure_angle_deg: 20.0,
    face_width_mm: 7.5,
    bore_diameter_mm: 5.0,
    d_shaft_flat_mm: 0.5,
    hub_diameter_mm: 14.0,
    hub_height_mm: 5.0,
    backlash_mm: 0.12,
    torque_nm: 0.6,
    rpm: 300.0,
    target_safety_factor: 2.0,
    operating_temp_c: 45.0,
    material_id: "petg_cf",
    print_orientation: "xy_flat",
    nozzle_diameter_mm: 0.4,
    printer_clearance_mm: 0.15,
  },
  gt2_pulley_40t: {
    preset: "gt2_pulley_40t",
    gear_kind: "gt2_pulley",
    teeth: 40,
    module_mm: 2.0,
    pressure_angle_deg: 20.0,
    face_width_mm: 9.0,
    bore_diameter_mm: 8.0,
    d_shaft_flat_mm: 0.7,
    hub_diameter_mm: 20.0,
    hub_height_mm: 6.0,
    backlash_mm: 0.12,
    torque_nm: 1.8,
    rpm: 150.0,
    target_safety_factor: 2.0,
    operating_temp_c: 45.0,
    material_id: "pa12_cf",
    print_orientation: "xy_flat",
    nozzle_diameter_mm: 0.4,
    printer_clearance_mm: 0.15,
  },
};

const ARM_META = {
  parol6_desktop: {
    label: "PAROL6 6-DOF",
    subtitle: "Desktop industrial 6-DOF · 1 kg",
    icon: `<svg viewBox="0 0 24 24" fill="none"><path d="M5 20H15M8 20V16L13 10L18 6" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"/><circle cx="8" cy="16" r="2" stroke="currentColor" stroke-width="1.5"/><circle cx="13" cy="10" r="1.8" stroke="currentColor" stroke-width="1.5"/><circle cx="18" cy="6" r="1.5" fill="currentColor"/></svg>`,
  },
  ar4_industrial: {
    label: "AR4-MK5 Industrial",
    subtitle: "Alcance extendido · 2.2 kg",
    icon: `<svg viewBox="0 0 24 24" fill="none"><path d="M4 20H14M7 20V14L14 7L20 5" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/><circle cx="7" cy="14" r="2.2" stroke="currentColor" stroke-width="1.6"/><circle cx="14" cy="7" r="2" stroke="currentColor" stroke-width="1.6"/></svg>`,
  },
  so_arm_lerobot: {
    label: "SO-ARM LeRobot",
    subtitle: "Compacto servo bus · 0.5 kg",
    icon: `<svg viewBox="0 0 24 24" fill="none"><path d="M6 20H14M10 20V15L15 11L18 9" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"/><circle cx="10" cy="15" r="1.8" stroke="currentColor" stroke-width="1.5"/><circle cx="15" cy="11" r="1.6" stroke="currentColor" stroke-width="1.5"/></svg>`,
  },
  cobot_hollow_ur: {
    label: "Cobot Hueco UR",
    subtitle: "Cobot tubular QDD · 3.0 kg",
    icon: `<svg viewBox="0 0 24 24" fill="none"><path d="M4 20H15M8 20V15L12 8L19 7" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"/><circle cx="8" cy="15" r="2.3" stroke="currentColor" stroke-width="1.6"/><circle cx="12" cy="8" r="2.1" stroke="currentColor" stroke-width="1.6"/><circle cx="19" cy="7" r="1.6" stroke="currentColor" stroke-width="1.5"/></svg>`,
  },
};

const FALLBACK_ARM_DEFAULTS = {
  parol6_desktop: {
    preset: "parol6_desktop",
    dof: 6,
    payload_kg: 1.0,
    base_height_mm: 95.0,
    upper_arm_mm: 200.0,
    forearm_mm: 170.0,
    wrist_mm: 75.0,
    link_style: "hollow_cobot",
    shoulder_joint_deg: 35.0,
    elbow_joint_deg: -55.0,
    wrist_joint_deg: 20.0,
    gear_ratio_shoulder: 20.0,
    gear_ratio_elbow: 10.0,
    max_accel_rad_s2: 12.0,
    material_id: "petg_cf",
    print_orientation: "xy_flat",
    printer_id: "bambu_x1c",
  },
  ar4_industrial: {
    preset: "ar4_industrial",
    dof: 6,
    payload_kg: 2.2,
    base_height_mm: 120.0,
    upper_arm_mm: 280.0,
    forearm_mm: 240.0,
    wrist_mm: 90.0,
    link_style: "organic_ibeam",
    shoulder_joint_deg: 30.0,
    elbow_joint_deg: -60.0,
    wrist_joint_deg: 30.0,
    gear_ratio_shoulder: 40.0,
    gear_ratio_elbow: 20.0,
    max_accel_rad_s2: 10.0,
    material_id: "pa12_cf",
    print_orientation: "xy_flat",
    printer_id: "voron_24",
  },
  so_arm_lerobot: {
    preset: "so_arm_lerobot",
    dof: 5,
    payload_kg: 0.5,
    base_height_mm: 70.0,
    upper_arm_mm: 140.0,
    forearm_mm: 125.0,
    wrist_mm: 55.0,
    link_style: "rounded_truss",
    shoulder_joint_deg: 40.0,
    elbow_joint_deg: -65.0,
    wrist_joint_deg: 25.0,
    gear_ratio_shoulder: 8.0,
    gear_ratio_elbow: 6.0,
    max_accel_rad_s2: 18.0,
    material_id: "petg_cf",
    print_orientation: "xy_flat",
    printer_id: "bambu_x1c",
  },
  cobot_hollow_ur: {
    preset: "cobot_hollow_ur",
    dof: 6,
    payload_kg: 3.0,
    base_height_mm: 135.0,
    upper_arm_mm: 320.0,
    forearm_mm: 270.0,
    wrist_mm: 105.0,
    link_style: "hollow_cobot",
    shoulder_joint_deg: 25.0,
    elbow_joint_deg: -50.0,
    wrist_joint_deg: 25.0,
    gear_ratio_shoulder: 50.0,
    gear_ratio_elbow: 30.0,
    max_accel_rad_s2: 10.0,
    material_id: "pa12_cf",
    print_orientation: "xy_flat",
    printer_id: "voron_24",
  },
};

const FALLBACK_END_PRESETS = {
  nema17: {
    label: "NEMA 17",
    outer_diameter_mm: 46.0,
    bore_diameter_mm: 22.0,
    bolt_count: 4,
    bolt_pcd_mm: 31.0,
    bolt_size: "M3",
    use_heat_inserts: true,
  },
  nema23: {
    label: "NEMA 23",
    outer_diameter_mm: 62.0,
    bore_diameter_mm: 38.1,
    bolt_count: 4,
    bolt_pcd_mm: 47.14,
    bolt_size: "M5",
    use_heat_inserts: true,
  },
  bearing_608: {
    label: "Rod. 608",
    outer_diameter_mm: 32.0,
    bore_diameter_mm: 22.0,
    bolt_count: 0,
    bolt_pcd_mm: 26.0,
    bolt_size: "M3",
    use_heat_inserts: false,
  },
  bearing_6805: {
    label: "Rod. 6805",
    outer_diameter_mm: 48.0,
    bore_diameter_mm: 37.0,
    bolt_count: 0,
    bolt_pcd_mm: 42.0,
    bolt_size: "M3",
    use_heat_inserts: false,
  },
  custom: {
    label: "Custom",
  },
};

const FALLBACK_ARCH_DEFAULTS = {
  articulated_6dof: {
    architecture: "articulated_6dof",
    length_mm: 180,
    root_width_mm: 38,
    tip_width_mm: 24,
    thickness_mm: 20,
    organic_taper: 0.75,
    cable_channel: true,
    root_end: { ...FALLBACK_END_PRESETS.nema17, preset: "nema17" },
    tip_end: { ...FALLBACK_END_PRESETS.bearing_608, preset: "bearing_608" },
    tip_load_n: 25,
    angular_accel_rad_s2: 15,
    target_safety_factor: 2.0,
    max_deflection_mm: 0.5,
    operating_temp_c: 45,
    material_id: "petg_cf",
    print_orientation: "xy_flat",
    nozzle_diameter_mm: 0.4,
    printer_clearance_mm: 0.15,
  },
  scara: {
    architecture: "scara",
    length_mm: 220,
    root_width_mm: 52,
    tip_width_mm: 34,
    thickness_mm: 24,
    organic_taper: 0.65,
    cable_channel: true,
    root_end: { ...FALLBACK_END_PRESETS.nema23, preset: "nema23" },
    tip_end: { ...FALLBACK_END_PRESETS.bearing_6805, preset: "bearing_6805" },
    tip_load_n: 35,
    angular_accel_rad_s2: 30,
    target_safety_factor: 2.2,
    max_deflection_mm: 0.35,
    operating_temp_c: 50,
    material_id: "pa12_cf",
    print_orientation: "xy_flat",
    nozzle_diameter_mm: 0.4,
    printer_clearance_mm: 0.15,
  },
  delta_rod: {
    architecture: "delta_rod",
    length_mm: 240,
    root_width_mm: 22,
    tip_width_mm: 20,
    thickness_mm: 12,
    organic_taper: 0.85,
    cable_channel: false,
    root_end: { ...FALLBACK_END_PRESETS.bearing_608, preset: "bearing_608" },
    tip_end: { ...FALLBACK_END_PRESETS.bearing_608, preset: "bearing_608" },
    tip_load_n: 12,
    angular_accel_rad_s2: 60,
    target_safety_factor: 2.0,
    max_deflection_mm: 0.4,
    operating_temp_c: 35,
    material_id: "petg_cf",
    print_orientation: "xy_flat",
    nozzle_diameter_mm: 0.4,
    printer_clearance_mm: 0.15,
  },
  robot_leg: {
    architecture: "robot_leg",
    length_mm: 200,
    root_width_mm: 46,
    tip_width_mm: 26,
    thickness_mm: 22,
    organic_taper: 0.8,
    cable_channel: true,
    root_end: { ...FALLBACK_END_PRESETS.nema23, preset: "nema23" },
    tip_end: { ...FALLBACK_END_PRESETS.bearing_608, preset: "bearing_608" },
    tip_load_n: 80,
    angular_accel_rad_s2: 45,
    target_safety_factor: 2.5,
    max_deflection_mm: 0.8,
    operating_temp_c: 55,
    material_id: "pa12_cf",
    print_orientation: "xy_flat",
    nozzle_diameter_mm: 0.6,
    printer_clearance_mm: 0.15,
  },
};

function clamp(val, min, max, fallback) {
  const n = Number(val);
  if (!Number.isFinite(n)) return fallback;
  return Math.min(Math.max(n, min), max);
}

export class ControlsController {
  constructor({ onArchitectureChange, onStepFileSelected, onClearStepReference } = {}) {
    this.onArchitectureChange = onArchitectureChange || (() => {});
    this.onStepFileSelected = onStepFileSelected || (() => {});
    this.onClearStepReference = onClearStepReference || (() => {});
    this.materialsMap = {};
    this.printersMap = {};
    this.archPresets = structuredClone(FALLBACK_ARCH_DEFAULTS);
    this.endPresets = structuredClone(FALLBACK_END_PRESETS);
    this.mountPresets = structuredClone(FALLBACK_MOUNT_DEFAULTS);
    this.gearPresets = structuredClone(FALLBACK_GEAR_DEFAULTS);
    this.armPresets = structuredClone(FALLBACK_ARM_DEFAULTS);
    this.activeFamily = "links";
    this.currentArchitecture = "articulated_6dof";
    this.currentMountActuator = "nema17";
    this.currentGearPreset = "spur_nema17";
    this.currentArmPreset = "parol6_desktop";
    this.currentOrientation = "xy_flat";
    this.referenceVolumeCm3 = null;

    this._bindStepUpload();
    this._bindSliders();
    this._bindGearLivePreview();
    this._bindEndTabs();
    this._bindEndControls("root");
    this._bindEndControls("tip");
    this._bindOrientationPills();
    this._bindMaterialSelect();
    this._bindPrinterSelect();
    this._renderArchitectureGrid();
    this._renderMountPresetGrid();
    this._renderGearPresetGrid();
    this._renderArmPresetGrid();
  }

  setFamily(family) {
    this.activeFamily = ["links", "mounts", "gears", "arm"].includes(family) ? family : "links";
    document.getElementById("family-controls-links")?.classList.toggle("hidden", this.activeFamily !== "links");
    document.getElementById("family-controls-mounts")?.classList.toggle("hidden", this.activeFamily !== "mounts");
    document.getElementById("family-controls-gears")?.classList.toggle("hidden", this.activeFamily !== "gears");
    document.getElementById("family-controls-arm")?.classList.toggle("hidden", this.activeFamily !== "arm");

    const headerLabel = document.getElementById("header-arch-label");
    if (headerLabel) {
      if (this.activeFamily === "mounts") {
        headerLabel.textContent = MOUNT_META[this.currentMountActuator]?.label || this.currentMountActuator;
      } else if (this.activeFamily === "gears") {
        headerLabel.textContent = GEAR_META[this.currentGearPreset]?.label || this.currentGearPreset;
      } else if (this.activeFamily === "arm") {
        headerLabel.textContent = ARM_META[this.currentArmPreset]?.label || this.currentArmPreset;
      } else {
        headerLabel.textContent = ARCH_META[this.currentArchitecture]?.label || this.currentArchitecture;
      }
    }
  }

  _bindStepUpload() {
    const dropzone = document.getElementById("step-dropzone");
    const fileInput = document.getElementById("step-file-input");
    const clearBtn = document.getElementById("btn-clear-step");

    const triggerFile = (file) => {
      if (!file) return;
      const lower = file.name.toLowerCase();
      if (!lower.endsWith(".step") && !lower.endsWith(".stp")) {
        this.onStepFileSelected(null, "Solo se admiten archivos .STEP o .STP.");
        return;
      }
      this.onStepFileSelected(file, null);
    };

    dropzone?.addEventListener("click", () => fileInput?.click());
    dropzone?.addEventListener("keydown", (e) => {
      if (e.key === "Enter" || e.key === " ") {
        e.preventDefault();
        fileInput?.click();
      }
    });

    fileInput?.addEventListener("change", () => {
      const file = fileInput.files?.[0];
      if (file) triggerFile(file);
      fileInput.value = "";
    });

    dropzone?.addEventListener("dragover", (e) => {
      e.preventDefault();
      dropzone.classList.add("dragover");
    });
    dropzone?.addEventListener("dragleave", () => {
      dropzone.classList.remove("dragover");
    });
    dropzone?.addEventListener("drop", (e) => {
      e.preventDefault();
      dropzone.classList.remove("dragover");
      const file = e.dataTransfer?.files?.[0];
      if (file) triggerFile(file);
    });

    clearBtn?.addEventListener("click", () => {
      this.clearStepReference();
      this.onClearStepReference();
    });
  }

  setStepUploadBusy(busy) {
    const dropzone = document.getElementById("step-dropzone");
    dropzone?.classList.toggle("busy", Boolean(busy));
  }

  setStepReference(analysis) {
    const card = document.getElementById("step-summary-card");
    const nameEl = document.getElementById("step-filename");
    const bboxEl = document.getElementById("step-bbox");
    const volEl = document.getElementById("step-volume");
    const holesEl = document.getElementById("step-holes");
    const notesEl = document.getElementById("step-notes");

    if (!analysis) {
      this.clearStepReference();
      return;
    }

    const refVol = analysis.suggested_request?.reference_volume_cm3 ?? analysis.volume_cm3;
    this.referenceVolumeCm3 = Number.isFinite(Number(refVol)) && Number(refVol) > 0 ? Number(refVol) : null;

    if (nameEl) nameEl.textContent = analysis.filename || "referencia.step";
    if (bboxEl) {
      bboxEl.textContent = `${Math.round(analysis.bbox_length_mm)}×${Math.round(
        analysis.bbox_width_mm
      )}×${Math.round(analysis.bbox_thickness_mm)} mm`;
    }
    if (volEl) volEl.textContent = `${Number(analysis.volume_cm3 || 0).toFixed(1)} cm³`;
    if (holesEl) {
      const holes = analysis.detected_holes || [];
      if (holes.length) {
        const diams = holes
          .slice(0, 2)
          .map((h) => `Ø${Number(h.diameter_mm).toFixed(1)}`)
          .join(", ");
        holesEl.textContent = `${holes.length} (${diams})`;
      } else {
        holesEl.textContent = "0";
      }
    }
    if (notesEl) {
      notesEl.innerHTML = "";
      for (const note of analysis.notes || []) {
        const li = document.createElement("li");
        li.textContent = note;
        notesEl.appendChild(li);
      }
    }

    card?.classList.remove("hidden");

    if (analysis.suggested_request) {
      this.applyDesignParams(analysis.suggested_request);
    }
  }

  clearStepReference() {
    this.referenceVolumeCm3 = null;
    const card = document.getElementById("step-summary-card");
    card?.classList.add("hidden");
  }

  _bindSliders() {
    const sliderConfigs = [
      { id: "length_mm", suffix: " mm", decimals: 0 },
      { id: "root_width_mm", suffix: " mm", decimals: 0 },
      { id: "tip_width_mm", suffix: " mm", decimals: 0 },
      { id: "thickness_mm", suffix: " mm", decimals: 0 },
      { id: "organic_taper", suffix: "%", decimals: 0 },
      { id: "topopt_volume_fraction", suffix: "%", decimals: 0 },
      { id: "printer_clearance_mm", suffix: " mm", decimals: 2 },
      // Monturas Mecatrónicas
      { id: "mount-wall_thickness_mm", suffix: " mm", decimals: 1 },
      { id: "mount-base_depth_mm", suffix: " mm", decimals: 0 },
      { id: "mount-shaft_height_mm", suffix: " mm", decimals: 0 },
      { id: "mount-gusset_thickness_mm", suffix: " mm", decimals: 1 },
      // Transmisión (Engranajes / Poleas GT2)
      { id: "gear-teeth", suffix: "", decimals: 0 },
      { id: "gear-module_mm", suffix: " mm", decimals: 1 },
      { id: "gear-face_width_mm", suffix: " mm", decimals: 1 },
      { id: "gear-backlash_mm", suffix: " mm", decimals: 2 },
      // Brazo Completo (Ensamblaje 4-6 DOF)
      { id: "arm-base_height_mm", suffix: " mm", decimals: 0 },
      { id: "arm-upper_arm_mm", suffix: " mm", decimals: 0 },
      { id: "arm-forearm_mm", suffix: " mm", decimals: 0 },
      { id: "arm-wrist_mm", suffix: " mm", decimals: 0 },
      { id: "arm-shoulder_joint_deg", suffix: "°", decimals: 0 },
      { id: "arm-elbow_joint_deg", suffix: "°", decimals: 0 },
      { id: "arm-wrist_joint_deg", suffix: "°", decimals: 0 },
    ];

    for (const cfg of sliderConfigs) {
      const input = document.getElementById(cfg.id);
      const readout = document.getElementById(`val-${cfg.id}`);
      if (!input || !readout) continue;
      const update = () => {
        const num = Number(input.value);
        readout.textContent = `${num.toFixed(cfg.decimals)}${cfg.suffix}`;
      };
      input.addEventListener("input", update);
      update();
    }
  }

  _bindEndTabs() {
    const tabBtns = document.querySelectorAll("[data-end-tab]");
    const panels = document.querySelectorAll("[data-end-panel]");
    tabBtns.forEach((btn) => {
      btn.addEventListener("click", () => {
        const target = btn.dataset.endTab;
        tabBtns.forEach((b) => {
          const active = b.dataset.endTab === target;
          b.classList.toggle("active", active);
          b.setAttribute("aria-selected", String(active));
        });
        panels.forEach((p) => {
          p.classList.toggle("active", p.dataset.endPanel === target);
        });
      });
    });
  }

  _bindEndControls(side) {
    const presetSel = document.getElementById(`${side}-preset`);
    const manualFields = [
      "outer_diameter_mm",
      "bore_diameter_mm",
      "bolt_count",
      "bolt_pcd_mm",
      "bolt_size",
      "use_heat_inserts",
    ];

    presetSel?.addEventListener("change", () => {
      const presetKey = presetSel.value;
      const presetData = this.endPresets[presetKey];
      if (presetKey !== "custom" && presetData) {
        this._setEndFormValues(side, { preset: presetKey, ...presetData });
      }
      this._updateEndTabSummary(side);
    });

    for (const field of manualFields) {
      const el = document.getElementById(`${side}-${field}`);
      if (!el) continue;
      el.addEventListener("input", () => {
        if (presetSel && presetSel.value !== "custom") {
          presetSel.value = "custom";
        }
        this._updateEndTabSummary(side);
      });
      el.addEventListener("change", () => {
        this._updateEndTabSummary(side);
      });
    }
    this._updateEndTabSummary(side);
  }

  _bindOrientationPills() {
    const pills = document.querySelectorAll("[data-orientation]");
    pills.forEach((pill) => {
      pill.addEventListener("click", () => {
        this.currentOrientation = pill.dataset.orientation;
        pills.forEach((p) => p.classList.toggle("active", p === pill));
      });
    });
  }

  _bindMaterialSelect() {
    const sel = document.getElementById("material_id");
    sel?.addEventListener("change", () => {
      this._renderMaterialBadge(sel.value);
    });
  }

  _bindPrinterSelect() {
    const sel = document.getElementById("printer_id");
    sel?.addEventListener("change", () => {
      this._applyPrinterDefaults(sel.value, true);
    });
  }

  setPrinters({ map, list }, selectId = null) {
    this.printersMap = map || {};
    const sel = document.getElementById("printer_id");
    if (!sel || !list?.length) return;

    const currentVal = selectId || sel.value || "bambu_x1c";
    sel.innerHTML = "";
    for (const p of list) {
      const opt = document.createElement("option");
      opt.value = p.id;
      const bedTag =
        p.bed_x_mm === p.bed_y_mm && p.bed_y_mm === p.bed_z_mm
          ? `${Math.round(p.bed_x_mm)}³`
          : `${Math.round(p.bed_x_mm)}×${Math.round(p.bed_y_mm)}×${Math.round(p.bed_z_mm)}`;
      opt.textContent = `${p.name || p.id} (${bedTag} mm)${p.is_custom ? " ★" : ""}`;
      sel.appendChild(opt);
    }

    if (this.printersMap[currentVal]) {
      sel.value = currentVal;
    } else if (this.printersMap.bambu_x1c) {
      sel.value = "bambu_x1c";
    }
    this._applyPrinterDefaults(sel.value, Boolean(selectId));
  }

  _applyPrinterDefaults(printerId, updateHardware = false) {
    const p = this.printersMap[printerId];
    if (!p) return;

    const bedEl = document.getElementById("printer-bed-chip");
    const encEl = document.getElementById("printer-enclosure-chip");
    const delBtn = document.getElementById("btn-del-printer");

    if (bedEl) {
      bedEl.textContent = `Cama ${Math.round(p.bed_x_mm)}×${Math.round(p.bed_y_mm)}×${Math.round(p.bed_z_mm)} mm`;
    }
    if (encEl) {
      encEl.textContent = p.enclosed ? "Cerrada" : "Abierta";
      encEl.classList.toggle("enclosed", Boolean(p.enclosed));
      encEl.classList.toggle("open", !p.enclosed);
    }
    if (delBtn) {
      delBtn.classList.toggle("hidden", !p.is_custom);
    }

    if (updateHardware) {
      if (p.default_nozzle_mm !== undefined) {
        const nozzleSel = document.getElementById("nozzle_diameter_mm");
        if (nozzleSel) {
          const valStr = String(Number(p.default_nozzle_mm));
          const exists = Array.from(nozzleSel.options).some((o) => o.value === valStr);
          if (!exists) {
            const opt = document.createElement("option");
            opt.value = valStr;
            opt.textContent = `${valStr} mm (Perfil)`;
            nozzleSel.appendChild(opt);
          }
          nozzleSel.value = valStr;
        }
      }
      if (p.clearance_mm !== undefined) {
        this._setInputValue("printer_clearance_mm", p.clearance_mm);
      }
    }
  }

  _renderArchitectureGrid() {
    const grid = document.getElementById("arch-preset-grid");
    if (!grid) return;
    grid.innerHTML = "";

    for (const [archId, meta] of Object.entries(ARCH_META)) {
      const backendPreset = this.archPresets[archId];
      const title = backendPreset?.name || meta.label;
      const subtitle = backendPreset?.description || meta.subtitle;

      const btn = document.createElement("button");
      btn.type = "button";
      btn.className = `arch-card${archId === this.currentArchitecture ? " active" : ""}`;
      btn.dataset.archId = archId;
      btn.innerHTML = `
        <div class="arch-card-head">
          ${meta.icon}
        </div>
        <span class="arch-card-title">${title}</span>
        <span class="arch-card-sub">${subtitle}</span>
      `;
      btn.addEventListener("click", () => {
        this.applyArchitecturePreset(archId);
      });
      grid.appendChild(btn);
    }
  }

  /**
   * Carga materiales desde GET /api/materials y actualiza el selector + badge.
   */
  setMaterials({ map, list }, selectId = null) {
    this.materialsMap = map || {};
    const sel = document.getElementById("material_id");
    if (!sel || !list?.length) return;

    const currentVal = selectId || sel.value || "petg_cf";
    sel.innerHTML = "";
    for (const mat of list) {
      const opt = document.createElement("option");
      opt.value = mat.id;
      const cat = (mat.category || "fdm").toUpperCase();
      opt.textContent = `${mat.name || mat.id} [${cat}]${mat.is_custom ? " ★" : ""}`;
      sel.appendChild(opt);
    }

    if (this.materialsMap[currentVal]) {
      sel.value = currentVal;
    } else if (this.materialsMap.petg_cf) {
      sel.value = "petg_cf";
    }
    this._renderMaterialBadge(sel.value);
  }

  _renderMaterialBadge(materialId) {
    const mat = this.materialsMap[materialId];
    if (!mat) return;

    const swatch = document.getElementById("mat-swatch");
    const cat = document.getElementById("mat-category");
    const cost = document.getElementById("mat-cost");
    const density = document.getElementById("mat-density");
    const youngs = document.getElementById("mat-youngs");
    const yieldEl = document.getElementById("mat-yield");
    const temp = document.getElementById("mat-temp");
    const notes = document.getElementById("mat-notes");

    if (swatch) swatch.style.background = mat.color_hex || "#38bdf8";
    if (cat) cat.textContent = (mat.category || "FDM").toUpperCase();
    if (cost) cost.textContent = `$${Number(mat.cost_per_kg_usd ?? 0).toFixed(0)}/kg`;
    if (density) density.textContent = `${Number(mat.density_g_cm3 ?? 0).toFixed(2)} g/cm³`;
    if (youngs) {
      youngs.textContent = `${Math.round(mat.youngs_xy_mpa ?? 0)} / ${Math.round(mat.youngs_z_mpa ?? 0)} MPa`;
    }
    if (yieldEl) {
      yieldEl.textContent = `${Math.round(mat.yield_xy_mpa ?? 0)} / ${Math.round(mat.yield_z_mpa ?? 0)} MPa`;
    }
    if (temp) temp.textContent = `${Math.round(mat.max_temp_c ?? 0)} °C`;
    if (notes) notes.textContent = mat.notes || "";
    const delMatBtn = document.getElementById("btn-del-material");
    if (delMatBtn) delMatBtn.classList.toggle("hidden", !mat.is_custom);
  }

  /**
   * Fusiona presets cargados desde GET /api/presets con los valores de respaldo.
   */
  setPresets({ architectures, endInterfaces }) {
    if (endInterfaces && typeof endInterfaces === "object") {
      for (const [key, val] of Object.entries(endInterfaces)) {
        if (val && typeof val === "object") {
          const flatEnd = val.params || val.defaults || val;
          this.endPresets[key] = {
            ...(this.endPresets[key] || {}),
            ...flatEnd,
          };
        }
      }
    }

    if (architectures && typeof architectures === "object") {
      const entries = Array.isArray(architectures)
        ? architectures.map((a) => [a.id || a.architecture, a])
        : Object.entries(architectures);

      for (const [key, val] of entries) {
        if (!key || !val || typeof val !== "object") continue;
        const flatParams = val.defaults || val.default_request || val.params || val;
        this.archPresets[key] = {
          ...(this.archPresets[key] || {}),
          ...flatParams,
          // Conservamos etiquetas en español para la interfaz
          name: ARCH_META[key]?.label || val.name,
          description: ARCH_META[key]?.subtitle || val.description || val.tagline,
        };
      }
    }

    this._renderArchitectureGrid();
  }

  applyArchitecturePreset(archId) {
    const preset = this.archPresets[archId] || FALLBACK_ARCH_DEFAULTS[archId];
    if (!preset) return;
    this.applyDesignParams({ ...preset, architecture: archId });
    this.onArchitectureChange(archId);
  }

  applyDesignParams(params) {
    if (!params) return;

    const archId = params.architecture || this.currentArchitecture;
    this.currentArchitecture = archId;

    // Actualizar cards activas
    document.querySelectorAll(".arch-card").forEach((card) => {
      card.classList.toggle("active", card.dataset.archId === archId);
    });

    // Actualizar header pill
    const headerLabel = document.getElementById("header-arch-label");
    if (headerLabel) {
      headerLabel.textContent = ARCH_META[archId]?.label || params.name || archId;
    }

    // Geometría
    this._setInputValue("length_mm", params.length_mm);
    this._setInputValue("root_width_mm", params.root_width_mm);
    this._setInputValue("tip_width_mm", params.tip_width_mm);
    this._setInputValue("thickness_mm", params.thickness_mm);
    if (params.organic_taper !== undefined) {
      const pct = params.organic_taper <= 1 ? Math.round(params.organic_taper * 100) : Math.round(params.organic_taper);
      this._setInputValue("organic_taper", pct);
    }
    if (params.topopt_volume_fraction !== undefined) {
      const vfPct =
        params.topopt_volume_fraction <= 1
          ? Math.round(params.topopt_volume_fraction * 100)
          : Math.round(params.topopt_volume_fraction);
      this._setInputValue("topopt_volume_fraction", vfPct);
    }
    this._setCheckbox("cable_channel", params.cable_channel);

    // Extremos
    if (params.root_end) this._setEndFormValues("root", params.root_end);
    if (params.tip_end) this._setEndFormValues("tip", params.tip_end);

    // Cargas
    this._setInputValue("tip_load_n", params.tip_load_n);
    this._setInputValue("angular_accel_rad_s2", params.angular_accel_rad_s2);
    this._setInputValue("max_deflection_mm", params.max_deflection_mm);
    this._setInputValue("target_safety_factor", params.target_safety_factor);
    this._setInputValue("operating_temp_c", params.operating_temp_c);

    // Material y manufactura
    if (params.material_id && this.materialsMap[params.material_id]) {
      this._setInputValue("material_id", params.material_id);
      this._renderMaterialBadge(params.material_id);
    }
    if (params.print_orientation) {
      this.currentOrientation = params.print_orientation;
      document.querySelectorAll("[data-orientation]").forEach((p) => {
        p.classList.toggle("active", p.dataset.orientation === this.currentOrientation);
      });
    }
    if (params.nozzle_diameter_mm !== undefined) {
      this._setInputValue("nozzle_diameter_mm", String(params.nozzle_diameter_mm));
    }
    if (params.printer_clearance_mm !== undefined) {
      this._setInputValue("printer_clearance_mm", params.printer_clearance_mm);
    }
  }

  _setInputValue(id, value) {
    if (value === undefined || value === null) return;
    const el = document.getElementById(id);
    if (!el) return;
    el.value = String(value);
    el.dispatchEvent(new Event("input", { bubbles: true }));
  }

  _setCheckbox(id, value) {
    if (value === undefined || value === null) return;
    const el = document.getElementById(id);
    if (!el) return;
    el.checked = Boolean(value);
  }

  _setEndFormValues(side, data) {
    if (!data) return;
    const presetEl = document.getElementById(`${side}-preset`);
    if (presetEl && data.preset) presetEl.value = data.preset;

    const mapFields = [
      "outer_diameter_mm",
      "bore_diameter_mm",
      "bolt_count",
      "bolt_pcd_mm",
      "bolt_size",
    ];
    for (const f of mapFields) {
      if (data[f] !== undefined && data[f] !== null) {
        const el = document.getElementById(`${side}-${f}`);
        if (el) el.value = String(data[f]);
      }
    }
    if (data.use_heat_inserts !== undefined) {
      this._setCheckbox(`${side}-use_heat_inserts`, data.use_heat_inserts);
    }
    this._updateEndTabSummary(side);
  }

  _updateEndTabSummary(side) {
    const summaryEl = document.getElementById(`tab-summary-${side}`);
    const presetVal = document.getElementById(`${side}-preset`)?.value || "custom";
    const od = Number(document.getElementById(`${side}-outer_diameter_mm`)?.value || 0);
    if (!summaryEl) return;
    const shortLabel = FALLBACK_END_PRESETS[presetVal]?.label || presetVal;
    summaryEl.textContent = `${shortLabel} · Ø${Math.round(od)}`;
  }

  _readEndInterface(side) {
    const preset = document.getElementById(`${side}-preset`)?.value || "custom";
    const outer = clamp(document.getElementById(`${side}-outer_diameter_mm`)?.value, 14, 120, 36);
    const bore = clamp(document.getElementById(`${side}-bore_diameter_mm`)?.value, 3, Math.min(90, outer - 2), 22);
    const boltCount = Math.round(clamp(document.getElementById(`${side}-bolt_count`)?.value, 0, 8, 4));
    const boltPcd = clamp(document.getElementById(`${side}-bolt_pcd_mm`)?.value, 10, 105, 29);
    const boltSize = document.getElementById(`${side}-bolt_size`)?.value || "M3";
    const useHeatInserts = Boolean(document.getElementById(`${side}-use_heat_inserts`)?.checked);

    return {
      preset,
      outer_diameter_mm: outer,
      bore_diameter_mm: bore,
      bolt_count: boltCount,
      bolt_pcd_mm: boltPcd,
      bolt_size: boltSize,
      use_heat_inserts: useHeatInserts,
    };
  }

  /**
   * Construye el payload JSON exacto que espera LinkDesignRequest en backend/app/schemas.py.
   */
  getPayload() {
    return {
      architecture: this.currentArchitecture,
      length_mm: clamp(document.getElementById("length_mm")?.value, 60, 450, 180),
      root_width_mm: clamp(document.getElementById("root_width_mm")?.value, 16, 110, 38),
      tip_width_mm: clamp(document.getElementById("tip_width_mm")?.value, 12, 90, 24),
      thickness_mm: clamp(document.getElementById("thickness_mm")?.value, 8, 60, 20),
      organic_taper: clamp(Number(document.getElementById("organic_taper")?.value || 75) / 100, 0, 1, 0.75),
      cable_channel: Boolean(document.getElementById("cable_channel")?.checked),

      root_end: this._readEndInterface("root"),
      tip_end: this._readEndInterface("tip"),

      tip_load_n: clamp(document.getElementById("tip_load_n")?.value, 1, 500, 25),
      angular_accel_rad_s2: clamp(document.getElementById("angular_accel_rad_s2")?.value, 0, 200, 15),
      target_safety_factor: clamp(document.getElementById("target_safety_factor")?.value, 1.1, 10, 2.0),
      max_deflection_mm: clamp(document.getElementById("max_deflection_mm")?.value, 0.05, 10, 0.5),
      operating_temp_c: clamp(document.getElementById("operating_temp_c")?.value, -10, 150, 45),

      material_id: document.getElementById("material_id")?.value || "petg_cf",
      printer_id: document.getElementById("printer_id")?.value || "bambu_x1c",
      print_orientation: this.currentOrientation,
      nozzle_diameter_mm: clamp(document.getElementById("nozzle_diameter_mm")?.value, 0.2, 1.0, 0.4),
      printer_clearance_mm: clamp(document.getElementById("printer_clearance_mm")?.value, 0, 0.5, 0.15),

      reference_volume_cm3: this.referenceVolumeCm3,
      topopt_volume_fraction: clamp(
        Number(document.getElementById("topopt_volume_fraction")?.value || 52) / 100,
        0.3,
        0.75,
        0.52
      ),
    };
  }

  _renderMountPresetGrid() {
    const grid = document.getElementById("mount-preset-grid");
    if (!grid) return;
    grid.innerHTML = "";

    for (const [actId, meta] of Object.entries(MOUNT_META)) {
      const preset = this.mountPresets[actId];
      const title = meta.label || preset?.name;
      const subtitle = meta.subtitle || preset?.subtitle;

      const btn = document.createElement("button");
      btn.type = "button";
      btn.className = `arch-card${actId === this.currentMountActuator ? " active" : ""}`;
      btn.dataset.mountActuator = actId;
      btn.innerHTML = `
        <div class="arch-card-head">
          ${meta.icon}
        </div>
        <span class="arch-card-title">${title}</span>
        <span class="arch-card-sub">${subtitle}</span>
      `;
      btn.addEventListener("click", () => {
        this.applyMountPreset(actId);
      });
      grid.appendChild(btn);
    }
  }

  setMountPresets(rawPresets) {
    if (!rawPresets || typeof rawPresets !== "object") return;
    const entries = Array.isArray(rawPresets)
      ? rawPresets.map((p) => [p.id || p.actuator_type, p])
      : Object.entries(rawPresets);

    for (const [key, val] of entries) {
      if (!key || !val || typeof val !== "object") continue;
      const flat = val.defaults || val.params || val;
      this.mountPresets[key] = {
        ...(this.mountPresets[key] || {}),
        ...flat,
        name: MOUNT_META[key]?.label || val.name,
        subtitle: MOUNT_META[key]?.subtitle || val.subtitle || val.description,
      };
    }
    this._renderMountPresetGrid();
  }

  applyMountPreset(actuatorId) {
    const preset = this.mountPresets[actuatorId] || FALLBACK_MOUNT_DEFAULTS[actuatorId];
    if (!preset) return;

    this.currentMountActuator = actuatorId;

    document.querySelectorAll("[data-mount-actuator]").forEach((card) => {
      card.classList.toggle("active", card.dataset.mountActuator === actuatorId);
    });

    if (this.activeFamily === "mounts") {
      const headerLabel = document.getElementById("header-arch-label");
      if (headerLabel) {
        headerLabel.textContent = MOUNT_META[actuatorId]?.label || preset.name || actuatorId;
      }
    }

    this._setInputValue("mount-base_type", preset.base_type);
    this._setInputValue("mount-wall_thickness_mm", preset.wall_thickness_mm);
    this._setInputValue("mount-base_depth_mm", preset.base_depth_mm);
    this._setInputValue("mount-shaft_height_mm", preset.shaft_height_mm);
    this._setInputValue("mount-gusset_thickness_mm", preset.gusset_thickness_mm);
    this._setCheckbox("mount-use_heat_inserts", preset.use_heat_inserts);
    this._setCheckbox("mount-ventilation_slots", preset.ventilation_slots);

    this._setInputValue("mount-radial_load_n", preset.radial_load_n);
    this._setInputValue("mount-motor_torque_nm", preset.motor_torque_nm);
    this._setInputValue("mount-motor_temp_c", preset.motor_temp_c);
    this._setInputValue("mount-max_deflection_mm", preset.max_deflection_mm);
    this._setInputValue("mount-target_safety_factor", preset.target_safety_factor);

    // Compartidos de Material y Manufactura 3D
    if (preset.material_id && this.materialsMap[preset.material_id]) {
      this._setInputValue("material_id", preset.material_id);
      this._renderMaterialBadge(preset.material_id);
    }
    if (preset.print_orientation) {
      this.currentOrientation = preset.print_orientation;
      document.querySelectorAll("[data-orientation]").forEach((p) => {
        p.classList.toggle("active", p.dataset.orientation === this.currentOrientation);
      });
    }
    if (preset.nozzle_diameter_mm !== undefined) {
      this._setInputValue("nozzle_diameter_mm", String(preset.nozzle_diameter_mm));
    }
    if (preset.printer_clearance_mm !== undefined) {
      this._setInputValue("printer_clearance_mm", preset.printer_clearance_mm);
    }
  }

  /**
   * Construye el payload JSON exacto que espera MountDesignRequest en backend/app/schemas.py.
   */
  getMountPayload() {
    return {
      actuator_type: this.currentMountActuator,
      base_type: document.getElementById("mount-base_type")?.value || "profile_2020",
      wall_thickness_mm: clamp(document.getElementById("mount-wall_thickness_mm")?.value, 2.5, 16, 5.0),
      base_depth_mm: clamp(document.getElementById("mount-base_depth_mm")?.value, 24, 100, 42.0),
      shaft_height_mm: clamp(document.getElementById("mount-shaft_height_mm")?.value, 20, 90, 32.0),
      gusset_thickness_mm: clamp(document.getElementById("mount-gusset_thickness_mm")?.value, 2.0, 14, 4.0),
      use_heat_inserts: Boolean(document.getElementById("mount-use_heat_inserts")?.checked),
      ventilation_slots: Boolean(document.getElementById("mount-ventilation_slots")?.checked),

      radial_load_n: clamp(document.getElementById("mount-radial_load_n")?.value, 1, 500, 35.0),
      motor_torque_nm: clamp(document.getElementById("mount-motor_torque_nm")?.value, 0, 20, 0.6),
      target_safety_factor: clamp(document.getElementById("mount-target_safety_factor")?.value, 1.1, 10, 2.0),
      max_deflection_mm: clamp(document.getElementById("mount-max_deflection_mm")?.value, 0.05, 5, 0.3),
      motor_temp_c: clamp(document.getElementById("mount-motor_temp_c")?.value, 15, 130, 58.0),

      material_id: document.getElementById("material_id")?.value || "petg_cf",
      printer_id: document.getElementById("printer_id")?.value || "bambu_x1c",
      print_orientation: this.currentOrientation,
      nozzle_diameter_mm: clamp(document.getElementById("nozzle_diameter_mm")?.value, 0.2, 1.0, 0.4),
      printer_clearance_mm: clamp(document.getElementById("printer_clearance_mm")?.value, 0, 0.5, 0.15),
    };
  }

  _bindGearLivePreview() {
    const kindEl = document.getElementById("gear-gear_kind");
    const teethEl = document.getElementById("gear-teeth");
    const modEl = document.getElementById("gear-module_mm");
    const update = () => this._updateGearDiameterPreview();
    kindEl?.addEventListener("change", update);
    teethEl?.addEventListener("input", update);
    modEl?.addEventListener("input", update);
    update();
  }

  _updateGearDiameterPreview() {
    const kind = document.getElementById("gear-gear_kind")?.value || "spur_involute";
    const teeth = clamp(document.getElementById("gear-teeth")?.value, 12, 80, 24);
    const mod = clamp(document.getElementById("gear-module_mm")?.value, 0.8, 6.0, 2.0);
    const modGroup = document.getElementById("gear-module-group");
    if (modGroup) {
      modGroup.style.opacity = kind === "gt2_pulley" ? "0.45" : "1";
    }

    let dp = mod * teeth;
    let dOut = dp + 2 * mod;
    if (kind === "gt2_pulley") {
      dp = (2.0 * teeth) / Math.PI;
      dOut = dp - 0.508 + 2.4;
    }

    const dpEl = document.getElementById("gear-dp-val");
    const doEl = document.getElementById("gear-do-val");
    if (dpEl) dpEl.textContent = `${dp.toFixed(1)} mm`;
    if (doEl) doEl.textContent = `${dOut.toFixed(1)} mm`;
  }

  _renderGearPresetGrid() {
    const grid = document.getElementById("gear-preset-grid");
    if (!grid) return;
    grid.innerHTML = "";

    for (const [presetId, meta] of Object.entries(GEAR_META)) {
      const preset = this.gearPresets[presetId];
      const title = meta.label || preset?.name;
      const subtitle = meta.subtitle || preset?.subtitle;

      const btn = document.createElement("button");
      btn.type = "button";
      btn.className = `arch-card${presetId === this.currentGearPreset ? " active" : ""}`;
      btn.dataset.gearPreset = presetId;
      btn.innerHTML = `
        <div class="arch-card-head">
          ${meta.icon}
        </div>
        <span class="arch-card-title">${title}</span>
        <span class="arch-card-sub">${subtitle}</span>
      `;
      btn.addEventListener("click", () => {
        this.applyGearPreset(presetId);
      });
      grid.appendChild(btn);
    }
  }

  setGearPresets(rawPresets) {
    if (!rawPresets || typeof rawPresets !== "object") return;
    const entries = Array.isArray(rawPresets)
      ? rawPresets.map((p) => [p.id || p.preset, p])
      : Object.entries(rawPresets);

    for (const [key, val] of entries) {
      if (!key || !val || typeof val !== "object") continue;
      const flat = val.defaults || val.params || val;
      this.gearPresets[key] = {
        ...(this.gearPresets[key] || {}),
        ...flat,
        name: GEAR_META[key]?.label || val.name,
        subtitle: GEAR_META[key]?.subtitle || val.subtitle || val.description,
      };
    }
    this._renderGearPresetGrid();
  }

  applyGearPreset(presetId) {
    const preset = this.gearPresets[presetId] || FALLBACK_GEAR_DEFAULTS[presetId];
    if (!preset) return;

    this.currentGearPreset = presetId;

    document.querySelectorAll("[data-gear-preset]").forEach((card) => {
      card.classList.toggle("active", card.dataset.gearPreset === presetId);
    });

    if (this.activeFamily === "gears") {
      const headerLabel = document.getElementById("header-arch-label");
      if (headerLabel) {
        headerLabel.textContent = GEAR_META[presetId]?.label || preset.name || presetId;
      }
    }

    this._setInputValue("gear-gear_kind", preset.gear_kind);
    this._setInputValue("gear-teeth", preset.teeth);
    this._setInputValue("gear-module_mm", preset.module_mm);
    this._setInputValue("gear-face_width_mm", preset.face_width_mm);
    this._setInputValue("gear-bore_diameter_mm", preset.bore_diameter_mm);
    this._setInputValue("gear-d_shaft_flat_mm", preset.d_shaft_flat_mm);
    this._setInputValue("gear-hub_diameter_mm", preset.hub_diameter_mm);
    this._setInputValue("gear-hub_height_mm", preset.hub_height_mm);
    this._setInputValue("gear-backlash_mm", preset.backlash_mm);

    this._setInputValue("gear-torque_nm", preset.torque_nm);
    this._setInputValue("gear-rpm", preset.rpm);
    this._setInputValue("gear-target_safety_factor", preset.target_safety_factor);
    this._setInputValue("gear-operating_temp_c", preset.operating_temp_c);

    // Compartidos de Material y Manufactura 3D
    if (preset.material_id && this.materialsMap[preset.material_id]) {
      this._setInputValue("material_id", preset.material_id);
      this._renderMaterialBadge(preset.material_id);
    }
    if (preset.print_orientation) {
      this.currentOrientation = preset.print_orientation;
      document.querySelectorAll("[data-orientation]").forEach((p) => {
        p.classList.toggle("active", p.dataset.orientation === this.currentOrientation);
      });
    }
    if (preset.nozzle_diameter_mm !== undefined) {
      this._setInputValue("nozzle_diameter_mm", String(preset.nozzle_diameter_mm));
    }
    if (preset.printer_clearance_mm !== undefined) {
      this._setInputValue("printer_clearance_mm", preset.printer_clearance_mm);
    }

    this._updateGearDiameterPreview();
  }

  /**
   * Construye el payload JSON exacto que espera GearDesignRequest en backend/app/schemas.py.
   */
  getGearPayload() {
    return {
      preset: this.currentGearPreset,
      gear_kind: document.getElementById("gear-gear_kind")?.value || "spur_involute",
      teeth: Math.round(clamp(document.getElementById("gear-teeth")?.value, 12, 80, 24)),
      module_mm: clamp(document.getElementById("gear-module_mm")?.value, 0.8, 6.0, 2.0),
      pressure_angle_deg: 20.0,
      face_width_mm: clamp(document.getElementById("gear-face_width_mm")?.value, 4.0, 35.0, 10.0),
      bore_diameter_mm: clamp(document.getElementById("gear-bore_diameter_mm")?.value, 3.0, 30.0, 5.0),
      d_shaft_flat_mm: clamp(document.getElementById("gear-d_shaft_flat_mm")?.value, 0.0, 2.0, 0.5),
      hub_diameter_mm: clamp(document.getElementById("gear-hub_diameter_mm")?.value, 10.0, 60.0, 16.0),
      hub_height_mm: clamp(document.getElementById("gear-hub_height_mm")?.value, 0.0, 20.0, 5.0),
      backlash_mm: clamp(document.getElementById("gear-backlash_mm")?.value, 0.0, 0.5, 0.15),

      torque_nm: clamp(document.getElementById("gear-torque_nm")?.value, 0.05, 50.0, 1.2),
      rpm: clamp(document.getElementById("gear-rpm")?.value, 1.0, 3000.0, 120.0),
      target_safety_factor: clamp(document.getElementById("gear-target_safety_factor")?.value, 1.1, 10.0, 2.0),
      operating_temp_c: clamp(document.getElementById("gear-operating_temp_c")?.value, 10.0, 130.0, 45.0),

      material_id: document.getElementById("material_id")?.value || "petg_cf",
      printer_id: document.getElementById("printer_id")?.value || "bambu_x1c",
      print_orientation: this.currentOrientation,
      nozzle_diameter_mm: clamp(document.getElementById("nozzle_diameter_mm")?.value, 0.2, 1.0, 0.4),
      printer_clearance_mm: clamp(document.getElementById("printer_clearance_mm")?.value, 0.0, 0.5, 0.15),
    };
  }

  _renderArmPresetGrid() {
    const grid = document.getElementById("arm-preset-grid");
    if (!grid) return;
    grid.innerHTML = "";

    for (const [presetId, meta] of Object.entries(ARM_META)) {
      const preset = this.armPresets[presetId];
      const title = meta.label || preset?.name;
      const subtitle = meta.subtitle || preset?.subtitle;

      const btn = document.createElement("button");
      btn.type = "button";
      btn.className = `arch-card${presetId === this.currentArmPreset ? " active" : ""}`;
      btn.dataset.armPreset = presetId;
      btn.innerHTML = `
        <div class="arch-card-head">
          ${meta.icon}
        </div>
        <span class="arch-card-title">${title}</span>
        <span class="arch-card-sub">${subtitle}</span>
      `;
      btn.addEventListener("click", () => {
        this.applyArmPreset(presetId);
      });
      grid.appendChild(btn);
    }
  }

  setArmPresets(rawPresets) {
    if (!rawPresets || typeof rawPresets !== "object") return;
    const entries = Array.isArray(rawPresets)
      ? rawPresets.map((p) => [p.id || p.preset, p])
      : Object.entries(rawPresets);

    for (const [key, val] of entries) {
      if (!key || !val || typeof val !== "object") continue;
      const flat = val.defaults || val.params || val;
      this.armPresets[key] = {
        ...(this.armPresets[key] || {}),
        ...flat,
        name: ARM_META[key]?.label || val.name,
        subtitle: ARM_META[key]?.subtitle || val.subtitle || val.description,
      };
    }
    this._renderArmPresetGrid();
  }

  applyArmPreset(presetId) {
    const preset = this.armPresets[presetId] || FALLBACK_ARM_DEFAULTS[presetId];
    if (!preset) return;

    this.currentArmPreset = presetId;

    document.querySelectorAll("[data-arm-preset]").forEach((card) => {
      card.classList.toggle("active", card.dataset.armPreset === presetId);
    });

    if (this.activeFamily === "arm") {
      const headerLabel = document.getElementById("header-arch-label");
      if (headerLabel) {
        headerLabel.textContent = ARM_META[presetId]?.label || preset.name || presetId;
      }
    }

    this._setInputValue("arm-dof", preset.dof);
    this._setInputValue("arm-payload_kg", preset.payload_kg);
    this._setInputValue("arm-link_style", preset.link_style);
    this._setInputValue("arm-base_height_mm", preset.base_height_mm);
    this._setInputValue("arm-upper_arm_mm", preset.upper_arm_mm);
    this._setInputValue("arm-forearm_mm", preset.forearm_mm);
    this._setInputValue("arm-wrist_mm", preset.wrist_mm);
    this._setInputValue("arm-shoulder_joint_deg", preset.shoulder_joint_deg);
    this._setInputValue("arm-elbow_joint_deg", preset.elbow_joint_deg);
    this._setInputValue("arm-wrist_joint_deg", preset.wrist_joint_deg);
    this._setInputValue("arm-gear_ratio_shoulder", preset.gear_ratio_shoulder);
    this._setInputValue("arm-gear_ratio_elbow", preset.gear_ratio_elbow);
    this._setInputValue("arm-max_accel_rad_s2", preset.max_accel_rad_s2);

    if (preset.material_id && this.materialsMap[preset.material_id]) {
      this._setInputValue("material_id", preset.material_id);
      this._renderMaterialBadge(preset.material_id);
    }
    if (preset.printer_id && this.printersMap[preset.printer_id]) {
      this._setInputValue("printer_id", preset.printer_id);
      this._applyPrinterDefaults(preset.printer_id, false);
    }
    if (preset.print_orientation) {
      this.currentOrientation = preset.print_orientation;
      document.querySelectorAll("[data-orientation]").forEach((p) => {
        p.classList.toggle("active", p.dataset.orientation === this.currentOrientation);
      });
    }
  }

  /**
   * Construye el payload JSON exacto que espera ArmAssemblyRequest en backend/app/schemas.py.
   */
  getArmPayload() {
    return {
      preset: this.currentArmPreset,
      dof: Math.round(clamp(document.getElementById("arm-dof")?.value, 4, 6, 6)),
      payload_kg: clamp(document.getElementById("arm-payload_kg")?.value, 0.1, 8.0, 1.0),
      base_height_mm: clamp(document.getElementById("arm-base_height_mm")?.value, 50, 200, 95),
      upper_arm_mm: clamp(document.getElementById("arm-upper_arm_mm")?.value, 100, 400, 200),
      forearm_mm: clamp(document.getElementById("arm-forearm_mm")?.value, 80, 350, 170),
      wrist_mm: clamp(document.getElementById("arm-wrist_mm")?.value, 40, 160, 75),
      link_style: document.getElementById("arm-link_style")?.value || "hollow_cobot",

      shoulder_joint_deg: clamp(document.getElementById("arm-shoulder_joint_deg")?.value, -60, 90, 35),
      elbow_joint_deg: clamp(document.getElementById("arm-elbow_joint_deg")?.value, -130, 130, -55),
      wrist_joint_deg: clamp(document.getElementById("arm-wrist_joint_deg")?.value, -90, 90, 20),

      gear_ratio_shoulder: clamp(document.getElementById("arm-gear_ratio_shoulder")?.value, 4, 120, 20),
      gear_ratio_elbow: clamp(document.getElementById("arm-gear_ratio_elbow")?.value, 4, 100, 10),
      max_accel_rad_s2: clamp(document.getElementById("arm-max_accel_rad_s2")?.value, 1, 100, 12),

      material_id: document.getElementById("material_id")?.value || "petg_cf",
      print_orientation: this.currentOrientation,
      printer_id: document.getElementById("printer_id")?.value || "bambu_x1c",
    };
  }
}
