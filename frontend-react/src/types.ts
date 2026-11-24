export type FamilyId = "links" | "mounts" | "gears" | "arm" | "projects";

export interface AlertItem {
  level: "info" | "warning" | "critical";
  code: string;
  message: string;
}

export interface VariantMetrics {
  volume_cm3: number;
  mass_g: number;
  mass_savings_pct: number;
  reference_savings_pct?: number | null;
  rotational_inertia_kg_cm2: number;
  max_stress_mpa: number;
  effective_yield_mpa: number;
  safety_factor: number;
  tip_deflection_mm: number;
  dynamic_torque_nm: number;
  estimated_cost_usd: number;
  stiffness_to_weight_score: number;
  print_without_supports: boolean;
  passes_requirements: boolean;
  stress_profile?: number[];
}

export interface VariantResult {
  variant_id: string;
  name: string;
  tagline: string;
  is_valid_solid: boolean;
  metrics: VariantMetrics;
  glb_url: string;
  step_url: string;
  stl_url: string;
  threemf_url?: string | null;
  alerts: AlertItem[];
}

export interface GenerateResponse {
  design_hash: string;
  recommended_variant: string;
  material_name: string;
  pitch_diameter_mm?: number;
  outer_diameter_mm?: number;
  global_alerts: AlertItem[];
  variants: VariantResult[];
}

export interface FEAResult {
  design_hash: string;
  variant_id: string;
  solver_name: string;
  node_count: number;
  element_count: number;
  solve_time_s: number;
  max_von_mises_mpa: number;
  p95_von_mises_mpa: number;
  max_displacement_mm: number;
  effective_yield_mpa: number;
  fea_safety_factor: number;
  passes_fea: boolean;
  is_orthotropic?: boolean;
  first_natural_freq_hz?: number | null;
  buckling_critical_load_n?: number | null;
  buckling_safety_factor?: number | null;
  fea_glb_url: string;
  notes: string[];
}

export interface JointTorqueSummary {
  joint_id: string;
  joint_name: string;
  segment_length_mm: number;
  segment_mass_g: number;
  motor_mass_g: number;
  static_torque_nm: number;
  dynamic_torque_nm: number;
  gear_ratio: number;
  required_motor_torque_nm: number;
  recommended_actuator: string;
  safety_factor: number;
}

export interface BOMItem {
  category: string;
  part_name: string;
  spec: string;
  qty: number;
  unit_cost_usd: number;
  total_cost_usd: number;
}

export interface ArmAssemblyResponse {
  design_hash: string;
  preset_name: string;
  material_name: string;
  dof: number;
  max_reach_mm: number;
  total_arm_mass_kg: number;
  printed_mass_g: number;
  estimated_total_cost_usd: number;
  tcp_position_mm: number[];
  tcp_deflection_mm: number;
  joints: JointTorqueSummary[];
  bom: BOMItem[];
  global_alerts: AlertItem[];
  glb_url: string;
  step_url: string;
  stl_url: string;
  urdf_url: string;
  mujoco_url: string;
  bom_csv_url: string;
}

export interface CopilotResponse {
  summary: string;
  parsed_constraints: string[];
  optimized_request: Record<string, any>;
  iterations: Array<{
    step: number;
    root_width_mm: number;
    thickness_mm: number;
    mass_g: number;
    safety_factor: number;
    tip_deflection_mm: number;
    passes: boolean;
  }>;
  converged: boolean;
  fea_verified: boolean;
  fea_result?: FEAResult | null;
}

export interface BlueprintAnalysisResponse {
  filename: string;
  extraction_mode: "multimodal_llm" | "cv_offline";
  detected_aspect_ratio: number;
  detected_holes_count: number;
  notes: string[];
  copilot_result: CopilotResponse;
}

export interface GyroidResult {
  design_hash: string;
  cell_size_mm: number;
  voxel_grid: number[];
  triangle_count: number;
  volume_cm3: number;
  mass_g: number;
  solve_time_s: number;
  glb_url: string;
  stl_url: string;
  threemf_url: string;
  notes: string[];
}

// v3.0: Ensamblador Modular Real + Contrato de Acoples + Cascada con Masas Reales
export type ArmSlotKey =
  | "shoulder_mount"
  | "shoulder_gear"
  | "upper_arm"
  | "elbow_gear"
  | "forearm";

export interface ModularArmSlots {
  shoulder_mount?: Record<string, any> | null;
  shoulder_mount_variant: string;
  shoulder_gear?: Record<string, any> | null;
  shoulder_gear_variant: string;
  upper_arm?: Record<string, any> | null;
  upper_arm_variant: string;
  elbow_gear?: Record<string, any> | null;
  elbow_gear_variant: string;
  forearm?: Record<string, any> | null;
  forearm_variant: string;
}

export type KinematicsType =
  | "articulated"
  | "scara"
  | "cartesian_gantry"
  | "delta_parallel"
  | "continuum_pcc"
  | "bimanual_dual_arm"
  | "stewart_hexapod"
  | "soft_pneumatic_bellows";

export type EnvironmentMode = "earth" | "space_zero_g" | "underwater_rov";

export interface PrintPlateInfo {
  plate_index?: number;
  plate_number?: number;
  filename?: string;
  url?: string;
  parts: string[];
  part_count?: number;
  estimated_filament_g?: number;
  estimated_hours?: number;
}

export interface PrintReadinessSummary {
  fits_bed: boolean;
  printer_name: string;
  bed_mm: number[];
  total_plates: number;
  total_filament_g: number;
  estimated_print_hours: number;
  layer_height_mm: number;
  wall_loops: number;
  infill_pct: number;
  infill_pattern: string;
  nozzle_temp_c: number;
  bed_temp_c: number;
  plates: PrintPlateInfo[];
  warnings: string[];
  // v7.1 MSLA Resin fields
  process_type?: "fdm" | "msla_resin";
  exposure_s?: number | null;
  bottom_exposure_s?: number | null;
  recommended_tilt_deg?: number | null;
  // v10.0 Auto-Dovetail Split
  split_parts_count?: number;
}

export interface ModularArmRequest {
  dof: number;
  payload_kg: number;
  material_id: string;
  printer_id?: string | null;
  base_height_mm: number;
  wrist_mm: number;
  max_accel_rad_s2: number;
  joint_angles_deg: [number, number, number];
  slots: ModularArmSlots;
  auto_fix?: boolean;
  mechatronics?: MechatronicsConfig | null;
  // v5.0 & v6.0 Universal Arms
  kinematics_type?: KinematicsType;
  z_stroke_mm?: number;
  wrist_3dof?: boolean;
  cnc_plate_mode?: boolean;
  remote_base_motors?: boolean;
  four_bar_parallelogram?: boolean;
  bevel_differential_wrist?: boolean;
  vacuum_eoat?: boolean;
  // v7.0 Environments & Counterbalance
  environment_mode?: EnvironmentMode;
  gravity_m_s2?: number;
  counterbalance_j2?: boolean;
  // v10.0 Industrial Cobots (DH offsets, Torque Sensors, 24V Brakes, Auto-Dovetail Split)
  dh_lateral_offsets_mm?: number[];
  joint_torque_sensors?: boolean;
  electromagnetic_brakes?: boolean;
  auto_dovetail_split?: boolean;
}

export interface CompatibilityIssue {
  severity: "ok" | "warning" | "critical";
  slot: string;
  parameter: string;
  current_value: string;
  suggested_value: string;
  message: string;
}

export interface PartPhysicsMetric {
  slot: string;
  part_name: string;
  variant: string;
  mass_g: number;
  volume_mm3: number;
  safety_factor: number;
  torque_nm: number;
  status: "ok" | "warning" | "critical";
}

export interface ModularArmResponse {
  design_hash: string;
  compatible: boolean;
  auto_fixed: boolean;
  auto_fix_notes: string[];
  compatibility_issues: CompatibilityIssue[];
  per_part_metrics: PartPhysicsMetric[];
  total_mass_g: number;
  reach_mm: number;
  shoulder_torque_nm: number;
  min_safety_factor: number;
  tcp_position_mm: number[];
  explode_vectors: Record<string, number[]>;
  files: Record<string, string>;
  slots_used: ModularArmSlots;
  joints: JointTorqueSummary[];
  bom: BOMItem[];
  mechatronics?: MechatronicsResponse | null;
  print_readiness?: PrintReadinessSummary | null;
}

// v4.0/v5.0/v7.1: Mecatrónica Completa — Motores, Curvas Torque–RPM, Presupuesto Eléctrico y BOM Maestro
export type BusArchitecture =
  | "can_fd_bus"
  | "stm32_step_dir"
  | "arduino_pca9685_i2c"
  | "uart_serial_bus"
  | "teensy_dm542"
  | "esp32_pca9685_5v";
export type MotorType =
  | "stepper"
  | "closed_loop_stepper"
  | "bldc_qdd"
  | "rc_servo"
  | "serial_bus_servo";

export interface JointActuatorSpec {
  motor_id: string;
  reducer_id: string;
  target_rpm: number;
  parallel_count?: number;
}

export interface MechatronicsConfig {
  bus_architecture: BusArchitecture;
  system_voltage_v: number;
  joint_actuators: Record<string, JointActuatorSpec>;
}

export interface TorqueSpeedPoint {
  rpm: number;
  continuous_torque_nm: number;
  peak_torque_nm: number;
}

export interface JointMechatronicsReport {
  joint_id: string;
  joint_name: string;
  motor_id: string;
  motor_name: string;
  motor_type: MotorType;
  parallel_count?: number;
  reducer_id: string;
  reducer_name: string;
  gear_ratio: number;
  efficiency: number;
  operating_rpm: number;
  required_rms_torque_nm: number;
  required_peak_torque_nm: number;
  available_continuous_torque_nm: number;
  available_peak_torque_nm: number;
  torque_margin_pct: number;
  thermal_margin_pct: number;
  continuous_current_a: number;
  peak_current_a: number;
  status: "ok" | "warning" | "critical";
  torque_speed_curve: TorqueSpeedPoint[];
}

export interface PowerBudget {
  bus_architecture: BusArchitecture;
  system_voltage_v: number;
  continuous_current_a: number;
  peak_current_a: number;
  continuous_power_w: number;
  peak_power_w: number;
  recommended_psu: string;
  psu_cost_usd: number;
  power_wire_awg: string;
  signal_wire_awg: string;
  total_harness_length_m: number;
  fuse_rating_a: number;
}

export type MasterBomCategory =
  | "printed_cnc"
  | "actuators_reducers"
  | "electronics_wiring"
  | "fasteners_hardware";

export interface MasterBomItem {
  category: MasterBomCategory;
  category_label: string;
  part_code: string;
  part_name: string;
  spec: string;
  qty: number;
  unit_cost_usd: number;
  total_cost_usd: number;
}

export interface MechatronicsResponse {
  bus_architecture: BusArchitecture;
  joint_reports: JointMechatronicsReport[];
  power_budget: PowerBudget;
  master_bom: MasterBomItem[];
  total_bom_cost_usd: number;
  notes: string[];
}

export interface ActuatorCatalogItem {
  id: string;
  name: string;
  motor_type: MotorType;
  voltage_v: number;
  rated_torque_nm: number;
  peak_torque_nm: number;
  no_load_rpm: number;
  corner_rpm: number;
  rated_current_a: number;
  peak_current_a: number;
  mass_g: number;
  unit_cost_usd: number;
}

export interface ReducerCatalogItem {
  id: string;
  name: string;
  ratio: number;
  efficiency: number;
  unit_cost_usd?: number;
}

export interface ArchitectureCatalogItem {
  id: BusArchitecture;
  name: string;
  description?: string;
}

export interface MechatronicsCatalog {
  actuators: ActuatorCatalogItem[];
  reducers: ReducerCatalogItem[];
  architectures: ArchitectureCatalogItem[];
}

// v5.0: Base de Datos de Brazos Robóticos Reales
export interface ArmDatabaseEntry {
  id: string;
  name: string;
  manufacturer_or_project: string;
  category: string;
  kinematics_type: string;
  dof: number;
  reach_mm: number;
  payload_kg: number;
  repeatability_mm: number;
  mass_kg: number;
  primary_materials: string[];
  actuators_summary: string;
  transmission_summary: string;
  electronics_summary: string;
  estimated_cost_usd: number;
  roboforge_preset_id: string | null;
  roboforge_replicability_pct: number;
  supported_today: string[];
  missing_capabilities: string[];
}

export interface ArmDatabaseResponse {
  schema_version?: string;
  total_arms?: number;
  average_replicability_pct?: number;
  arms: ArmDatabaseEntry[];
}

// v8.0: Cinemática Inversa 3D, Calibración Pie de Rey y Control USB en Vivo
export interface ArmIkRequest {
  target_x_mm: number;
  target_y_mm: number;
  target_z_mm: number;
  l1_mm: number;
  l2_mm: number;
  l3_mm: number;
  base_height_mm: number;
  kinematics_type?: string;
  elbow_up?: boolean;
}

export interface ArmIkResponse {
  reachable: boolean;
  reach_error_mm: number;
  tcp_mm: number[];
  j0_base_deg: number;
  j1_shoulder_deg: number;
  j2_elbow_deg: number;
  j3_wrist_deg: number;
  z_mm?: number;
}

export interface CaliperCalibrateRequest {
  measured_m3_mm: number;
  measured_bore_mm: number;
  nominal_m3_mm?: number;
  nominal_bore_mm?: number;
  process_type?: "fdm" | "msla_resin";
}

export interface CaliperCalibrateResponse {
  status: "perfect" | "tight" | "loose";
  xy_hole_compensation_mm: number;
  recommended_clearance_mm: number;
  exposure_delta_s?: number | null;
  mean_error_mm?: number;
  diagnosis?: string;
  message?: string;
  slicer_tip?: string;
}

export interface HardwarePortInfo {
  port: string;
  description: string;
}

export interface LiveArmPose {
  j0_base: number;
  j1_shoulder: number;
  j2_elbow: number;
  j3_wrist: number;
  gripper_pct: number;
}

export interface HardwarePoseRequest extends LiveArmPose {
  port: string;
  baudrate?: number;
}

export interface HardwarePoseResponse {
  ok: boolean;
  frame: string;
  port: string;
  mode: string;
  error?: string;
}

// v9.0: Auditor DFM Goldfish + Familias Multi-Proyecto (FPV Drone, Rover 4WD, Camera Gimbal)
export type DFMStatus = "pass" | "warn" | "fail";

export interface DFMCheckItem {
  id: string;
  label: string;
  status: DFMStatus;
  detail: string;
}

export interface DFMAuditReport {
  status: DFMStatus;
  summary: string;
  checks: DFMCheckItem[];
}

export type ProjectFamilyId = "fpv_drone" | "rover_4wd" | "camera_gimbal";

export interface ProjectPresetInfo {
  id: string;
  name: string;
  subtitle?: string;
  defaults?: Record<string, any>;
  material_id?: string;
  printer_id?: string;
}

export interface ProjectFamilyInfo {
  id: ProjectFamilyId;
  name: string;
  subtitle: string;
  icon?: string;
  presets: ProjectPresetInfo[];
}

export interface ProjectGenerateRequest {
  preset_id: string;
  material_id: string;
  printer_id: string;
  params: Record<string, any>;
}

export interface ProjectGenerateResponse {
  family_id: string;
  preset_id: string;
  design_hash: string;
  name: string;
  metrics: Record<string, any>;
  bom: MasterBomItem[];
  total_bom_cost_usd: number;
  dfm_audit: DFMAuditReport;
  explode_vectors: Record<string, number[]>;
  files: Record<string, string>;
}
