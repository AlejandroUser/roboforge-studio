export type FamilyId = "links" | "mounts" | "gears" | "arm";

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
}
