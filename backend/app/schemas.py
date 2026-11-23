from __future__ import annotations

from typing import Literal, Optional
from pydantic import BaseModel, Field


ArmArchitecture = Literal["articulated_6dof", "scara", "delta_rod", "robot_leg"]
PrintOrientation = Literal["xy_flat", "z_vertical"]
EndPreset = Literal["nema17", "nema23", "bearing_608", "bearing_6805", "custom"]
VariantKind = Literal[
    "tapered_solid",
    "organic_ibeam",
    "rounded_truss",
    "hollow_cobot",
    "organic_topopt",
]

# Hito 3B: Familia 2 — Monturas Mecatrónicas
MountActuatorType = Literal["nema17", "nema23", "servo_mg996r", "profile_2020_gusset"]
MountBaseType = Literal["profile_2020", "profile_2040", "flat_plate"]
MountVariantKind = Literal["mount_standard", "mount_gusseted", "mount_lightweight"]

# Hito 3C: Familia 3 — Transmisión (Engranajes Involuta y Poleas GT2)
GearPresetType = Literal["spur_nema17", "spur_heavy", "gt2_pulley_20t", "gt2_pulley_40t"]
GearKind = Literal["spur_involute", "gt2_pulley"]
GearVariantKind = Literal["gear_solid", "gear_webbed", "gear_spoked"]


class EndInterfaceParams(BaseModel):
    preset: EndPreset = "bearing_608"
    outer_diameter_mm: float = Field(36.0, ge=14.0, le=120.0)
    bore_diameter_mm: float = Field(22.0, ge=3.0, le=90.0)
    bolt_count: int = Field(4, ge=0, le=8)
    bolt_pcd_mm: float = Field(29.0, ge=10.0, le=105.0)
    bolt_size: Literal["M3", "M4", "M5"] = "M3"
    use_heat_inserts: bool = True


class LinkDesignRequest(BaseModel):
    architecture: ArmArchitecture = "articulated_6dof"
    length_mm: float = Field(180.0, ge=60.0, le=450.0)
    root_width_mm: float = Field(38.0, ge=16.0, le=110.0)
    tip_width_mm: float = Field(24.0, ge=12.0, le=90.0)
    thickness_mm: float = Field(20.0, ge=8.0, le=60.0)
    organic_taper: float = Field(0.75, ge=0.0, le=1.0, description="0=recto, 1=curva parabólica biónica")
    cable_channel: bool = True

    root_end: EndInterfaceParams = Field(
        default_factory=lambda: EndInterfaceParams(
            preset="nema17",
            outer_diameter_mm=46.0,
            bore_diameter_mm=22.0,
            bolt_count=4,
            bolt_pcd_mm=31.0,
            bolt_size="M3",
            use_heat_inserts=True,
        )
    )
    tip_end: EndInterfaceParams = Field(
        default_factory=lambda: EndInterfaceParams(
            preset="bearing_608",
            outer_diameter_mm=32.0,
            bore_diameter_mm=22.0,
            bolt_count=0,
            bolt_pcd_mm=26.0,
            bolt_size="M3",
            use_heat_inserts=False,
        )
    )

    # Cargas y objetivos
    tip_load_n: float = Field(25.0, ge=1.0, le=500.0)
    angular_accel_rad_s2: float = Field(15.0, ge=0.0, le=200.0)
    target_safety_factor: float = Field(2.0, ge=1.1, le=10.0)
    max_deflection_mm: float = Field(0.5, ge=0.05, le=10.0)
    operating_temp_c: float = Field(45.0, ge=-10.0, le=150.0)

    # Material y manufactura
    material_id: str = "petg_cf"
    print_orientation: PrintOrientation = "xy_flat"
    nozzle_diameter_mm: float = Field(0.4, ge=0.2, le=1.0)
    printer_clearance_mm: float = Field(0.15, ge=0.0, le=0.5)

    # Hito 2: Referencia .STEP opcional y fracción de volumen TopOpt
    reference_volume_cm3: Optional[float] = Field(None, gt=0.0)
    topopt_volume_fraction: float = Field(0.52, ge=0.30, le=0.75)

    # Hito 4: Perfil de impresora 3D opcional para DFM (cama y cámara cerrada)
    printer_id: Optional[str] = None


class AlertItem(BaseModel):
    level: Literal["info", "warning", "critical"]
    code: str
    message: str


class VariantMetrics(BaseModel):
    volume_cm3: float
    mass_g: float
    mass_savings_pct: float
    reference_savings_pct: Optional[float] = None
    rotational_inertia_kg_cm2: float
    max_stress_mpa: float
    effective_yield_mpa: float
    safety_factor: float
    tip_deflection_mm: float
    dynamic_torque_nm: float
    estimated_cost_usd: float
    stiffness_to_weight_score: float
    print_without_supports: bool
    passes_requirements: bool
    stress_profile: list[float] = Field(
        default_factory=list,
        description="Esfuerzo normalizado [0..1] en 12 estaciones de raíz (x=0) a punta (x=L) para mapa de calor 3D",
    )


class VariantResult(BaseModel):
    variant_id: VariantKind
    name: str
    tagline: str
    is_valid_solid: bool
    metrics: VariantMetrics
    glb_url: str
    step_url: str
    stl_url: str
    threemf_url: Optional[str] = None
    alerts: list[AlertItem] = Field(default_factory=list)


class GenerateResponse(BaseModel):
    design_hash: str
    recommended_variant: VariantKind
    material_name: str
    global_alerts: list[AlertItem] = Field(default_factory=list)
    variants: list[VariantResult]


class DetectedHole(BaseModel):
    center_x_mm: float
    center_y_mm: float
    diameter_mm: float


class StepReferenceAnalysis(BaseModel):
    filename: str
    bbox_length_mm: float
    bbox_width_mm: float
    bbox_thickness_mm: float
    volume_cm3: float
    detected_holes: list[DetectedHole] = Field(default_factory=list)
    suggested_request: LinkDesignRequest
    reference_glb_url: Optional[str] = None
    notes: list[str] = Field(default_factory=list)


# Hito 3A: Validación FEA 3D Real (Gmsh + 3D Tet4 Linear Elasticity FEM)
class FEARequest(BaseModel):
    design_hash: str
    variant_id: VariantKind
    request: LinkDesignRequest
    mesh_size_mm: float = Field(7.0, ge=3.0, le=15.0)
    load_direction: Literal["y_inplane", "z_outofplane"] = "y_inplane"
    deform_scale: float = Field(0.0, ge=0.0, le=100.0)


class FEAResult(BaseModel):
    design_hash: str
    variant_id: str
    solver_name: str
    node_count: int
    element_count: int
    solve_time_s: float
    max_von_mises_mpa: float
    p95_von_mises_mpa: float
    max_displacement_mm: float
    effective_yield_mpa: float
    fea_safety_factor: float
    passes_fea: bool
    is_orthotropic: bool = True
    first_natural_freq_hz: Optional[float] = None
    buckling_critical_load_n: Optional[float] = None
    buckling_safety_factor: Optional[float] = None
    fea_glb_url: str
    notes: list[str] = Field(default_factory=list)


# Hito 3B: Familia 2 — Monturas Mecatrónicas (NEMA 17/23, Servos, Perfil 2020)
class MountDesignRequest(BaseModel):
    actuator_type: MountActuatorType = "nema17"
    base_type: MountBaseType = "profile_2020"
    wall_thickness_mm: float = Field(5.0, ge=2.5, le=16.0)
    base_depth_mm: float = Field(42.0, ge=24.0, le=100.0)
    shaft_height_mm: float = Field(32.0, ge=20.0, le=90.0)
    gusset_thickness_mm: float = Field(4.0, ge=2.0, le=14.0)
    use_heat_inserts: bool = False
    ventilation_slots: bool = True

    # Cargas y ambiente del motor
    radial_load_n: float = Field(35.0, ge=1.0, le=500.0)
    motor_torque_nm: float = Field(0.6, ge=0.0, le=20.0)
    target_safety_factor: float = Field(2.0, ge=1.1, le=10.0)
    max_deflection_mm: float = Field(0.3, ge=0.05, le=5.0)
    motor_temp_c: float = Field(58.0, ge=15.0, le=130.0)

    # Material y manufactura
    material_id: str = "petg_cf"
    print_orientation: PrintOrientation = "xy_flat"
    nozzle_diameter_mm: float = Field(0.4, ge=0.2, le=1.0)
    printer_clearance_mm: float = Field(0.15, ge=0.0, le=0.5)
    printer_id: Optional[str] = None


class MountVariantResult(BaseModel):
    variant_id: MountVariantKind
    name: str
    tagline: str
    is_valid_solid: bool
    metrics: VariantMetrics
    glb_url: str
    step_url: str
    stl_url: str
    threemf_url: Optional[str] = None
    alerts: list[AlertItem] = Field(default_factory=list)


class MountGenerateResponse(BaseModel):
    design_hash: str
    recommended_variant: MountVariantKind
    material_name: str
    global_alerts: list[AlertItem] = Field(default_factory=list)
    variants: list[MountVariantResult]


class MountFEARequest(BaseModel):
    design_hash: str
    variant_id: MountVariantKind
    request: MountDesignRequest
    mesh_size_mm: float = Field(5.5, ge=2.5, le=12.0)
    load_direction: Literal["y_inplane", "z_outofplane"] = "y_inplane"
    deform_scale: float = Field(0.0, ge=0.0, le=100.0)


# Hito 3C: Familia 3 — Transmisión (Engranajes Involuta y Poleas GT2)
class GearDesignRequest(BaseModel):
    preset: GearPresetType = "spur_nema17"
    gear_kind: GearKind = "spur_involute"
    teeth: int = Field(24, ge=12, le=80)
    module_mm: float = Field(2.0, ge=0.8, le=6.0)
    pressure_angle_deg: float = Field(20.0, ge=14.5, le=25.0)
    face_width_mm: float = Field(10.0, ge=4.0, le=35.0)
    bore_diameter_mm: float = Field(5.0, ge=3.0, le=30.0)
    d_shaft_flat_mm: float = Field(0.5, ge=0.0, le=2.0)
    hub_diameter_mm: float = Field(16.0, ge=10.0, le=60.0)
    hub_height_mm: float = Field(5.0, ge=0.0, le=20.0)
    backlash_mm: float = Field(0.15, ge=0.0, le=0.5)

    # Cargas de transmisión
    torque_nm: float = Field(1.2, ge=0.05, le=50.0)
    rpm: float = Field(120.0, ge=1.0, le=3000.0)
    target_safety_factor: float = Field(2.0, ge=1.1, le=10.0)
    operating_temp_c: float = Field(45.0, ge=10.0, le=130.0)

    # Material y manufactura 3D
    material_id: str = "petg_cf"
    print_orientation: PrintOrientation = "xy_flat"
    nozzle_diameter_mm: float = Field(0.4, ge=0.2, le=1.0)
    printer_clearance_mm: float = Field(0.15, ge=0.0, le=0.5)
    printer_id: Optional[str] = None


class GearVariantResult(BaseModel):
    variant_id: GearVariantKind
    name: str
    tagline: str
    is_valid_solid: bool
    metrics: VariantMetrics
    glb_url: str
    step_url: str
    stl_url: str
    threemf_url: Optional[str] = None
    alerts: list[AlertItem] = Field(default_factory=list)


class GearGenerateResponse(BaseModel):
    design_hash: str
    recommended_variant: GearVariantKind
    material_name: str
    pitch_diameter_mm: float
    outer_diameter_mm: float
    global_alerts: list[AlertItem] = Field(default_factory=list)
    variants: list[GearVariantResult]


class GearFEARequest(BaseModel):
    design_hash: str
    variant_id: GearVariantKind
    request: GearDesignRequest
    mesh_size_mm: float = Field(4.5, ge=2.0, le=10.0)
    load_direction: Literal["y_inplane", "z_outofplane"] = "y_inplane"
    deform_scale: float = Field(0.0, ge=0.0, le=100.0)


# Hito 4: Perfiles de Impresora 3D y Materiales Personalizados
class PrinterProfileCreate(BaseModel):
    id: str = Field(..., pattern=r"^[a-z0-9_]{2,32}$")
    name: str = Field(..., min_length=2, max_length=64)
    bed_x_mm: float = Field(256.0, ge=100.0, le=600.0)
    bed_y_mm: float = Field(256.0, ge=100.0, le=600.0)
    bed_z_mm: float = Field(256.0, ge=100.0, le=600.0)
    default_nozzle_mm: float = Field(0.4, ge=0.2, le=1.0)
    clearance_mm: float = Field(0.15, ge=0.0, le=0.5)
    max_bed_temp_c: float = Field(110.0, ge=40.0, le=150.0)
    enclosed: bool = True
    notes: str = ""


class PrinterProfile(PrinterProfileCreate):
    is_custom: bool = False


class CustomMaterialCreate(BaseModel):
    id: str = Field(..., pattern=r"^[a-z0-9_]{2,32}$")
    name: str = Field(..., min_length=2, max_length=64)
    category: Literal["fdm", "cnc"] = "fdm"
    density_g_cm3: float = Field(..., gt=0.5, le=10.0)
    youngs_xy_mpa: float = Field(..., gt=200.0, le=250000.0)
    youngs_z_mpa: float = Field(..., gt=200.0, le=250000.0)
    yield_xy_mpa: float = Field(..., gt=10.0, le=1500.0)
    yield_z_mpa: float = Field(..., gt=10.0, le=1500.0)
    max_temp_c: float = Field(..., ge=40.0, le=600.0)
    cost_per_kg_usd: float = Field(..., ge=5.0, le=500.0)
    color_hex: str = Field("#38bdf8", pattern=r"^#[0-9a-fA-F]{6}$")
    notes: str = ""


# Hito 5 + v5.0: Cadena Cinemática Multi-Eslabón (Brazo Completo 4–7 DOF + SCARA) + URDF/MuJoCo + BOM
ArmPresetType = Literal[
    "parol6_desktop",
    "ar4_industrial",
    "so_arm_lerobot",
    "cobot_hollow_ur",
    "fabri_arm_5",
    "pybot_scara",
    "openarm_7dof",
    "arctos_6dof",
    "bcn3d_moveo",
]


class ArmAssemblyRequest(BaseModel):
    preset: ArmPresetType = "parol6_desktop"
    dof: int = Field(6, ge=4, le=7)
    payload_kg: float = Field(1.0, ge=0.1, le=8.0)
    base_height_mm: float = Field(95.0, ge=50.0, le=200.0)
    upper_arm_mm: float = Field(200.0, ge=100.0, le=400.0)
    forearm_mm: float = Field(170.0, ge=80.0, le=350.0)
    wrist_mm: float = Field(75.0, ge=40.0, le=160.0)
    link_style: Literal["organic_ibeam", "hollow_cobot", "rounded_truss", "tapered_solid"] = "hollow_cobot"

    # Ángulos de cinemática directa (grados)
    shoulder_joint_deg: float = Field(35.0, ge=-60.0, le=90.0)
    elbow_joint_deg: float = Field(-55.0, ge=-130.0, le=130.0)
    wrist_joint_deg: float = Field(20.0, ge=-90.0, le=90.0)

    # Transmisión y dinámica
    gear_ratio_shoulder: float = Field(20.0, ge=4.0, le=120.0)
    gear_ratio_elbow: float = Field(10.0, ge=4.0, le=100.0)
    max_accel_rad_s2: float = Field(12.0, ge=1.0, le=100.0)

    # Material e impresora
    material_id: str = "petg_cf"
    print_orientation: PrintOrientation = "xy_flat"
    printer_id: Optional[str] = "bambu_x1c"


class JointTorqueSummary(BaseModel):
    joint_id: str
    joint_name: str
    segment_length_mm: float
    segment_mass_g: float
    motor_mass_g: float
    static_torque_nm: float
    dynamic_torque_nm: float
    gear_ratio: float
    required_motor_torque_nm: float
    recommended_actuator: str
    safety_factor: float


class BOMItem(BaseModel):
    category: Literal["Impresión 3D", "Actuadores", "Rodamientos", "Tornillería", "Transmisión"]
    part_name: str
    spec: str
    qty: int
    unit_cost_usd: float
    total_cost_usd: float


class ArmAssemblyResponse(BaseModel):
    design_hash: str
    preset_name: str
    material_name: str
    dof: int
    max_reach_mm: float
    total_arm_mass_kg: float
    printed_mass_g: float
    estimated_total_cost_usd: float
    tcp_position_mm: list[float]
    tcp_deflection_mm: float
    joints: list[JointTorqueSummary]
    bom: list[BOMItem]
    global_alerts: list[AlertItem] = Field(default_factory=list)
    glb_url: str
    step_url: str
    stl_url: str
    urdf_url: str
    mujoco_url: str
    bom_csv_url: str


# Hito 5 + Eje 4: Copiloto de Optimización Automática en Bucle Cerrado (Analítico + FEA 3D + Planos 2D)
class CopilotRequest(BaseModel):
    prompt: str = Field(..., min_length=4, max_length=600)
    current_request: Optional[LinkDesignRequest] = None
    verify_with_fea: bool = False


class CopilotIteration(BaseModel):
    step: int
    root_width_mm: float
    thickness_mm: float
    mass_g: float
    safety_factor: float
    tip_deflection_mm: float
    passes: bool


class CopilotResponse(BaseModel):
    summary: str
    parsed_constraints: list[str]
    optimized_request: LinkDesignRequest
    iterations: list[CopilotIteration]
    converged: bool
    fea_verified: bool = False
    fea_result: Optional[FEAResult] = None


class BlueprintAnalysisResponse(BaseModel):
    filename: str
    extraction_mode: Literal["multimodal_llm", "cv_offline"]
    detected_aspect_ratio: float
    detected_holes_count: int
    notes: list[str] = Field(default_factory=list)
    copilot_result: CopilotResponse


# Sprint 6C: Motor Implícito SDF + Celosía Gyroid (TPMS) guiada por campo físico (estilo PicoGK / nTop)
class GyroidRequest(BaseModel):
    request: LinkDesignRequest
    cell_size_mm: float = Field(14.0, ge=6.0, le=30.0)
    shell_thickness_mm: float = Field(2.4, ge=1.2, le=6.0)
    min_density: float = Field(0.28, ge=0.15, le=0.60)
    max_density: float = Field(0.68, ge=0.35, le=0.90)
    voxel_size_mm: float = Field(1.6, ge=0.8, le=3.0)


class GyroidResult(BaseModel):
    design_hash: str
    cell_size_mm: float
    voxel_grid: list[int]
    triangle_count: int
    volume_cm3: float
    mass_g: float
    solve_time_s: float
    glb_url: str
    stl_url: str
    threemf_url: str
    notes: list[str] = Field(default_factory=list)


# v3.0: Ensamblador Modular Real + Contrato de Acoples + Cascada con Masas Reales
class ModularArmSlots(BaseModel):
    shoulder_mount: Optional[MountDesignRequest] = None
    shoulder_mount_variant: MountVariantKind = "mount_gusseted"
    shoulder_gear: Optional[GearDesignRequest] = None
    shoulder_gear_variant: GearVariantKind = "gear_spoked"
    upper_arm: Optional[LinkDesignRequest] = None
    upper_arm_variant: VariantKind = "organic_ibeam"
    elbow_gear: Optional[GearDesignRequest] = None
    elbow_gear_variant: GearVariantKind = "gear_webbed"
    forearm: Optional[LinkDesignRequest] = None
    forearm_variant: VariantKind = "hollow_cobot"


class ModularArmRequest(BaseModel):
    dof: int = Field(6, ge=4, le=7)
    kinematics_type: Literal["articulated", "scara"] = "articulated"
    z_stroke_mm: float = Field(default=120.0, ge=40.0, le=300.0)
    wrist_3dof: bool = True
    cnc_plate_mode: bool = False
    payload_kg: float = Field(1.0, ge=0.1, le=8.0)
    material_id: str = "petg_cf"
    printer_id: Optional[str] = "bambu_x1c"
    base_height_mm: float = Field(95.0, ge=50.0, le=200.0)
    wrist_mm: float = Field(75.0, ge=40.0, le=160.0)
    max_accel_rad_s2: float = Field(12.0, ge=1.0, le=100.0)
    joint_angles_deg: list[float] = Field(
        default_factory=lambda: [35.0, -55.0, 20.0],
        min_length=3,
        max_length=3,
    )
    slots: ModularArmSlots = Field(default_factory=ModularArmSlots)
    auto_fix: bool = False
    mechatronics: Optional["MechatronicsConfig"] = None


class CompatibilityIssue(BaseModel):
    severity: Literal["ok", "warning", "critical"]
    slot: str
    parameter: str
    current_value: str
    suggested_value: str
    message: str


class PartPhysicsMetric(BaseModel):
    slot: str
    part_name: str
    variant: str
    mass_g: float
    volume_mm3: float
    safety_factor: float
    torque_nm: float
    status: Literal["ok", "warning", "critical"]


class ModularArmResponse(BaseModel):
    design_hash: str
    compatible: bool
    auto_fixed: bool
    auto_fix_notes: list[str] = Field(default_factory=list)
    compatibility_issues: list[CompatibilityIssue] = Field(default_factory=list)
    per_part_metrics: list[PartPhysicsMetric] = Field(default_factory=list)
    total_mass_g: float
    reach_mm: float
    shoulder_torque_nm: float
    min_safety_factor: float
    tcp_position_mm: list[float] = Field(default_factory=list)
    explode_vectors: dict[str, list[float]] = Field(default_factory=dict)
    files: dict[str, str] = Field(default_factory=dict)
    slots_used: ModularArmSlots
    joints: list[JointTorqueSummary] = Field(default_factory=list)
    bom: list[BOMItem] = Field(default_factory=list)
    mechatronics: Optional["MechatronicsResponse"] = None


# v4.0 + v4.1 + v5.0: Mecatrónica Universal — Motores, Servos Bus UART, Teensy DM542T, Curvas Torque–RPM y BOM Maestro
BusArchitectureType = Literal[
    "can_fd_bus",
    "stm32_step_dir",
    "arduino_pca9685_i2c",
    "uart_serial_bus",
    "teensy_dm542",
]


class JointActuatorSpec(BaseModel):
    motor_id: str = "nema17_mks_servo42d"
    reducer_id: str = "planetary_10"
    target_rpm: float = Field(30.0, ge=1.0, le=300.0)
    parallel_count: int = Field(default=1, ge=1, le=2)


class MechatronicsConfig(BaseModel):
    bus_architecture: BusArchitectureType = "can_fd_bus"
    system_voltage_v: float = Field(24.0, ge=5.0, le=60.0)
    joint_actuators: dict[str, JointActuatorSpec] = Field(default_factory=dict)


class TorqueSpeedPoint(BaseModel):
    rpm: float
    continuous_torque_nm: float
    peak_torque_nm: float


class JointMechatronicsReport(BaseModel):
    joint_id: str
    joint_name: str
    motor_id: str
    motor_name: str
    motor_type: Literal[
        "stepper",
        "closed_loop_stepper",
        "bldc_qdd",
        "rc_servo",
        "serial_bus_servo",
    ]
    parallel_count: int = 1
    reducer_id: str
    reducer_name: str
    gear_ratio: float
    efficiency: float
    operating_rpm: float
    required_rms_torque_nm: float
    required_peak_torque_nm: float
    available_continuous_torque_nm: float
    available_peak_torque_nm: float
    torque_margin_pct: float
    thermal_margin_pct: float
    continuous_current_a: float
    peak_current_a: float
    status: Literal["ok", "warning", "critical"]
    torque_speed_curve: list[TorqueSpeedPoint] = Field(default_factory=list)


class PowerBudget(BaseModel):
    bus_architecture: BusArchitectureType
    system_voltage_v: float
    continuous_current_a: float
    peak_current_a: float
    continuous_power_w: float
    peak_power_w: float
    recommended_psu: str
    psu_cost_usd: float
    power_wire_awg: str
    signal_wire_awg: str
    total_harness_length_m: float
    fuse_rating_a: float


class MasterBomItem(BaseModel):
    category: Literal[
        "printed_cnc",
        "actuators_reducers",
        "electronics_wiring",
        "fasteners_hardware",
    ]
    category_label: str
    part_code: str
    part_name: str
    spec: str
    qty: int
    unit_cost_usd: float
    total_cost_usd: float


class MechatronicsResponse(BaseModel):
    bus_architecture: BusArchitectureType
    joint_reports: list[JointMechatronicsReport] = Field(default_factory=list)
    power_budget: PowerBudget
    master_bom: list[MasterBomItem] = Field(default_factory=list)
    total_bom_cost_usd: float
    notes: list[str] = Field(default_factory=list)


ModularArmRequest.model_rebuild()
ModularArmResponse.model_rebuild()
