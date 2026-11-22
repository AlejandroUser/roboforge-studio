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


class MountVariantResult(BaseModel):
    variant_id: MountVariantKind
    name: str
    tagline: str
    is_valid_solid: bool
    metrics: VariantMetrics
    glb_url: str
    step_url: str
    stl_url: str
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
