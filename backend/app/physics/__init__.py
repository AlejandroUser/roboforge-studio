from .arm_compatibility import (
    auto_match_interfaces,
    evaluate_modular_arm,
    resolve_default_slots,
    validate_arm_compatibility,
)
from .arm_kinematics import evaluate_arm_assembly, load_arm_presets
from .copilot import run_closed_loop_copilot
from .dfm_rules import evaluate_global_alerts
from .fea_solver import run_gear_fea, run_mount_fea, run_step_fea
from .firmware_gen import generate_arduino_pca9685_ino
from .gear_mechanics import (
    compute_gear_diameters,
    evaluate_gear_global_alerts,
    evaluate_gear_metrics,
    load_gear_presets,
)
from .mechanics import evaluate_variant_metrics, load_materials, load_presets
from .mechatronics import (
    ACTUATOR_CATALOG,
    REDUCER_CATALOG,
    evaluate_mechatronics,
    generate_circuit_schematic_svg,
    generate_lerobot_config_json,
    generate_wireviz_yaml,
    get_mechatronics_catalog,
    master_bom_to_csv,
    solve_belt_center_distance,
)
from .mount_mechanics import (
    evaluate_mount_global_alerts,
    evaluate_mount_metrics,
    load_mount_presets,
)
from .profiles import (
    delete_custom_material,
    delete_custom_printer,
    evaluate_printer_dfm_alerts,
    load_printers,
    save_custom_material,
    save_custom_printer,
)
from .topopt import extract_organic_void_polygons, run_simp_cantilever

__all__ = [
    "load_materials",
    "load_presets",
    "load_mount_presets",
    "load_gear_presets",
    "load_arm_presets",
    "load_printers",
    "save_custom_printer",
    "delete_custom_printer",
    "save_custom_material",
    "delete_custom_material",
    "compute_gear_diameters",
    "evaluate_variant_metrics",
    "evaluate_mount_metrics",
    "evaluate_gear_metrics",
    "evaluate_arm_assembly",
    "resolve_default_slots",
    "validate_arm_compatibility",
    "auto_match_interfaces",
    "evaluate_modular_arm",
    "evaluate_global_alerts",
    "evaluate_mount_global_alerts",
    "evaluate_gear_global_alerts",
    "evaluate_printer_dfm_alerts",
    "run_closed_loop_copilot",
    "run_simp_cantilever",
    "extract_organic_void_polygons",
    "run_step_fea",
    "run_mount_fea",
    "run_gear_fea",
    "ACTUATOR_CATALOG",
    "REDUCER_CATALOG",
    "get_mechatronics_catalog",
    "evaluate_mechatronics",
    "generate_circuit_schematic_svg",
    "generate_wireviz_yaml",
    "master_bom_to_csv",
    "solve_belt_center_distance",
    "generate_lerobot_config_json",
    "generate_arduino_pca9685_ino",
]
