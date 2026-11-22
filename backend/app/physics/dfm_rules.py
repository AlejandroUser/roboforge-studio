from __future__ import annotations

from ..schemas import AlertItem, EndInterfaceParams, LinkDesignRequest
from .profiles import evaluate_printer_dfm_alerts

CF_MATERIALS = {"petg_cf", "pa12_cf"}


def _check_end_interface(label: str, end: EndInterfaceParams) -> list[AlertItem]:
    alerts: list[AlertItem] = []
    radial_wall_mm = (end.outer_diameter_mm - end.bore_diameter_mm) / 2.0
    if radial_wall_mm < 2.0:
        alerts.append(
            AlertItem(
                level="critical",
                code="END_WALL_TOO_THIN",
                message=(
                    f"{label} boss radial wall ({radial_wall_mm:.1f} mm) is below 2.0 mm "
                    "and risks cracking under bearing or motor mounting loads."
                ),
            )
        )

    if end.bolt_count > 0:
        min_pcd = end.bore_diameter_mm + 4.0
        max_pcd = end.outer_diameter_mm - 4.0
        if end.bolt_pcd_mm < min_pcd or end.bolt_pcd_mm > max_pcd:
            alerts.append(
                AlertItem(
                    level="warning",
                    code="BOLT_PCD_CLEARANCE",
                    message=(
                        f"{label} bolt PCD ({end.bolt_pcd_mm:.1f} mm) leaves insufficient "
                        "edge distance relative to bore or outer boss diameter."
                    ),
                )
            )
    return alerts


def evaluate_global_alerts(req: LinkDesignRequest, material: dict) -> list[AlertItem]:
    """
    Evaluate global DFM, thermal, print-orientation, nozzle, and geometry sanity alerts
    that apply across all structural variants for the given design request and material.
    """
    alerts: list[AlertItem] = []

    max_temp_c = float(material["max_temp_c"])
    mat_name = str(material["name"])
    mat_id = str(material["id"])
    category = str(material.get("category", "fdm"))

    # 1. Thermal & motor heat alerts
    if req.operating_temp_c >= max_temp_c:
        alerts.append(
            AlertItem(
                level="critical",
                code="THERMAL_LIMIT_EXCEEDED",
                message=(
                    f"Operating temperature ({req.operating_temp_c:.0f}°C) meets or exceeds "
                    f"{mat_name} heat deflection limit ({max_temp_c:.0f}°C). "
                    "Switch to PETG-CF, ABS/ASA, PA12-CF, or Aluminum."
                ),
            )
        )
    elif req.operating_temp_c >= max_temp_c - 10.0:
        alerts.append(
            AlertItem(
                level="warning",
                code="THERMAL_MARGIN_LOW",
                message=(
                    f"Operating temperature ({req.operating_temp_c:.0f}°C) is within 10°C of "
                    f"{mat_name} thermal limit ({max_temp_c:.0f}°C); creep may occur near motors."
                ),
            )
        )
    elif (
        req.root_end.preset in ("nema17", "nema23")
        and max_temp_c <= 60.0
        and req.operating_temp_c >= 45.0
    ):
        alerts.append(
            AlertItem(
                level="warning",
                code="MOTOR_THERMAL_RISK",
                message=(
                    f"Stepper motor flange ({req.root_end.preset.upper()}) can reach 55–65°C "
                    f"during sustained holding torque, risking softening in {mat_name}."
                ),
            )
        )

    # 2. FDM print orientation & anisotropy alerts
    if category == "fdm" and req.print_orientation == "z_vertical":
        yield_xy = float(material["yield_xy_mpa"])
        yield_z = float(material["yield_z_mpa"])
        loss_pct = (1.0 - yield_z / max(yield_xy, 1e-6)) * 100.0
        alerts.append(
            AlertItem(
                level="warning",
                code="Z_ORIENTATION_WEAKNESS",
                message=(
                    f"Vertical Z-axis printing aligns bending stress across layer lines, "
                    f"reducing {mat_name} effective yield by ~{loss_pct:.0f}%. "
                    "Use 'xy_flat' for maximum bending strength."
                ),
            )
        )

    # 3. Nozzle & wall thickness DFM alerts (FDM)
    if category == "fdm":
        if mat_id in CF_MATERIALS and req.nozzle_diameter_mm < 0.4:
            alerts.append(
                AlertItem(
                    level="warning",
                    code="CF_NOZZLE_CLOG_RISK",
                    message=(
                        f"{mat_name} contains chopped carbon fibers and requires at least a "
                        "0.4 mm hardened steel nozzle to prevent clogging."
                    ),
                )
            )

        min_recommended_thickness = req.nozzle_diameter_mm * 12.0
        if req.thickness_mm < min_recommended_thickness:
            alerts.append(
                AlertItem(
                    level="warning",
                    code="THIN_SECTION_FOR_NOZZLE",
                    message=(
                        f"Link thickness ({req.thickness_mm:.1f} mm) is thin for a "
                        f"{req.nozzle_diameter_mm:.2f} mm nozzle (recommend >= "
                        f"{min_recommended_thickness:.1f} mm for robust perimeters)."
                    ),
                )
            )

        if req.printer_clearance_mm < 0.1:
            alerts.append(
                AlertItem(
                    level="info",
                    code="TIGHT_PRINTER_CLEARANCE",
                    message=(
                        f"Printer clearance ({req.printer_clearance_mm:.2f} mm) is below 0.10 mm; "
                        "bearings and heat-set inserts may require post-reaming."
                    ),
                )
            )

    # 4. Geometry sanity checks
    if req.tip_width_mm > req.root_width_mm:
        alerts.append(
            AlertItem(
                level="warning",
                code="INVERSE_TAPER",
                message=(
                    f"Tip width ({req.tip_width_mm:.1f} mm) exceeds root width "
                    f"({req.root_width_mm:.1f} mm); cantilever links should be wider at the root."
                ),
            )
        )

    slenderness = req.length_mm / max(req.root_width_mm, 1.0)
    if slenderness > 10.0:
        alerts.append(
            AlertItem(
                level="warning",
                code="HIGH_SLENDERNESS_RATIO",
                message=(
                    f"High link slenderness ratio (L/h = {slenderness:.1f} > 10) increases "
                    "tip deflection and lateral buckling sensitivity."
                ),
            )
        )

    if req.cable_channel and req.thickness_mm < 12.0:
        alerts.append(
            AlertItem(
                level="warning",
                code="CABLE_CHANNEL_THIN_LINK",
                message=(
                    f"Internal cable channel in a {req.thickness_mm:.1f} mm thick link "
                    "leaves thin web walls; increase thickness to >= 12 mm or disable channel."
                ),
            )
        )

    alerts.extend(_check_end_interface("Root", req.root_end))
    alerts.extend(_check_end_interface("Tip", req.tip_end))

    # 5. Printer build volume & enclosure compatibility
    part_x_mm = req.length_mm + 0.5 * (
        req.root_end.outer_diameter_mm + req.tip_end.outer_diameter_mm
    )
    part_y_mm = max(
        req.root_width_mm,
        req.tip_width_mm,
        req.root_end.outer_diameter_mm,
        req.tip_end.outer_diameter_mm,
    )
    part_z_mm = req.thickness_mm
    if req.print_orientation == "z_vertical":
        part_x_mm, part_z_mm = part_z_mm, part_x_mm
    alerts.extend(
        evaluate_printer_dfm_alerts(
            req.printer_id, material, part_x_mm, part_y_mm, part_z_mm
        )
    )

    return alerts
