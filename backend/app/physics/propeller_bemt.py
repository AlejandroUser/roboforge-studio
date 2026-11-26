"""Blade Element Momentum Theory (BEMT) & Root Structural Solver for Drone Propellers.

Implements:
- Embedded low-Reynolds airfoil geometry & polars (NACA 4412, Clark-Y, Eppler E63)
  with guaranteed printable trailing-edge thickness.
- Radial chord, twist/pitch, sweep, and rake station distributions.
- Iterative BEMT solver with Prandtl tip/hub-loss corrections for hover & axial climb.
- Toroidal loop aerodynamic & acoustic correction model (MIT Lincoln Lab inspired).
- Centrifugal + aerodynamic flapwise bending root stress analysis for FDM/MSLA/PA-CF blades.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

AIR_DENSITY_KG_M3 = 1.225  # Sea-level ISA
SPEED_OF_SOUND_MS = 340.3  # Sea-level ISA


@dataclass(frozen=True)
class AirfoilSpec:
    id: str
    name: str
    camber_m: float      # Max camber as fraction of chord
    camber_p: float      # Position of max camber (0..1)
    thickness_t: float   # Max thickness as fraction of chord
    flat_bottom: bool    # True for Clark-Y style flat lower surface
    cl0: float           # Zero-angle lift coefficient
    cl_alpha: float      # Lift curve slope (1/rad)
    cl_max: float        # Pre-stall max lift coefficient
    cd0: float           # Zero-lift profile drag at low Re (~5e4 - 1.5e5)
    cd_k: float          # Induced profile drag quadratic coefficient


AIRFOIL_LIBRARY: dict[str, AirfoilSpec] = {
    "naca4412": AirfoilSpec(
        id="naca4412",
        name="NACA 4412 (Estándar FPV / Multirrotor)",
        camber_m=0.04,
        camber_p=0.40,
        thickness_t=0.12,
        flat_bottom=False,
        cl0=0.42,
        cl_alpha=5.55,
        cl_max=1.38,
        cd0=0.018,
        cd_k=0.044,
    ),
    "clark_y": AirfoilSpec(
        id="clark_y",
        name="Clark-Y (Alto empuje · Base plana imprimible)",
        camber_m=0.035,
        camber_p=0.35,
        thickness_t=0.117,
        flat_bottom=True,
        cl0=0.36,
        cl_alpha=5.60,
        cl_max=1.35,
        cd0=0.016,
        cd_k=0.042,
    ),
    "eppler_e63": AirfoilSpec(
        id="eppler_e63",
        name="Eppler E63 (Alta eficiencia · Long-Range)",
        camber_m=0.052,
        camber_p=0.42,
        thickness_t=0.092,
        flat_bottom=False,
        cl0=0.52,
        cl_alpha=5.45,
        cl_max=1.44,
        cd0=0.015,
        cd_k=0.039,
    ),
}


@dataclass
class BladeStation:
    r_frac: float          # Normalized radius r/R
    r_mm: float            # Physical radius (mm)
    chord_mm: float        # Local chord length (mm)
    twist_deg: float       # Local geometric pitch angle beta (deg)
    thickness_mm: float    # Local max airfoil thickness (mm)
    sweep_mm: float        # In-plane sweep offset (mm)
    rake_mm: float         # Out-of-plane axial rake offset (mm)
    inflow_deg: float = 0.0
    alpha_deg: float = 0.0
    cl: float = 0.0
    cd: float = 0.0
    dT_N: float = 0.0
    dQ_Nm: float = 0.0


@dataclass
class PropellerBemtResult:
    diameter_inch: float
    pitch_inch: float
    blade_count: int
    airfoil_id: str
    prop_style: str
    rpm: float
    thrust_n: float
    thrust_g: float
    torque_nm: float
    power_w: float
    figure_of_merit: float
    efficiency_g_per_w: float
    tip_speed_ms: float
    tip_mach: float
    blade_mass_g: float
    total_prop_mass_g: float
    centrifugal_force_n: float
    root_bending_nm: float
    root_stress_mpa: float
    root_safety_factor: float
    estimated_noise_dba: float
    warnings: list[str] = field(default_factory=list)
    stations: list[BladeStation] = field(default_factory=list)

    def to_metrics_dict(self) -> dict[str, Any]:
        return {
            "diameter_inch": round(self.diameter_inch, 2),
            "pitch_inch": round(self.pitch_inch, 2),
            "blade_count": self.blade_count,
            "airfoil_id": self.airfoil_id,
            "prop_style": self.prop_style,
            "rpm": round(self.rpm, 0),
            "thrust_g": round(self.thrust_g, 1),
            "thrust_n": round(self.thrust_n, 2),
            "torque_nm": round(self.torque_nm, 4),
            "power_w": round(self.power_w, 1),
            "figure_of_merit": round(self.figure_of_merit, 3),
            "efficiency_g_per_w": round(self.efficiency_g_per_w, 2),
            "tip_speed_ms": round(self.tip_speed_ms, 1),
            "tip_mach": round(self.tip_mach, 3),
            "total_prop_mass_g": round(self.total_prop_mass_g, 2),
            "centrifugal_force_n": round(self.centrifugal_force_n, 1),
            "root_stress_mpa": round(self.root_stress_mpa, 2),
            "root_safety_factor": round(self.root_safety_factor, 2),
            "estimated_noise_dba": round(self.estimated_noise_dba, 1),
        }


def sample_airfoil_coords(
    airfoil_id: str = "naca4412",
    n_per_surface: int = 16,
    min_te_ratio: float = 0.032,
) -> list[tuple[float, float]]:
    """Return closed 2D airfoil coordinates normalized to chord=1, pivoted at 25% chord.

    Guarantees a finite trailing-edge thickness (`min_te_ratio * chord`) so 3D-printed
    and CNC-milled blades have no degenerate zero-thickness knife edges.
    Coordinates are ordered counter-clockwise starting from trailing edge top -> LE -> TE bottom.
    """
    spec = AIRFOIL_LIBRARY.get(airfoil_id, AIRFOIL_LIBRARY["naca4412"])
    m = spec.camber_m
    p = max(0.05, min(0.95, spec.camber_p))
    t = spec.thickness_t

    # Cosine clustering concentrates points near leading and trailing edges
    xs = [0.5 * (1.0 - math.cos(math.pi * i / n_per_surface)) for i in range(n_per_surface + 1)]

    upper: list[tuple[float, float]] = []
    lower: list[tuple[float, float]] = []

    for x in xs:
        # Classic NACA thickness distribution with finite TE thickness coefficient
        yt = 5.0 * t * (
            0.2969 * math.sqrt(max(x, 0.0))
            - 0.1260 * x
            - 0.3516 * x * x
            + 0.2843 * x * x * x
            - 0.1015 * x * x * x * x
        )
        # Ensure printable trailing edge thickness ramp near x -> 1
        yt = max(yt, 0.5 * min_te_ratio * (x ** 1.5))

        if x < p:
            yc = (m / (p * p)) * (2.0 * p * x - x * x)
            dyc_dx = (2.0 * m / (p * p)) * (p - x)
        else:
            yc = (m / ((1.0 - p) ** 2)) * ((1.0 - 2.0 * p) + 2.0 * p * x - x * x)
            dyc_dx = (2.0 * m / ((1.0 - p) ** 2)) * (p - x)

        theta = math.atan(dyc_dx)
        xu = x - yt * math.sin(theta)
        yu = yc + yt * math.cos(theta)
        xl = x + yt * math.sin(theta)
        yl = yc - yt * math.cos(theta)

        if spec.flat_bottom and x > 0.12:
            # Blend lower surface toward a flat bottom for Clark-Y character
            yl = max(yl, -0.25 * t * (1.0 - 0.65 * x))

        # Pivot at aerodynamic center (0.25 chord)
        upper.append((xu - 0.25, yu))
        lower.append((xl - 0.25, yl))

    # Build closed CCW loop: TE_top -> LE -> TE_bottom (avoiding duplicate LE point)
    pts = list(reversed(upper)) + lower[1:]
    return pts


def compute_blade_stations(
    diameter_inch: float,
    pitch_inch: float,
    blade_count: int = 3,
    airfoil_id: str = "naca4412",
    hub_radius_mm: float = 6.5,
    n_stations: int = 10,
    chord_scale: float = 1.0,
    sweep_deg: float = 0.0,
    rake_deg: float = 0.0,
) -> list[BladeStation]:
    """Generate radial aerodynamic stations (r, chord, twist, thickness, sweep, rake)."""
    spec = AIRFOIL_LIBRARY.get(airfoil_id, AIRFOIL_LIBRARY["naca4412"])
    radius_mm = (diameter_inch * 25.4) / 2.0
    pitch_mm = pitch_inch * 25.4
    r_start = max(hub_radius_mm * 1.15, radius_mm * 0.16)
    r_end = radius_mm * 0.985

    # Max chord scales inversely with blade count to keep reasonable rotor solidity
    base_max_chord = radius_mm * 0.25 * ((2.5 / max(blade_count, 2)) ** 0.35) * chord_scale

    stations: list[BladeStation] = []
    for i in range(n_stations):
        u = i / max(n_stations - 1, 1)  # 0 at root, 1 at tip
        r_mm = r_start + u * (r_end - r_start)
        r_frac = r_mm / radius_mm

        # Smooth multirotor planform: narrow root -> peak at ~45% span -> rounded tapered tip
        planform = math.sin(math.pi * (0.14 + 0.78 * u)) ** 0.78
        planform = max(0.32, planform)
        chord_mm = max(4.0, base_max_chord * planform)

        # Geometric pitch angle beta(r) = atan(Pitch / (2*pi*r))
        beta_deg = math.degrees(math.atan2(pitch_mm, 2.0 * math.pi * r_mm))
        beta_deg = max(7.0, min(58.0, beta_deg))

        # Root reinforcement: thicken airfoil near hub for centrifugal + flapwise bending strength
        t_factor = 1.0 + 0.85 * ((1.0 - u) ** 2.0)
        thickness_mm = max(1.1, chord_mm * spec.thickness_t * t_factor)

        # Optional sweep (in-plane curve) and rake (axial coning)
        span_from_root = r_mm - r_start
        sweep_mm = span_from_root * math.tan(math.radians(sweep_deg)) * (u ** 1.3)
        rake_mm = span_from_root * math.tan(math.radians(rake_deg))

        stations.append(
            BladeStation(
                r_frac=r_frac,
                r_mm=r_mm,
                chord_mm=chord_mm,
                twist_deg=beta_deg,
                thickness_mm=thickness_mm,
                sweep_mm=sweep_mm,
                rake_mm=rake_mm,
            )
        )
    return stations


def solve_propeller_bemt(
    diameter_inch: float = 5.0,
    pitch_inch: float = 4.3,
    blade_count: int = 3,
    rpm: float = 18000.0,
    airfoil_id: str = "naca4412",
    prop_style: str = "conventional",
    v_axial_ms: float = 0.0,
    hub_radius_mm: float = 6.5,
    chord_scale: float = 1.0,
    material_density_g_cm3: float = 1.25,
    material_yield_z_mpa: float = 38.0,
    n_stations: int = 14,
) -> PropellerBemtResult:
    """Solve propeller hover/climb thrust, torque, power, FM, and root structural stress via BEMT."""
    spec = AIRFOIL_LIBRARY.get(airfoil_id, AIRFOIL_LIBRARY["naca4412"])
    style = "toroidal" if str(prop_style).strip().lower() == "toroidal" else "conventional"

    radius_m = (diameter_inch * 0.0254) / 2.0
    omega = max(rpm, 500.0) * 2.0 * math.pi / 60.0
    v_tip = omega * radius_m
    tip_mach = v_tip / SPEED_OF_SOUND_MS

    stations = compute_blade_stations(
        diameter_inch=diameter_inch,
        pitch_inch=pitch_inch,
        blade_count=blade_count,
        airfoil_id=airfoil_id,
        hub_radius_mm=hub_radius_mm,
        n_stations=n_stations,
        chord_scale=chord_scale,
    )

    # Zero-lift angle of attack (rad)
    alpha0 = -spec.cl0 / spec.cl_alpha
    lambda_c = max(0.0, v_axial_ms) / max(v_tip, 1e-6)
    rho = AIR_DENSITY_KG_M3

    # Toroidal blades behave as closed dual-arm loops with reduced tip-vortex losses
    # and higher effective solidity / wetted drag area
    eff_blades = blade_count * (1.35 if style == "toroidal" else 1.0)
    tip_loss_bonus = 1.06 if style == "toroidal" else 1.0
    profile_drag_mult = 1.12 if style == "toroidal" else 1.0

    total_thrust_n = 0.0
    total_torque_nm = 0.0
    blade_mass_kg = 0.0
    centrifugal_n = 0.0
    root_bending_nm = 0.0

    rho_mat = material_density_g_cm3 * 1000.0  # kg/m^3
    r_hub_m = hub_radius_mm / 1000.0

    dr_m = (stations[-1].r_mm - stations[0].r_mm) / 1000.0 / max(len(stations) - 1, 1)

    for st in stations:
        r_m = st.r_mm / 1000.0
        r_hat = max(0.05, min(0.995, st.r_frac))
        c_m = st.chord_mm / 1000.0
        beta = math.radians(st.twist_deg)
        sigma_local = (eff_blades * c_m) / (math.pi * radius_m)

        # Iterative inflow ratio lambda(r) with Prandtl tip + hub loss
        F = 0.92
        lam = max(0.04, 0.5 * lambda_c + 0.08)
        for _ in range(18):
            term_a = (sigma_local * spec.cl_alpha) / (16.0 * max(F, 0.15)) - 0.5 * lambda_c
            radicand = max(0.0, term_a * term_a + (sigma_local * spec.cl_alpha / (8.0 * max(F, 0.15))) * max(beta - alpha0, 0.01) * r_hat)
            lam_new = math.sqrt(radicand) - term_a
            lam = 0.65 * max(lam_new, 1e-4) + 0.35 * lam
            phi = math.atan2(lam, r_hat)
            sin_phi = max(math.sin(phi), 0.04)

            f_tip = 0.5 * eff_blades * (1.0 - r_hat) / (r_hat * sin_phi)
            f_hub = 0.5 * eff_blades * max(r_hat - (r_hub_m / radius_m), 0.01) / (r_hat * sin_phi)
            F_tip = (2.0 / math.pi) * math.acos(min(1.0, max(0.0, math.exp(-min(f_tip, 25.0)))))
            F_hub = (2.0 / math.pi) * math.acos(min(1.0, max(0.0, math.exp(-min(f_hub, 25.0)))))
            F = min(1.0, max(0.18, F_tip * F_hub * tip_loss_bonus))

        phi = math.atan2(lam, r_hat)
        alpha = beta - phi

        # Lift & drag with smooth post-stall model
        cl_lin = spec.cl0 + spec.cl_alpha * alpha
        if cl_lin > spec.cl_max:
            overshoot = cl_lin - spec.cl_max
            cl = max(0.45, spec.cl_max - 0.65 * overshoot)
            cd = (spec.cd0 + spec.cd_k * (spec.cl_max ** 2) + 0.85 * (overshoot ** 1.5)) * profile_drag_mult
        elif cl_lin < -0.5:
            cl = -0.5
            cd = (spec.cd0 + 0.25) * profile_drag_mult
        else:
            cl = cl_lin
            cd = (spec.cd0 + spec.cd_k * (cl * cl)) * profile_drag_mult

        # Local resultant velocity W
        v_t = omega * r_m
        v_a = lam * v_tip
        w_sq = v_t * v_t + v_a * v_a

        # Elemental thrust & torque across all blades
        dL = 0.5 * rho * w_sq * c_m * cl * dr_m
        dD = 0.5 * rho * w_sq * c_m * cd * dr_m

        dT_blade = (dL * math.cos(phi) - dD * math.sin(phi)) * F
        dQ_blade = (dL * math.sin(phi) + dD * math.cos(phi)) * r_m

        dT_all = max(0.0, eff_blades * dT_blade)
        dQ_all = max(0.0, eff_blades * dQ_blade)

        total_thrust_n += dT_all
        total_torque_nm += dQ_all

        st.inflow_deg = round(math.degrees(phi), 2)
        st.alpha_deg = round(math.degrees(alpha), 2)
        st.cl = round(cl, 3)
        st.cd = round(cd, 4)
        st.dT_N = round(dT_all, 4)
        st.dQ_Nm = round(dQ_all, 5)

        # Structural integration per blade (airfoil area ~ 0.68 * chord * thickness)
        area_m2 = 0.68 * c_m * (st.thickness_mm / 1000.0)
        if style == "toroidal":
            area_m2 *= 1.55  # closed loop has two load-bearing arms
        dm_kg = rho_mat * area_m2 * dr_m
        blade_mass_kg += dm_kg
        centrifugal_n += dm_kg * (omega * omega) * r_m
        root_bending_nm += max(0.0, dT_blade) * max(0.0, r_m - r_hub_m)

    power_w = max(0.1, total_torque_nm * omega)
    disk_area_m2 = math.pi * (radius_m ** 2)
    ideal_power_w = (max(total_thrust_n, 0.0) ** 1.5) / math.sqrt(2.0 * rho * disk_area_m2)
    figure_of_merit = min(0.92, ideal_power_w / power_w) if power_w > 0.1 else 0.0
    thrust_g = total_thrust_n / 9.80665 * 1000.0
    eff_g_per_w = thrust_g / power_w

    # Root cross-section stress (axial centrifugal + flapwise bending)
    root_st = stations[0]
    root_c_m = root_st.chord_mm / 1000.0
    root_t_m = root_st.thickness_mm / 1000.0
    root_area_m2 = max(1e-7, 0.68 * root_c_m * root_t_m * (1.8 if style == "toroidal" else 1.0))
    # Section modulus Z ≈ 0.075 * c * t^2 for airfoil in flapwise bending
    root_z_m3 = max(1e-10, 0.075 * root_c_m * (root_t_m ** 2) * (2.1 if style == "toroidal" else 1.0))

    sigma_cent_mpa = (centrifugal_n / root_area_m2) / 1e6
    sigma_bend_mpa = (root_bending_nm / root_z_m3) / 1e6
    root_stress_mpa = sigma_cent_mpa + sigma_bend_mpa
    root_sf = min(25.0, material_yield_z_mpa / max(root_stress_mpa, 0.05))

    # Hub mass approximation
    hub_mass_kg = rho_mat * math.pi * ((r_hub_m ** 2) - (0.0025 ** 2)) * 0.0065
    total_prop_mass_g = (blade_count * blade_mass_kg + max(0.0, hub_mass_kg)) * 1000.0

    # Empirical acoustic estimate (dBA at 1 m) based on tip speed and blade count
    base_noise = 52.0 + 34.0 * math.log10(max(v_tip / 30.0, 0.35)) + 3.0 * math.log10(max(blade_count, 2) / 2.0)
    if style == "toroidal":
        base_noise -= 6.5
    estimated_noise_dba = max(42.0, min(105.0, base_noise))

    warnings: list[str] = []
    if tip_mach > 0.72:
        warnings.append(
            f"Velocidad de punta elevada (Mach {tip_mach:.2f}, {v_tip:.0f} m/s): riesgo de pérdidas por compresibilidad y ruido extremo."
        )
    if root_sf < 1.5:
        warnings.append(
            f"Factor de seguridad en raíz bajo (FS={root_sf:.2f} a {rpm:.0f} RPM, σ={root_stress_mpa:.1f} MPa). Aumenta cuerda/espesor o usa PA-CF/Resina técnica."
        )
    elif root_sf < 2.2:
        warnings.append(
            f"Margen estructural moderado en raíz (FS={root_sf:.2f}). Recomendado imprimir con 100% relleno y orientación horizontal."
        )

    return PropellerBemtResult(
        diameter_inch=diameter_inch,
        pitch_inch=pitch_inch,
        blade_count=blade_count,
        airfoil_id=spec.id,
        prop_style=style,
        rpm=rpm,
        thrust_n=total_thrust_n,
        thrust_g=thrust_g,
        torque_nm=total_torque_nm,
        power_w=power_w,
        figure_of_merit=figure_of_merit,
        efficiency_g_per_w=eff_g_per_w,
        tip_speed_ms=v_tip,
        tip_mach=tip_mach,
        blade_mass_g=blade_mass_kg * 1000.0,
        total_prop_mass_g=total_prop_mass_g,
        centrifugal_force_n=centrifugal_n,
        root_bending_nm=root_bending_nm,
        root_stress_mpa=root_stress_mpa,
        root_safety_factor=root_sf,
        estimated_noise_dba=estimated_noise_dba,
        warnings=warnings,
        stations=stations,
    )
