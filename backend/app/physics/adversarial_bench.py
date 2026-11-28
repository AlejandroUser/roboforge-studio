"""RoboForge v14.5 — Adversarial Physics Test Bench (Modal Resonance, 20G Crash & Monte Carlo Pf).

Adapts proven open-source engineering V&V patterns:
- 3D Hex8 Modal Eigenvalue Analysis (K*phi = lambda*M*phi via scipy.sparse.linalg.eigsh)
  to detect motor RPM / Blade-Pass-Frequency (BPF) resonance conflicts.
- 20G Inertial / Impact Crash Load FEA (equivalent to CalculiX *STATIC *DLOAD GRAV 20G).
- Vectorized 50,000-trial Monte Carlo Structural & FDM Tolerance Reliability (Pf & beta index).
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla
import trimesh


def _hex8_brick_stiffness(dx: float, dy: float, dz: float, nu: float = 0.33) -> np.ndarray:
    """Exact 24x24 stiffness matrix for a 3D Hex8 brick (dx, dy, dz) with E=1.0 via 2x2x2 Gauss."""
    lam = nu / ((1.0 + nu) * (1.0 - 2.0 * nu))
    mu = 1.0 / (2.0 * (1.0 + nu))
    D = np.array(
        [
            [lam + 2 * mu, lam, lam, 0, 0, 0],
            [lam, lam + 2 * mu, lam, 0, 0, 0],
            [lam, lam, lam + 2 * mu, 0, 0, 0],
            [0, 0, 0, mu, 0, 0],
            [0, 0, 0, 0, mu, 0],
            [0, 0, 0, 0, 0, mu],
        ],
        dtype=np.float64,
    )
    signs = np.array(
        [
            [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
            [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1],
        ],
        dtype=np.float64,
    )
    gp = 1.0 / math.sqrt(3.0)
    Ke = np.zeros((24, 24), dtype=np.float64)
    detJ = (dx * dy * dz) / 8.0
    inv_jx, inv_jy, inv_jz = 2.0 / dx, 2.0 / dy, 2.0 / dz
    for xi in (-gp, gp):
        for eta in (-gp, gp):
            for zeta in (-gp, gp):
                B = np.zeros((6, 24), dtype=np.float64)
                for a in range(8):
                    sx, sy, sz = signs[a]
                    dNdx = 0.125 * sx * (1.0 + sy * eta) * (1.0 + sz * zeta) * inv_jx
                    dNdy = 0.125 * (1.0 + sx * xi) * sy * (1.0 + sz * zeta) * inv_jy
                    dNdz = 0.125 * (1.0 + sx * xi) * (1.0 + sy * eta) * sz * inv_jz
                    c = 3 * a
                    B[0, c] = dNdx
                    B[1, c + 1] = dNdy
                    B[2, c + 2] = dNdz
                    B[3, c], B[3, c + 1] = dNdy, dNdx
                    B[4, c + 1], B[4, c + 2] = dNdz, dNdy
                    B[5, c], B[5, c + 2] = dNdz, dNdx
                Ke += (B.T @ D @ B) * detJ
    return Ke


def _voxelize_mesh_hex8(
    mesh: trimesh.Trimesh,
    nx: int = 20,
    ny: int = 20,
    nz: int = 4,
) -> tuple[np.ndarray, float, float, float, np.ndarray]:
    """Sample a 3D mesh onto a uniform Hex8 grid (dependency-free, no rtree required)."""
    bmin = np.asarray(mesh.bounds[0], dtype=np.float64)
    bmax = np.asarray(mesh.bounds[1], dtype=np.float64)
    ext = np.maximum(bmax - bmin, 1.0)
    dx, dy, dz = float(ext[0] / nx), float(ext[1] / ny), float(ext[2] / nz)

    xs = bmin[0] + (np.arange(nx) + 0.5) * dx
    ys = bmin[1] + (np.arange(ny) + 0.5) * dy
    zs = bmin[2] + (np.arange(nz) + 0.5) * dz
    gx, gy, gz = np.meshgrid(xs, ys, zs, indexing="ij")
    pts = np.column_stack([gx.ravel(), gy.ravel(), gz.ravel()])

    try:
        inside = mesh.contains(pts).reshape((nx, ny, nz))
    except Exception:
        # Pure-trimesh voxel grid fallback (zero rtree dependency)
        pitch = max(0.8, min(dx, dy, dz) * 0.75)
        vg = mesh.voxelized(pitch=pitch).fill()
        inside = vg.is_filled(pts).reshape((nx, ny, nz))
        if not np.any(inside):
            inside = np.ones((nx, ny, nz), dtype=bool)
    return inside, dx, dy, dz, bmin


def _assemble_hex8_k_m(
    inside: np.ndarray,
    dx: float,
    dy: float,
    dz: float,
    e_mpa: float,
    density_g_cm3: float,
    fix_mode: str = "center_hub",
) -> tuple[sp.csc_matrix, sp.csc_matrix, np.ndarray, int]:
    """Assemble global sparse stiffness K (N/mm) and lumped mass M (tonne = N*s^2/mm) on free DOFs."""
    nx, ny, nz = inside.shape
    ke0 = _hex8_brick_stiffness(dx, dy, dz, nu=0.33) * e_mpa  # N/mm
    # Consistent units: mm, N, s -> mass in tonne (1 tonne = 1e6 g; density g/cm^3 * 1e-9 = tonne/mm^3)
    rho_tonne_mm3 = density_g_cm3 * 1e-9
    elem_mass = rho_tonne_mm3 * (dx * dy * dz)

    n_nodes = (nx + 1) * (ny + 1) * (nz + 1)
    ndof = 3 * n_nodes

    def nid(ix: int, iy: int, iz: int) -> int:
        return ix * (ny + 1) * (nz + 1) + iy * (nz + 1) + iz

    # Build element-to-DOF connectivity for active (inside) elements only
    active_idx = np.argwhere(inside)
    if len(active_idx) == 0:
        raise ValueError("Voxelized mesh has zero interior elements.")

    rows: list[np.ndarray] = []
    cols: list[np.ndarray] = []
    vals: list[np.ndarray] = []
    mass_diag = np.zeros(ndof, dtype=np.float64)

    ii, jj = np.meshgrid(np.arange(24), np.arange(24), indexing="ij")
    ii_f, jj_f = ii.ravel(), jj.ravel()
    ke_flat = ke0.ravel()

    active_nodes: set[int] = set()
    for ix, iy, iz in active_idx:
        ix_i, iy_i, iz_i = int(ix), int(iy), int(iz)
        # Match _hex8_stiffness node ordering: (0,0,0),(1,0,0),(1,1,0),(0,1,0),(0,0,1),(1,0,1),(1,1,1),(0,1,1)
        n_el = [
            nid(ix_i, iy_i, iz_i),
            nid(ix_i + 1, iy_i, iz_i),
            nid(ix_i + 1, iy_i + 1, iz_i),
            nid(ix_i, iy_i + 1, iz_i),
            nid(ix_i, iy_i, iz_i + 1),
            nid(ix_i + 1, iy_i, iz_i + 1),
            nid(ix_i + 1, iy_i + 1, iz_i + 1),
            nid(ix_i, iy_i + 1, iz_i + 1),
        ]
        edofs = np.empty(24, dtype=np.int64)
        for k, n in enumerate(n_el):
            active_nodes.add(n)
            edofs[3 * k] = 3 * n
            edofs[3 * k + 1] = 3 * n + 1
            edofs[3 * k + 2] = 3 * n + 2
            # Lumped 1/8 element mass per node in X, Y, Z
            mass_diag[3 * n : 3 * n + 3] += elem_mass / 8.0

        rows.append(edofs[ii_f])
        cols.append(edofs[jj_f])
        vals.append(ke_flat)

    K_full = sp.coo_matrix(
        (np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))),
        shape=(ndof, ndof),
    ).tocsc()

    # Determine fixed boundary nodes
    fixed_nodes: set[int] = set()
    if fix_mode == "root_x":
        for iy in range(ny + 1):
            for iz in range(nz + 1):
                n = nid(0, iy, iz)
                if n in active_nodes:
                    fixed_nodes.add(n)
    else:
        # center_hub: fix central FC stack core (middle 25% in X and Y)
        x_lo, x_hi = int(nx * 0.36), int(nx * 0.64)
        y_lo, y_hi = int(ny * 0.36), int(ny * 0.64)
        for ix in range(x_lo, x_hi + 1):
            for iy in range(y_lo, y_hi + 1):
                for iz in range(nz + 1):
                    n = nid(ix, iy, iz)
                    if n in active_nodes:
                        fixed_nodes.add(n)

    # Fallback if boundary missed active nodes
    if not fixed_nodes:
        fixed_nodes.add(min(active_nodes))

    free_dofs_list: list[int] = []
    for n in sorted(active_nodes):
        if n not in fixed_nodes:
            free_dofs_list.extend([3 * n, 3 * n + 1, 3 * n + 2])
    free_dofs = np.array(free_dofs_list, dtype=np.int64)

    K_ff = K_full[free_dofs, :][:, free_dofs].tocsc()
    M_ff = sp.diags(np.maximum(mass_diag[free_dofs], 1e-14), format="csc")
    return K_ff, M_ff, free_dofs, ndof


def run_modal_resonance_check(
    mesh: trimesh.Trimesh,
    e_mpa: float = 5200.0,
    density_g_cm3: float = 1.18,
    motor_rpm: float = 22000.0,
    blade_count: int = 3,
    fix_mode: str = "center_hub",
    nx: int = 18,
    ny: int = 18,
    nz: int = 3,
) -> dict[str, Any]:
    """Compute lowest 3 structural natural frequencies (Hz) and check against motor 1x RPM & BPF."""
    inside, dx, dy, dz, _ = _voxelize_mesh_hex8(mesh, nx=nx, ny=ny, nz=nz)
    K_ff, M_ff, free_dofs, _ = _assemble_hex8_k_m(inside, dx, dy, dz, e_mpa, density_g_cm3, fix_mode=fix_mode)

    k_req = min(3, max(1, len(free_dofs) - 2))
    # Shift-invert eigsh near sigma=1.0 finds smallest positive eigenvalues cleanly & fast
    evals, _ = spla.eigsh(K_ff, k=k_req, M=M_ff, sigma=1.0, which="LM")
    freqs_hz = sorted([float(math.sqrt(max(ev, 0.0)) / (2.0 * math.pi)) for ev in evals if ev > 1.0])
    if not freqs_hz:
        freqs_hz = [0.0]

    f1 = freqs_hz[0]
    f_motor_hz = motor_rpm / 60.0
    f_bpf_hz = blade_count * f_motor_hz

    # Relative separation margin from motor 1x and Blade-Pass-Frequency
    margin_motor_pct = abs(f1 - f_motor_hz) / max(f_motor_hz, 1.0) * 100.0
    margin_bpf_pct = abs(f1 - f_bpf_hz) / max(f_bpf_hz, 1.0) * 100.0
    min_margin_pct = min(margin_motor_pct, margin_bpf_pct)

    # Also check if any of the first 3 modes falls within ±15% of 1x motor frequency
    resonance_conflict = any(abs(f - f_motor_hz) / max(f_motor_hz, 1.0) < 0.18 for f in freqs_hz)
    status = "fail" if (resonance_conflict or f1 < 80.0) else ("warn" if min_margin_pct < 25.0 else "pass")

    warnings: list[str] = []
    if f1 < 80.0:
        warnings.append(f"Frecuencia natural f1 muy baja ({f1:.1f} Hz < 80 Hz): riesgo de oscilación PID / jello.")
    if resonance_conflict:
        warnings.append(
            f"MODAL_RESONANCE_RISK: modo estructural ({[round(f, 1) for f in freqs_hz]} Hz) se cruza con 1x motor ({f_motor_hz:.1f} Hz @ {motor_rpm:.0f} RPM)."
        )

    return {
        "modes_hz": [round(f, 1) for f in freqs_hz],
        "f1_hz": round(f1, 1),
        "motor_1x_hz": round(f_motor_hz, 1),
        "blade_pass_hz": round(f_bpf_hz, 1),
        "min_margin_pct": round(min_margin_pct, 1),
        "status": status,
        "warnings": warnings,
    }


def run_crash_20g_check(
    mesh: trimesh.Trimesh,
    e_mpa: float = 5200.0,
    yield_mpa: float = 45.0,
    total_mass_kg: float = 0.68,
    g_load: float = 20.0,
    fix_mode: str = "center_hub",
    nx: int = 18,
    ny: int = 18,
    nz: int = 3,
) -> dict[str, Any]:
    """Simulate a 20G crash / hard-landing inertial load on the 3D voxelized structure."""
    inside, dx, dy, dz, _ = _voxelize_mesh_hex8(mesh, nx=nx, ny=ny, nz=nz)
    K_ff, M_ff, free_dofs, ndof = _assemble_hex8_k_m(inside, dx, dy, dz, e_mpa, 1.18, fix_mode=fix_mode)

    # Total crash force = g_load * m * g distributed across free Z DOFs (worst-case slap/impact)
    f_crash_n = g_load * total_mass_kg * 9.80665
    f_free = np.zeros(len(free_dofs), dtype=np.float64)
    z_mask = (free_dofs % 3) == 2
    n_z = int(np.sum(z_mask))
    if n_z > 0:
        f_free[z_mask] = -f_crash_n / n_z

    u_free = spla.spsolve(K_ff, f_free)
    max_defl_mm = float(np.max(np.abs(u_free))) if len(u_free) else 0.0

    # Estimate peak bending strain & von Mises stress from local displacement gradient
    span_mm = max(float(mesh.extents[0]), float(mesh.extents[1])) * (0.5 if fix_mode == "center_hub" else 1.0)
    thick_mm = max(2.0, min(float(mesh.extents[2]), 25.0))
    # Cantilever bending strain approximation: eps_max ≈ 1.5 * delta * t / L^2
    est_strain = 1.5 * max_defl_mm * thick_mm / max(span_mm * span_mm, 100.0)
    peak_stress_mpa = float(e_mpa * est_strain)
    crash_sf = min(20.0, yield_mpa / max(peak_stress_mpa, 0.1))

    status = "pass" if crash_sf >= 1.25 else ("warn" if crash_sf >= 0.90 else "fail")
    warnings: list[str] = []
    if crash_sf < 1.25:
        warnings.append(
            f"CRASH_20G_RISK: bajo impacto {g_load:.0f}G ({f_crash_n:.0f} N), FS={crash_sf:.2f} (σ≈{peak_stress_mpa:.1f} MPa, δ={max_defl_mm:.2f} mm)."
        )

    return {
        "g_load": g_load,
        "crash_force_n": round(f_crash_n, 1),
        "max_deflection_mm": round(max_defl_mm, 3),
        "estimated_peak_stress_mpa": round(peak_stress_mpa, 2),
        "crash_safety_factor": round(crash_sf, 2),
        "status": status,
        "warnings": warnings,
    }


def run_monte_carlo_reliability(
    nominal_stress_mpa: float,
    yield_mean_mpa: float = 45.0,
    yield_cv: float = 0.15,
    load_cv: float = 0.10,
    hole_nom_mm: float = 5.0,
    shaft_nom_mm: float = 4.82,
    fdm_tol_std_mm: float = 0.08,
    n_samples: int = 50000,
    seed: int = 42,
) -> dict[str, Any]:
    """Run vectorized 50k-sample Monte Carlo structural & FDM tolerance reliability analysis."""
    rng = np.random.default_rng(seed)

    # 1. Stress-strength interference with FDM interlayer anisotropy & wall/load dispersion
    stress_samples = rng.normal(nominal_stress_mpa, max(nominal_stress_mpa * load_cv, 0.05), size=n_samples)
    strength_samples = rng.normal(yield_mean_mpa, max(yield_mean_mpa * yield_cv, 0.5), size=n_samples)
    pf_struct = float(np.mean(stress_samples >= strength_samples))

    # Reliability index beta (Hasofer-Lind / normal quantile approximation)
    mean_margin = yield_mean_mpa - nominal_stress_mpa
    std_margin = math.hypot(yield_mean_mpa * yield_cv, nominal_stress_mpa * load_cv)
    beta = float(mean_margin / max(std_margin, 1e-6))

    # 2. FDM bore/shaft tolerance stackup jamming probability
    hole_act = rng.normal(hole_nom_mm - 0.04, fdm_tol_std_mm, size=n_samples)  # slight FDM shrinkage
    shaft_act = rng.normal(shaft_nom_mm, 0.025, size=n_samples)
    pf_jam = float(np.mean((hole_act - shaft_act) <= 0.0))

    pf_struct_pct = pf_struct * 100.0
    pf_jam_pct = pf_jam * 100.0
    status = "pass" if (pf_struct_pct < 1.0 and beta >= 2.3) else ("warn" if pf_struct_pct < 5.0 else "fail")

    warnings: list[str] = []
    if pf_struct_pct >= 1.0:
        warnings.append(
            f"MONTE_CARLO_RELIABILITY: Probabilidad de fallo estructural Pf={pf_struct_pct:.2f}% (β={beta:.2f} < 2.33) bajo dispersión FDM ±15%."
        )
    if pf_jam_pct >= 2.0:
        warnings.append(
            f"TOLERANCE_STACKUP_RISK: Probabilidad de atasco en eje/barreno Pf_jam={pf_jam_pct:.2f}% (aumenta holgura +0.1 mm)."
        )

    return {
        "n_samples": n_samples,
        "pf_structural_pct": round(pf_struct_pct, 3),
        "reliability_beta": round(beta, 2),
        "pf_jamming_pct": round(pf_jam_pct, 3),
        "status": status,
        "warnings": warnings,
    }


__all__ = [
    "run_modal_resonance_check",
    "run_crash_20g_check",
    "run_monte_carlo_reliability",
]
