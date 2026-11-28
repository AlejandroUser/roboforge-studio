"""3D Multi-Case SIMP Topology Optimization (Hex8) with Langelaar 45-deg AM Overhang Filter."""

from __future__ import annotations

import time
from dataclasses import dataclass
from functools import lru_cache

import numpy as np
from scipy import sparse
from scipy.ndimage import gaussian_filter, maximum_filter
from scipy.sparse.linalg import LinearOperator, cg


@dataclass
class Simp3DResult:
    rho_3d: np.ndarray                 # shape (nx, ny, nz) float64 in [0, 1]
    compliance_history: list[float]
    final_volume_fraction: float
    mass_reduction_pct: float
    elapsed_s: float
    nx: int
    ny: int
    nz: int
    overhang_filter_45: bool


@lru_cache(maxsize=4)
def _hex8_element_stiffness(nu: float = 0.3) -> np.ndarray:
    """Exact 24x24 stiffness matrix for a unit 8-node hexahedral (Hex8) element (E=1.0) via 2x2x2 Gauss quadrature."""
    # Constitutive matrix D for 3D isotropic elasticity (E=1)
    c_iso = 1.0 / ((1.0 + nu) * (1.0 - 2.0 * nu))
    d_mat = c_iso * np.array(
        [
            [1.0 - nu, nu, nu, 0.0, 0.0, 0.0],
            [nu, 1.0 - nu, nu, 0.0, 0.0, 0.0],
            [nu, nu, 1.0 - nu, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.5 * (1.0 - 2.0 * nu), 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.5 * (1.0 - 2.0 * nu), 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.5 * (1.0 - 2.0 * nu)],
        ],
        dtype=np.float64,
    )

    # Natural coordinates of the 8 Hex8 nodes
    xi_n = np.array([-1, 1, 1, -1, -1, 1, 1, -1], dtype=np.float64)
    eta_n = np.array([-1, -1, 1, 1, -1, -1, 1, 1], dtype=np.float64)
    zeta_n = np.array([-1, -1, -1, -1, 1, 1, 1, 1], dtype=np.float64)

    gp = 1.0 / np.sqrt(3.0)
    ke = np.zeros((24, 24), dtype=np.float64)
    # Jacobian for unit cube [0,1]^3 vs [-1,1]^3 is diag(0.5, 0.5, 0.5) => detJ = 1/8, dN/dx = 2 * dN/dxi
    det_j = 0.125

    for xi in (-gp, gp):
        for eta in (-gp, gp):
            for zeta in (-gp, gp):
                dn_dxi = 0.125 * xi_n * (1.0 + eta_n * eta) * (1.0 + zeta_n * zeta)
                dn_deta = 0.125 * eta_n * (1.0 + xi_n * xi) * (1.0 + zeta_n * zeta)
                dn_dzeta = 0.125 * zeta_n * (1.0 + xi_n * xi) * (1.0 + eta_n * eta)

                dn_dx = 2.0 * dn_dxi
                dn_dy = 2.0 * dn_deta
                dn_dz = 2.0 * dn_dzeta

                b_mat = np.zeros((6, 24), dtype=np.float64)
                for a in range(8):
                    col = 3 * a
                    b_mat[0, col] = dn_dx[a]
                    b_mat[1, col + 1] = dn_dy[a]
                    b_mat[2, col + 2] = dn_dz[a]
                    b_mat[3, col] = dn_dy[a]
                    b_mat[3, col + 1] = dn_dx[a]
                    b_mat[4, col + 1] = dn_dz[a]
                    b_mat[4, col + 2] = dn_dy[a]
                    b_mat[5, col] = dn_dz[a]
                    b_mat[5, col + 2] = dn_dx[a]

                ke += (b_mat.T @ d_mat @ b_mat) * det_j

    return 0.5 * (ke + ke.T)


def _apply_langelaar_overhang_45(
    rho: np.ndarray,
    preserve_mask: np.ndarray,
    obstacle_mask: np.ndarray,
) -> np.ndarray:
    """
    Causal layer-by-layer Langelaar AM overhang filter along Z (build direction).
    Each voxel at layer z can only have density <= max density in the 3x3 cone beneath it at z-1.
    """
    out = rho.copy()
    nz = out.shape[2]
    for z in range(1, nz):
        support_below = maximum_filter(out[:, :, z - 1], size=3, mode="nearest")
        out[:, :, z] = np.minimum(out[:, :, z], support_below)
        # Preserve zones (bearings/bosses) remain solid; obstacles remain void
        out[:, :, z][preserve_mask[:, :, z]] = 1.0
        out[:, :, z][obstacle_mask[:, :, z]] = 0.001
    return out


def run_simp_3d_link(
    length_mm: float,
    root_width_mm: float,
    tip_width_mm: float,
    thickness_mm: float,
    load_n: float,
    lateral_load_n: float = 0.0,
    angular_accel_rad_s2: float = 0.0,
    root_od_mm: float = 36.0,
    tip_od_mm: float = 28.0,
    root_bore_mm: float = 12.0,
    tip_bore_mm: float = 10.0,
    volume_fraction: float = 0.38,
    overhang_filter_45: bool = True,
    nx: int = 30,
    ny: int = 10,
    nz: int = 10,
    n_iter: int = 18,
) -> Simp3DResult:
    """
    Run 3D Multi-Case SIMP Topology Optimization (Hex8) with Jacobi-preconditioned CG
    (warm-started), Preserve/Obstacle masks, and optional Langelaar 45-deg AM overhang filter.

    Returns Simp3DResult with `rho_3d` of shape (nx, ny, nz) in [0, 1].
    """
    t0 = time.perf_counter()

    nx = max(12, min(48, int(nx)))
    ny = max(6, min(20, int(ny)))
    nz = max(6, min(20, int(nz)))
    n_iter = max(6, min(40, int(n_iter)))
    volfrac = float(np.clip(volume_fraction, 0.20, 0.80))
    penal = 3.0
    e_min = 1e-4
    e_max = 1.0

    n_el = nx * ny * nz
    n_nodes = (nx + 1) * (ny + 1) * (nz + 1)
    n_dofs = 3 * n_nodes

    ke = _hex8_element_stiffness(nu=0.3)
    ke_flat = ke.ravel()

    # Node numbering: (nx+1, ny+1, nz+1) in C-order
    node_grid = np.arange(n_nodes, dtype=np.int32).reshape((nx + 1, ny + 1, nz + 1))

    # Build element-to-DOF connectivity (n_el, 24)
    n0 = node_grid[:-1, :-1, :-1].ravel()
    n1 = node_grid[1:, :-1, :-1].ravel()
    n2 = node_grid[1:, 1:, :-1].ravel()
    n3 = node_grid[:-1, 1:, :-1].ravel()
    n4 = node_grid[:-1, :-1, 1:].ravel()
    n5 = node_grid[1:, :-1, 1:].ravel()
    n6 = node_grid[1:, 1:, 1:].ravel()
    n7 = node_grid[:-1, 1:, 1:].ravel()

    enodes = np.column_stack((n0, n1, n2, n3, n4, n5, n6, n7))  # (n_el, 8)
    edof_mat = np.empty((n_el, 24), dtype=np.int32)
    for a in range(8):
        edof_mat[:, 3 * a] = 3 * enodes[:, a]
        edof_mat[:, 3 * a + 1] = 3 * enodes[:, a] + 1
        edof_mat[:, 3 * a + 2] = 3 * enodes[:, a] + 2

    # Physical element centers for Preserve / Obstacle / Tapered-Envelope masks
    max_w = max(float(root_width_mm), float(tip_width_mm), float(root_od_mm), float(tip_od_mm))
    xs = (np.arange(nx, dtype=np.float64) + 0.5) / nx * float(length_mm)
    ys = ((np.arange(ny, dtype=np.float64) + 0.5) / ny - 0.5) * max_w
    zs = ((np.arange(nz, dtype=np.float64) + 0.5) / nz - 0.5) * float(thickness_mm)
    X, Y, _Z = np.meshgrid(xs, ys, zs, indexing="ij")

    r_root = np.hypot(X, Y)
    r_tip = np.hypot(X - float(length_mm), Y)
    half_w_taper = 0.5 * (
        float(root_width_mm) + (float(tip_width_mm) - float(root_width_mm)) * (X / max(float(length_mm), 1.0))
    )

    obstacle_mask = (r_root <= float(root_bore_mm) * 0.48) | (r_tip <= float(tip_bore_mm) * 0.48)
    preserve_mask = (
        (r_root <= float(root_od_mm) * 0.52)
        | (r_tip <= float(tip_od_mm) * 0.52)
        | (X <= max(float(length_mm) / nx, float(root_od_mm) * 0.25))
        | (X >= float(length_mm) - max(float(length_mm) / nx, float(tip_od_mm) * 0.25))
    ) & (~obstacle_mask)
    outside_taper_mask = (np.abs(Y) > (half_w_taper + 0.5 * (max_w / ny))) & (~preserve_mask)
    passive_void_mask = obstacle_mask | outside_taper_mask

    # Boundary conditions: clamp root face (ix == 0)
    fixed_nodes = node_grid[0, :, :].ravel()
    fixed_dofs = np.concatenate((3 * fixed_nodes, 3 * fixed_nodes + 1, 3 * fixed_nodes + 2))
    free_mask = np.ones(n_dofs, dtype=bool)
    free_mask[fixed_dofs] = False
    free_dofs = np.nonzero(free_mask)[0].astype(np.int32)
    n_free = len(free_dofs)

    # Map global DOF -> free DOF index (-1 for fixed) for ultra-fast free-DOF sparse assembly
    dof_to_free = np.full(n_dofs, -1, dtype=np.int32)
    dof_to_free[free_dofs] = np.arange(n_free, dtype=np.int32)

    i_glob = np.repeat(edof_mat, 24, axis=1).ravel()
    j_glob = np.tile(edof_mat, (1, 24)).ravel()
    i_free = dof_to_free[i_glob]
    j_free = dof_to_free[j_glob]
    keep_nz = (i_free >= 0) & (j_free >= 0)
    i_free_k = i_free[keep_nz]
    j_free_k = j_free[keep_nz]

    # Multi-case loads at tip center (ix == nx, iy near mid, iz near mid)
    base_load = max(float(load_n), 1.0)
    inertial_equiv_n = max(0.0, (base_load / 9.81) * (float(length_mm) / 1000.0) * float(angular_accel_rad_s2))
    fz_total = -(base_load + inertial_equiv_n)
    fy_total = max(float(lateral_load_n), 0.25 * base_load)

    tip_y_slice = slice(max(0, ny // 2 - 1), min(ny + 1, ny // 2 + 2))
    tip_z_slice = slice(max(0, nz // 2 - 1), min(nz + 1, nz // 2 + 2))
    tip_nodes = node_grid[nx, tip_y_slice, tip_z_slice].ravel()
    n_tip = max(len(tip_nodes), 1)

    f_vert = np.zeros(n_dofs, dtype=np.float64)
    f_vert[3 * tip_nodes + 2] = fz_total / n_tip

    f_lat = np.zeros(n_dofs, dtype=np.float64)
    f_lat[3 * tip_nodes + 1] = fy_total / n_tip

    f_vert_free = f_vert[free_dofs]
    f_lat_free = f_lat[free_dofs]

    # Initialize density field
    design_mask = (~preserve_mask) & (~passive_void_mask)
    rho = np.full((nx, ny, nz), volfrac, dtype=np.float64)
    rho[preserve_mask] = 1.0
    rho[passive_void_mask] = 0.001

    u_vert = np.zeros(n_dofs, dtype=np.float64)
    u_lat = np.zeros(n_dofs, dtype=np.float64)
    u_vert_free = np.zeros(n_free, dtype=np.float64)
    u_lat_free = np.zeros(n_free, dtype=np.float64)

    compliance_history: list[float] = []
    target_design_mean = volfrac

    for it in range(n_iter):
        rho_phys = gaussian_filter(rho, sigma=0.75, mode="nearest")
        rho_phys[preserve_mask] = 1.0
        rho_phys[passive_void_mask] = 0.001
        if overhang_filter_45:
            rho_phys = _apply_langelaar_overhang_45(rho_phys, preserve_mask, passive_void_mask)

        x_flat = rho_phys.ravel()
        e_elem = e_min + (x_flat**penal) * (e_max - e_min)

        # Assemble free-DOF stiffness matrix K_free directly
        vals_all = np.outer(e_elem, ke_flat).ravel()
        k_free = sparse.coo_matrix(
            (vals_all[keep_nz], (i_free_k, j_free_k)),
            shape=(n_free, n_free),
            dtype=np.float64,
        ).tocsr()

        diag_inv = 1.0 / np.maximum(k_free.diagonal(), 1e-12)
        m_precond = LinearOperator(
            shape=(n_free, n_free),
            matvec=lambda v, d=diag_inv: d * v,
            dtype=np.float64,
        )

        # Solve both load cases via warm-started Jacobi-CG
        u_vert_free, _ = cg(k_free, f_vert_free, x0=u_vert_free, rtol=1.5e-3, maxiter=200, M=m_precond)
        u_lat_free, _ = cg(k_free, f_lat_free, x0=u_lat_free, rtol=1.5e-3, maxiter=200, M=m_precond)

        u_vert[free_dofs] = u_vert_free
        u_lat[free_dofs] = u_lat_free

        # Element strain energies for both load cases
        uv_el = u_vert[edof_mat]
        ul_el = u_lat[edof_mat]
        ce_v = np.einsum("ij,jk,ik->i", uv_el, ke, uv_el)
        ce_l = np.einsum("ij,jk,ik->i", ul_el, ke, ul_el)

        c_total = float(np.dot(e_elem, ce_v) + 0.55 * np.dot(e_elem, ce_l))
        compliance_history.append(round(c_total, 4))

        dc_flat = -penal * (e_max - e_min) * (x_flat ** (penal - 1.0)) * (ce_v + 0.55 * ce_l)
        dc_3d = gaussian_filter(dc_flat.reshape((nx, ny, nz)), sigma=0.85, mode="nearest")

        # Optimality Criteria (OC) update over free design voxels
        l1, l2 = 1e-12, 1e12
        move = 0.22
        rho_new = rho.copy()
        dc_design = np.minimum(dc_3d[design_mask], -1e-12)
        rho_design = rho[design_mask]

        if rho_design.size > 0:
            for _ in range(28):
                lmid = 0.5 * (l1 + l2)
                b_val = np.sqrt(np.maximum(1e-12, -dc_design / max(lmid, 1e-12)))
                cand = np.clip(
                    rho_design * b_val,
                    np.maximum(0.001, rho_design - move),
                    np.minimum(1.0, rho_design + move),
                )
                if cand.mean() > target_design_mean:
                    l1 = lmid
                else:
                    l2 = lmid
                if (l2 - l1) / (l1 + l2 + 1e-12) < 1e-3:
                    break
            rho_new[design_mask] = cand

        # Enforce Y-symmetry for clean bionic robotic links
        rho_new = 0.5 * (rho_new + rho_new[:, ::-1, :])
        rho_new[preserve_mask] = 1.0
        rho_new[passive_void_mask] = 0.001

        change = float(np.max(np.abs(rho_new - rho)))
        rho = rho_new
        if it >= 8 and change < 0.015:
            break

    # Final filtered & overhang-compliant density field
    rho_final = gaussian_filter(rho, sigma=0.65, mode="nearest")
    rho_final[preserve_mask] = 1.0
    rho_final[passive_void_mask] = 0.001
    if overhang_filter_45:
        rho_final = _apply_langelaar_overhang_45(rho_final, preserve_mask, passive_void_mask)
    rho_final = np.clip(rho_final, 0.0, 1.0)

    active_domain = ~passive_void_mask
    final_vf = float(rho_final[active_domain].mean()) if np.any(active_domain) else volfrac
    mass_red_pct = max(0.0, min(85.0, (1.0 - final_vf) * 100.0))
    elapsed_s = time.perf_counter() - t0

    return Simp3DResult(
        rho_3d=rho_final,
        compliance_history=compliance_history,
        final_volume_fraction=round(final_vf, 3),
        mass_reduction_pct=round(mass_red_pct, 1),
        elapsed_s=round(elapsed_s, 3),
        nx=nx,
        ny=ny,
        nz=nz,
        overhang_filter_45=overhang_filter_45,
    )
