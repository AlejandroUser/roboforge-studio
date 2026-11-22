from __future__ import annotations

from typing import Callable

import numpy as np
from scipy import sparse
from scipy.sparse.linalg import spsolve
from skimage.measure import approximate_polygon, find_contours


def _q4_element_stiffness(nu: float = 0.3) -> np.ndarray:
    """Plane-stress 4-node bilinear quadrilateral (Q4) element stiffness matrix (E=1)."""
    k = np.array(
        [
            1 / 2 - nu / 6,
            1 / 8 + nu / 8,
            -1 / 4 - nu / 12,
            -1 / 8 + 3 * nu / 8,
            -1 / 4 + nu / 12,
            -1 / 8 - nu / 8,
            nu / 6,
            1 / 8 - 3 * nu / 8,
        ],
        dtype=np.float64,
    )
    ke = (
        1.0
        / (1.0 - nu**2)
        * np.array(
            [
                [k[0], k[1], k[2], k[3], k[4], k[5], k[6], k[7]],
                [k[1], k[0], k[7], k[6], k[5], k[4], k[3], k[2]],
                [k[2], k[7], k[0], k[5], k[6], k[3], k[4], k[1]],
                [k[3], k[6], k[5], k[0], k[7], k[2], k[1], k[4]],
                [k[4], k[5], k[6], k[7], k[0], k[1], k[2], k[3]],
                [k[5], k[4], k[3], k[2], k[1], k[0], k[7], k[6]],
                [k[6], k[3], k[4], k[1], k[2], k[7], k[0], k[5]],
                [k[7], k[2], k[1], k[4], k[3], k[6], k[5], k[0]],
            ],
            dtype=np.float64,
        )
    )
    return ke


def _build_sensitivity_filter(
    nx: int, ny: int, rmin: float
) -> tuple[sparse.csc_matrix, np.ndarray]:
    """Build sparse convolution filter matrix H and normalization vector Hs."""
    n_el = nx * ny
    radius = int(np.ceil(rmin))
    max_entries = n_el * ((2 * radius + 1) ** 2)
    i_h = np.empty(max_entries, dtype=np.int32)
    j_h = np.empty(max_entries, dtype=np.int32)
    s_h = np.empty(max_entries, dtype=np.float64)

    idx = 0
    for i1 in range(nx):
        i2_min = max(i1 - (radius - 1), 0)
        i2_max = min(i1 + radius, nx)
        for j1 in range(ny):
            e1 = i1 * ny + j1
            j2_min = max(j1 - (radius - 1), 0)
            j2_max = min(j1 + radius, ny)
            for i2 in range(i2_min, i2_max):
                dx2 = (i1 - i2) ** 2
                for j2 in range(j2_min, j2_max):
                    dist = np.sqrt(dx2 + (j1 - j2) ** 2)
                    weight = rmin - dist
                    if weight > 0.0:
                        i_h[idx] = e1
                        j_h[idx] = i2 * ny + j2
                        s_h[idx] = weight
                        idx += 1

    h_mat = sparse.csc_matrix(
        (s_h[:idx], (i_h[:idx], j_h[:idx])), shape=(n_el, n_el), dtype=np.float64
    )
    hs = np.asarray(h_mat.sum(axis=1)).ravel()
    return h_mat, hs


def run_simp_cantilever(
    nx: int = 44,
    ny: int = 16,
    volfrac: float = 0.52,
    penal: float = 3.0,
    rmin: float = 1.6,
    n_iter: int = 24,
) -> np.ndarray:
    """
    2D SIMP topology optimization for a cantilever beam (left edge clamped,
    vertical tip load at right-edge center) using scipy.sparse.linalg.spsolve,
    sensitivity filtering, and Optimality Criteria (OC) updates.

    Returns a (ny, nx) float64 density array in [0.001, 1.0].
    """
    volfrac = float(np.clip(volfrac, 0.25, 0.85))
    n_el = nx * ny
    n_dofs = 2 * (nx + 1) * (ny + 1)

    ke = _q4_element_stiffness(nu=0.3)

    # Element DOF connectivity matrix (n_el, 8)
    nodenrs = np.arange((nx + 1) * (ny + 1)).reshape((nx + 1, ny + 1))
    edof_vec = (2 * nodenrs[:-1, :-1] + 2).ravel()
    edof_mat = np.tile(edof_vec[:, None], (1, 8)) + np.array(
        [0, 1, 2 * ny + 2, 2 * ny + 3, 2 * ny, 2 * ny + 1, -2, -1], dtype=np.int32
    )

    i_k = np.kron(edof_mat, np.ones((1, 8), dtype=np.int32)).ravel()
    j_k = np.kron(edof_mat, np.ones((8, 1), dtype=np.int32)).ravel()
    ke_flat = ke.ravel()

    # Boundary conditions: clamp left edge (i = 0, all j = 0..ny)
    fixed_nodes = nodenrs[0, :]
    fixed_dofs = np.union1d(2 * fixed_nodes, 2 * fixed_nodes + 1)
    all_dofs = np.arange(n_dofs, dtype=np.int32)
    free_dofs = np.setdiff1d(all_dofs, fixed_dofs)

    # Vertical load at right edge center (i = nx, j = ny // 2)
    f = np.zeros(n_dofs, dtype=np.float64)
    center_j = ny // 2
    if ny % 2 == 0:
        load_node = nodenrs[nx, center_j]
        f[2 * load_node + 1] = -1.0
    else:
        f[2 * nodenrs[nx, center_j] + 1] = -0.5
        f[2 * nodenrs[nx, center_j + 1] + 1] = -0.5

    f_free = f[free_dofs]
    u = np.zeros(n_dofs, dtype=np.float64)

    h_mat, hs = _build_sensitivity_filter(nx, ny, rmin)

    e_min = 1e-9
    e_max = 1.0
    x = np.full(n_el, volfrac, dtype=np.float64)

    for _ in range(n_iter):
        # Assemble global stiffness matrix K
        s_k = np.outer(e_min + np.power(x, penal) * (e_max - e_min), ke_flat).ravel()
        k_mat = sparse.csc_matrix((s_k, (i_k, j_k)), shape=(n_dofs, n_dofs))
        k_free = k_mat[free_dofs, :][:, free_dofs]

        u[free_dofs] = spsolve(k_free, f_free)

        # Element compliance sensitivity dc
        u_el = u[edof_mat]
        ce = np.einsum("ij,jk,ik->i", u_el, ke, u_el)
        dc = -penal * (e_max - e_min) * np.power(x, penal - 1.0) * ce

        # Sensitivity filter
        dc = (h_mat @ (x * dc)) / (hs * np.maximum(1e-3, x))

        # Optimality Criteria (OC) bisection update
        l1 = 0.0
        l2 = 1e9
        move = 0.2
        x_new = x
        while (l2 - l1) / (l1 + l2 + 1e-12) > 1e-3:
            lmid = 0.5 * (l2 + l1)
            b_val = np.sqrt(np.maximum(1e-12, -dc / max(lmid, 1e-12)))
            x_new = np.clip(
                x * b_val,
                np.maximum(0.001, x - move),
                np.minimum(1.0, x + move),
            )
            if x_new.mean() > volfrac:
                l1 = lmid
            else:
                l2 = lmid

        # Enforce vertical symmetry for clean bionic robotic link cutouts
        grid = x_new.reshape((nx, ny))
        grid = 0.5 * (grid + grid[:, ::-1])
        change = float(np.max(np.abs(grid.ravel() - x)))
        x = grid.ravel()
        if change < 0.01:
            break

    # Reshape to (ny, nx) where row=y (0..ny-1) and col=x (0..nx-1)
    return x.reshape((nx, ny)).T.copy()


def _polygon_area(pts: np.ndarray) -> float:
    x = pts[:, 0]
    y = pts[:, 1]
    return 0.5 * float(np.abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def _chaikin_smooth_closed(pts: np.ndarray, iterations: int = 1) -> np.ndarray:
    """Corner-cutting Chaikin smoothing on a closed 2D polygon (open vertex list)."""
    arr = pts
    for _ in range(iterations):
        n = len(arr)
        if n < 3:
            break
        nxt = np.roll(arr, -1, axis=0)
        q = 0.75 * arr + 0.25 * nxt
        r = 0.25 * arr + 0.75 * nxt
        out = np.empty((2 * n, 2), dtype=np.float64)
        out[0::2] = q
        out[1::2] = r
        arr = out
    return arr


def _dedupe_consecutive(pts: np.ndarray, min_dist: float = 0.45) -> np.ndarray:
    """Remove consecutive points closer than min_dist (including wrap-around)."""
    if len(pts) < 3:
        return pts
    kept = [pts[0]]
    for pt in pts[1:]:
        if np.hypot(pt[0] - kept[-1][0], pt[1] - kept[-1][1]) >= min_dist:
            kept.append(pt)
    if len(kept) >= 2 and np.hypot(kept[-1][0] - kept[0][0], kept[-1][1] - kept[0][1]) < min_dist:
        kept.pop()
    return np.asarray(kept, dtype=np.float64)


def extract_organic_void_polygons(
    density_grid: np.ndarray,
    span_x0: float,
    span_x1: float,
    half_w_fn: Callable[[float], float],
    margin_mm: float = 3.2,
    threshold: float = 0.42,
) -> list[list[tuple[float, float]]]:
    """
    Extract smooth, closed organic void polygons from a 2D SIMP density grid
    using skimage.measure.find_contours on (1.0 - density_grid).

    All returned polygons are strictly contained within:
        span_x0 <= x <= span_x1
        abs(y) <= half_w_fn(x) - margin_mm

    Each polygon is returned as a list of (x, y) tuples in mm (open vertex ring,
    without repeating the first point at the end, ready for CAD Spline/Polyline).
    """
    if span_x1 <= span_x0 + 4.0:
        return []

    ny, nx = density_grid.shape
    if ny < 3 or nx < 3:
        return []

    # Pad with 0.0 (solid outside the design span) so all void contours are closed loops
    void_field = np.pad(
        1.0 - np.clip(density_grid, 0.0, 1.0),
        pad_width=1,
        mode="constant",
        constant_values=0.0,
    )
    level = 1.0 - float(np.clip(threshold, 0.05, 0.95))
    raw_contours = find_contours(void_field, level=level)

    span_len = span_x1 - span_x0
    polygons: list[list[tuple[float, float]]] = []

    for contour in raw_contours:
        if len(contour) < 5:
            continue

        # Convert padded (row, col) to normalized coordinates u in [0, 1], v in [0, 1]
        u = np.clip((contour[:, 1] - 1.0) / max(nx - 1, 1), 0.0, 1.0)
        v = np.clip((contour[:, 0] - 1.0) / max(ny - 1, 1), 0.0, 1.0)

        xs = span_x0 + u * span_len
        half_ws = np.array(
            [max(0.0, float(half_w_fn(float(x_val))) - margin_mm) for x_val in xs],
            dtype=np.float64,
        )
        ys = (2.0 * v - 1.0) * half_ws

        pts = np.column_stack((xs, ys))

        # Drop duplicate closing vertex before smoothing/simplification
        if np.hypot(pts[0, 0] - pts[-1, 0], pts[0, 1] - pts[-1, 1]) < 1e-3:
            pts = pts[:-1]
        if len(pts) < 4:
            continue

        # Simplify noisy staircase edges, then apply 1 Chaikin pass for smooth bionic curves
        approx = approximate_polygon(pts, tolerance=0.55)
        approx = _dedupe_consecutive(approx, min_dist=0.6)
        if len(approx) < 4:
            continue

        smooth = _chaikin_smooth_closed(approx, iterations=1)
        smooth = _dedupe_consecutive(smooth, min_dist=0.5)
        if len(smooth) < 4:
            continue

        # Strictly clamp every vertex inside [span_x0, span_x1] and |y| <= half_w_fn(x) - margin_mm
        smooth[:, 0] = np.clip(smooth[:, 0], span_x0, span_x1)
        for idx in range(len(smooth)):
            max_y = max(0.0, float(half_w_fn(float(smooth[idx, 0]))) - margin_mm)
            if max_y <= 0.0:
                smooth[idx, 1] = 0.0
            else:
                smooth[idx, 1] = float(np.clip(smooth[idx, 1], -max_y, max_y))

        # Filter out degenerate or tiny voids that could destabilize OCCT booleans
        bbox_w = float(np.max(smooth[:, 0]) - np.min(smooth[:, 0]))
        bbox_h = float(np.max(smooth[:, 1]) - np.min(smooth[:, 1]))
        area = _polygon_area(smooth)
        if bbox_w < 2.5 or bbox_h < 2.0 or area < 12.0:
            continue

        polygons.append([(round(float(p[0]), 3), round(float(p[1]), 3)) for p in smooth])

    # Sort left-to-right by centroid x for deterministic CAD construction
    polygons.sort(key=lambda poly: sum(p[0] for p in poly) / len(poly))
    return polygons
