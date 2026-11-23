from __future__ import annotations

import threading
import time
from pathlib import Path

import gmsh
import numpy as np
import trimesh
from scipy import sparse
from scipy.sparse.linalg import spsolve

from ..schemas import (
    FEAResult,
    GearDesignRequest,
    GearVariantKind,
    LinkDesignRequest,
    MountDesignRequest,
    MountVariantKind,
    VariantKind,
)
from .gear_mechanics import compute_gear_diameters

_GMSH_LOCK = threading.Lock()
POISSON_RATIO = 0.33


def _mesh_step_tet4(
    step_path: Path, mesh_size_mm: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Import a STEP solid into Gmsh (thread-safe), generate a 3D Tet4 mesh, and
    return (coords[N,3], tets[E,4], surface_tris[F,3]) with 0-based node indices.
    Uses OCC geometry healing and progressive 2D/3D mesher fallbacks so filleted
    CAD solids with small bolt holes mesh reliably.
    """
    attempts = [
        # (min_size, max_size, curvature_pts, alg2d, alg3d)
        (min(mesh_size_mm * 0.6, 2.2), float(mesh_size_mm), 12, 6, 1),
        (min(mesh_size_mm * 0.3, 1.4), float(mesh_size_mm), 16, 1, 1),
        (0.8, float(mesh_size_mm) * 0.85, 20, 1, 10),
    ]

    last_err: Exception | None = None
    with _GMSH_LOCK:
        for min_sz, max_sz, curv, alg2d, alg3d in attempts:
            gmsh.initialize(interruptible=False)
            try:
                gmsh.option.setNumber("General.Terminal", 0)
                gmsh.option.setNumber("Geometry.OCCFixDegenerated", 1)
                gmsh.option.setNumber("Geometry.OCCFixSmallEdges", 1)
                gmsh.option.setNumber("Geometry.OCCFixSmallFaces", 1)
                gmsh.option.setNumber("Mesh.MeshSizeMin", min_sz)
                gmsh.option.setNumber("Mesh.MeshSizeMax", max_sz)
                gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", curv)
                gmsh.option.setNumber("Mesh.Algorithm", alg2d)
                gmsh.option.setNumber("Mesh.Algorithm3D", alg3d)
                gmsh.option.setNumber("Mesh.Optimize", 1)
                gmsh.model.add("roboforge_fea")

                gmsh.model.occ.importShapes(str(step_path))
                gmsh.model.occ.synchronize()
                gmsh.model.mesh.generate(3)

                node_tags, node_coords, _ = gmsh.model.mesh.getNodes()
                if len(node_tags) == 0:
                    raise RuntimeError("Gmsh generated 0 nodes from STEP geometry.")

                coords = np.asarray(node_coords, dtype=np.float64).reshape((-1, 3))
                tags_int = np.asarray(node_tags, dtype=np.int64)
                tag_to_idx = np.full(int(tags_int.max()) + 1, -1, dtype=np.int32)
                tag_to_idx[tags_int] = np.arange(len(tags_int), dtype=np.int32)

                _, tet_node_tags = gmsh.model.mesh.getElementsByType(4)
                if len(tet_node_tags) == 0:
                    raise RuntimeError("Gmsh generated 0 Tet4 elements from STEP geometry.")
                tets = tag_to_idx[np.asarray(tet_node_tags, dtype=np.int64).reshape((-1, 4))]

                _, tri_node_tags = gmsh.model.mesh.getElementsByType(2)
                if len(tri_node_tags) > 0:
                    tris = tag_to_idx[
                        np.asarray(tri_node_tags, dtype=np.int64).reshape((-1, 3))
                    ]
                else:
                    tris = np.empty((0, 3), dtype=np.int32)
                last_err = None
                break
            except Exception as exc:
                last_err = exc
            finally:
                gmsh.finalize()

    if last_err is not None:
        raise RuntimeError(f"Gmsh 3D meshing failed after fallbacks: {last_err}") from last_err

    # Keep only nodes referenced by 3D Tet4 elements to avoid singular disconnected DOFs
    used_nodes = np.unique(tets)
    if len(used_nodes) < len(coords):
        old_to_new = np.full(len(coords), -1, dtype=np.int32)
        old_to_new[used_nodes] = np.arange(len(used_nodes), dtype=np.int32)
        coords = coords[used_nodes]
        tets = old_to_new[tets]
        if len(tris) > 0:
            tris = old_to_new[tris]
            tris = tris[np.all(tris >= 0, axis=1)]

    return coords, tets, tris


def _extract_boundary_tris_from_tets(tets: np.ndarray) -> np.ndarray:
    """Fallback boundary triangle extraction if surface elements are absent."""
    faces = np.vstack(
        [
            tets[:, [0, 2, 1]],
            tets[:, [0, 1, 3]],
            tets[:, [1, 2, 3]],
            tets[:, [0, 3, 2]],
        ]
    )
    sorted_faces = np.sort(faces, axis=1)
    _, idx, counts = np.unique(
        sorted_faces, axis=0, return_index=True, return_counts=True
    )
    return faces[idx[counts == 1]]


def _elasticity_matrix_3d(youngs_mpa: float, nu: float = POISSON_RATIO) -> np.ndarray:
    """Isotropic 3D linear elasticity constitutive matrix D (6x6, MPa)."""
    lam = (youngs_mpa * nu) / ((1.0 + nu) * (1.0 - 2.0 * nu))
    mu = youngs_mpa / (2.0 * (1.0 + nu))
    d_mat = np.zeros((6, 6), dtype=np.float64)
    d_mat[0, 0] = d_mat[1, 1] = d_mat[2, 2] = lam + 2.0 * mu
    d_mat[0, 1] = d_mat[0, 2] = d_mat[1, 0] = d_mat[1, 2] = d_mat[2, 0] = d_mat[2, 1] = lam
    d_mat[3, 3] = d_mat[4, 4] = d_mat[5, 5] = mu
    return d_mat


def _orthotropic_elasticity_matrix_3d(
    youngs_xy_mpa: float,
    youngs_z_mpa: float,
    print_orientation: str = "xy_flat",
    nu_xy: float = POISSON_RATIO,
) -> tuple[np.ndarray, bool]:
    """
    Transversely isotropic (orthotropic) 3D stiffness tensor D (6x6, MPa)
    for layered FDM/anisotropic materials.
    - xy_flat: layers in XY plane, weak build axis along Z.
    - z_vertical: weak build axis aligned with X (longitudinal span).
    Falls back cleanly to isotropic if E_xy == E_z.
    """
    e_p = max(float(youngs_xy_mpa), 100.0)
    e_t = max(float(youngs_z_mpa), 100.0)
    if abs(e_p - e_t) / max(e_p, e_t) < 0.02:
        return _elasticity_matrix_3d(e_p, nu_xy), False

    # Thermodynamically stable out-of-plane Poisson ratio
    nu_pt = min(0.30, 0.42 * np.sqrt(e_p / e_t) * (e_t / e_p))
    nu_tp = nu_pt * (e_t / e_p)
    g_p = e_p / (2.0 * (1.0 + nu_xy))
    g_t = (e_p * e_t) / (e_p + e_t + 2.0 * nu_pt * e_t)

    # Compliance matrix S in Voigt order [xx, yy, zz, xy, yz, xz] (Z = weak axis)
    s_mat = np.zeros((6, 6), dtype=np.float64)
    s_mat[0, 0] = 1.0 / e_p
    s_mat[1, 1] = 1.0 / e_p
    s_mat[2, 2] = 1.0 / e_t
    s_mat[0, 1] = s_mat[1, 0] = -nu_xy / e_p
    s_mat[0, 2] = s_mat[2, 0] = -nu_tp / e_t
    s_mat[1, 2] = s_mat[2, 1] = -nu_tp / e_t
    s_mat[3, 3] = 1.0 / g_p
    s_mat[4, 4] = 1.0 / g_t
    s_mat[5, 5] = 1.0 / g_t

    d_mat = np.linalg.inv(s_mat)

    # If printed vertically along X, swap X <-> Z in Voigt indices:
    # [xx, yy, zz, xy, yz, xz] -> [zz, yy, xx, yz, xy, xz] (perm = [2, 1, 0, 4, 3, 5])
    if print_orientation == "z_vertical":
        perm = [2, 1, 0, 4, 3, 5]
        d_mat = d_mat[np.ix_(perm, perm)]

    # Verify positive definiteness
    if np.min(np.linalg.eigvalsh(0.5 * (d_mat + d_mat.T))) <= 0.0:
        return _elasticity_matrix_3d(min(e_p, e_t), nu_xy), False

    return d_mat, True


def _von_mises_colormap(values: np.ndarray, vmax: float) -> np.ndarray:
    """
    Map scalar stress values in [0, vmax] to RGBA uint8 colors along the
    engineering heatmap palette: blue -> cyan -> green -> yellow -> red.
    """
    stops = np.array([0.0, 0.25, 0.50, 0.75, 1.0], dtype=np.float64)
    rgb_stops = np.array(
        [
            [37, 99, 235],   # blue
            [6, 182, 212],   # cyan
            [34, 197, 94],   # green
            [250, 204, 21],  # yellow
            [239, 68, 68],   # red
        ],
        dtype=np.float64,
    )
    t = np.clip(values / max(vmax, 1e-6), 0.0, 1.0)
    r = np.interp(t, stops, rgb_stops[:, 0])
    g = np.interp(t, stops, rgb_stops[:, 1])
    b = np.interp(t, stops, rgb_stops[:, 2])
    a = np.full_like(r, 255.0)
    return np.column_stack((r, g, b, a)).astype(np.uint8)


def _solve_tet4_and_export(
    *,
    coords: np.ndarray,
    tets: np.ndarray,
    tris: np.ndarray,
    fixed_nodes: np.ndarray,
    load_nodes: np.ndarray,
    load_n: float,
    load_axis: int,
    youngs_eff_mpa: float,
    yield_eff_mpa: float,
    target_safety_factor: float,
    max_deflection_mm: float,
    mesh_size_mm: float,
    deform_scale: float,
    output_glb_path: Path,
    design_hash: str,
    variant_id: str,
    notes: list[str],
    t0: float,
    youngs_xy_mpa: float | None = None,
    youngs_z_mpa: float | None = None,
    density_g_cm3: float = 1.29,
    print_orientation: str = "xy_flat",
) -> FEAResult:
    """
    Shared vectorized 3D Tet4 linear elasticity solver, nodal Von Mises post-processor,
    and vertex-colored GLB exporter (preserving COLOR_0 + optional deform_scale).
    """
    n_nodes = len(coords)

    # Vectorized Tet4 shape-function gradients and volumes
    tet_coords = coords[tets]  # (E, 4, 3)
    ones = np.ones((len(tets), 4, 1), dtype=np.float64)
    x_mat = np.concatenate([ones, tet_coords], axis=2)  # (E, 4, 4)

    dets = np.linalg.det(x_mat)
    vols = np.abs(dets) / 6.0
    valid_mask = vols > 1e-9
    if not np.all(valid_mask):
        tets = tets[valid_mask]
        x_mat = x_mat[valid_mask]
        vols = vols[valid_mask]

    n_elems = len(tets)
    if n_elems == 0:
        raise RuntimeError("No valid positive-volume Tet4 elements found.")

    inv_x = np.linalg.inv(x_mat)
    grad_n = inv_x[:, 1:4, :].transpose(0, 2, 1)  # (E, 4, 3)

    b_mat = np.zeros((n_elems, 6, 12), dtype=np.float64)
    for i in range(4):
        gx = grad_n[:, i, 0]
        gy = grad_n[:, i, 1]
        gz = grad_n[:, i, 2]
        c0, c1, c2 = 3 * i, 3 * i + 1, 3 * i + 2
        b_mat[:, 0, c0] = gx
        b_mat[:, 1, c1] = gy
        b_mat[:, 2, c2] = gz
        b_mat[:, 3, c0] = gy
        b_mat[:, 3, c1] = gx
        b_mat[:, 4, c1] = gz
        b_mat[:, 4, c2] = gy
        b_mat[:, 5, c0] = gz
        b_mat[:, 5, c2] = gx

    e_xy = float(youngs_xy_mpa) if youngs_xy_mpa is not None else float(youngs_eff_mpa)
    e_z = float(youngs_z_mpa) if youngs_z_mpa is not None else float(youngs_eff_mpa)
    d_mat, is_orthotropic = _orthotropic_elasticity_matrix_3d(e_xy, e_z, print_orientation, POISSON_RATIO)
    db_mat = np.einsum("ij,ejk->eik", d_mat, b_mat)  # (E, 6, 12)
    ke = np.einsum("eji,ejk->eik", b_mat, db_mat) * vols[:, None, None]  # (E, 12, 12)

    n_dofs = 3 * n_nodes
    edofs = np.empty((n_elems, 12), dtype=np.int32)
    edofs[:, 0::3] = 3 * tets
    edofs[:, 1::3] = 3 * tets + 1
    edofs[:, 2::3] = 3 * tets + 2

    rows = np.broadcast_to(edofs[:, :, None], (n_elems, 12, 12)).ravel()
    cols = np.broadcast_to(edofs[:, None, :], (n_elems, 12, 12)).ravel()
    k_global = sparse.coo_matrix((ke.ravel(), (rows, cols)), shape=(n_dofs, n_dofs)).tocsr()

    f_vec = np.zeros(n_dofs, dtype=np.float64)
    f_vec[3 * load_nodes + load_axis] = -float(load_n) / float(len(load_nodes))

    fixed_dofs = np.concatenate([3 * fixed_nodes, 3 * fixed_nodes + 1, 3 * fixed_nodes + 2])
    free_dofs = np.setdiff1d(np.arange(n_dofs, dtype=np.int32), fixed_dofs)

    k_ff = k_global[free_dofs, :][:, free_dofs].tocsc()
    u_vec = np.zeros(n_dofs, dtype=np.float64)
    u_vec[free_dofs] = spsolve(k_ff, f_vec[free_dofs])

    if not np.all(np.isfinite(u_vec)):
        raise RuntimeError("FEM linear system produced non-finite displacements.")

    # 3D Modal Analysis: 1st natural frequency (Hz) using lumped nodal mass matrix
    # Consistent units: mm, N, MPa (N/mm²), mass in tonne (1 g/cm³ = 1e-9 tonne/mm³)
    first_natural_freq_hz: float | None = None
    try:
        from scipy.sparse.linalg import eigsh

        rho_tonne_mm3 = max(float(density_g_cm3), 0.1) * 1e-9
        nodal_mass = np.zeros(n_nodes, dtype=np.float64)
        elem_node_mass = (vols * rho_tonne_mm3) / 4.0
        for k in range(4):
            np.add.at(nodal_mass, tets[:, k], elem_node_mass)
        m_diag = np.repeat(np.maximum(nodal_mass, 1e-18), 3)
        m_ff = sparse.diags(m_diag[free_dofs], format="csc")
        eigvals, _ = eigsh(k_ff, k=1, M=m_ff, sigma=0.0, which="LM", tol=1e-2, maxiter=200)
        if len(eigvals) > 0 and np.isfinite(eigvals[0]) and eigvals[0] > 0.0:
            first_natural_freq_hz = round(float(np.sqrt(eigvals[0]) / (2.0 * np.pi)), 1)
    except Exception:
        first_natural_freq_hz = None

    u_nodes = u_vec.reshape((n_nodes, 3))
    max_displacement_mm = float(np.max(np.linalg.norm(u_nodes, axis=1)))

    # Element stress (Voigt 6-component) and Von Mises equivalent stress
    u_el = u_vec[edofs]
    sigma = np.einsum("eij,ej->ei", db_mat, u_el)
    sx, sy, sz, txy, tyz, txz = (
        sigma[:, 0],
        sigma[:, 1],
        sigma[:, 2],
        sigma[:, 3],
        sigma[:, 4],
        sigma[:, 5],
    )
    vm_el = np.sqrt(
        np.maximum(
            0.0,
            0.5
            * (
                (sx - sy) ** 2
                + (sy - sz) ** 2
                + (sz - sx) ** 2
                + 6.0 * (txy**2 + tyz**2 + txz**2)
            ),
        )
    )

    # Volume-weighted nodal averaging for smooth surface Von Mises field
    nodal_vm_num = np.zeros(n_nodes, dtype=np.float64)
    nodal_vm_den = np.zeros(n_nodes, dtype=np.float64)
    for k in range(4):
        np.add.at(nodal_vm_num, tets[:, k], vm_el * vols)
        np.add.at(nodal_vm_den, tets[:, k], vols)
    nodal_vm = nodal_vm_num / np.maximum(nodal_vm_den, 1e-12)

    max_von_mises_mpa = float(np.max(nodal_vm))
    p95_von_mises_mpa = float(np.percentile(nodal_vm, 95))
    fea_safety_factor = yield_eff_mpa / max(p95_von_mises_mpa, 1e-6)
    passes_fea = (
        fea_safety_factor >= target_safety_factor
        and max_displacement_mm <= max_deflection_mm
    )

    # Export vertex-colored surface mesh to GLB with COLOR_0 preserved
    surf_nodes, inv_idx = np.unique(tris, return_inverse=True)
    deform_scale_clamped = max(0.0, min(float(deform_scale), 100.0))
    surf_verts = coords[surf_nodes] + deform_scale_clamped * u_nodes[surf_nodes]
    surf_faces = inv_idx.reshape((-1, 3))
    surf_vm = nodal_vm[surf_nodes]
    if deform_scale_clamped > 0.0:
        notes.append(
            f"Deformación visual amplificada ×{deform_scale_clamped:.0f} sobre la malla 3D."
        )

    color_scale_max = max(p95_von_mises_mpa * 1.35, 0.1)
    vertex_colors = _von_mises_colormap(surf_vm, color_scale_max)

    mesh = trimesh.Trimesh(
        vertices=surf_verts,
        faces=surf_faces,
        vertex_colors=vertex_colors,
        process=False,
    )
    mesh.fix_normals()
    output_glb_path.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(str(output_glb_path), file_type="glb")

    solve_time_s = time.perf_counter() - t0
    tensor_label = (
        f"tensor ortotrópico 6×6 (Exy={e_xy:.0f} / Ez={e_z:.0f} MPa, {print_orientation})"
        if is_orthotropic
        else f"tensor isotrópico (E={e_xy:.0f} MPa)"
    )
    notes.append(f"Malla Tet4 Gmsh (h={mesh_size_mm:.1f} mm, {tensor_label}).")
    if first_natural_freq_hz is not None:
        if first_natural_freq_hz < 45.0:
            notes.append(
                f"⚠ Análisis Modal: 1ª frecuencia natural baja ({first_natural_freq_hz:.1f} Hz < 45 Hz) — riesgo de resonancia con motores paso a paso."
            )
        else:
            notes.append(
                f"Análisis Modal: 1ª frecuencia natural f₁ = {first_natural_freq_hz:.1f} Hz (rigidez dinámica saludable)."
            )
    if passes_fea:
        notes.append(
            f"Validación FEA aprobada: FS={fea_safety_factor:.2f} (≥ {target_safety_factor:.2f}) y δ={max_displacement_mm:.3f} mm (≤ {max_deflection_mm:.2f} mm)."
        )
    else:
        notes.append(
            f"Revisar diseño: FS FEA={fea_safety_factor:.2f} (objetivo {target_safety_factor:.2f}), δ={max_displacement_mm:.3f} mm (límite {max_deflection_mm:.2f} mm)."
        )

    return FEAResult(
        design_hash=design_hash,
        variant_id=variant_id,
        solver_name=(
            "Gmsh Tet4 + FEM 3D Ortotrópico 6×6 + Modal"
            if is_orthotropic
            else "Gmsh Tet4 + FEM 3D Isotrópico + Modal"
        ),
        node_count=int(n_nodes),
        element_count=int(n_elems),
        solve_time_s=round(solve_time_s, 3),
        max_von_mises_mpa=round(max_von_mises_mpa, 2),
        p95_von_mises_mpa=round(p95_von_mises_mpa, 2),
        max_displacement_mm=round(max_displacement_mm, 4),
        effective_yield_mpa=round(yield_eff_mpa, 2),
        fea_safety_factor=round(fea_safety_factor, 2),
        passes_fea=passes_fea,
        is_orthotropic=is_orthotropic,
        first_natural_freq_hz=first_natural_freq_hz,
        fea_glb_url=f"/api/files/{design_hash}/{output_glb_path.name}",
        notes=notes,
    )


def run_step_fea(
    step_path: Path,
    output_glb_path: Path,
    req: LinkDesignRequest,
    design_hash: str,
    variant_id: VariantKind,
    material: dict,
    mesh_size_mm: float = 7.0,
    load_direction: str = "y_inplane",
    deform_scale: float = 0.0,
) -> FEAResult:
    """
    Run a real 3D Tet4 Linear Elasticity FEM analysis on a robotic link STEP file.
    """
    t0 = time.perf_counter()
    step_path = Path(step_path)
    output_glb_path = Path(output_glb_path)

    coords, tets, tris = _mesh_step_tet4(step_path, mesh_size_mm=mesh_size_mm)
    if len(tris) == 0:
        tris = _extract_boundary_tris_from_tets(tets)

    n_nodes = len(coords)

    if req.print_orientation == "z_vertical":
        youngs_eff_mpa = float(material["youngs_z_mpa"])
        yield_eff_mpa = float(material["yield_z_mpa"])
    else:
        youngs_eff_mpa = float(material["youngs_xy_mpa"])
        yield_eff_mpa = float(material["yield_xy_mpa"])

    # Boundary conditions: fixed root bore/boss nodes
    notes: list[str] = []
    r_root = np.hypot(coords[:, 0], coords[:, 1])
    fixed_nodes = np.where(r_root <= float(req.root_end.bore_diameter_mm) * 0.65)[0]
    if len(fixed_nodes) < 8:
        x_min = float(np.min(coords[:, 0]))
        x_thresh = x_min + max(6.0, float(req.root_end.outer_diameter_mm) * 0.25)
        fixed_nodes = np.where(coords[:, 0] <= x_thresh)[0]
        if len(fixed_nodes) < 8:
            fixed_nodes = np.argsort(coords[:, 0])[: max(12, n_nodes // 40)]
        notes.append(
            f"Empotramiento aplicado por respaldo en los {len(fixed_nodes)} nodos del extremo raíz."
        )
    else:
        notes.append(
            f"Empotramiento real (u=0) en {len(fixed_nodes)} nodos del alojamiento raíz."
        )

    # Tip distributed load nodes
    r_tip = np.hypot(coords[:, 0] - float(req.length_mm), coords[:, 1])
    load_nodes = np.where(r_tip <= float(req.tip_end.bore_diameter_mm) * 0.65)[0]
    load_nodes = np.setdiff1d(load_nodes, fixed_nodes)
    if len(load_nodes) < 4:
        x_max = float(np.max(coords[:, 0]))
        x_thresh = x_max - max(6.0, float(req.tip_end.outer_diameter_mm) * 0.25)
        load_nodes = np.setdiff1d(np.where(coords[:, 0] >= x_thresh)[0], fixed_nodes)
        if len(load_nodes) < 4:
            candidates = np.setdiff1d(np.argsort(coords[:, 0])[::-1], fixed_nodes)
            load_nodes = candidates[: max(8, n_nodes // 50)]

    load_axis = 2 if load_direction == "z_outofplane" else 1
    axis_label = "-Z (fuera del plano)" if load_axis == 2 else "-Y (en el plano)"
    notes.append(
        f"Carga de {req.tip_load_n:.1f} N distribuida en {len(load_nodes)} nodos de la punta en dirección {axis_label}."
    )

    return _solve_tet4_and_export(
        coords=coords,
        tets=tets,
        tris=tris,
        fixed_nodes=fixed_nodes,
        load_nodes=load_nodes,
        load_n=float(req.tip_load_n),
        load_axis=load_axis,
        youngs_eff_mpa=youngs_eff_mpa,
        yield_eff_mpa=yield_eff_mpa,
        youngs_xy_mpa=float(material["youngs_xy_mpa"]),
        youngs_z_mpa=float(material["youngs_z_mpa"]),
        density_g_cm3=float(material.get("density_g_cm3", 1.29)),
        print_orientation=req.print_orientation,
        target_safety_factor=float(req.target_safety_factor),
        max_deflection_mm=float(req.max_deflection_mm),
        mesh_size_mm=mesh_size_mm,
        deform_scale=deform_scale,
        output_glb_path=output_glb_path,
        design_hash=design_hash,
        variant_id=variant_id,
        notes=notes,
        t0=t0,
    )


def run_mount_fea(
    step_path: Path,
    output_glb_path: Path,
    req: MountDesignRequest,
    design_hash: str,
    variant_id: MountVariantKind,
    material: dict,
    mesh_size_mm: float = 5.5,
    load_direction: str = "y_inplane",
    deform_scale: float = 0.0,
) -> FEAResult:
    """
    Run a real 3D Tet4 Linear Elasticity FEM analysis on an L-bracket mechatronic
    mount STEP file:
    - Clamps bottom face of base plate (z <= min_z + 1.5 mm).
    - Distributes radial_load_n on the upper zone of the vertical motor faceplate
      (y <= min_y + wall_thickness_mm + 1.5 mm and z >= min_z + 0.5*(max_z - min_z)).
    """
    t0 = time.perf_counter()
    step_path = Path(step_path)
    output_glb_path = Path(output_glb_path)

    coords, tets, tris = _mesh_step_tet4(step_path, mesh_size_mm=mesh_size_mm)
    if len(tris) == 0:
        tris = _extract_boundary_tris_from_tets(tets)

    n_nodes = len(coords)

    if req.print_orientation == "z_vertical":
        youngs_eff_mpa = float(material["youngs_z_mpa"])
        yield_eff_mpa = float(material["yield_z_mpa"])
    else:
        youngs_eff_mpa = float(material["youngs_xy_mpa"])
        yield_eff_mpa = float(material["yield_xy_mpa"])

    notes: list[str] = []
    min_z = float(np.min(coords[:, 2]))
    max_z = float(np.max(coords[:, 2]))
    min_y = float(np.min(coords[:, 1]))

    # Clamp bottom face of base plate
    fixed_nodes = np.where(coords[:, 2] <= min_z + 1.5)[0]
    if len(fixed_nodes) < 8:
        fixed_nodes = np.argsort(coords[:, 2])[: max(12, n_nodes // 30)]
        notes.append(
            f"Empotramiento de base aplicado por respaldo en {len(fixed_nodes)} nodos inferiores."
        )
    else:
        notes.append(
            f"Empotramiento real (u=0) en {len(fixed_nodes)} nodos de la cara inferior de la placa base."
        )

    # Load upper half of vertical motor plate
    z_mid = min_z + 0.5 * (max_z - min_z)
    y_plate_max = min_y + float(req.wall_thickness_mm) + 1.5
    load_mask = (coords[:, 1] <= y_plate_max) & (coords[:, 2] >= z_mid)
    load_nodes = np.setdiff1d(np.where(load_mask)[0], fixed_nodes)
    if len(load_nodes) < 4:
        candidates = np.setdiff1d(np.argsort(coords[:, 2])[::-1], fixed_nodes)
        load_nodes = candidates[: max(8, n_nodes // 40)]

    load_axis = 2 if load_direction == "z_outofplane" else 1
    axis_label = "-Z (vertical / peso motor)" if load_axis == 2 else "-Y (tensión radial)"
    notes.append(
        f"Carga radial de {req.radial_load_n:.1f} N distribuida en {len(load_nodes)} nodos de la cara del motor en dirección {axis_label}."
    )

    return _solve_tet4_and_export(
        coords=coords,
        tets=tets,
        tris=tris,
        fixed_nodes=fixed_nodes,
        load_nodes=load_nodes,
        load_n=float(req.radial_load_n),
        load_axis=load_axis,
        youngs_eff_mpa=youngs_eff_mpa,
        yield_eff_mpa=yield_eff_mpa,
        youngs_xy_mpa=float(material["youngs_xy_mpa"]),
        youngs_z_mpa=float(material["youngs_z_mpa"]),
        density_g_cm3=float(material.get("density_g_cm3", 1.29)),
        print_orientation=req.print_orientation,
        target_safety_factor=float(req.target_safety_factor),
        max_deflection_mm=float(req.max_deflection_mm),
        mesh_size_mm=mesh_size_mm,
        deform_scale=deform_scale,
        output_glb_path=output_glb_path,
        design_hash=design_hash,
        variant_id=variant_id,
        notes=notes,
        t0=t0,
    )


def run_gear_fea(
    step_path: Path,
    output_glb_path: Path,
    req: GearDesignRequest,
    design_hash: str,
    variant_id: GearVariantKind,
    material: dict,
    mesh_size_mm: float = 4.5,
    load_direction: str = "y_inplane",
    deform_scale: float = 0.0,
) -> FEAResult:
    """
    Run a real 3D Tet4 Linear Elasticity FEM analysis on an involute gear or GT2
    pulley STEP file:
    - Clamps central shaft bore nodes (r = hypot(x, y) <= bore_diameter_mm * 0.65).
    - Applies tangential tooth load F_t = 2000 * torque_nm / d_p (in -Y for y_inplane
      or -Z for z_outofplane) on the right outer sector (x >= 0.78 * r_max).
    """
    t0 = time.perf_counter()
    step_path = Path(step_path)
    output_glb_path = Path(output_glb_path)

    coords, tets, tris = _mesh_step_tet4(step_path, mesh_size_mm=mesh_size_mm)
    if len(tris) == 0:
        tris = _extract_boundary_tris_from_tets(tets)

    n_nodes = len(coords)

    if req.print_orientation == "z_vertical":
        youngs_eff_mpa = float(material["youngs_z_mpa"])
        yield_eff_mpa = float(material["yield_z_mpa"])
    else:
        youngs_eff_mpa = float(material["youngs_xy_mpa"])
        yield_eff_mpa = float(material["yield_xy_mpa"])

    d_p, _ = compute_gear_diameters(req)
    f_t_n = (2000.0 * float(req.torque_nm)) / max(d_p, 1.0)
    max_allowable_deflection = (
        max(0.20, 0.12 * float(req.module_mm))
        if req.gear_kind == "spur_involute"
        else 0.18
    )

    notes: list[str] = []
    radii = np.hypot(coords[:, 0], coords[:, 1])
    r_max = float(np.max(radii))

    # Clamp central shaft bore nodes
    fixed_nodes = np.where(radii <= float(req.bore_diameter_mm) * 0.65)[0]
    if len(fixed_nodes) < 8:
        fixed_nodes = np.argsort(radii)[: max(12, n_nodes // 30)]
        notes.append(
            f"Empotramiento del eje aplicado por respaldo en {len(fixed_nodes)} nodos interiores."
        )
    else:
        notes.append(
            f"Empotramiento real (u=0) en {len(fixed_nodes)} nodos del agujero central del eje."
        )

    # Apply tangential force on right outer tooth sector (x >= 0.78 * r_max)
    load_nodes = np.setdiff1d(np.where(coords[:, 0] >= 0.78 * r_max)[0], fixed_nodes)
    if len(load_nodes) < 4:
        candidates = np.setdiff1d(np.argsort(coords[:, 0])[::-1], fixed_nodes)
        load_nodes = candidates[: max(8, n_nodes // 40)]

    load_axis = 2 if load_direction == "z_outofplane" else 1
    axis_label = "-Z (axial)" if load_axis == 2 else "-Y (tangencial en diente)"
    notes.append(
        f"Fuerza tangencial Ft={f_t_n:.1f} N (τ={req.torque_nm:.2f} N·m, dp={d_p:.1f} mm) aplicada en {len(load_nodes)} nodos del sector exterior en dirección {axis_label}."
    )

    return _solve_tet4_and_export(
        coords=coords,
        tets=tets,
        tris=tris,
        fixed_nodes=fixed_nodes,
        load_nodes=load_nodes,
        load_n=f_t_n,
        load_axis=load_axis,
        youngs_eff_mpa=youngs_eff_mpa,
        yield_eff_mpa=yield_eff_mpa,
        youngs_xy_mpa=float(material["youngs_xy_mpa"]),
        youngs_z_mpa=float(material["youngs_z_mpa"]),
        density_g_cm3=float(material.get("density_g_cm3", 1.29)),
        print_orientation=req.print_orientation,
        target_safety_factor=float(req.target_safety_factor),
        max_deflection_mm=max_allowable_deflection,
        mesh_size_mm=mesh_size_mm,
        deform_scale=deform_scale,
        output_glb_path=output_glb_path,
        design_hash=design_hash,
        variant_id=variant_id,
        notes=notes,
        t0=t0,
    )
