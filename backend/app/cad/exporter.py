"""Export build123d CAD variants to STEP, STL, and GLB with deterministic hash caching."""

from __future__ import annotations

import hashlib
import json
import math
import sys
import tempfile
import zipfile
from pathlib import Path
from typing import Any

import numpy as np
import trimesh
from build123d import Compound, Part, export_step, export_stl

from ..schemas import (
    GearDesignRequest,
    GearVariantKind,
    LinkDesignRequest,
    MountDesignRequest,
    MountVariantKind,
    VariantKind,
)
from .threemf_exporter import compute_modifier_box_for_mesh, export_smart_3mf

# Project root: handles both source checkout and PyInstaller frozen bundle
if getattr(sys, "frozen", False) and hasattr(sys, "_MEIPASS"):
    PROJECT_ROOT = Path(sys._MEIPASS)
    EXPORTS_ROOT = Path(sys.executable).resolve().parent / "exports"
else:
    PROJECT_ROOT = Path(__file__).resolve().parents[3]
    EXPORTS_ROOT = PROJECT_ROOT / "exports"

# Distinct metallic/engineering PBR appearance per variant
VARIANT_PBR_STYLES: dict[str, dict[str, Any]] = {
    "tapered_solid": {
        "baseColorFactor": [132, 162, 198, 255],  # Anodized titanium steel-blue
        "metallicFactor": 0.65,
        "roughnessFactor": 0.34,
    },
    "organic_ibeam": {
        "baseColorFactor": [64, 184, 174, 255],  # Aerospace cyan-teal alloy
        "metallicFactor": 0.60,
        "roughnessFactor": 0.32,
    },
    "rounded_truss": {
        "baseColorFactor": [224, 134, 68, 255],  # Industrial bronze-copper alloy
        "metallicFactor": 0.58,
        "roughnessFactor": 0.35,
    },
    "hollow_cobot": {
        "baseColorFactor": [168, 152, 228, 255],  # Satin cobot slate-violet
        "metallicFactor": 0.52,
        "roughnessFactor": 0.28,
    },
    "organic_topopt": {
        "baseColorFactor": [240, 198, 72, 255],  # Technical titanium-gold SIMP alloy
        "metallicFactor": 0.68,
        "roughnessFactor": 0.28,
    },
    # Family 2: Mechatronic Mounts
    "mount_standard": {
        "baseColorFactor": [126, 154, 192, 255],  # Anodized slate aluminum
        "metallicFactor": 0.64,
        "roughnessFactor": 0.34,
    },
    "mount_gusseted": {
        "baseColorFactor": [62, 176, 206, 255],  # Precision CNC cyan-steel
        "metallicFactor": 0.66,
        "roughnessFactor": 0.30,
    },
    "mount_lightweight": {
        "baseColorFactor": [82, 204, 146, 255],  # Bionic anodized emerald-titanium
        "metallicFactor": 0.62,
        "roughnessFactor": 0.30,
    },
    # Family 3: Transmission (Gears & GT2 Pulleys)
    "gear_solid": {
        "baseColorFactor": [142, 164, 194, 255],  # Hardened steel-blue
        "metallicFactor": 0.72,
        "roughnessFactor": 0.30,
    },
    "gear_webbed": {
        "baseColorFactor": [72, 188, 212, 255],  # Machined cyan-alloy
        "metallicFactor": 0.68,
        "roughnessFactor": 0.28,
    },
    "gear_spoked": {
        "baseColorFactor": [236, 178, 72, 255],  # Precision brass-titanium
        "metallicFactor": 0.70,
        "roughnessFactor": 0.26,
    },
}


def compute_design_hash(req: LinkDesignRequest) -> str:
    """Compute a deterministic 16-char SHA-256 hex digest for a LinkDesignRequest."""
    canonical = json.dumps(req.model_dump(), sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:16]


def compute_mount_design_hash(req: MountDesignRequest) -> str:
    """Compute a deterministic 16-char SHA-256 hex digest for a MountDesignRequest."""
    canonical = "mount:" + json.dumps(req.model_dump(), sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:16]


def compute_gear_design_hash(req: GearDesignRequest) -> str:
    """Compute a deterministic 16-char SHA-256 hex digest for a GearDesignRequest."""
    canonical = "gear:" + json.dumps(req.model_dump(), sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:16]


def get_export_dir(design_hash: str, exports_root: Path | None = None) -> Path:
    root = exports_root or EXPORTS_ROOT
    out_dir = root / design_hash
    out_dir.mkdir(parents=True, exist_ok=True)
    return out_dir


# In-memory cache of polished studio meshes (vertices, faces, uvs) to avoid repeated temp STL I/O
_STUDIO_MESH_CACHE: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]] = {}


def compute_box_projected_uvs(mesh: trimesh.Trimesh, scale_mm: float = 50.0) -> np.ndarray:
    """Compute physical-scale triplanar / box-projected UVs (mm / scale_mm) in pure NumPy (<2 ms)."""
    verts = np.asarray(mesh.vertices, dtype=np.float32)
    if len(verts) == 0:
        return np.zeros((0, 2), dtype=np.float32)
    normals = np.abs(np.asarray(mesh.vertex_normals, dtype=np.float32))
    if len(normals) != len(verts):
        normals = np.tile(np.array([[0.0, 0.0, 1.0]], dtype=np.float32), (len(verts), 1))
    dom = np.argmax(normals, axis=1)
    inv_scale = np.float32(1.0 / max(1.0, float(scale_mm)))
    uvs = np.empty((len(verts), 2), dtype=np.float32)
    mx = dom == 0
    my = dom == 1
    mz = dom == 2
    uvs[mx] = verts[mx][:, [1, 2]] * inv_scale
    uvs[my] = verts[my][:, [0, 2]] * inv_scale
    uvs[mz] = verts[mz][:, [0, 1]] * inv_scale
    return uvs


def infer_node_semantics(
    node_name: str,
    printable_names: set[str] | None = None,
    material_hint: str | None = None,
) -> dict[str, Any]:
    """Infer semantic glTF extras (role, printable, material_hint) for a CAD assembly node."""
    low = (node_name or "").lower()
    if any(k in low for k in ("motor", "servo")):
        role, default_mat = "actuator", "anodized_dark"
    elif "bearing" in low:
        role, default_mat = "bearing", "chrome_steel"
    elif any(k in low for k in ("fastener", "screw", "bolt", "standoff")):
        role, default_mat = "fastener", "stainless_steel"
    elif "horn" in low:
        role, default_mat = "servo_horn", "anodized_red"
    elif "torque_sensor" in low:
        role, default_mat = "torque_sensor", "copper_gauge"
    elif "em_brake" in low:
        role, default_mat = "em_brake", "dark_steel"
    elif "iso9409" in low or "flange" in low:
        role, default_mat = "tool_flange", "machined_steel"
    elif any(k in low for k in ("controller", "pcb", "mcu")):
        role, default_mat = "electronics", "enclosure_polymer"
    elif any(k in low for k in ("gear", "cycloidal", "planetary", "belt", "capstan", "bevel")):
        role, default_mat = "transmission", material_hint or "machined_alloy"
    elif any(k in low for k in ("propeller", "blade", "hub")):
        role, default_mat = "propulsion", material_hint or "carbon_composite"
    elif any(k in low for k in ("gripper", "nozzle", "forceps")):
        role, default_mat = "end_effector", material_hint or "engineering_polymer"
    else:
        role, default_mat = "structural", material_hint or "engineering_polymer"

    is_printable = (node_name in printable_names) if printable_names is not None else (role in ("structural", "propulsion", "end_effector", "transmission"))
    return {
        "role": role,
        "printable": bool(is_printable),
        "material_hint": material_hint if is_printable and material_hint else default_mat,
    }


def apply_studio_mesh_polish(
    mesh: trimesh.Trimesh,
    rgba: list[int],
    metallic: float,
    roughness: float,
    name: str,
    *,
    role: str | None = None,
    printable: bool | None = None,
    material_hint: str | None = None,
    crease_angle_deg: float = 32.0,
    uv_scale_mm: float = 50.0,
) -> trimesh.Trimesh:
    """Apply KeyShot/Shapr3D studio polish: crease-split normals (32°), physical box UVs, PBR material, and glTF extras."""
    mesh.fix_normals()
    try:
        mesh = trimesh.graph.smooth_shade(mesh, angle=math.radians(crease_angle_deg))
    except Exception:
        pass
    # Strip bulky smooth_shade component map before writing glTF extras
    mesh.metadata.pop("original_components", None)
    uvs = compute_box_projected_uvs(mesh, scale_mm=uv_scale_mm)
    mat = trimesh.visual.material.PBRMaterial(
        name=name,
        baseColorFactor=rgba,
        metallicFactor=metallic,
        roughnessFactor=roughness,
    )
    mesh.visual = trimesh.visual.TextureVisuals(uv=uvs, material=mat)
    if role is not None:
        mesh.metadata["role"] = role
    if printable is not None:
        mesh.metadata["printable"] = bool(printable)
    if material_hint is not None:
        mesh.metadata["material_hint"] = material_hint
    return mesh


def part_to_studio_trimesh(
    part: Part | Compound,
    rgba: list[int],
    metallic: float,
    roughness: float,
    name: str,
    *,
    role: str | None = None,
    printable: bool | None = None,
    material_hint: str | None = None,
    cache_key: str | None = None,
    out_raw_holder: list[trimesh.Trimesh] | None = None,
) -> trimesh.Trimesh:
    """Tessellate a build123d solid at studio quality (tol=0.09, ang=0.14) with smooth/sharp crease normals,
    box-projected physical UVs, PBR material, semantic glTF extras, and optional in-memory mesh caching.
    If out_raw_holder is provided, appends the pre-crease-split welded mesh for watertight DFM auditing.
    """
    if cache_key and cache_key in _STUDIO_MESH_CACHE and out_raw_holder is None:
        verts, faces, uvs = _STUDIO_MESH_CACHE[cache_key]
        mesh = trimesh.Trimesh(vertices=verts.copy(), faces=faces.copy(), process=False)
        mat = trimesh.visual.material.PBRMaterial(
            name=name,
            baseColorFactor=rgba,
            metallicFactor=metallic,
            roughnessFactor=roughness,
        )
        mesh.visual = trimesh.visual.TextureVisuals(uv=uvs.copy(), material=mat)
        if role is not None:
            mesh.metadata["role"] = role
        if printable is not None:
            mesh.metadata["printable"] = bool(printable)
        if material_hint is not None:
            mesh.metadata["material_hint"] = material_hint
        return mesh

    with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as tmp:
        tmp_path = Path(tmp.name)
    try:
        export_stl(part, str(tmp_path), tolerance=0.09, angular_tolerance=0.14)
        loaded = trimesh.load(str(tmp_path), force="mesh")
        mesh = loaded.dump(concatenate=True) if isinstance(loaded, trimesh.Scene) else loaded
        if not isinstance(mesh, trimesh.Trimesh):
            raise ValueError(f"Expected Trimesh for {name}")
        if out_raw_holder is not None:
            raw_copy = mesh.copy()
            raw_copy.fix_normals()
            out_raw_holder.append(raw_copy)
        mesh = apply_studio_mesh_polish(
            mesh,
            rgba=rgba,
            metallic=metallic,
            roughness=roughness,
            name=name,
            role=role,
            printable=printable,
            material_hint=material_hint,
        )
        if cache_key:
            if len(_STUDIO_MESH_CACHE) > 128:
                _STUDIO_MESH_CACHE.clear()
            uv_arr = getattr(mesh.visual, "uv", None)
            if uv_arr is not None:
                _STUDIO_MESH_CACHE[cache_key] = (
                    np.asarray(mesh.vertices, dtype=np.float32).copy(),
                    np.asarray(mesh.faces, dtype=np.int64).copy(),
                    np.asarray(uv_arr, dtype=np.float32).copy(),
                )
        return mesh
    finally:
        if tmp_path.exists():
            try:
                tmp_path.unlink()
            except OSError:
                pass


def _stl_to_glb(stl_path: Path, glb_path: Path, variant_id: VariantKind) -> None:
    """Convert an STL mesh to a studio-polished PBR GLB file with crease normals and physical UVs."""
    loaded = trimesh.load(str(stl_path), force="mesh")
    if isinstance(loaded, trimesh.Scene):
        mesh = loaded.dump(concatenate=True)
    else:
        mesh = loaded

    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"Expected Trimesh from {stl_path}, got {type(mesh)}")

    style = VARIANT_PBR_STYLES.get(
        variant_id,
        {
            "baseColorFactor": [160, 175, 195, 255],
            "metallicFactor": 0.6,
            "roughnessFactor": 0.35,
        },
    )
    mesh = apply_studio_mesh_polish(
        mesh,
        rgba=style["baseColorFactor"],
        metallic=style["metallicFactor"],
        roughness=style["roughnessFactor"],
        name=f"roboforge_{variant_id}",
        role="structural",
        printable=True,
        material_hint=str(variant_id),
    )
    mesh.export(str(glb_path), file_type="glb")


def export_variants(
    design_hash: str,
    variants_data: dict[VariantKind, dict[str, Any]],
    exports_root: Path | None = None,
) -> dict[VariantKind, dict[str, Path]]:
    """Export all generated CAD variants to .step, .stl, and .glb under exports/<hash>/."""
    out_dir = get_export_dir(design_hash, exports_root=exports_root)
    exported_paths: dict[VariantKind, dict[str, Path]] = {}

    for variant_id, data in variants_data.items():
        part = data["part"]
        step_path = out_dir / f"{variant_id}.step"
        stl_path = out_dir / f"{variant_id}.stl"
        glb_path = out_dir / f"{variant_id}.glb"
        threemf_path = out_dir / f"{variant_id}.3mf"

        # Cache check: only re-export if any file is missing or empty
        if not (step_path.exists() and step_path.stat().st_size > 0):
            export_step(part, str(step_path))

        if not (stl_path.exists() and stl_path.stat().st_size > 0):
            export_stl(part, str(stl_path), tolerance=0.08, angular_tolerance=0.15)

        if not (glb_path.exists() and glb_path.stat().st_size > 0):
            _stl_to_glb(stl_path, glb_path, variant_id)

        if not (threemf_path.exists() and threemf_path.stat().st_size > 0):
            mod_center, mod_size = compute_modifier_box_for_mesh(stl_path, str(variant_id))
            export_smart_3mf(
                stl_path=stl_path,
                output_3mf_path=threemf_path,
                part_name=f"RoboForge_{variant_id}",
                modifier_box_center=mod_center,
                modifier_box_size=mod_size,
            )

        exported_paths[variant_id] = {
            "step": step_path,
            "stl": stl_path,
            "glb": glb_path,
            "3mf": threemf_path,
        }

    # Persist lightweight metadata cache for instant repeated /api/generate calls
    meta_path = out_dir / "metadata.json"
    meta_payload = {
        variant_id: {
            "name": data["name"],
            "tagline": data["tagline"],
            "volume_mm3": data["volume_mm3"],
            "is_valid": data["is_valid"],
        }
        for variant_id, data in variants_data.items()
    }
    meta_path.write_text(json.dumps(meta_payload, indent=2, ensure_ascii=False), encoding="utf-8")

    return exported_paths


def load_cached_metadata(
    design_hash: str,
    exports_root: Path | None = None,
) -> dict[VariantKind, dict[str, Any]] | None:
    """Return cached variant metadata if all 4 variants and all 12 export files exist."""
    root = exports_root or EXPORTS_ROOT
    out_dir = root / design_hash
    meta_path = out_dir / "metadata.json"
    if not meta_path.exists():
        return None

    try:
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
    except Exception:
        return None

    expected_variants: tuple[VariantKind, ...] = (
        "tapered_solid",
        "organic_ibeam",
        "rounded_truss",
        "hollow_cobot",
        "organic_topopt",
    )
    for vid in expected_variants:
        if vid not in meta:
            return None
        for ext in ("step", "stl", "glb", "3mf"):
            fp = out_dir / f"{vid}.{ext}"
            if not (fp.exists() and fp.stat().st_size > 0):
                return None

    return meta


def export_mount_variants(
    design_hash: str,
    variants_data: dict[MountVariantKind, dict[str, Any]],
    exports_root: Path | None = None,
) -> dict[MountVariantKind, dict[str, Path]]:
    """Export all generated mount CAD variants to .step, .stl, and .glb."""
    return export_variants(design_hash, variants_data, exports_root=exports_root)  # type: ignore[arg-type,return-value]


def load_cached_mount_metadata(
    design_hash: str,
    exports_root: Path | None = None,
) -> dict[MountVariantKind, dict[str, Any]] | None:
    """Return cached mount metadata if all 3 mount variants and all 9 export files exist."""
    root = exports_root or EXPORTS_ROOT
    out_dir = root / design_hash
    meta_path = out_dir / "metadata.json"
    if not meta_path.exists():
        return None

    try:
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
    except Exception:
        return None

    expected_variants: tuple[MountVariantKind, ...] = (
        "mount_standard",
        "mount_gusseted",
        "mount_lightweight",
    )
    for vid in expected_variants:
        if vid not in meta:
            return None
        for ext in ("step", "stl", "glb", "3mf"):
            fp = out_dir / f"{vid}.{ext}"
            if not (fp.exists() and fp.stat().st_size > 0):
                return None

    return meta


def export_gear_variants(
    design_hash: str,
    variants_data: dict[GearVariantKind, dict[str, Any]],
    exports_root: Path | None = None,
) -> dict[GearVariantKind, dict[str, Path]]:
    """Export all generated gear/pulley CAD variants to .step, .stl, and .glb."""
    return export_variants(design_hash, variants_data, exports_root=exports_root)  # type: ignore[arg-type,return-value]


def load_cached_gear_metadata(
    design_hash: str,
    exports_root: Path | None = None,
) -> dict[GearVariantKind, dict[str, Any]] | None:
    """Return cached gear metadata if all 3 gear variants and all 9 export files exist."""
    root = exports_root or EXPORTS_ROOT
    out_dir = root / design_hash
    meta_path = out_dir / "metadata.json"
    if not meta_path.exists():
        return None

    try:
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
    except Exception:
        return None

    expected_variants: tuple[GearVariantKind, ...] = (
        "gear_solid",
        "gear_webbed",
        "gear_spoked",
    )
    for vid in expected_variants:
        if vid not in meta:
            return None
        for ext in ("step", "stl", "glb", "3mf"):
            fp = out_dir / f"{vid}.{ext}"
            if not (fp.exists() and fp.stat().st_size > 0):
                return None

    return meta


def build_arm_manufacturing_zip(
    out_dir: Path,
    included_files: list[Path],
    zip_filename: str = "arm_manufacturing_pack.zip",
) -> Path:
    """Package individual part STEP/3MF files, full arm STEP, URDF, MuJoCo XML, electronics & BOM into a clean ZIP."""
    out_dir.mkdir(parents=True, exist_ok=True)
    zip_path = out_dir / zip_filename
    seen_names: set[str] = set()

    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        included_list: list[str] = []
        for fp in included_files:
            if not (fp.exists() and fp.is_file() and fp.stat().st_size > 0):
                continue
            # Avoid nesting the separate 3D-parts ZIP inside the manufacturing ZIP
            if fp.name == "arm_stl_parts_pack.zip" or fp.name == zip_filename:
                continue
            arcname = fp.name
            if arcname in seen_names:
                continue
            seen_names.add(arcname)
            zf.write(fp, arcname=arcname)
            included_list.append(arcname)

        readme_lines = [
            "# Kit de Manufactura e Ingeniería — RoboForge Studio Pro v7.0",
            "",
            "Este paquete contiene los archivos CAD editables (`.STEP`), simulación (`.URDF`/`.XML`),",
            "electrónica (`.SVG`/`.YML`/`.INO`/`.JSON`), lista de compras (`BOM .CSV`) y guía de impresión.",
            "",
            "*(Para imprimir pieza por pieza en SatelLite / Chitubox / Cura, usa también el paquete dedicado `arm_stl_parts_pack.zip`.)*",
            "",
            "## Archivos incluidos",
            *[f"- `{name}`" for name in sorted(included_list)],
            "",
        ]
        zf.writestr("00_LEEME_KIT_MANUFACTURA.md", "\n".join(readme_lines))

    return zip_path
