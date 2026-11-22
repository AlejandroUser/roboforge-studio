"""Export build123d CAD variants to STEP, STL, and GLB with deterministic hash caching."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

import trimesh
from build123d import export_step, export_stl

from ..schemas import (
    GearDesignRequest,
    GearVariantKind,
    LinkDesignRequest,
    MountDesignRequest,
    MountVariantKind,
    VariantKind,
)
from .threemf_exporter import compute_modifier_box_for_mesh, export_smart_3mf

# Project root: .../roboforge/
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


def _stl_to_glb(stl_path: Path, glb_path: Path, variant_id: VariantKind) -> None:
    """Convert an STL mesh to a styled PBR GLB file using trimesh."""
    loaded = trimesh.load(str(stl_path), force="mesh")
    if isinstance(loaded, trimesh.Scene):
        mesh = loaded.dump(concatenate=True)
    else:
        mesh = loaded

    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"Expected Trimesh from {stl_path}, got {type(mesh)}")

    # Ensure clean outward-facing normals
    mesh.fix_normals()

    style = VARIANT_PBR_STYLES.get(
        variant_id,
        {
            "baseColorFactor": [160, 175, 195, 255],
            "metallicFactor": 0.6,
            "roughnessFactor": 0.35,
        },
    )
    material = trimesh.visual.material.PBRMaterial(
        name=f"roboforge_{variant_id}",
        baseColorFactor=style["baseColorFactor"],
        metallicFactor=style["metallicFactor"],
        roughnessFactor=style["roughnessFactor"],
    )
    mesh.visual = trimesh.visual.TextureVisuals(material=material)
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
