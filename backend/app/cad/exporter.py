"""Export build123d CAD variants to STEP, STL, and GLB with deterministic hash caching."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

import trimesh
from build123d import export_step, export_stl

from ..schemas import LinkDesignRequest, VariantKind

# Project root: .../roboforge/
PROJECT_ROOT = Path(__file__).resolve().parents[3]
EXPORTS_ROOT = PROJECT_ROOT / "exports"

# Distinct metallic/engineering PBR appearance per variant
VARIANT_PBR_STYLES: dict[VariantKind, dict[str, Any]] = {
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
        "baseColorFactor": [228, 168, 66, 255],  # Industrial amber-gold titanium
        "metallicFactor": 0.58,
        "roughnessFactor": 0.35,
    },
    "hollow_cobot": {
        "baseColorFactor": [168, 152, 228, 255],  # Satin cobot slate-violet
        "metallicFactor": 0.52,
        "roughnessFactor": 0.28,
    },
}


def compute_design_hash(req: LinkDesignRequest) -> str:
    """Compute a deterministic 16-char SHA-256 hex digest for a LinkDesignRequest."""
    canonical = json.dumps(req.model_dump(), sort_keys=True, separators=(",", ":"))
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

        # Cache check: only re-export if any file is missing or empty
        if not (step_path.exists() and step_path.stat().st_size > 0):
            export_step(part, str(step_path))

        if not (stl_path.exists() and stl_path.stat().st_size > 0):
            export_stl(part, str(stl_path), tolerance=0.08, angular_tolerance=0.15)

        if not (glb_path.exists() and glb_path.stat().st_size > 0):
            _stl_to_glb(stl_path, glb_path, variant_id)

        exported_paths[variant_id] = {
            "step": step_path,
            "stl": stl_path,
            "glb": glb_path,
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
    )
    for vid in expected_variants:
        if vid not in meta:
            return None
        for ext in ("step", "stl", "glb"):
            fp = out_dir / f"{vid}.{ext}"
            if not (fp.exists() and fp.stat().st_size > 0):
                return None

    return meta
