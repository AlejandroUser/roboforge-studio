"""Smart multi-object .3MF exporter with 100% infill structural modifier zones."""

from __future__ import annotations

import xml.sax.saxutils as saxutils
import zipfile
from pathlib import Path

import numpy as np
import trimesh

CONTENT_TYPES_XML = """<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml" />
  <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml" />
</Types>
"""

RELS_XML = """<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel" />
</Relationships>
"""


def _load_single_trimesh(stl_path: Path) -> trimesh.Trimesh:
    loaded = trimesh.load(str(stl_path), force="mesh")
    if isinstance(loaded, trimesh.Scene):
        mesh = loaded.dump(concatenate=True)
    else:
        mesh = loaded
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"Expected Trimesh from {stl_path}, got {type(mesh)}")
    return mesh


def _mesh_to_3mf_object_xml(object_id: int, name: str, mesh: trimesh.Trimesh) -> str:
    """Serialize a Trimesh into a 3MF <object> XML block."""
    safe_name = saxutils.escape(name, {'"': "&quot;"})
    verts = np.asarray(mesh.vertices, dtype=np.float64)
    faces = np.asarray(mesh.faces, dtype=np.int64)

    lines = [
        f'    <object id="{object_id}" type="model" name="{safe_name}">',
        "      <mesh>",
        "        <vertices>",
    ]
    for v in verts:
        lines.append(f'          <vertex x="{v[0]:.4f}" y="{v[1]:.4f}" z="{v[2]:.4f}" />')
    lines.append("        </vertices>")
    lines.append("        <triangles>")
    for f in faces:
        lines.append(f'          <triangle v1="{int(f[0])}" v2="{int(f[1])}" v3="{int(f[2])}" />')
    lines.extend(
        [
            "        </triangles>",
            "      </mesh>",
            "    </object>",
        ]
    )
    return "\n".join(lines)


def compute_modifier_box_for_mesh(
    stl_path: Path,
    variant_id: str,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Compute the structural 100%-infill modifier box (center, size) for a given variant."""
    mesh = _load_single_trimesh(stl_path)
    bmin, bmax = mesh.bounds
    size = np.maximum(bmax - bmin, 1.0)
    center = 0.5 * (bmin + bmax)

    if variant_id.startswith("mount_"):
        # Critical 90-degree corner junction between base plate and vertical motor faceplate
        box_size = (
            float(size[0] + 1.0),
            float(max(10.0, size[1] * 0.35)),
            float(max(10.0, size[2] * 0.40)),
        )
        box_center = (
            float(center[0]),
            float(bmin[1] + box_size[1] / 2.0),
            float(bmin[2] + box_size[2] / 2.0),
        )
    elif variant_id.startswith("gear_"):
        # Critical central shaft hub & keyway/D-shaft torque zone
        hub_xy = float(max(10.0, min(size[0], size[1]) * 0.45))
        box_size = (hub_xy, hub_xy, float(size[2] + 1.0))
        box_center = (float(center[0]), float(center[1]), float(center[2]))
    else:
        # Robotic arm links: root boss & neck maximum bending moment zone
        root_len = float(max(18.0, size[0] * 0.28))
        box_size = (root_len, float(size[1] + 1.0), float(size[2] + 1.0))
        box_center = (
            float(bmin[0] + root_len / 2.0),
            float(center[1]),
            float(center[2]),
        )

    return box_center, box_size


def export_smart_3mf(
    stl_path: Path,
    output_3mf_path: Path,
    part_name: str,
    modifier_box_center: tuple[float, float, float],
    modifier_box_size: tuple[float, float, float],
    modifier_name: str = "RoboForge_Refuerzo_100pct_Modifier",
) -> Path:
    """Write a valid multi-object .3MF OPC ZIP containing the part mesh + 100% infill modifier box."""
    part_mesh = _load_single_trimesh(stl_path)

    mod_mesh = trimesh.creation.box(extents=modifier_box_size)
    mod_mesh.apply_translation(modifier_box_center)

    obj1_xml = _mesh_to_3mf_object_xml(1, part_name, part_mesh)
    obj2_xml = _mesh_to_3mf_object_xml(2, modifier_name, mod_mesh)

    model_xml = (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<model unit="millimeter" xml:lang="es-ES" '
        'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">\n'
        '  <metadata name="Application">RoboForge Studio</metadata>\n'
        '  <metadata name="Description">Pieza CAD con zona modificadora de refuerzo 100% infill para OrcaSlicer/Bambu Studio/PrusaSlicer</metadata>\n'
        "  <resources>\n"
        f"{obj1_xml}\n"
        f"{obj2_xml}\n"
        "  </resources>\n"
        "  <build>\n"
        '    <item objectid="1" />\n'
        '    <item objectid="2" />\n'
        "  </build>\n"
        "</model>\n"
    )

    output_3mf_path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output_3mf_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("[Content_Types].xml", CONTENT_TYPES_XML)
        zf.writestr("_rels/.rels", RELS_XML)
        zf.writestr("3D/3dmodel.model", model_xml)

    return output_3mf_path
