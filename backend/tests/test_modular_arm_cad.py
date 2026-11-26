"""Pytest suite for v3.0 Modular Arm CAD Assembler, semantic GLB nodes, and Manufacturing .ZIP."""

from __future__ import annotations

import time
import zipfile
from pathlib import Path

import trimesh

from backend.app.cad.arm_assembly import build_modular_arm_assembly, compute_modular_arm_hash
from backend.app.physics.mechanics import load_materials
from backend.app.schemas import (
    GearDesignRequest,
    LinkDesignRequest,
    ModularArmRequest,
    ModularArmSlots,
    MountDesignRequest,
)


REQUIRED_SEMANTIC_NODES = {
    "base_pedestal",
    "shoulder_mount",
    "shoulder_gear",
    "upper_arm_link",
    "elbow_gear",
    "forearm_link",
    "wrist_j4_roll",
    "wrist_j5_pitch",
    "wrist_j6_flange",
}


def test_modular_arm_cad_build_semantic_glb_and_subpart_cache(tmp_path: Path):
    """Modular arm must instantiate real solids, export semantic GLB nodes, and cache sub-parts."""
    materials = load_materials()
    mat = materials["petg_cf"]

    req = ModularArmRequest(
        dof=6,
        payload_kg=1.2,
        material_id="petg_cf",
        joint_angles_deg=[30.0, -50.0, 20.0],
        slots=ModularArmSlots(
            shoulder_mount=MountDesignRequest(actuator_type="nema17", base_type="profile_2020"),
            shoulder_mount_variant="mount_gusseted",
            shoulder_gear=GearDesignRequest(gear_kind="spur_involute", teeth=24, module_mm=2.0),
            shoulder_gear_variant="gear_spoked",
            upper_arm=LinkDesignRequest(length_mm=200.0, thickness_mm=20.0),
            upper_arm_variant="organic_ibeam",
            elbow_gear=GearDesignRequest(gear_kind="spur_involute", teeth=20, module_mm=2.0),
            elbow_gear_variant="gear_webbed",
            forearm=LinkDesignRequest(length_mm=165.0, thickness_mm=16.0),
            forearm_variant="hollow_cobot",
        ),
    )

    h1 = compute_modular_arm_hash(req)
    res = build_modular_arm_assembly(req, mat, tmp_path / h1, h1)

    # 1. All real volumes positive
    vols = res["real_volumes_mm3"]
    for key in ("base_pedestal", "shoulder_mount", "shoulder_gear", "upper_arm_link", "elbow_gear", "forearm_link", "wrist_tube"):
        assert vols[key] > 100.0, f"Invalid volume for {key}: {vols[key]}"

    # 2. GLB is a multi-node trimesh.Scene containing all required semantic node names
    glb_path = res["glb_path"]
    assert glb_path.exists() and glb_path.stat().st_size > 2000
    scene = trimesh.load(str(glb_path), force="scene")
    assert isinstance(scene, trimesh.Scene)
    scene_nodes = set(scene.graph.nodes_geometry)
    missing = REQUIRED_SEMANTIC_NODES - scene_nodes
    assert not missing, f"Missing semantic GLB nodes: {missing} (found: {scene_nodes})"

    # 3. Explode vectors present for every semantic node (3D float lists)
    explode_vecs = res["explode_vectors"]
    for node_name in REQUIRED_SEMANTIC_NODES:
        assert node_name in explode_vecs
        assert len(explode_vecs[node_name]) == 3

    # 4. Individual STEP & 3MF files exported
    indiv_names = {p.name for p in res["individual_files"]}
    for expected in (
        "shoulder_mount.step",
        "shoulder_mount.3mf",
        "shoulder_gear.step",
        "shoulder_gear.3mf",
        "upper_arm_link.step",
        "upper_arm_link.3mf",
        "elbow_gear.step",
        "elbow_gear.3mf",
        "forearm_link.step",
        "forearm_link.3mf",
    ):
        assert expected in indiv_names, f"Missing individual export: {expected}"

    # 5. Sub-part cache accelerates angle-only rebuild
    req2 = req.model_copy(update={"joint_angles_deg": [40.0, -60.0, 20.0]})
    h2 = compute_modular_arm_hash(req2)
    t0 = time.perf_counter()
    res2 = build_modular_arm_assembly(req2, mat, tmp_path / h2, h2)
    elapsed = time.perf_counter() - t0
    assert res2["glb_path"].exists()
    assert elapsed < 12.0  # Cached sub-parts skip OCCT solid rebuilds


def test_modular_arm_api_assemble_automatch_and_zip():
    """Verify POST /api/arm/auto-match, POST /api/arm/assemble, and downloadable manufacturing .ZIP."""
    from fastapi.testclient import TestClient

    from backend.app.main import app

    client = TestClient(app)

    # 1. Auto-match an incompatible request (NEMA23 6.35mm shaft vs 5.0mm gear bore)
    bad_req = ModularArmRequest(
        slots=ModularArmSlots(
            shoulder_mount=MountDesignRequest(actuator_type="nema23"),
            shoulder_gear=GearDesignRequest(bore_diameter_mm=5.0, teeth=24, module_mm=2.0),
        )
    )
    r_match = client.post("/api/arm/auto-match", json=bad_req.model_dump())
    assert r_match.status_code == 200, r_match.text
    match_data = r_match.json()
    assert match_data["auto_fixed"] is True
    assert len(match_data["auto_fix_notes"]) >= 1

    # 2. Assemble with auto_fix=True
    req_auto = bad_req.model_copy(update={"auto_fix": True})
    r_asm = client.post("/api/arm/assemble", json=req_auto.model_dump())
    assert r_asm.status_code == 200, r_asm.text
    data = r_asm.json()

    assert data["auto_fixed"] is True
    assert data["total_mass_g"] > 100.0
    assert data["reach_mm"] > 300.0
    assert len(data["per_part_metrics"]) >= 5
    assert set(REQUIRED_SEMANTIC_NODES).issubset(set(data["explode_vectors"].keys()))

    # 3. Download GLB and Manufacturing ZIP pack
    r_glb = client.get(data["files"]["glb"])
    assert r_glb.status_code == 200 and len(r_glb.content) > 2000

    zip_url = data["files"]["zip"]
    assert zip_url.endswith("/arm_manufacturing_pack.zip")
    r_zip = client.get(zip_url)
    assert r_zip.status_code == 200 and len(r_zip.content) > 2000

    # 4. Inspect ZIP contents
    import io

    with zipfile.ZipFile(io.BytesIO(r_zip.content), "r") as zf:
        znames = set(zf.namelist())
        for must_have in (
            "arm_assembly.step",
            "arm_assembly.urdf",
            "arm_mujoco.xml",
            "arm_bom.csv",
            "upper_arm_link.step",
            "upper_arm_link.3mf",
            "forearm_link.step",
            "forearm_link.3mf",
            "shoulder_mount.step",
            "shoulder_mount.3mf",
            "shoulder_gear.step",
            "shoulder_gear.3mf",
            "elbow_gear.step",
            "elbow_gear.3mf",
        ):
            assert must_have in znames, f"Missing {must_have} in manufacturing ZIP: {sorted(znames)}"
