from __future__ import annotations

from fastapi.testclient import TestClient

from backend.app.main import app
from backend.app.schemas import GyroidRequest, LinkDesignRequest


def test_sdf_gyroid_endpoint_and_downloads() -> None:
    client = TestClient(app)
    payload = GyroidRequest(
        request=LinkDesignRequest(length_mm=160.0, material_id="petg_cf"),
        cell_size_mm=14.0,
        voxel_size_mm=2.0,
    ).model_dump(mode="json")

    res = client.post("/api/sdf/gyroid", json=payload)
    assert res.status_code == 200, res.text
    data = res.json()

    assert data["design_hash"].startswith("gyr_")
    assert data["triangle_count"] > 1000
    assert data["volume_cm3"] > 1.0
    assert data["mass_g"] > 1.0
    assert len(data["voxel_grid"]) == 3

    for key in ("glb_url", "stl_url", "threemf_url"):
        dl = client.get(data[key])
        assert dl.status_code == 200
        assert len(dl.content) > 500
