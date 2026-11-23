from __future__ import annotations

import io
from fastapi.testclient import TestClient
from PIL import Image, ImageDraw

from backend.app.main import app


def test_copilot_closed_loop_with_real_3d_fea() -> None:
    client = TestClient(app)
    res = client.post(
        "/api/copilot/optimize",
        json={
            "prompt": "Eslabón de 160 mm para 25 N en PETG-CF con FS >= 2.0",
            "verify_with_fea": True,
        },
    )
    assert res.status_code == 200, res.text
    data = res.json()
    assert data["converged"] is True
    assert data["fea_verified"] is True
    assert data["fea_result"] is not None
    assert data["fea_result"]["passes_fea"] is True
    assert data["fea_result"]["first_natural_freq_hz"] is not None


def test_copilot_2d_blueprint_png_analysis() -> None:
    # Draw a synthetic 2D blueprint PNG of a robotic link with 2 mounting holes
    img = Image.new("L", (400, 160), 255)
    draw = ImageDraw.Draw(img)
    draw.rounded_rectangle([40, 50, 360, 110], radius=28, fill=30)
    draw.ellipse([58, 66, 86, 94], fill=255)
    draw.ellipse([314, 66, 342, 94], fill=255)
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    buf.seek(0)

    client = TestClient(app)
    res = client.post(
        "/api/copilot/blueprint",
        files={"file": ("plano_brazo.png", buf.getvalue(), "image/png")},
        data={"prompt": "35 N en PA12-CF", "verify_with_fea": "false"},
    )
    assert res.status_code == 200, res.text
    data = res.json()
    assert data["filename"] == "plano_brazo.png"
    assert data["detected_aspect_ratio"] > 3.5
    assert data["detected_holes_count"] >= 2
    assert data["copilot_result"]["converged"] is True
