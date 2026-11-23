"""Generate custom RoboForge .ico and compile standalone Windows .exe via PyInstaller."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

from PIL import Image, ImageDraw

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DESKTOP_DIR = PROJECT_ROOT / "desktop"
ICO_PATH = DESKTOP_DIR / "roboforge.ico"


def generate_icon(ico_path: Path) -> Path:
    """Create a crisp dark-engineering multi-resolution Windows .ico for RoboForge Studio."""
    size = 256
    img = Image.new("RGBA", (size, size), (9, 13, 20, 255))
    draw = ImageDraw.Draw(img)

    # Rounded dark studio frame
    draw.rounded_rectangle([14, 14, 242, 242], radius=48, fill=(15, 23, 42, 255), outline=(56, 189, 248, 220), width=8)
    # Robotic arm kinematic link motif
    draw.line([(72, 184), (176, 80)], fill=(226, 232, 240, 255), width=22)
    # Root joint (cyan)
    draw.ellipse([44, 156, 100, 212], fill=(9, 13, 20, 255), outline=(56, 189, 248, 255), width=14)
    # Tip joint (emerald)
    draw.ellipse([152, 56, 200, 104], fill=(9, 13, 20, 255), outline=(16, 185, 129, 255), width=12)
    # Accent sparks
    draw.line([(176, 30), (176, 48)], fill=(16, 185, 129, 255), width=8)
    draw.line([(208, 80), (226, 80)], fill=(16, 185, 129, 255), width=8)

    ico_path.parent.mkdir(parents=True, exist_ok=True)
    img.save(str(ico_path), format="ICO", sizes=[(16, 16), (32, 32), (48, 48), (64, 64), (128, 128), (256, 256)])
    return ico_path


def build_exe() -> None:
    generate_icon(ICO_PATH)
    print(f"[RoboForge] Icon generated: {ICO_PATH}")

    cmd = [
        sys.executable,
        "-m",
        "PyInstaller",
        "--noconfirm",
        "--onedir",
        "--windowed",
        "--name",
        "RoboForgeStudio",
        "--icon",
        str(ICO_PATH),
        "--add-data",
        f"{PROJECT_ROOT / 'frontend'};frontend",
        "--add-data",
        f"{PROJECT_ROOT / 'backend' / 'app' / 'data'};backend/app/data",
        "--collect-all",
        "build123d",
        "--collect-all",
        "OCP",
        "--collect-all",
        "gmsh",
        "--collect-all",
        "webview",
        "--hidden-import",
        "uvicorn.logging",
        "--hidden-import",
        "uvicorn.loops.auto",
        "--hidden-import",
        "uvicorn.protocols.http.auto",
        "--hidden-import",
        "uvicorn.lifespan.on",
        str(DESKTOP_DIR / "app_desktop.py"),
    ]
    print("[RoboForge] Running PyInstaller...")
    subprocess.run(cmd, cwd=str(PROJECT_ROOT), check=True)
    exe_path = PROJECT_ROOT / "dist" / "RoboForgeStudio" / "RoboForgeStudio.exe"
    print(f"[RoboForge] Standalone executable ready: {exe_path}")


if __name__ == "__main__":
    build_exe()
