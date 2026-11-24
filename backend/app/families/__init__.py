"""Registry of RoboForge v9.0 Multi-Project Families (FPV Drones, Rovers 4WD, Camera Gimbals)."""

from __future__ import annotations

from typing import Any

from .base import ProjectFamilyPlugin, compute_project_hash
from .camera_gimbal import CameraGimbalPlugin
from .fpv_drone import FpvDronePlugin
from .rover_4wd import Rover4WDPlugin

FAMILY_REGISTRY: dict[str, ProjectFamilyPlugin] = {
    "fpv_drone": FpvDronePlugin(),
    "rover_4wd": Rover4WDPlugin(),
    "camera_gimbal": CameraGimbalPlugin(),
}


def get_family_plugin(family_id: str) -> ProjectFamilyPlugin | None:
    """Return the registered ProjectFamilyPlugin for family_id, or None."""
    return FAMILY_REGISTRY.get((family_id or "").strip().lower())


def list_family_infos() -> list[dict[str, Any]]:
    """Return serialized family metadata + presets for GET /api/projects/families."""
    return [plugin.get_info() for plugin in FAMILY_REGISTRY.values()]


__all__ = [
    "FAMILY_REGISTRY",
    "ProjectFamilyPlugin",
    "compute_project_hash",
    "get_family_plugin",
    "list_family_infos",
]
