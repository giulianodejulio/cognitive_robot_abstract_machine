"""
Validation for a stationary, localized waypoint acquisition.
"""

# %% imports
from dataclasses import dataclass
import math
import re
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from nav_msgs.msg import OccupancyGrid


# %% capture evidence
@dataclass(frozen=True)
class WaypointEvidence:
    """
    Freshness and wheel motion associated with a candidate pose.
    """

    pose_age: float
    """
    Absolute transform timestamp age in seconds.
    """

    wheel_age: float
    """
    Absolute wheel timestamp age in seconds.
    """

    linear_velocity: float
    """
    Measured forward velocity in metres per second.
    """

    angular_velocity: float
    """
    Measured yaw rate in radians per second.
    """

    def blockers(self, name: str, existing: list[str], key: str) -> list[str]:
        """
        Explain why a candidate must not be persisted.
        """
        reasons = []
        if re.fullmatch(r"[A-Za-z][A-Za-z0-9_-]*", name) is None:
            reasons.append(
                "Use a name starting with a letter, followed by letters, digits, underscores or hyphens"
            )
        if key in existing:
            reasons.append("Waypoint already exists; no overwrite allowed")
        if not all(
            math.isfinite(value)
            for value in (
                self.pose_age,
                self.wheel_age,
                self.linear_velocity,
                self.angular_velocity,
            )
        ):
            reasons.append("Nonfinite acquisition evidence")
        if abs(self.pose_age) > 0.5 or abs(self.wheel_age) > 0.5:
            reasons.append(
                f"Pose or wheel feedback is older than 0.5 seconds: pose_age={self.pose_age:.3f}s; wheel_age={self.wheel_age:.3f}s"
            )
        if abs(self.linear_velocity) >= 0.005 or abs(self.angular_velocity) >= 0.01:
            reasons.append("Robot is not stationary")
        return reasons


# %% map identity by published cells
@dataclass(frozen=True)
class MapFile:
    """
    An Eulero occupancy-grid YAML candidate.
    """

    filename: str
    """
    Native YAML path for the candidate map.
    """

    def matches(self, message: OccupancyGrid) -> bool:
        """
        Compare published geometry and cells with a trinary map file.
        """
        path = Path(self.filename)
        if not path.is_file():
            return False
        configuration = yaml.safe_load(path.read_text())
        if configuration.get("mode", "trinary") != "trinary":
            return False
        image_path = path.parent / configuration["image"]
        if not image_path.is_file():
            return False
        image = np.asarray(Image.open(image_path).convert("L"), dtype=float)
        if image.shape != (message.info.height, message.info.width):
            return False
        origin = configuration["origin"]
        position = message.info.origin.position
        rotation = message.info.origin.orientation
        if not math.isclose(
            message.info.resolution, configuration["resolution"], abs_tol=1e-7
        ):
            return False
        if not all(
            math.isclose(a, b, abs_tol=1e-7)
            for a, b in zip(
                (position.x, position.y, rotation.z, rotation.w),
                (
                    origin[0],
                    origin[1],
                    math.sin(origin[2] / 2),
                    math.cos(origin[2] / 2),
                ),
            )
        ):
            return False
        occupancy = (
            image / 255 if configuration.get("negate", 0) else (255 - image) / 255
        )
        cells = np.full(image.shape, -1, dtype=np.int8)
        cells[occupancy > configuration["occupied_thresh"]] = 100
        cells[occupancy < configuration["free_thresh"]] = 0
        return np.array_equal(
            np.flipud(cells).ravel(), np.asarray(message.data, dtype=np.int8)
        )
