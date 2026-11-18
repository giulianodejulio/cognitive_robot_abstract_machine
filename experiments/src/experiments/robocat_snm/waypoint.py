"""
Read named cabin targets through the existing native SSH connection.
"""

# %% imports
from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from importlib.resources import files
import json
import math
from pathlib import Path
import re
import shlex
import subprocess
from typing_extensions import Any

from geometry_msgs.msg import Pose as GeometryPose
from rosidl_runtime_py.set_message import set_message_fields
from experiments.robocat_snm.exceptions import EntryError
from semantic_digital_twin.spatial_types import Pose


# %% correlated native reply
class WaypointReply(StrEnum):
    """
    Marker separating native waypoint data from transport logging.
    """

    RESULT = "WAYPOINT_RESULT "


@dataclass(frozen=True)
class SavedWaypoint:
    """
    A saved native waypoint resolved in the currently localized map.
    """

    waypoint_key: str
    """
    Full Eulero building/floor/name identity.
    """

    frame_id: str
    """
    Coordinate frame of the resolved target.
    """

    pose: GeometryPose
    """
    Native target position and quaternion.
    """

    @classmethod
    def from_json(cls, document: dict[str, Any]) -> SavedWaypoint:
        """
        Decode the native reply without discarding its reference frame.
        """
        pose = GeometryPose()
        set_message_fields(pose, document["pose"])
        return cls(document["waypoint_key"], document["frame_id"], pose)

    def in_map(self, expected_key: str) -> Pose:
        """
        Require the requested identity and finite normalized planar pose.
        """
        position, quaternion = self.pose.position, self.pose.orientation
        values = (
            position.x,
            position.y,
            position.z,
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        )
        if self.waypoint_key != expected_key or self.frame_id != "map":
            raise EntryError("Inside waypoint reply has a different identity or frame")
        if (
            not all(math.isfinite(value) for value in values)
            or abs(sum(value * value for value in values[3:]) - 1) > 1e-5
        ):
            raise EntryError("Inside waypoint has invalid coordinates or quaternion")
        if (
            abs(position.z) > 0.01
            or abs(quaternion.x) > 1e-5
            or abs(quaternion.y) > 1e-5
        ):
            raise EntryError("Inside waypoint must be planar")
        return Pose.from_xyz_quaternion(*values)


# %% existing native transport
@dataclass
class WaypointConnection:
    """
    Resolve an Eulero waypoint without creating a movement action client.
    """

    socket: Path = Path("/workspace/cram/.robocat-planner/ssh.sock")
    """
    Authenticated host SSH master socket shared with the container.
    """

    robot: str = "pal@10.68.0.1"
    """
    Native robot endpoint already authenticated by the host.
    """

    def read(self, name: str, parent_key: str) -> Pose:
        """
        Resolve one enabled waypoint from the saved staging floor.
        """
        key = name if "/" in name else parent_key + "/" + name
        if (
            key.rpartition("/")[0] != parent_key
            or re.fullmatch(r"[A-Za-z][A-Za-z0-9_-]*", key.rpartition("/")[2]) is None
        ):
            raise EntryError(
                "Inside waypoint must be a valid name on the staging floor"
            )
        if not self.socket.is_socket():
            raise EntryError(
                "Run bash scripts/connect_robocat_planner.sh on the laptop host first"
            )
        package = files("experiments.robocat_snm")
        command = [
            "bash",
            "-c",
            package.joinpath("native_remote.sh").read_text(),
            "bash",
            package.joinpath("native_waypoint.py").read_text(),
            key,
        ]
        reply = subprocess.run(
            [
                "ssh",
                "-S",
                str(self.socket),
                "-o",
                "BatchMode=yes",
                "-o",
                "StrictHostKeyChecking=yes",
                "-o",
                "ControlMaster=no",
                "-o",
                "ConnectTimeout=3",
                self.robot,
                shlex.join(command),
            ],
            capture_output=True,
            text=True,
            timeout=15,
        )
        lines = [
            line.removeprefix(WaypointReply.RESULT)
            for line in reply.stdout.splitlines()
            if line.startswith(WaypointReply.RESULT)
        ]
        if reply.returncode != 0 or len(lines) != 1:
            raise EntryError(
                "Could not read inside waypoint from Eulero: "
                + (reply.stderr + reply.stdout)[-1000:]
            )
        return SavedWaypoint.from_json(json.loads(lines[0])).in_map(key)
