"""
Request paths through the robot's native planner using an existing SSH connection.
"""

# %% imports
from __future__ import annotations

import json
import math
import shlex
import subprocess
import tempfile
import time
from contextlib import ExitStack
from copy import deepcopy
from dataclasses import dataclass
from enum import StrEnum
from importlib.resources import files
from pathlib import Path

import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path as NavigationPath
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from typing_extensions import Callable

from experiments.robocat_waypoints.model import WaypointFrame


# %% native CLI response
class PlannerOutput(StrEnum):
    """
    Markers emitted by the installed native ros2 action CLI.
    """

    RESULT = "Result:\n    "
    STATUS = "Goal finished with status: "
    SUCCESS = "SUCCEEDED"
    PATH = "path"


@dataclass
class PlannerReply:
    """
    A correlated planning result or a reason why no usable path was returned.
    """

    path: NavigationPath | None = None
    """
    Path returned by this planning action, never by a different topic publisher.
    """

    reason: str | None = None
    """
    Transport, planning or response failure explanation.
    """

    def connect_goal(self, goal: PoseStamped) -> NavigationPath | None:
        """
        Connect the exact goal after removing a backwards terminal approach.
        """
        if self.path is None:
            return None
        path = deepcopy(self.path)
        target = goal.pose.position
        while len(path.poses) > 1:
            previous = path.poses[-2].pose.position
            last = path.poses[-1].pose.position
            progress = (
                (last.x - previous.x) * (target.x - last.x)
                + (last.y - previous.y) * (target.y - last.y)
                + (last.z - previous.z) * (target.z - last.z)
            )
            if progress >= 0:
                break
            path.poses.pop()
        path.poses.append(deepcopy(goal))
        return path

    @classmethod
    def from_output(cls, output: str, returncode: int) -> PlannerReply:
        """
        Decode the owned native action result without accepting failed actions.
        """
        if returncode != 0:
            return cls(
                reason=f"Native planner transport failed (exit {returncode}): {output[-512:].strip()}"
            )
        _, marker, body = output.partition(PlannerOutput.RESULT)
        document, status_marker, status = body.rpartition(PlannerOutput.STATUS)
        if not marker or not status_marker or status.strip() != PlannerOutput.SUCCESS:
            return cls(reason=f"Planning did not succeed: {output[-512:].strip()}")
        payload = yaml.safe_load(document)
        if not isinstance(payload, dict) or PlannerOutput.PATH not in payload:
            return cls(reason="Native planner returned no Path payload")
        path = NavigationPath()
        set_message_fields(path, payload[PlannerOutput.PATH])
        if not path.poses:
            return cls(reason="Native planner returned an empty path")
        return cls(path=path)

    def matches(
        self,
        goal: PoseStamped,
        position_tolerance: float = 0.02,
        angular_tolerance_degrees: float = 2.0,
    ) -> bool:
        """
        Require the returned endpoint to describe the selected map goal.
        """
        if (
            self.path is None
            or not self.path.poses
            or self.path.header.frame_id != goal.header.frame_id
        ):
            return False
        end = self.path.poses[-1].pose
        start = goal.pose
        if (
            math.dist(
                (start.position.x, start.position.y, start.position.z),
                (end.position.x, end.position.y, end.position.z),
            )
            > position_tolerance
        ):
            return False
        a, b = start.orientation, end.orientation
        values = (a.x, a.y, a.z, a.w, b.x, b.y, b.z, b.w)
        if not all(math.isfinite(value) for value in values):
            return False
        if any(
            abs(sum(value * value for value in part) - 1) > 1e-5
            for part in (values[:4], values[4:])
        ):
            return False
        dot = abs(sum(x * y for x, y in zip(values[:4], values[4:])))
        return 2 * math.acos(min(1.0, dot)) <= math.radians(angular_tolerance_degrees)


# %% bounded native planning process
@dataclass
class PlannerProcess:
    """
    Stop only the local SSH child owned by this request.
    """

    process: subprocess.Popen[str]
    """
    The owned process; it never publishes robot velocity commands.
    """

    def stop(self) -> None:
        """
        Terminate a pending transport and reap it with a bounded grace interval.
        """
        if self.process.poll() is not None:
            return
        self.process.terminate()
        deadline = time.monotonic() + 2.0
        while self.process.poll() is None and time.monotonic() < deadline:
            time.sleep(0.02)
        if self.process.poll() is None:
            self.process.kill()
        self.process.wait()


@dataclass
class NativePlanner:
    """
    Call only ComputePathToPose through a host-authenticated SSH master socket.
    """

    socket: Path = Path("/workspace/cram/.robocat-planner/ssh.sock")
    """
    Shared socket created on the host without copying SSH credentials.
    """

    robot: str = "pal@10.68.0.1"
    """
    Native robot connection; authentication is owned by the host SSH master.
    """

    planner_id: str = "GridBased"
    """
    Planner plugin selected in the native PAL planner server.
    """

    timeout: float = 16.0
    """
    Maximum local duration of one planning request in seconds.
    """

    def request(
        self, goal: PoseStamped, still_current: Callable[[], bool] | None = None
    ) -> PlannerReply:
        """
        Compute a path without commanding motion; cancel stale local requests.
        """
        if not self.socket.is_socket():
            return PlannerReply(
                reason="Planner SSH connection missing: run bash scripts/connect_robocat_planner.sh on the laptop host"
            )
        if goal.header.frame_id != WaypointFrame.MAP:
            return PlannerReply(reason="Planner goal must be in map")
        position, rotation = goal.pose.position, goal.pose.orientation
        values = (
            position.x,
            position.y,
            position.z,
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )
        if (
            not all(math.isfinite(value) for value in values)
            or abs(sum(value * value for value in values[3:]) - 1) > 1e-5
        ):
            return PlannerReply(
                reason="Planner goal must contain a finite pose and a unit quaternion"
            )
        native_goal = ComputePathToPose.Goal(
            goal=goal, planner_id=self.planner_id, use_start=False
        )
        script = (
            files("experiments.robocat_waypoints")
            .joinpath("planner_remote.sh")
            .read_text()
        )
        with ExitStack() as cleanup:
            stdin = cleanup.enter_context(tempfile.TemporaryFile(mode="w+t"))
            stdout = cleanup.enter_context(tempfile.TemporaryFile(mode="w+t"))
            stderr = cleanup.enter_context(tempfile.TemporaryFile(mode="w+t"))
            json.dump(message_to_ordereddict(native_goal), stdin, allow_nan=False)
            stdin.seek(0)
            process = subprocess.Popen(
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
                    f"bash -c {shlex.quote(script)}",
                ],
                stdin=stdin,
                stdout=stdout,
                stderr=stderr,
                text=True,
            )
            cleanup.callback(PlannerProcess(process).stop)
            deadline = time.monotonic() + self.timeout
            while process.poll() is None and time.monotonic() < deadline:
                if still_current is not None and not still_current():
                    return PlannerReply(
                        reason="Planning canceled because the selection or map changed"
                    )
                time.sleep(0.05)
            if process.poll() is None:
                return PlannerReply(reason="Native planner request timed out")
            stdout.seek(0)
            stderr.seek(0)
            output = stdout.read()
            if process.returncode != 0:
                output += stderr.read()
            return PlannerReply.from_output(output, process.returncode)
