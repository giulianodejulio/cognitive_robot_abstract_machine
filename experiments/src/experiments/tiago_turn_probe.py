"""Compare command and odometry during one explicitly requested real pure turn."""

# %% imports
from __future__ import annotations

import argparse
import json
import math
import signal
import subprocess
import time
from contextlib import ExitStack
from dataclasses import dataclass, field
from enum import StrEnum
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from experiments.tiago_goal_logging import PlanarPose
from krrood.adapters.json_serializer import to_json


# %% aggregate results without recording streams
class ProbeTopic(StrEnum):
    """The three existing streams observed by this probe."""

    COMMAND = "/cmd_vel"
    WHEELS = "/mobile_base_controller/odom"
    LASER = "/direct_laser_odometry/odom"


@dataclass
class OdometryResult:
    """Displacement relative to one source's own initial pose."""

    samples: int
    """Number of messages in this measurement window."""
    frame: str
    """Reported parent frame."""
    child_frame: str
    """Reported moving frame."""
    forward: float
    """Final forward displacement in initial body axes, in meters."""
    lateral: float
    """Final leftward displacement in initial body axes, in meters."""
    displacement: float
    """Norm of the final planar displacement, in meters."""
    maximum_displacement: float
    """Largest displacement from the start observed during the window."""
    yaw_degrees: float
    """Shortest final yaw change relative to the start, in degrees."""
    maximum_absolute_linear_x: float
    """Largest reported absolute body forward speed, in m/s."""
    maximum_absolute_angular_z: float
    """Largest reported absolute yaw speed, in rad/s."""


@dataclass
class OdometryTrace:
    """Keep two messages and maxima, never a trajectory recording."""

    initial: Odometry | None = None
    """Initial message for the current measurement window."""
    latest: Odometry | None = None
    """Latest received message."""
    received_at: float = 0
    """Latest local monotonic receipt time."""
    samples: int = 0
    """Messages received in the current window."""
    maximum_displacement: float = 0
    """Peak distance from the window's initial position."""
    maximum_linear: float = 0
    """Peak absolute forward speed."""
    maximum_angular: float = 0
    """Peak absolute yaw speed."""

    def receive(self, message: Odometry) -> None:
        """Update only the latest sample and scalar aggregates."""
        if self.initial is None:
            self.initial = message
        self.latest = message
        self.received_at = time.monotonic()
        self.samples += 1
        initial, current = self.initial.pose.pose.position, message.pose.pose.position
        self.maximum_displacement = max(
            self.maximum_displacement,
            math.hypot(current.x - initial.x, current.y - initial.y),
        )
        self.maximum_linear = max(
            self.maximum_linear, abs(message.twist.twist.linear.x)
        )
        self.maximum_angular = max(
            self.maximum_angular, abs(message.twist.twist.angular.z)
        )

    def reset(self) -> None:
        """Start the next window from the last received pose."""
        self.initial = self.latest
        self.samples = 0
        self.maximum_displacement = self.maximum_linear = self.maximum_angular = 0

    def current(self) -> bool:
        """Require finite recent feedback before or during the owned motion."""
        if self.latest is None or time.monotonic() - self.received_at > 2:
            return False
        pose = PlanarPose.from_ros(self.latest.pose.pose)
        return all(
            math.isfinite(value)
            for value in (
                pose.x,
                pose.y,
                pose.yaw,
                self.latest.twist.twist.linear.x,
                self.latest.twist.twist.angular.z,
            )
        )

    def result(self) -> OdometryResult:
        """Summarize one source without comparing its absolute origin to another."""
        if self.initial is None or self.latest is None:
            raise RuntimeError("Missing odometry samples")
        start = PlanarPose.from_ros(self.initial.pose.pose)
        end = PlanarPose.from_ros(self.latest.pose.pose)
        dx, dy = end.x - start.x, end.y - start.y
        return OdometryResult(
            samples=self.samples,
            frame=self.latest.header.frame_id,
            child_frame=self.latest.child_frame_id,
            forward=math.cos(start.yaw) * dx + math.sin(start.yaw) * dy,
            lateral=-math.sin(start.yaw) * dx + math.cos(start.yaw) * dy,
            displacement=math.hypot(dx, dy),
            maximum_displacement=self.maximum_displacement,
            yaw_degrees=math.degrees(
                math.atan2(math.sin(end.yaw - start.yaw), math.cos(end.yaw - start.yaw))
            ),
            maximum_absolute_linear_x=self.maximum_linear,
            maximum_absolute_angular_z=self.maximum_angular,
        )


@dataclass
class CommandResult:
    """Counts and extrema of commands observed in one window."""

    samples: int
    """A zero count means command velocity is unknown, not proven zero."""
    maximum_absolute_linear_x: float
    """Largest absolute commanded forward velocity in m/s."""
    maximum_absolute_linear_y: float
    """Largest absolute commanded lateral velocity in m/s."""
    maximum_absolute_angular_z: float
    """Largest absolute commanded yaw velocity in rad/s."""


@dataclass
class CommandTrace:
    """Keep command maxima without retaining individual messages."""

    samples: int = 0
    """Messages received in this window."""
    maximum_linear_x: float = 0
    """Peak absolute forward command."""
    maximum_linear_y: float = 0
    """Peak absolute lateral command."""
    maximum_angular_z: float = 0
    """Peak absolute yaw command."""

    def receive(self, message: Twist) -> None:
        """Retain extrema of both positive and negative commands."""
        self.samples += 1
        self.maximum_linear_x = max(self.maximum_linear_x, abs(message.linear.x))
        self.maximum_linear_y = max(self.maximum_linear_y, abs(message.linear.y))
        self.maximum_angular_z = max(self.maximum_angular_z, abs(message.angular.z))

    def reset(self) -> None:
        """Clear aggregates between baseline and movement."""
        self.samples = 0
        self.maximum_linear_x = self.maximum_linear_y = self.maximum_angular_z = 0

    def result(self) -> CommandResult:
        """Report sample coverage alongside command extrema."""
        return CommandResult(
            self.samples,
            self.maximum_linear_x,
            self.maximum_linear_y,
            self.maximum_angular_z,
        )


@dataclass
class MeasurementWindow:
    """Aggregates for the same observation interval."""

    command: CommandResult
    """Observed Giskard input commands to the mux."""
    wheels: OdometryResult
    """Wheel odometry displacement."""
    laser: OdometryResult
    """Laser odometry displacement."""


@dataclass
class ProbeResult:
    """One isolated turn, including its stationary baseline."""

    angle: float
    """Requested relative yaw in degrees."""
    publishers: list[str]
    """Discovered publishers on the command topic."""
    baseline: MeasurementWindow
    """One second of stationary feedback before the goal."""
    motion: MeasurementWindow | None
    """Goal execution plus one second after its result, if executed."""
    return_code: int | None
    """Exit status of the owned foreground goal command."""


# %% explicit CLI authorization
@dataclass
class ProbeOptions:
    """Choose read-only inspection or one small real turn."""

    execute: bool = False
    """Whether to dispatch a real goal."""
    angle: float = 30
    """Relative rotation, restricted to at most thirty degrees."""
    output: Path = Path("/tmp/tiago-turn-probe.json")
    """Aggregate report location; no stream recording is written."""

    @classmethod
    def parse(cls, arguments: list[str] | None = None) -> ProbeOptions:
        """Require explicit mode and reject nonfinite or oversized turns."""
        parser = argparse.ArgumentParser(description=__doc__)
        mode = parser.add_mutually_exclusive_group(required=True)
        mode.add_argument("--check", action="store_true")
        mode.add_argument("--execute", action="store_true")
        parser.add_argument("--angle", type=float, default=cls().angle)
        parser.add_argument("--output", type=Path, default=cls().output)
        options = parser.parse_args(arguments)
        if not math.isfinite(options.angle) or not 0 < abs(options.angle) <= 30:
            parser.error("--angle must be finite, nonzero and within +/-30 degrees")
        return cls(execute=options.execute, angle=options.angle, output=options.output)


# %% one lightweight observer, reusing the already-running client for control
@dataclass
class TurnProbe:
    """Observe three streams and own at most one cancellable CLI request."""

    node: Node
    """Subscription-only ROS node."""
    wheels: OdometryTrace = field(default_factory=OdometryTrace)
    """Wheel feedback aggregates."""
    laser: OdometryTrace = field(default_factory=OdometryTrace)
    """Laser feedback aggregates."""
    command: CommandTrace = field(default_factory=CommandTrace)
    """Command extrema."""
    sender: Path = field(
        default_factory=lambda: Path(__file__).resolve().parents[3]
        / "scripts/tiago-goal"
    )
    """Existing sender; this observer never publishes a twist."""

    def __post_init__(self) -> None:
        self.node.create_subscription(
            Twist, ProbeTopic.COMMAND, self.command.receive, qos_profile_sensor_data
        )
        self.node.create_subscription(
            Odometry, ProbeTopic.WHEELS, self.wheels.receive, qos_profile_sensor_data
        )
        self.node.create_subscription(
            Odometry, ProbeTopic.LASER, self.laser.receive, qos_profile_sensor_data
        )

    def collect(self, seconds: float) -> None:
        """Spin a bounded interval without console output or recording."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def window(self) -> MeasurementWindow:
        """Snapshot the scalar aggregates for all three topics."""
        return MeasurementWindow(
            self.command.result(), self.wheels.result(), self.laser.result()
        )

    def reset(self) -> None:
        """Preserve the latest poses as the next interval's starting points."""
        self.command.reset()
        self.wheels.reset()
        self.laser.reset()

    def cancel_owned(self, process: subprocess.Popen) -> None:
        """Request cancellation of only this probe's foreground command."""
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=12)

    def run(self, options: ProbeOptions) -> ProbeResult:
        """Measure a baseline and, after the existing preflight, one pure turn."""
        deadline = time.monotonic() + 8
        while (
            not (self.wheels.current() and self.laser.current())
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self.node, timeout_sec=0.05)
        if not (self.wheels.current() and self.laser.current()):
            raise RuntimeError(
                "Missing fresh finite wheel/laser odometry; no goal sent"
            )
        self.reset()
        self.collect(1)
        publishers = [
            f"{info.node_namespace}/{info.node_name}"
            for info in self.node.get_publishers_info_by_topic(ProbeTopic.COMMAND)
        ]
        baseline = self.window()
        if not options.execute:
            return ProbeResult(options.angle, publishers, baseline, None, None)
        ready = subprocess.run(
            [str(self.sender), "--ready"], capture_output=True, text=True, timeout=5
        )
        if (
            ready.returncode != 0
            or baseline.wheels.maximum_absolute_linear_x >= 0.005
            or baseline.wheels.maximum_absolute_angular_z >= 0.01
        ):
            raise RuntimeError(
                f"Robot/client not ready; no goal sent: {ready.stdout} {ready.stderr}"
            )
        self.collect(0.2)
        self.reset()
        with ExitStack() as cleanup:
            process = subprocess.Popen(
                [str(self.sender), "--turn", f"{options.angle:g}", "--pure-turn"],
                start_new_session=True,
            )
            cleanup.callback(self.cancel_owned, process)
            deadline = time.monotonic() + 30
            cancel_sent = False
            while process.poll() is None:
                rclpy.spin_once(self.node, timeout_sec=0.02)
                if not cancel_sent and (
                    time.monotonic() >= deadline
                    or not self.wheels.current()
                    or not self.laser.current()
                    or self.command.maximum_linear_x > 0.005
                ):
                    process.send_signal(signal.SIGINT)
                    cancel_sent = True
                    deadline = time.monotonic() + 10
                if cancel_sent and time.monotonic() >= deadline:
                    raise TimeoutError("Owned turn cancellation was not confirmed")
            self.collect(1)
            return ProbeResult(
                options.angle, publishers, baseline, self.window(), process.returncode
            )


# %% real execution only through explicit CLI mode
def main() -> None:
    """Observe without motion by default, or execute the explicitly selected turn."""
    options = ProbeOptions.parse()
    with ExitStack() as cleanup:
        rclpy.init()
        cleanup.callback(rclpy.shutdown)
        node = rclpy.create_node("tiago_turn_probe")
        cleanup.callback(node.destroy_node)
        result = TurnProbe(node).run(options)
        report = json.dumps(to_json(result), indent=2)
        options.output.write_text(report)
        print(report, flush=True)


if __name__ == "__main__":
    main()
