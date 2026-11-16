"""Own one Nav2 goal in the robot's native ROS environment."""

# %% imports
from __future__ import annotations

import argparse
import math
import os
import select
import signal
import sys
import time
from contextlib import ExitStack
from dataclasses import dataclass, field
from enum import Enum
from types import FrameType

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default
from rclpy.task import Future
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterValue


# %% native navigation lifecycle
class NavigationStatus(str, Enum):
    """Line-oriented state exported by the owned native navigation process."""

    WAITING = "waiting"
    NAVIGATING = "navigating"
    SUCCEEDED = "succeeded"
    IDLE = "idle"
    BUSY = "busy"
    FAILED = "failed"
    CANCELED = "canceled"
    HEARTBEAT = "heartbeat"
    CANCEL = "cancel"
    ROUTING_READY = "routing_ready"
    BOOTSTRAP_READY = "bootstrap_ready"
    BOOTSTRAP_BLOCKED = "bootstrap_blocked"

    def __str__(self) -> str:
        """Return the wire value on both native Python 3.10 and local Python 3.12."""
        return self.value


@dataclass
class NativeNavigation:
    """Cancel only the owned Nav2 goal if its supervising connection disappears."""

    node: Node
    """Native Humble node."""
    goal: NavigateToPose.Goal
    """Configured staging pose."""
    execute: bool
    """Whether this process may submit a goal."""
    confirm_navigation_idle: bool = False
    """Operator confirmation for read-only preflight when Nav2 status is unavailable."""
    bootstrap_navigation: bool = False
    """Explicit authorization of a first goal without a received native idle status."""
    bootstrap_maximum_age: float = 0.5
    """Maximum age of the supervisor's stationary/free-mux permission in seconds."""
    bootstrap_at: float | None = field(default=None, init=False)
    """Receipt time of the latest unrevoked bootstrap permission."""
    timeout: float = 180.0
    """Maximum duration of the owned navigation, in seconds."""
    heartbeat_timeout: float = 2.0
    """Maximum absence of supervisor heartbeats."""
    client: ActionClient = field(init=False)
    """Action client for the native navigation stack."""
    subscription: object = field(init=False)
    """Subscription retaining active-goal status."""
    statuses: GoalStatusArray | None = field(default=None, init=False)
    """Latest native action status, including goals owned by other clients."""
    pending: Future | None = field(default=None, init=False)
    """Outstanding acceptance request."""
    handle: ClientGoalHandle | None = field(default=None, init=False)
    """Accepted owned goal."""
    result: Future | None = field(default=None, init=False)
    """Owned result request."""
    cancellation: Future | None = field(default=None, init=False)
    """Cancellation acknowledgment request."""
    last_heartbeat: float = field(default_factory=time.monotonic, init=False)
    """Monotonic receipt time of the last supervisor heartbeat."""
    stop_requested: bool = field(default=False, init=False)
    """An interruption that must survive delayed goal acceptance."""
    status: NavigationStatus = field(default=NavigationStatus.WAITING, init=False)
    """Last correlated lifecycle result."""
    parameters: object = field(init=False)
    """Native mux configuration service client."""
    routing: Future | None = field(default=None, init=False)
    """Configuration query whose reply must match both separate mux inputs."""
    routing_ready: bool = field(default=False, init=False)
    """Whether the native mux exposes separate navigation and entry inputs."""
    input_buffer: bytes = field(default=b"", init=False)
    """Incomplete supervisor command, without buffered stale heartbeats."""

    def __post_init__(self) -> None:
        self.client = ActionClient(self.node, NavigateToPose, "/navigate_to_pose")
        self.subscription = self.node.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self.receive_status,
            qos_profile_action_status_default,
        )
        self.parameters = self.node.create_client(
            GetParameters, "/twist_mux/get_parameters"
        )

    def receive_status(self, message: GoalStatusArray) -> None:
        """Retain native ownership information without canceling unrelated clients."""
        self.statuses = message

    def receive_command(self, command: str, now: float) -> None:
        """Retain supervisor liveness and revoke a first-goal permit when interlocks change."""
        if command == NavigationStatus.HEARTBEAT:
            self.last_heartbeat = now
        elif (
            command == NavigationStatus.BOOTSTRAP_READY
            and self.bootstrap_navigation
            and self.execute
        ):
            self.bootstrap_at = now
        elif command == NavigationStatus.BOOTSTRAP_BLOCKED:
            self.bootstrap_at = None
        else:
            self.stop_requested = True

    @property
    def active_goals(self) -> list[GoalStatus]:
        """All native navigation goals that have not reached a terminal state."""
        if self.statuses is None:
            return []
        return [
            item
            for item in self.statuses.status_list
            if item.status
            in (
                GoalStatus.STATUS_ACCEPTED,
                GoalStatus.STATUS_EXECUTING,
                GoalStatus.STATUS_CANCELING,
            )
        ]

    def tick(self, now: float, deadline: float) -> None:
        """Process delayed acceptance, result and watchdog cancellation."""
        if now - self.last_heartbeat > self.heartbeat_timeout or now > deadline:
            self.stop_requested = True
        if self.routing is None and self.parameters.service_is_ready():
            self.routing = self.parameters.call_async(
                GetParameters.Request(
                    names=[
                        "topics.navigation.topic",
                        "topics.navigation.priority",
                        "topics.snm.topic",
                        "topics.snm.priority",
                    ]
                )
            )
        if self.routing is not None and self.routing.done():
            if self.routing.exception() is not None:
                self.status = NavigationStatus.FAILED
                return
            values = self.routing.result().values
            self.routing_ready = (
                len(values) == 4
                and values[0].string_value.lstrip("/") == "cmd_vel"
                and values[1].integer_value == 10
                and values[2].string_value.lstrip("/") == "snm_vel"
                and values[3].integer_value == 20
            )
            if not self.routing_ready:
                self.status = NavigationStatus.FAILED
                return
        if self.pending is not None and self.pending.done() and self.handle is None:
            if (
                self.pending.exception() is not None
                or not self.pending.result().accepted
            ):
                self.status = NavigationStatus.FAILED
                return
            self.handle = self.pending.result()
            self.result = self.handle.get_result_async()
        if (
            self.stop_requested
            and self.handle is not None
            and self.cancellation is None
        ):
            self.cancellation = self.handle.cancel_goal_async()
        if self.result is not None and self.result.done():
            if self.result.exception() is not None:
                self.status = NavigationStatus.FAILED
            else:
                code = self.result.result().status
                self.status = (
                    NavigationStatus.SUCCEEDED
                    if code == GoalStatus.STATUS_SUCCEEDED
                    else (
                        NavigationStatus.CANCELED
                        if code == GoalStatus.STATUS_CANCELED
                        else NavigationStatus.FAILED
                    )
                )
        if self.status is not NavigationStatus.WAITING or self.stop_requested:
            return
        bootstrap_ready = (
            self.execute
            and self.bootstrap_navigation
            and self.bootstrap_at is not None
            and now - self.bootstrap_at <= self.bootstrap_maximum_age
        )
        if (
            (
                self.statuses is None
                and not self.confirm_navigation_idle
                and not bootstrap_ready
            )
            or not self.client.server_is_ready()
            or not self.routing_ready
        ):
            return
        if self.active_goals:
            self.status = NavigationStatus.FAILED
            return
        if self.bootstrap_navigation and self.execute and not bootstrap_ready:
            return
        if not self.execute:
            self.status = NavigationStatus.SUCCEEDED
            return
        self.goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        if self.statuses is None:
            print(
                "BOOTSTRAP AUTHORIZED: first Nav2 goal; supervisor confirms stopped base and free mux; native idle status is not certified",
                file=sys.stderr,
                flush=True,
            )
        self.pending = self.client.send_goal_async(self.goal)
        self.status = NavigationStatus.NAVIGATING

    def run(self) -> None:
        """Keep native status available until the supervisor closes its connection."""
        deadline = time.monotonic() + self.timeout
        report_at = 0.0
        stop_at: float | None = None
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            now = time.monotonic()
            while select.select([sys.stdin], [], [], 0)[0]:
                chunk = os.read(sys.stdin.fileno(), 4096)
                if not chunk:
                    self.stop_requested = True
                    break
                self.input_buffer += chunk
                while b"\n" in self.input_buffer:
                    command, _, self.input_buffer = self.input_buffer.partition(b"\n")
                    self.receive_command(command.decode().strip(), now)
            self.tick(now, deadline)
            if not self.stop_requested and now >= report_at:
                print(self.status, flush=True)
                if self.statuses is not None or (
                    self.confirm_navigation_idle and not self.execute
                ):
                    print(
                        (
                            NavigationStatus.BUSY
                            if self.active_goals
                            else NavigationStatus.IDLE
                        ),
                        flush=True,
                    )
                if self.routing_ready:
                    print(NavigationStatus.ROUTING_READY, flush=True)
                report_at = now + 0.2
            if self.stop_requested:
                if stop_at is None:
                    stop_at = now + 5.0
                if (
                    self.pending is None
                    or (self.result is not None and self.result.done())
                    or self.status is NavigationStatus.FAILED
                    or now >= stop_at
                ):
                    return
            if self.status in (
                NavigationStatus.SUCCEEDED,
                NavigationStatus.FAILED,
                NavigationStatus.CANCELED,
            ):
                deadline = math.inf

    def request_stop(self, number: int, frame: FrameType | None) -> None:
        """Retain a native termination request until the owned goal is canceled."""
        self.stop_requested = True

    def close(self) -> None:
        """Drain owned cancellation even when the status output connection fails."""
        if self.pending is None:
            return
        self.stop_requested = True
        deadline = time.monotonic() + 5
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            self.tick(time.monotonic(), math.inf)
            if self.status is NavigationStatus.FAILED or (
                self.result is not None and self.result.done()
            ):
                return


# %% native command entry point
@dataclass(frozen=True)
class WheelSetting:
    """One controller parameter and its experimentally validated multiplier."""

    parameter: str
    """Native controller parameter name."""
    multiplier: float
    """Required value before the next configure transition."""


@dataclass
class WheelCalibration:
    """Require the experimentally validated native wheel multipliers."""

    node: Node
    """Native node querying the controller parameters before navigation starts."""
    radius_multiplier: float = 2.0
    """Validated left and right radius multipliers."""
    separation_multiplier: float = 1.22
    """Validated separation multiplier."""

    @property
    def settings(self) -> list[WheelSetting]:
        """The native parameter names and validated values for both wheels."""
        return [
            WheelSetting("left_wheel_radius_multiplier", self.radius_multiplier),
            WheelSetting("right_wheel_radius_multiplier", self.radius_multiplier),
            WheelSetting("wheel_separation_multiplier", self.separation_multiplier),
        ]

    def read(self) -> list[ParameterValue] | None:
        """Read current native values without changing controller state."""
        client = self.node.create_client(
            GetParameters, "/mobile_base_controller/get_parameters"
        )
        if not client.wait_for_service(timeout_sec=3):
            return None
        future = client.call_async(
            GetParameters.Request(names=[item.parameter for item in self.settings])
        )
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3)
        if not future.done() or future.exception() is not None:
            return None
        return future.result().values

    def verify(self) -> bool:
        """Reject missing or reset calibration without changing robot parameters."""
        values = self.read()
        return (
            values is not None
            and len(values) == len(self.settings)
            and all(
                math.isclose(value.double_value, setting.multiplier, abs_tol=1e-6)
                for value, setting in zip(values, self.settings)
            )
        )


def main() -> None:
    """Accept only finite staging coordinates in the native map frame."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--yaw", type=float, required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm-navigation-idle", action="store_true")
    parser.add_argument("--bootstrap-navigation", action="store_true")
    parser.add_argument("--timeout", type=float, default=180.0)
    options = parser.parse_args()
    if options.execute and options.confirm_navigation_idle:
        parser.error("--confirm-navigation-idle cannot authorize navigation execution")
    if options.bootstrap_navigation and not options.execute:
        parser.error("--bootstrap-navigation requires explicit --execute")
    if not all(math.isfinite(value) for value in (options.x, options.y, options.yaw)):
        parser.error("Staging coordinates must be finite")
    if not math.isfinite(options.timeout) or options.timeout <= 0:
        parser.error("Navigation timeout must be positive and finite")
    with ExitStack() as cleanup:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        cleanup.callback(rclpy.shutdown)
        node = rclpy.create_node("robocat_snm_navigation")
        cleanup.callback(node.destroy_node)
        if not WheelCalibration(node).verify():
            print(
                "Native wheel calibration unavailable or differs from 2.0 / 2.0 / 1.22",
                file=sys.stderr,
                flush=True,
            )
            print(NavigationStatus.FAILED, flush=True)
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x, goal.pose.pose.position.y = options.x, options.y
        goal.pose.pose.orientation.z = math.sin(options.yaw / 2)
        goal.pose.pose.orientation.w = math.cos(options.yaw / 2)
        navigation = NativeNavigation(
            node,
            goal,
            options.execute,
            confirm_navigation_idle=options.confirm_navigation_idle,
            bootstrap_navigation=options.bootstrap_navigation,
            timeout=options.timeout,
        )
        cleanup.callback(navigation.close)
        for number in (signal.SIGINT, signal.SIGTERM):
            signal.signal(number, navigation.request_stop)
        navigation.run()


if __name__ == "__main__":
    main()
