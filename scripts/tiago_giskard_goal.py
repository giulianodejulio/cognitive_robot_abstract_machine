"""Send relative straight goals through a one-shot or persistent TIAGo client."""

# %% imports
from __future__ import annotations

import argparse
import fcntl
import math
import os
import re
import select
import stat
import sys
import time
from _thread import LockType
from concurrent.futures import Future, ThreadPoolExecutor
from contextlib import ExitStack
from dataclasses import dataclass, field
from functools import partial
from pathlib import Path
from threading import Lock
from typing_extensions import Callable

from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from coraplex.alternative_motion_mappings.tiago_single_arm import SingleArmBaseMotion
from coraplex.datastructures.dataclasses import Context
from coraplex.execution_environment import real_robot
from coraplex.plans.factories import execute_single
from coraplex.robot_plans import MoveMotion
from experiments.tiago_goal_logging import GoalErrorLog, GoalErrorPhase, PlanarPose
from experiments.tiago_pure_turn import ZeroForwardVelocity
from giskardpy.middleware.ros2 import rospy
from giskardpy.middleware.ros2.exceptions import ExecutionCanceledException
from giskardpy.middleware.ros2.graceful_shutdown import GracefulShutdownSignals
from giskardpy.middleware.ros2.python_interface import GiskardWrapper
from giskardpy.motion_statechart.goals.cartesian_goals import (
    CartesianPoseStraight,
    DifferentialDriveBaseGoal,
)
from giskardpy.motion_statechart.goals.templates import Parallel
from giskardpy.motion_statechart.tasks.cartesian_tasks import (
    CartesianOrientation,
    CartesianPosition,
    CartesianPositionStraight,
)
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer
from semantic_digital_twin.robots.tiago_single_arm import TiagoSingleArm
from semantic_digital_twin.spatial_types import Pose


# %% request arguments
@dataclass
class GoalOptions:
    """The distance and explicit opt-in of a real forward motion request."""

    distance: float = 0.3
    """Forward distance in meters along the initial heading."""
    execute: bool = False
    """Whether the operator explicitly requested execution."""
    check: bool = False
    """Whether to inspect preflight only, without connecting to the goal server."""
    interactive: bool = False
    """Whether to keep the connected client waiting for terminal input."""
    listen: Path | None = None
    """The local FIFO on which to accept separate shell requests."""

    @classmethod
    def parse(cls, arguments: list[str] | None = None) -> GoalOptions:
        """Validate arguments before initializing any ROS interfaces."""
        defaults = cls()
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument(
            "--distance",
            type=float,
            default=defaults.distance,
            metavar="METERS",
            help="Positive forward distance in meters (default: 0.3)",
        )
        mode = parser.add_mutually_exclusive_group()
        mode.add_argument(
            "--execute",
            action="store_true",
            help="Send a real motion goal after preflight checks",
        )
        mode.add_argument(
            "--check",
            action="store_true",
            help="Check feedback and mux without sending a goal",
        )
        mode.add_argument(
            "--interactive",
            action="store_true",
            help="Keep connections warm and read distances from this terminal",
        )
        mode.add_argument(
            "--listen",
            type=Path,
            metavar="FIFO",
            help="Keep connections warm and read distances from a local FIFO",
        )
        options = cls(**vars(parser.parse_args(arguments)))
        if not math.isfinite(options.distance) or options.distance <= 0:
            parser.error("--distance must be positive and finite")
        if not (
            options.execute
            or options.check
            or options.interactive
            or options.listen is not None
        ):
            parser.error("--execute and current operator authorization are required")
        return options

    def preflight(self, feedback: BaseFeedback) -> None:
        """Require motion readiness for one-shot commands, not persistent startup."""
        if self.listen is not None or self.interactive:
            return
        deadline = time.monotonic() + 10
        while not feedback.ready() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not feedback.ready():
            raise RuntimeError(
                "Preflight failed:\n- " + "\n- ".join(feedback.blocking_reasons())
            )


# %% fixed-heading straight motion
@dataclass(frozen=True)
class MotionTolerances:
    """Completion thresholds shared by the goal tasks and request log."""

    position: float = 0.02  # 0.01 original value
    """Position tolerance in meters."""
    orientation_degrees: float = 2 # 5: safe value. 1: original value
    """Orientation tolerance in degrees."""

    def describe(self) -> str:
        """Describe the configured position and orientation thresholds."""
        return f"{self.position:g} m / {self.orientation_degrees:g} deg"


@dataclass
class StraightBaseMotion(SingleArmBaseMotion):
    """Track a straight line and fixed orientation with configured thresholds."""

    tolerances: MotionTolerances = MotionTolerances()
    """Immutable completion thresholds used by this motion mapping."""

    @property
    def _motion_chart(self) -> Parallel:
        goal = CartesianPoseStraight(
            root_link=self.world.root, tip_link=self.robot.root, goal_pose=self.target
        )
        return Parallel(
            name="CartesianPoseStraight",
            nodes=[
                CartesianPositionStraight(
                    name="position",
                    root_link=goal.root_link,
                    tip_link=goal.tip_link,
                    goal_point=goal.goal_pose.position,
                    weight=goal.weight,
                    binding_policy=goal.binding_policy,
                    threshold=self.tolerances.position,
                ),
                CartesianOrientation(
                    name="orientation",
                    root_link=goal.root_link,
                    tip_link=goal.tip_link,
                    goal_orientation=goal.goal_pose.rotation_matrix,
                    weight=goal.weight,
                    binding_policy=goal.binding_policy,
                    threshold=math.radians(self.tolerances.orientation_degrees),
                ),
            ],
        )


# %% native differential-drive phases
@dataclass
class DifferentialBaseMotion(SingleArmBaseMotion):
    """Reach a relative base pose with native differential-drive phases."""

    @property
    def _motion_chart(self) -> DifferentialDriveBaseGoal:
        tolerances = StraightBaseMotion.tolerances
        return DifferentialDriveBaseGoal(
            goal_pose=self.target,
            threshold=tolerances.position,
            orientation_threshold=math.radians(tolerances.orientation_degrees),
        )


# %% rotation while holding the initial position
@dataclass
class InPlaceTurnMotion(SingleArmBaseMotion):
    """Reach a heading while holding the supplied initial odometry position."""

    @property
    def _motion_chart(self) -> Parallel:
        tolerances = StraightBaseMotion.tolerances
        return Parallel(
            name="InPlaceTurn",
            nodes=[
                CartesianPosition(
                    name="position",
                    root_link=self.world.root,
                    tip_link=self.robot.root,
                    goal_point=self.target.position,
                    threshold=tolerances.position,
                ),
                CartesianOrientation(
                    name="orientation",
                    root_link=self.world.root,
                    tip_link=self.robot.root,
                    goal_orientation=self.target.rotation_matrix,
                    threshold=math.radians(tolerances.orientation_degrees),
                ),
            ],
        )


# %% optional pure rotation experiment
@dataclass
class PureTurnMotion(SingleArmBaseMotion):
    """Reach a heading with zero longitudinal velocity and no position recovery."""

    @property
    def _motion_chart(self) -> Parallel:
        return Parallel(
            name="PureTurn",
            nodes=[
                CartesianOrientation(
                    name="orientation",
                    root_link=self.world.root,
                    tip_link=self.robot.root,
                    goal_orientation=self.target.rotation_matrix,
                    threshold=math.radians(
                        StraightBaseMotion.tolerances.orientation_degrees
                    ),
                ),
                ZeroForwardVelocity(name="zero_forward_velocity"),
            ],
        )


# %% one timestamp per request
@dataclass
class WheelOnset:
    """Measure first significant wheel feedback after accepting a local request."""

    minimum_linear_speed: float = 0.005
    """Linear velocity threshold in meters per second."""
    minimum_angular_speed: float = 0.01
    """Angular velocity threshold in radians per second."""
    requested_at: float | None = field(default=None, init=False)
    """Local monotonic time of request reception."""
    observed_at: float | None = field(default=None, init=False)
    """Local monotonic time of first significant wheel feedback."""
    lock: LockType = field(default_factory=Lock, init=False, repr=False)
    """Protect the request timestamp from ROS callback races."""

    def arm(self, requested_at: float) -> None:
        """Reset the single onset timestamp for a new request."""
        with self.lock:
            self.observed_at = None
            self.requested_at = requested_at

    def observe(self, message: Odometry, received_at: float | None = None) -> None:
        """Retain only the first callback reporting significant measured velocity."""
        received_at = time.monotonic() if received_at is None else received_at
        twist = message.twist.twist
        moving = (
            abs(twist.linear.x) >= self.minimum_linear_speed
            or abs(twist.angular.z) >= self.minimum_angular_speed
        )
        with self.lock:
            if (
                self.requested_at is not None
                and self.observed_at is None
                and moving
                and received_at >= self.requested_at
            ):
                self.observed_at = received_at

    @property
    def latency(self) -> float | None:
        """Request-to-wheel feedback interval, including transport and publication delay."""
        with self.lock:
            if self.requested_at is None or self.observed_at is None:
                return None
            return self.observed_at - self.requested_at


# %% real feedback and safety checks
@dataclass
class BaseFeedback:
    """Keep only the latest real samples needed for preflight and cancellation."""

    node: Node
    """The spinning ROS node owning the subscriptions."""
    emergency: Bool | None = field(default=None, init=False)
    """The latched emergency state."""
    joy: Bool | None = field(default=None, init=False)
    """The latched teleoperation priority state."""
    odometry: Odometry | None = field(default=None, init=False)
    """The laser odometry used by the server."""
    joints: JointState | None = field(default=None, init=False)
    """The real joint positions and sample timestamp."""
    wheel_odometry: Odometry | None = field(default=None, init=False)
    """Latest wheel sample for checking rest and estimating onset."""
    stationary_since: float | None = field(default=None, init=False)
    """Receipt time of the first sample in the current stationary interval."""
    stationary_samples: int = field(default=0, init=False)
    """Number of consecutive wheel samples below movement thresholds."""
    wheel_received_at: float | None = field(default=None, init=False)
    """Receipt time of the latest wheel message."""
    onset: WheelOnset = field(default_factory=WheelOnset, init=False)
    """One onset measurement; no traffic history is retained."""
    mux: dict[str, str] = field(default_factory=dict, init=False)
    """The latest mux lock diagnostic values."""
    subscriptions: list = field(default_factory=list, init=False)
    """Subscriptions retained for the client's lifetime."""

    def __post_init__(self) -> None:
        self.subscriptions = [
            self.node.create_subscription(
                Bool,
                "/power/is_emergency",
                self.receive_emergency,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            ),
            self.node.create_subscription(
                Bool,
                "/joy_priority",
                self.receive_joy,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            ),
            self.node.create_subscription(
                Odometry, "/direct_laser_odometry/odom", self.receive_odometry, 1
            ),
            self.node.create_subscription(
                JointState, "/joint_states", self.receive_joints, 1
            ),
            self.node.create_subscription(
                DiagnosticArray, "/diagnostics", self.receive_diagnostics, 1
            ),
            self.node.create_subscription(
                Odometry, "/mobile_base_controller/odom", self.receive_wheels, 1
            ),
        ]

    def receive_emergency(self, message: Bool) -> None:
        """Update the emergency interlock."""
        self.emergency = message

    def receive_joy(self, message: Bool) -> None:
        """Update the operator control lock."""
        self.joy = message

    def receive_odometry(self, message: Odometry) -> None:
        """Update the measured base pose."""
        self.odometry = message

    def receive_joints(self, message: JointState) -> None:
        """Update the measured joint configuration."""
        self.joints = message

    def receive_wheels(self, message: Odometry) -> None:
        """Update wheel state and capture at most one movement timestamp."""
        now = time.monotonic()
        if self.wheel_received_at is None or now - self.wheel_received_at > 2:
            self.stationary_since = None
            self.stationary_samples = 0
        self.wheel_received_at = now
        self.wheel_odometry = message
        self.onset.observe(message)
        twist = message.twist.twist
        if (
            abs(twist.linear.x) < self.onset.minimum_linear_speed
            and abs(twist.angular.z) < self.onset.minimum_angular_speed
        ):
            if self.stationary_since is None:
                self.stationary_since = now
            self.stationary_samples += 1
        else:
            self.stationary_since = None
            self.stationary_samples = 0

    def wheels_settled(self) -> bool:
        """Require at least three fresh stationary samples spanning 0.2 seconds."""
        return (
            self.wheels_stationary()
            and self.stationary_since is not None
            and self.stationary_samples >= 3
            and time.monotonic() - self.stationary_since >= 0.2
        )

    def wheels_stationary(self) -> bool:
        """Require recent finite wheel feedback below the onset thresholds."""
        message = self.wheel_odometry
        if message is None:
            return False
        now = self.node.get_clock().now().nanoseconds / 1e9
        age = now - (message.header.stamp.sec + message.header.stamp.nanosec / 1e9)
        twist = message.twist.twist
        return (
            abs(age) <= 2
            and abs(twist.linear.x) < self.onset.minimum_linear_speed
            and abs(twist.angular.z) < self.onset.minimum_angular_speed
        )

    def receive_diagnostics(self, message: DiagnosticArray) -> None:
        """Update only mux diagnostic values."""
        for status in message.status:
            if "twist_mux" in status.name:
                self.mux.update({item.key: item.value for item in status.values})

    def ready(self, require_free_mux: bool = True) -> bool:
        """Require finite, fresh feedback and released operator interlocks."""
        reasons = self.blocking_reasons(require_free_mux)
        return len(reasons) == 0

    def blocking_reasons(self, require_free_mux: bool = True) -> list[str]:
        """Describe missing samples, interlocks and timestamp offsets preventing motion."""
        reasons = []
        if self.emergency is None:
            reasons.append("missing /power/is_emergency (transient-local)")
        elif self.emergency.data:
            reasons.append("is_emergency=true")
        if self.joy is None:
            reasons.append("missing /joy_priority (transient-local)")
        elif self.joy.data:
            reasons.append("joy_priority=true")
        now = self.node.get_clock().now().nanoseconds / 1e9
        for topic, message in (
            ("/direct_laser_odometry/odom", self.odometry),
            ("/joint_states", self.joints),
        ):
            if message is None:
                reasons.append(f"missing {topic}")
                continue
            age = now - (message.header.stamp.sec + message.header.stamp.nanosec / 1e9)
            if abs(age) > 2:
                reasons.append(f"{topic}: timestamp offset {age:+.3f} s exceeds ±2 s")
        if self.joints is not None and (
            not self.joints.position
            or not all(math.isfinite(value) for value in self.joints.position)
        ):
            reasons.append("joint positions missing or nonfinite")
        if self.odometry is not None:
            pose = self.odometry.pose.pose
            if not all(
                math.isfinite(value)
                for value in (
                    pose.position.x,
                    pose.position.y,
                    pose.orientation.z,
                    pose.orientation.w,
                )
            ):
                reasons.append("odometry pose nonfinite")
        if require_free_mux:
            priority = self.mux.get("current priority")
            if priority is None:
                reasons.append("missing twist_mux diagnostics")
            elif priority != "0":
                reasons.append(f"mux current priority={priority} (expected 0)")
        reasons.extend(
            f"{key}: {value.strip()}"
            for key, value in self.mux.items()
            if key.startswith("lock ") and "locked" in value
        )
        return reasons


# %% cancellation and client lifecycle
@dataclass
class GoalCancellation:
    """Cancel only this client's outstanding action on interruption or timeout."""

    wrapper: GiskardWrapper
    """The client that owns the action handle."""
    execution: Future
    """The task waiting for the action result."""
    requested: bool = False
    """Whether cancellation was already sent."""

    def cancel(self) -> None:
        """Request cancellation when an accepted action is still running."""
        if (
            not self.execution.done()
            and not self.requested
            and self.wrapper._client._goal_handle is not None
        ):
            self.wrapper.cancel_goal_async()
            self.requested = True


# %% connected request session
@dataclass
class GoalSession:
    """Reuse one world, action client and worker for successive relative goals."""

    context: Context
    """The already connected CRAM context."""
    feedback: BaseFeedback
    """Latest measured state and operator interlocks."""
    pool: ThreadPoolExecutor
    """The worker retained across requests."""
    last_status: tuple[str, ...] | None = field(default=None, init=False)
    """The last reported blockers, to log only state transitions."""
    last_outcome: str = field(default="rejected", init=False)
    """The terminal outcome returned to the command that owns the request."""

    def report_status(self) -> None:
        """Report changes in permission to move while keeping the client connected."""
        reasons = self.feedback.blocking_reasons()
        if not self.feedback.wheels_stationary():
            reasons.append("wheel feedback missing, stale or robot already moving")
        summaries = set()
        operator_block = any(
            "joy_priority=true" in reason
            or "lock locks.joystick" in reason
            or "lock locks.charging" in reason
            for reason in reasons
        )
        for reason in reasons:
            if "joy_priority=true" in reason or "lock locks.joystick" in reason:
                summaries.add("teleop attiva")
            elif "lock locks.charging" in reason:
                summaries.add("ricarica attiva")
            elif reason.startswith("mux current priority") and operator_block:
                continue
            elif reason.startswith("wheel feedback"):
                summaries.add("ruote non ferme o feedback ruote assente")
            elif ": timestamp offset" in reason:
                summaries.add(
                    reason.partition(": timestamp offset")[0]
                    + ": timestamp fuori limite"
                )
            else:
                summaries.add(reason)
        status = tuple(sorted(summaries))
        if status == self.last_status:
            return
        self.last_status = status
        print(
            ("GOALS BLOCKED: " + "; ".join(status) if status else "GOALS ENABLED"),
            flush=True,
        )

    @staticmethod
    def parse_distance(line: str) -> float | None:
        """Accept only finite positive numeric requests."""
        if re.fullmatch(r"\+?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?", line) is None:
            return None
        distance = float(line)
        return distance if math.isfinite(distance) and distance > 0 else None

    def interact(self) -> None:
        """Read distances without reconnecting; quit or EOF closes the client."""
        while True:
            print("READY CLIENT: enter distance in meters, or quit", flush=True)
            line = sys.stdin.readline()
            requested_at = time.monotonic()
            if not line or line.strip() in ("quit", "exit"):
                return
            distance = self.parse_distance(line.strip())
            if distance is None:
                print(
                    "Rejected: expected a positive finite distance in meters",
                    flush=True,
                )
                continue
            self.send(distance, requested_at=requested_at)

    def send(
        self,
        distance: float,
        requested_at: float | None = None,
        cancel_requested: Callable[[], bool] | None = None,
        turn_degrees: float = 0,
        differential_drive: bool = False,
        pure_turn: bool = False,
    ) -> bool:
        """Reject a blocked request or execute it without closing the connection."""
        requested_at = time.monotonic() if requested_at is None else requested_at
        description = (
            f"turn={turn_degrees:g} deg" if turn_degrees else f"distance={distance:g} m"
        )
        print(
            f"CLIENT RECEIVED: {description}; time={time.time():.6f}",
            flush=True,
        )
        self.last_outcome = "rejected"
        if (
            not math.isfinite(distance)
            or not math.isfinite(turn_degrees)
            or abs(turn_degrees) > 180
            or (differential_drive and turn_degrees != 0)
            or (pure_turn and turn_degrees == 0)
            or (distance != 0 if turn_degrees else distance <= 0)
        ):
            print(
                "REJECTED: expected positive distance or a turn within +/-180 deg",
                flush=True,
            )
            return False
        feedback = self.feedback
        context = self.context
        if not feedback.ready():
            print(
                "REJECTED (not queued):\n- " + "\n- ".join(feedback.blocking_reasons()),
                flush=True,
            )
            return False
        if not feedback.wheels_stationary():
            print(
                "REJECTED (not queued): wheel feedback missing, stale or robot already moving",
                flush=True,
            )
            return False
        wrapper = context.giskard_wrapper
        feedback.onset.arm(requested_at)
        initial = feedback.odometry.pose.pose
        start = PlanarPose.from_ros(initial)
        yaw = start.yaw
        error_log = GoalErrorLog(
            start=start,
            distance=distance,
            requested_at=requested_at,
            turn_degrees=turn_degrees,
        )
        target = Pose.from_xyz_rpy(
            x=initial.position.x + distance * math.cos(yaw),
            y=initial.position.y + distance * math.sin(yaw),
            yaw=yaw + math.radians(turn_degrees),
            reference_frame=context.world.root,
        )
        with real_robot(collision_avoidance=True):
            motion = InPlaceTurnMotion(target) if turn_degrees else MoveMotion(target)
            movement = (
                f"{turn_degrees:g} deg turn; held position"
                if turn_degrees
                else f"{distance:g} m forward; fixed heading"
            )
            if differential_drive:
                motion = DifferentialBaseMotion(target)
                movement = f"{distance:g} m forward; DifferentialDriveBaseGoal; final heading equals initial"
            if pure_turn:
                motion = PureTurnMotion(target)
                movement = f"{turn_degrees:g} deg PURE TURN; solver forward velocity=0; no position recovery"
            plan = execute_single(motion, context=context)
            with ExitStack() as goal_cleanup:
                print(
                    f"SENDING: {movement}; tolerances {StraightBaseMotion.tolerances.describe()}; time={time.time():.6f}",
                    flush=True,
                )
                future = self.pool.submit(plan.perform)
                cancellation = GoalCancellation(wrapper, future)
                goal_cleanup.callback(cancellation.cancel)
                deadline = time.monotonic() + 60
                onset_reported = False
                operator_cancel = False
                while not future.done():
                    error_log.report(feedback.odometry, GoalErrorPhase.TRACKING)
                    if (
                        cancel_requested is not None
                        and cancel_requested()
                        and not operator_cancel
                    ):
                        operator_cancel = True
                        print(f"CANCEL REQUESTED: time={time.time():.6f}", flush=True)
                    if (
                        operator_cancel
                        or time.monotonic() >= deadline
                        or not feedback.ready(require_free_mux=False)
                    ):
                        cancellation.cancel()
                    if not onset_reported and feedback.onset.latency is not None:
                        print(
                            f"WHEEL ONSET: {feedback.onset.latency:.3f} s from local request (odometry receipt)",
                            flush=True,
                        )
                        onset_reported = True
                    time.sleep(0.05)
                error = future.exception()
                error_log.report(feedback.odometry, GoalErrorPhase.RESULT, force=True)
                if error is not None:
                    if isinstance(error, ExecutionCanceledException):
                        self.last_outcome = "canceled"
                        print(
                            f"CANCELED: time={time.time():.6f}; client remains connected",
                            flush=True,
                        )
                        return False
                    self.last_outcome = "failed"
                    print(
                        f"GOAL FAILED: {error!r}; client remains connected", flush=True
                    )
                    return False
                print(
                    f"SUCCESS: time={time.time():.6f}; elapsed={time.monotonic() - requested_at:.3f} s (client result)",
                    flush=True,
                )
        time.sleep(1)
        error_log.report(feedback.odometry, GoalErrorPhase.SETTLED, force=True)
        final = feedback.odometry.pose.pose
        dx = final.position.x - initial.position.x
        dy = final.position.y - initial.position.y
        print(
            f"FINAL POSE: forward={math.cos(yaw)*dx + math.sin(yaw)*dy:.3f} m, lateral={-math.sin(yaw)*dx + math.cos(yaw)*dy:.3f} m",
            flush=True,
        )
        print(f"REQUEST TO WHEEL FEEDBACK: {feedback.onset.latency} s", flush=True)
        self.last_outcome = "succeeded"
        return True


# %% local shell transport
@dataclass
class GoalRequest:
    """A forward distance with an optional local reply owned by its CLI process."""

    distance: float | None
    """Forward distance in meters, or None for a readiness query."""
    identifier: str | None = None
    """Unique CLI request identity for targeted cancellation."""
    reply: Path | None = None
    """Existing private status file created by the waiting command."""
    turn_degrees: float = 0
    """Relative yaw change; positive turns left, with zero translation."""
    differential_drive: bool = False
    """Whether to use the native three-phase differential-drive goal."""
    pure_turn: bool = False
    """Whether this turn requires zero longitudinal velocity without position recovery."""
    movement_reported: bool = field(default=False, init=False)
    """Whether the onset state was already sent to the waiting CLI."""

    @classmethod
    def parse(cls, line: str) -> GoalRequest | None:
        """Accept legacy distances or validated foreground request envelopes."""
        if not line.startswith(
            ("request ", "ready ", "turn ", "differential ", "pure-turn ")
        ):
            distance = GoalSession.parse_distance(line)
            return None if distance is None else cls(distance)
        fields = line.split()
        turn_degrees = 0
        if fields[0] == "ready" and len(fields) == 3:
            distance = None
            reply = Path(fields[2])
        elif fields[0] in ("turn", "pure-turn") and len(fields) == 4:
            if (
                re.fullmatch(
                    r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?", fields[2]
                )
                is None
            ):
                return None
            turn_degrees = float(fields[2])
            if not math.isfinite(turn_degrees) or not 0 < abs(turn_degrees) <= 180:
                return None
            distance = 0
            reply = Path(fields[3])
        elif fields[0] in ("request", "differential") and len(fields) == 4:
            distance = GoalSession.parse_distance(fields[2])
            if distance is None:
                return None
            reply = Path(fields[3])
        else:
            return None
        if not fields[1].startswith("tiago-goal."):
            return None
        if (
            reply.name != "status"
            or reply.parent.name != fields[1]
            or reply.parent.parent != Path("/tmp")
        ):
            return None
        if reply.is_symlink() or reply.parent.is_symlink() or not reply.is_file():
            return None
        if (
            reply.stat().st_uid != os.getuid()
            or reply.parent.stat().st_uid != os.getuid()
        ):
            return None
        return cls(
            distance,
            fields[1],
            reply,
            turn_degrees,
            differential_drive=fields[0] == "differential",
            pure_turn=fields[0] == "pure-turn",
        )

    def publish(self, outcome: str) -> None:
        """Reply only while the request owner's status file still exists."""
        if (
            self.reply is not None
            and self.reply.is_file()
            and not self.reply.is_symlink()
        ):
            self.reply.write_text(outcome + "\n")

    def report_movement(self, session: GoalSession) -> None:
        """Publish one measured onset state for an automatic cancellation timer."""
        if not self.movement_reported and session.feedback.onset.latency is not None:
            self.publish("moving")
            self.movement_reported = True


@dataclass
class GoalInbox:
    """Accept local FIFO commands without starting another Python/ROS client."""

    path: Path
    """Named pipe shared with shell commands in the same container."""
    pending: bytes = field(default=b"", init=False)
    """Partial input retained so a batched request and cancellation stay ordered."""

    def poll_cancel(self, descriptor: int, active_id: str | None = None) -> bool:
        """Read cancellation while busy and reject other requests without queuing."""
        cancel = False
        while select.select([descriptor], [], [], 0)[0]:
            self.pending += os.read(descriptor, 4096)
        while b"\n" in self.pending:
            raw, _, self.pending = self.pending.partition(b"\n")
            line = raw.decode(errors="replace").strip()
            if line == "cancel" or (
                active_id is not None and line == f"cancel {active_id}"
            ):
                cancel = True
            elif line.startswith("cancel "):
                print("CANCEL ignored: request is not the active goal", flush=True)
            elif line:
                request = GoalRequest.parse(line)
                if request is not None:
                    request.publish(
                        "blocked" if request.distance is None else "rejected"
                    )
                    if request.distance is None:
                        continue
                print(
                    "REJECTED while busy (not queued); use Ctrl+C in the owning CLI",
                    flush=True,
                )
        return cancel

    def serve(self, session: GoalSession) -> None:
        """Process one goal at a time and discard requests queued while busy."""
        if self.path.exists() or self.path.is_symlink():
            status = self.path.lstat()
            if not stat.S_ISFIFO(status.st_mode) or status.st_uid != os.getuid():
                raise ValueError(f"Refusing non-owned or non-FIFO path: {self.path}")
        else:
            os.mkfifo(self.path, mode=0o600)
        with ExitStack() as cleanup:
            descriptor = os.open(self.path, os.O_RDWR | os.O_NONBLOCK)
            cleanup.callback(os.close, descriptor)
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
            cleanup.callback(self.path.unlink, missing_ok=True)
            while True:
                print(f"READY CLIENT: printf '0.30\\n' > {self.path}", flush=True)
                while b"\n" not in self.pending:
                    session.report_status()
                    if select.select([descriptor], [], [], 0.5)[0]:
                        self.pending += os.read(descriptor, 4096)
                requested_at = time.monotonic()
                raw, _, self.pending = self.pending.partition(b"\n")
                line = raw.decode(errors="replace").strip()
                if line in ("quit", "exit"):
                    return
                if line == "cancel" or line.startswith("cancel "):
                    print("CANCEL: no active goal", flush=True)
                    continue
                request = GoalRequest.parse(line)
                if request is None:
                    print("Rejected: expected one positive finite distance", flush=True)
                    continue
                if request.distance is None:
                    request.publish(
                        "ready"
                        if session.feedback.ready()
                        and session.feedback.wheels_settled()
                        else "blocked"
                    )
                    continue
                if self.poll_cancel(descriptor, active_id=request.identifier):
                    request.publish("canceled")
                    continue
                request.publish("accepted")

                def read_control() -> bool:
                    """Report onset and read cancel commands while this goal is active."""
                    request.report_movement(session)
                    return self.poll_cancel(descriptor, active_id=request.identifier)

                send = (
                    partial(session.send, turn_degrees=request.turn_degrees)
                    if request.turn_degrees
                    else session.send
                )
                if request.differential_drive:
                    send = partial(session.send, differential_drive=True)
                if request.pure_turn:
                    send = partial(
                        session.send, turn_degrees=request.turn_degrees, pure_turn=True
                    )
                success = send(
                    request.distance,
                    requested_at=requested_at,
                    cancel_requested=read_control,
                )
                request.publish("succeeded" if success else session.last_outcome)
                self.poll_cancel(descriptor)


# %% startup once, request many times
def main() -> None:
    """Connect once and serve local requests or send a single explicit goal."""
    options = GoalOptions.parse()
    with ExitStack() as cleanup:
        rospy.init_node("tiago_giskard_goal")
        cleanup.callback(rospy.shutdown)
        cleanup.enter_context(GracefulShutdownSignals())
        node = rospy.get_node()
        feedback = BaseFeedback(node)
        options.preflight(feedback)
        if options.check:
            print("PREFLIGHT OK: no goal sent", flush=True)
            return
        world = fetch_world_from_service(node, timeout_seconds=12)
        if str(world.root.name) != "odom":
            raise RuntimeError("Expected the odom-only TIAGo bringup server")
        synchronizer = WorldSynchronizer(_world=world, node=node)
        cleanup.callback(synchronizer.close)
        robot = world.get_semantic_annotations_by_type(TiagoSingleArm)[0]
        context = Context(
            world=world,
            robot=robot,
            ros_node=node,
            alternative_motion_mappings=[StraightBaseMotion],
        )
        cleanup.callback(context.giskard_wrapper.close)
        pool = cleanup.enter_context(ThreadPoolExecutor(max_workers=1))
        pool.submit(time.monotonic).result()
        session = GoalSession(context=context, feedback=feedback, pool=pool)
        if options.listen is not None:
            GoalInbox(options.listen).serve(session)
        elif options.interactive:
            session.interact()
        else:
            if not session.send(options.distance):
                raise RuntimeError(
                    "Goal not completed; see rejection or failure details above"
                )


if __name__ == "__main__":
    main()
