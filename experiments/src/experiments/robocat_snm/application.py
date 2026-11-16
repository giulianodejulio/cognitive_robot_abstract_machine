"""Execute a supervised native Nav2 to Giskard handover without HPM."""

# %% imports
from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from concurrent.futures import Future, ThreadPoolExecutor
from contextlib import ExitStack
from dataclasses import dataclass, field, replace
from pathlib import Path
from enum import StrEnum
from types import FrameType

import yaml
from geometry_msgs.msg import Pose as GeometryPose
from rclpy.parameter_client import AsyncParameterClient
from rclpy.utilities import remove_ros_args
from rosidl_runtime_py.set_message import set_message_fields
from std_msgs.msg import String
from typing_extensions import Callable

from coraplex.datastructures.dataclasses import Context
from coraplex.execution_environment import real_robot
from coraplex.plans.factories import execute_single
from experiments.robocat_safety import RobocatEnvironment, FeedbackSource
from experiments.robocat_snm.exceptions import EntryError
from experiments.robocat_snm.door_passage import CalibratedDoorPassage
from experiments.robocat_snm.feedback import EntryInputs
from experiments.robocat_snm.geometry import EntryGeometry, StagingAlignment
from experiments.robocat_snm.native_navigation import NavigationStatus
from experiments.robocat_snm.navigation import NavigationConnection
from experiments.robocat_snm.policy import EntryPhase, EntryPolicy
from giskardpy.middleware.ros2 import rospy
from scripts.robocat_giskard_goal import RobocatStraightMotion
from scripts.tiago_giskard_goal import GoalCancellation, MotionTolerances
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer
from semantic_digital_twin.robots.robocat import Robocat
from semantic_digital_twin.spatial_types import Pose


# %% request configuration
@dataclass
class EntryOptions:
    """Explicit calibration and execution choices for one supervised trial."""

    staging: Path = Path("doc/robocat_elevator_staging.yaml")
    """Saved staging pose whose map alignment the operator has checked."""
    doorway_distance: float | None = None
    """Measured threshold distance along the staging waypoint's forward axis."""
    distance: float = 1.5
    """Inside target distance from staging, in metres."""
    feedback: FeedbackSource = FeedbackSource.WHEEL
    """Odometry used consistently for motion targets and entry geometry."""
    map_fingerprint: str | None = None
    """Expected active-map identity printed by the non-executing check."""
    execute: bool = False
    """Explicit authorization to submit movement goals."""
    confirm_navigation_idle: bool = False
    """Operator confirms Nav2 is idle for a read-only preflight check."""
    bootstrap_navigation: bool = False
    """Explicitly authorize the first Nav2 goal when no action status has been received."""
    entry_only: bool = False
    """Skip Nav2 submission while still requiring native navigation to be idle."""
    handover_only: bool = False
    """Stop after native staging arrival and verified handover; never submit Giskard motion."""
    handover_position_tolerance: float | None = None
    """Explicit staging position acceptance for handover-only diagnostic trials."""
    handover_orientation_tolerance_deg: float | None = None
    """Explicit staging heading acceptance in degrees for handover-only diagnostic trials."""
    staging_position_tolerance: float = StagingAlignment.position_tolerance
    """Maximum staging position error before elevator entry, in metres."""
    staging_orientation_tolerance_deg: float = math.degrees(
        StagingAlignment.orientation_tolerance
    )
    """Maximum staging heading error before elevator entry, in degrees."""
    doors_held_open: bool = False
    """Operator confirms the doors remain held open throughout this supervised trial."""
    pass_door_action: bool = False
    """Compare the native door action with the existing single-motion entry."""
    retain_door_clearance: bool = EntryPolicy.retain_door_clearance
    """Compare retained doorway authorization without changing the baseline motion."""
    allow_isolated_scan_gaps: bool = False
    """Explicitly permit single missing lidar rays with measured clear neighbours within one centimetre."""
    wait_timeout: float = 60.0
    """Maximum wait for stable entry conditions, in seconds."""
    motion_timeout: float = 60.0
    """Maximum entry motion duration before cancellation is requested."""
    navigation_timeout: float = 180.0
    """Maximum wait for native staging navigation, in seconds."""

    @classmethod
    def parse(cls, arguments: list[str] | None = None) -> EntryOptions:
        """Validate execution opt-in before creating action clients."""
        defaults = cls()
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("--staging", type=Path, default=defaults.staging)
        parser.add_argument("--doorway-distance", type=float)
        parser.add_argument("--distance", type=float, default=defaults.distance)
        parser.add_argument(
            "--feedback",
            type=FeedbackSource,
            choices=list(FeedbackSource),
            default=defaults.feedback,
        )
        parser.add_argument("--map-fingerprint")
        parser.add_argument("--execute", action="store_true")
        parser.add_argument("--confirm-navigation-idle", action="store_true")
        parser.add_argument(
            "--bootstrap-navigation",
            action="store_true",
            help="Explicitly authorize first Nav2 goal without initial action status, only with fresh interlocks and a stopped free base",
        )
        parser.add_argument("--entry-only", action="store_true")
        parser.add_argument(
            "--handover-only",
            action="store_true",
            help="Navigate to staging only with --execute; never send a Giskard goal",
        )
        parser.add_argument("--doors-held-open", action="store_true")
        door_mode = parser.add_mutually_exclusive_group()
        door_mode.add_argument(
            "--pass-door-action",
            action="store_true",
            help="Use native PassDoorAction, including its staging correction and latched doorway monitor",
        )
        door_mode.add_argument(
            "--retain-door-clearance",
            action="store_true",
            help="Retain stable doorway opening during the baseline motion; keep continuous path and safety checks",
        )
        parser.add_argument(
            "--allow-isolated-scan-gaps",
            action="store_true",
            help="Opt in to interior single-ray gaps bounded by clear measurements at most 1 cm apart",
        )
        parser.add_argument(
            "--staging-position-tolerance",
            type=float,
            default=defaults.staging_position_tolerance,
        )
        parser.add_argument(
            "--staging-orientation-tolerance-deg",
            type=float,
            default=defaults.staging_orientation_tolerance_deg,
        )
        parser.add_argument(
            "--handover-position-tolerance",
            type=float,
            help="Staging acceptance in metres; only with --handover-only",
        )
        parser.add_argument(
            "--handover-orientation-tolerance-deg",
            type=float,
            help="Staging heading acceptance in degrees; only with --handover-only",
        )
        parser.add_argument("--wait-timeout", type=float, default=defaults.wait_timeout)
        parser.add_argument(
            "--motion-timeout", type=float, default=defaults.motion_timeout
        )
        parser.add_argument(
            "--navigation-timeout", type=float, default=defaults.navigation_timeout
        )
        options = cls(**vars(parser.parse_args(arguments)))
        handover_tolerances = (
            options.handover_position_tolerance,
            options.handover_orientation_tolerance_deg,
        )
        if (
            any(value is not None for value in handover_tolerances)
            and not options.handover_only
        ):
            parser.error("Explicit handover tolerances require --handover-only")
        if any(
            value is not None and (not math.isfinite(value) or value <= 0)
            for value in handover_tolerances
        ):
            parser.error("Handover tolerances must be positive and finite")
        if options.confirm_navigation_idle and options.execute:
            parser.error(
                "--confirm-navigation-idle is only allowed for read-only checks"
            )
        if options.entry_only and options.handover_only:
            parser.error("--entry-only and --handover-only are mutually exclusive")
        if options.bootstrap_navigation and (not options.execute or options.entry_only):
            parser.error(
                "--bootstrap-navigation requires --execute and native staging navigation"
            )
        if options.execute and not options.map_fingerprint:
            parser.error("Execution requires --map-fingerprint")
        if (
            options.execute
            and not options.handover_only
            and (
                options.doorway_distance is None
                or not options.map_fingerprint
                or not options.doors_held_open
            )
        ):
            parser.error(
                "Execution requires --doorway-distance, --map-fingerprint and --doors-held-open"
            )
        values = (
            options.distance,
            options.staging_position_tolerance,
            options.staging_orientation_tolerance_deg,
            options.wait_timeout,
            options.motion_timeout,
            options.navigation_timeout,
        )
        if not all(math.isfinite(value) and value > 0 for value in values):
            parser.error("Distances and timeouts must be positive and finite")
        return options

    def verify_server_topics(self, feedback_topic: str, command_topic: str) -> None:
        """Reject command routing or pose feedback inconsistent with this request."""
        if feedback_topic != self.feedback.topic or command_topic != "/snm_vel":
            raise EntryError(
                f"Giskard feedback must be {self.feedback.topic} and command topic /snm_vel; "
                f"received {feedback_topic} and {command_topic}"
            )

    def entry_geometry(self, staging: Pose, doorway_distance: float) -> EntryGeometry:
        """Construct entry geometry with staging acceptance independent of goal completion."""
        return EntryGeometry(
            staging,
            doorway_distance,
            self.distance,
            staging_tolerance=self.staging_position_tolerance,
            staging_orientation_tolerance=math.radians(
                self.staging_orientation_tolerance_deg
            ),
        )

    def load_staging(self) -> Pose:
        """Read the saved native map waypoint with a finite unit quaternion."""
        document = yaml.safe_load(self.staging.read_text())
        if document["frame_id"] != "map":
            raise EntryError("The staging waypoint must be expressed in map")
        message = GeometryPose()
        set_message_fields(message, document["pose"])
        position, quaternion = message.position, message.orientation
        values = (
            position.x,
            position.y,
            position.z,
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        )
        if (
            not all(math.isfinite(value) for value in values)
            or abs(sum(value**2 for value in values[3:]) - 1) > 1e-5
        ):
            raise EntryError(
                "Staging pose contains nonfinite values or a non-unit quaternion"
            )
        return Pose.from_xyz_quaternion(*values)


# %% interruption and event output
@dataclass
class EntryStop:
    """Retain interruption until both owned actions have terminated."""

    requested: bool = False
    """Whether the operator has requested stopping."""

    def receive(self, number: int, frame: FrameType | None) -> None:
        """Record a signal without shutting down ROS before cancellation completes."""
        self.requested = True


@dataclass
class EntryLocalization:
    """Wait for a usable map pose without submitting any motion request."""

    inputs: EntryInputs
    """Current map and synchronized base feedback."""
    stop: EntryStop
    """Operator interruption state."""
    poll_seconds: float = 0.1
    """Interval between map pose checks."""

    def wait(self) -> bool:
        """Resume when localization is available, or return false on interruption."""
        announced = False
        while not self.stop.requested:
            if self.inputs.map_pose() is not None:
                print(
                    "LOCALIZATION READY: map pose available; continuing checks",
                    flush=True,
                )
                return True
            if not announced:
                print(
                    "WAITING FOR LOCALIZATION: set 2D Pose Estimate in RViz; Ctrl+C stops; no goal sent",
                    flush=True,
                )
                announced = True
            time.sleep(self.poll_seconds)
        return False


class TerminalColor(StrEnum):
    """Terminal-only emphasis for operational problems."""

    RED = "\033[31m"
    RESET = "\033[0m"


@dataclass
class EntryEvents:
    """Publish and print state transitions with both wall and monotonic timestamps."""

    publish: Callable[[String], None]
    """State publisher retained by the spinning ROS node."""
    previous: str = field(default="", init=False)
    """Previous state text, avoiding high-rate duplicate output."""

    def report(self, text: str, *, problem: bool = False) -> None:
        """Emit plain ROS events and emphasize problems only in an interactive terminal."""
        if text == self.previous:
            return
        self.previous = text
        line = f"SNM wall={time.time():.6f} monotonic={time.monotonic():.6f} {text}"
        displayed = (
            f"{TerminalColor.RED}{line}{TerminalColor.RESET}"
            if problem and sys.stdout.isatty()
            else line
        )
        print(displayed, flush=True)
        self.publish(String(data=line))


# %% handover and entry execution
@dataclass
class EntryMission:
    """Navigate to staging, then hand the stopped base to one SNM entry request."""

    options: EntryOptions
    """Configured trial limits and opt-in."""
    inputs: EntryInputs
    """Current scene and operator interlocks."""
    navigation: NavigationConnection
    """Owned native navigation action and continuing ownership supervision."""
    stop: EntryStop
    """Operator interruption shared by every phase."""
    events: EntryEvents
    """Lightweight transition output."""
    pool: ThreadPoolExecutor
    """One worker for the blocking native CRAM execution."""
    policy: EntryPolicy = field(default_factory=EntryPolicy)
    """One request's measured entry decisions."""
    motion: Future | None = field(default=None, init=False)
    """Owned CRAM execution future."""
    cancellation: GoalCancellation | None = field(default=None, init=False)
    """Idempotent cancellation of the accepted Giskard goal."""

    def approach(self) -> None:
        """Require native success, free mux and measured rest before handover."""
        if self.stop.requested:
            raise EntryError("Request interrupted before native navigation")
        self.events.report("NAV2: waiting for staging success and a stopped base")
        deadline = time.monotonic() + self.options.navigation_timeout
        self.navigation.start()
        while time.monotonic() < deadline and not self.stop.requested:
            current = self.navigation.poll(
                bootstrap_ready=(
                    self.options.bootstrap_navigation
                    and self.inputs.feedback.ready()
                    and self.inputs.feedback.wheels_settled()
                )
            )
            if not self.inputs.feedback.ready(require_free_mux=False):
                reasons = self.inputs.feedback.blocking_reasons(require_free_mux=False)
                details = (
                    "; ".join(reasons)
                    if reasons
                    else "Interlock changed before details could be collected"
                )
                self.events.report(f"NAV2 INTERLOCK: {details}", problem=True)
                raise EntryError(
                    "Native approach interrupted by lost feedback or an operator/driver interlock: "
                    + details
                )
            if self.navigation.status in (
                NavigationStatus.FAILED,
                NavigationStatus.CANCELED,
            ):
                raise EntryError(
                    f"Native navigation did not succeed: {self.navigation.status}"
                )
            if (
                current
                and self.navigation.status is NavigationStatus.SUCCEEDED
                and self.navigation.idle
                and self.navigation.routing_ready
            ):
                if (
                    self.inputs.feedback.ready()
                    and self.inputs.feedback.wheels_settled()
                ):
                    return
            time.sleep(0.1)
        raise EntryError(
            "Staging navigation was interrupted or did not yield a stopped base before its deadline"
        )

    def handover(self, staging: Pose) -> None:
        """Reach and certify staging without constructing or submitting an entry task."""
        self.approach()
        self.inputs.confirm_handover(
            StagingAlignment(
                staging,
                position_tolerance=(
                    self.options.handover_position_tolerance
                    if self.options.handover_position_tolerance is not None
                    else StagingAlignment.position_tolerance
                ),
                orientation_tolerance=(
                    math.radians(self.options.handover_orientation_tolerance_deg)
                    if self.options.handover_orientation_tolerance_deg is not None
                    else StagingAlignment.orientation_tolerance
                ),
            ),
            self.options.map_fingerprint,
        )
        if (
            self.stop.requested
            or not self.navigation.poll()
            or self.navigation.status is not NavigationStatus.SUCCEEDED
            or not self.navigation.idle
            or not self.navigation.routing_ready
            or not self.inputs.feedback.ready()
            or not self.inputs.feedback.wheels_settled()
        ):
            raise EntryError(
                "Handover conditions changed after staging verification; no Giskard goal sent"
            )
        self.events.report(
            "HANDOVER READY: Nav2 succeeded; navigation idle; mux free; base stopped; staging pose verified; no Giskard goal sent"
        )

    def run(self, geometry: EntryGeometry) -> EntryPhase:
        """Monitor a fixed, non-resumable entry through final verified arrest."""
        self.policy.retain_door_clearance = self.options.retain_door_clearance
        if self.options.retain_door_clearance:
            self.events.report(
                "ENTRY MODE: baseline motion; doorway opening retained after stable authorization"
            )
        self.approach()
        self.inputs.freeze(geometry, self.options.map_fingerprint)
        waiting_deadline = time.monotonic() + self.options.wait_timeout
        motion_deadline = math.inf
        stopping_deadline = math.inf
        with real_robot(collision_avoidance=True):
            while True:
                now = time.monotonic()
                current = self.navigation.poll()
                observation = self.inputs.observe(
                    current and self.navigation.idle and self.navigation.routing_ready
                )
                if self.motion is not None:
                    terminal = self.motion.done()
                    observation = replace(
                        observation,
                        motion_terminal=terminal,
                        motion_succeeded=terminal and self.motion.exception() is None,
                        motion_failed=terminal and self.motion.exception() is not None,
                        native_doorway_monitor=self.options.pass_door_action,
                    )
                else:
                    observation = replace(observation, motion_terminal=True)
                if self.stop.requested:
                    self.policy.cancel()
                deadline = (
                    motion_deadline if self.motion is not None else waiting_deadline
                )
                if now > deadline and self.policy.phase not in (
                    EntryPhase.STOPPING,
                    EntryPhase.FAILED,
                    EntryPhase.CANCELED,
                ):
                    self.policy.phase = EntryPhase.STOPPING
                    self.policy.reason = (
                        "Entry deadline exceeded; cancellation requested"
                    )
                self.policy.update(observation, now)
                description = (
                    f"phase={self.policy.phase}; {self.policy.reason}; door={observation.door_open}; "
                    f"passage={observation.passage_clear}; blockers={'; '.join(self.inputs.last_reasons)}"
                )
                self.events.report(
                    description,
                    problem=bool(self.inputs.last_reasons)
                    or self.policy.phase in (EntryPhase.STOPPING, EntryPhase.FAILED),
                )
                if self.policy.phase is EntryPhase.ENTERING and self.motion is None:
                    if (
                        not self.inputs.feedback.ready()
                        or not self.inputs.feedback.wheels_settled()
                    ):
                        self.policy.phase = EntryPhase.STOPPING
                        self.policy.reason = "The base or mux changed before submission"
                        continue
                    target = self.inputs.snapshot.feedback_target
                    target.reference_frame = self.inputs.context.world.root
                    motion = (
                        CalibratedDoorPassage.from_geometry(
                            self.inputs.frozen_geometry, self.inputs.context
                        )
                        if self.options.pass_door_action
                        else RobocatStraightMotion(
                            target,
                            tolerances=MotionTolerances(
                                geometry.position_tolerance,
                                math.degrees(geometry.orientation_tolerance),
                            ),
                        )
                    )
                    plan = execute_single(motion, context=self.inputs.context)
                    self.motion = self.pool.submit(plan.perform)
                    self.cancellation = GoalCancellation(
                        self.inputs.context.giskard_wrapper, self.motion
                    )
                    motion_deadline = now + self.options.motion_timeout
                    self.events.report(
                        f"GISKARD: submitting {'PassDoorAction (native staging correction, latched doorway)' if self.options.pass_door_action else 'fixed inside target'}; distance_from_staging={geometry.distance:g} m"
                    )
                if self.policy.phase is EntryPhase.STOPPING:
                    if stopping_deadline == math.inf:
                        stopping_deadline = now + 10
                    if self.cancellation is not None:
                        self.cancellation.cancel()
                    if now > stopping_deadline:
                        raise EntryError(
                            "Cancellation/arrest not confirmed; check the robot before any new goal"
                        )
                if self.policy.phase in (
                    EntryPhase.SUCCEEDED,
                    EntryPhase.FAILED,
                    EntryPhase.CANCELED,
                ):
                    if (
                        observation.motion_succeeded
                        and observation.stationary
                        and self.inputs.last_assessment is not None
                    ):
                        self.events.report(self.inputs.last_assessment.describe())
                    if (
                        self.motion is not None
                        and self.motion.done()
                        and self.motion.exception() is not None
                    ):
                        self.events.report(
                            f"GISKARD ERROR: {self.motion.exception()!r}", problem=True
                        )
                    return self.policy.phase
                time.sleep(0.1)

    def close(self) -> None:
        """Drain cancellation while the ROS context is still alive."""
        if self.cancellation is None:
            return
        deadline = time.monotonic() + 10
        while not self.motion.done() and time.monotonic() < deadline:
            self.cancellation.cancel()
            time.sleep(0.05)


# %% executable lifecycle
def main() -> None:
    """Check connections by default; submit real goals only with explicit execution."""
    options = EntryOptions.parse(remove_ros_args()[1:])
    RobocatEnvironment().validate()
    staging = options.load_staging()
    if options.doorway_distance is not None and not options.handover_only:
        EntryGeometry(staging, options.doorway_distance, options.distance).validate()
    with ExitStack() as cleanup:
        rospy.init_node("robocat_snm")
        cleanup.callback(rospy.shutdown)
        stop = EntryStop()
        for number in (signal.SIGINT, signal.SIGTERM):
            previous = signal.signal(number, stop.receive)
            cleanup.callback(signal.signal, number, previous)
        node = rospy.get_node()
        world = fetch_world_from_service(node, timeout_seconds=12)
        synchronizer = WorldSynchronizer(_world=world, node=node)
        cleanup.callback(synchronizer.close)
        robots = world.get_semantic_annotations_by_type(Robocat)
        if len(robots) != 1:
            raise EntryError("Giskard must expose exactly one Robocat")
        context = Context(world=world, robot=robots[0], ros_node=node)
        cleanup.callback(context.giskard_wrapper.close)
        inputs = EntryInputs(node, context, feedback_source=options.feedback)
        inputs.visibility.allow_isolated_gaps = options.allow_isolated_scan_gaps
        parameters = AsyncParameterClient(node, "giskard")
        settings = parameters.get_parameters(
            ["base_feedback_topic", "base_command_topic"]
        )
        deadline = time.monotonic() + 12
        while not stop.requested and time.monotonic() < deadline:
            if (
                settings.done()
                and inputs.scene is not None
                and inputs.feedback.ready()
                and inputs.feedback.wheels_settled()
            ):
                break
            time.sleep(0.1)
        if not settings.done() or settings.exception() is not None:
            raise EntryError(
                "Giskard configuration unavailable; restart the updated bringup"
            )
        values = settings.result().values
        if len(values) != 2:
            raise EntryError("Giskard feedback and command parameters are missing")
        options.verify_server_topics(values[0].string_value, values[1].string_value)
        print(
            f"FEEDBACK: {options.feedback.value} ({options.feedback.topic}); wheel stop checks retained",
            flush=True,
        )
        if (
            inputs.scene is None
            or not inputs.feedback.ready()
            or not inputs.feedback.wheels_settled()
        ):
            raise EntryError(
                "Initial feedback not ready: "
                + "; ".join(inputs.feedback.blocking_reasons())
            )
        if not EntryLocalization(inputs, stop).wait():
            return
        print(f"MAP FINGERPRINT: {inputs.scene.fingerprint}", flush=True)
        pose = inputs.map_pose()
        print(f"MAP POSE: {pose}; STAGING: {staging}", flush=True)
        if stop.requested:
            return
        if (
            options.map_fingerprint is not None
            and options.map_fingerprint != inputs.scene.fingerprint
        ):
            raise EntryError("The active map does not match the configured fingerprint")
        navigation = NavigationConnection(
            staging,
            execute=options.execute and not options.entry_only,
            confirm_navigation_idle=options.confirm_navigation_idle,
            bootstrap_navigation=options.bootstrap_navigation,
            timeout=options.navigation_timeout,
        )
        cleanup.callback(navigation.close)
        if not options.execute:
            navigation.start()
            deadline = time.monotonic() + 12
            while time.monotonic() < deadline and not stop.requested:
                if navigation.poll() and navigation.idle and navigation.routing_ready:
                    print(
                        f"CHECK OK: native mux inputs, idle navigation, {options.feedback.value}-feedback Giskard and interlocks; no goal sent",
                        flush=True,
                    )
                    return
                if navigation.status is NavigationStatus.FAILED:
                    raise EntryError(
                        "Native configuration check failed; verify separate navigation and snm mux inputs"
                    )
                time.sleep(0.1)
            raise EntryError("Native configuration check timed out; no goal was sent")
        events = EntryEvents(
            node.create_publisher(String, "/snm/entry_state", 10).publish
        )
        pool = cleanup.enter_context(ThreadPoolExecutor(max_workers=1))
        mission = EntryMission(options, inputs, navigation, stop, events, pool)
        cleanup.callback(mission.close)
        if options.handover_only:
            mission.handover(staging)
            return
        outcome = mission.run(options.entry_geometry(staging, options.doorway_distance))
        print(f"ENTRY RESULT: {outcome}", flush=True)
        if outcome is not EntryPhase.SUCCEEDED:
            raise EntryError(f"Entry terminated with {outcome}; no automatic restart")
