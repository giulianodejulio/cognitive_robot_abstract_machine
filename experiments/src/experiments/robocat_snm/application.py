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
from dataclasses import asdict, dataclass, field, replace
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
from experiments.robocat_snm.verbose import TraceLogger
from experiments.robocat_snm.profiles import EntryProfile, EntryProfileName
from experiments.robocat_snm.cabin_turn import (
    CabinTurnAuthorization,
    CabinTurnMeasurement,
    RobocatCabinTurnMotion,
)
from experiments.tiago_goal_logging import PlanarPose
from experiments.robocat_snm.geometry import EntryGeometry, StagingAlignment
from experiments.robocat_snm.native_navigation import NavigationStatus
from experiments.robocat_snm.navigation import NavigationConnection
from experiments.robocat_snm.waypoint import WaypointConnection
from experiments.robocat_wall import WallStopSettings
from experiments.robocat_snm.policy import EntryPhase, EntryPolicy
from giskardpy.middleware.ros2 import rospy
from experiments.robocat_snm.entry_motion import (
    RobocatEntryMotion as RobocatStraightMotion,
)
from scripts.tiago_giskard_goal import GoalCancellation, MotionTolerances
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer
from semantic_digital_twin.robots.robocat import Robocat
from semantic_digital_twin.spatial_types import Pose


# %% request configuration
@dataclass
class EntryOptions:
    """Explicit calibration and execution choices for one supervised trial."""

    profile: EntryProfileName | None = None
    """Named calibration whose values are defaults for explicit CLI overrides."""
    staging: Path = Path("doc/robocat_elevator_staging.yaml")
    """Saved staging pose whose map alignment the operator has checked."""
    doorway_distance: float | None = None
    """Measured threshold distance along the staging waypoint's forward axis."""
    distance: float = 1.5
    """Inside target distance from staging, in metres."""
    inside_waypoint: str | None = None
    """Eulero cabin waypoint name or full key, replacing a distance-only target."""
    position_tolerance: float = EntryGeometry.position_tolerance
    """Final longitudinal acceptance in metres."""
    orientation_tolerance_deg: float = math.degrees(EntryGeometry.orientation_tolerance)
    """Final target heading acceptance in degrees."""
    wall_stop_distance: float | None = None
    """Explicit calibrated base-to-cabin-wall stopping distance in metres."""
    feedback: FeedbackSource = FeedbackSource.WHEEL
    """Odometry used consistently for motion targets and entry geometry."""
    map_fingerprint: str | None = None
    """Expected active-map identity printed by the non-executing check."""
    turn_after_entry: bool = False
    """Opt in to a doorway-facing pure turn only after certified lidar arrival."""
    verbose: bool = False
    """Enable periodic timing and geometry evidence without changing entry decisions."""
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
        parser.add_argument(
            "--profile",
            type=EntryProfileName,
            choices=list(EntryProfileName),
            help="Load stable elevator calibration; explicit CLI options override it",
        )
        parser.add_argument("--staging", type=Path, default=defaults.staging)
        parser.add_argument("--doorway-distance", type=float)
        target = parser.add_mutually_exclusive_group()
        target.add_argument("--distance", type=float, default=argparse.SUPPRESS)
        target.add_argument(
            "--inside-waypoint",
            help="Saved Eulero cabin waypoint name or full floor key",
        )
        parser.add_argument(
            "--position-tolerance", type=float, default=defaults.position_tolerance
        )
        parser.add_argument(
            "--orientation-tolerance-deg",
            type=float,
            default=defaults.orientation_tolerance_deg,
        )
        parser.add_argument(
            "--feedback",
            type=FeedbackSource,
            choices=list(FeedbackSource),
            default=defaults.feedback,
        )
        parser.add_argument(
            "--wall-stop-distance",
            type=float,
            help="Calibrated base-to-back-wall distance; wheel feedback only",
        )
        parser.add_argument("--map-fingerprint")
        parser.add_argument(
            "--verbose",
            action="store_true",
            help="Print detailed timing, wall and passage diagnostics",
        )
        parser.add_argument(
            "--turn-after-entry",
            action="store_true",
            help="Turn toward the doorway after successful lidar-referenced entry and measured rest",
        )
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
        selection = parser.parse_args(arguments)
        if selection.profile is not None:
            calibration = asdict(EntryProfile.load(selection.profile))
            if "distance" in vars(selection):
                calibration["inside_waypoint"] = None
            if selection.pass_door_action:
                calibration["retain_door_clearance"] = False
            parser.set_defaults(**calibration)
        options = cls(**vars(parser.parse_args(arguments)))
        if options.turn_after_entry and (
            options.wall_stop_distance is None or options.handover_only
        ):
            parser.error(
                "--turn-after-entry requires lidar wall stopping and a complete entry request"
            )
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
        if options.wall_stop_distance is not None:
            if (
                not math.isfinite(options.wall_stop_distance)
                or options.wall_stop_distance
                <= EntryGeometry.front_extent
                + EntryGeometry.margin
                + options.position_tolerance
            ):
                parser.error(
                    "Wall stop reference must leave measured front clearance, including tolerance"
                )
            if options.feedback is not FeedbackSource.WHEEL or options.pass_door_action:
                parser.error(
                    "--wall-stop-distance requires wheel feedback and the baseline entry motion"
                )
        if options.confirm_navigation_idle and options.execute:
            parser.error(
                "--confirm-navigation-idle is only allowed for read-only checks"
            )
        if options.inside_waypoint is not None and options.pass_door_action:
            parser.error(
                "--inside-waypoint currently supports the baseline entry, not --pass-door-action"
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
            options.position_tolerance,
            options.orientation_tolerance_deg,
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

    def entry_geometry(
        self, staging: Pose, doorway_distance: float, inside: Pose | None = None
    ) -> EntryGeometry:
        """Construct entry geometry with staging acceptance independent of goal completion."""
        if self.inside_waypoint is not None and inside is None:
            raise EntryError("The requested inside waypoint has not been resolved")
        residual = None if inside is None else StagingAlignment(staging).measure(inside)
        return EntryGeometry(
            staging,
            doorway_distance,
            self.distance if residual is None else residual.longitudinal,
            target_lateral=0.0 if residual is None else residual.lateral,
            target_yaw=0.0 if residual is None else residual.yaw,
            position_tolerance=self.position_tolerance,
            orientation_tolerance=math.radians(self.orientation_tolerance_deg),
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

    trace_logger: TraceLogger = field(default_factory=TraceLogger, init=False)
    """Rate-limited detailed observation output, enabled only by the CLI flag."""

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
                "ENTRY MODE: Cartesian straight motion; doorway opening retained after stable authorization"
            )
        if self.options.wall_stop_distance is not None:
            self.events.report(
                f"COMPLETION: lidar distance within {geometry.position_tolerance:g} m of reference; measured wheel rest; lateral and yaw safety bounds retained (yaw <= 15 deg)"
            )
        elif not self.options.pass_door_action:
            self.events.report(
                f"COMPLETION: longitudinal tolerance={geometry.position_tolerance:g} m; "
                f"lateral tolerance={geometry.lateral_tolerance:g} m; "
                f"orientation tolerance={math.degrees(geometry.orientation_tolerance):g} deg; "
                "fresh measured wheel settling"
            )
        if self.options.wall_stop_distance is not None:
            self.events.report(
                f"LIDAR STOP: base-to-wall reference={self.options.wall_stop_distance:.4f} m; tolerance={geometry.position_tolerance:.4f} m; wheel tracking; nominal target adaptation bounded to 0.40 m"
            )
        if self.options.inside_waypoint is not None:
            self.events.report(
                f"INSIDE WAYPOINT: {self.options.inside_waypoint}; map target=({float(geometry.target.x):.4f}, {float(geometry.target.y):.4f}); yaw={math.degrees(float(geometry.target.yaw)):.3f} deg; staging-relative=({geometry.distance:.4f}, {geometry.target_lateral:.4f}) m"
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
                if self.options.verbose:
                    self.trace_logger.emit(
                        self.inputs.trace,
                        now,
                        problem=f"{self.policy.phase}: {self.policy.reason}: {self.inputs.last_reasons}",
                        force=self.policy.phase
                        in (
                            EntryPhase.SUCCEEDED,
                            EntryPhase.FAILED,
                            EntryPhase.CANCELED,
                        ),
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
                            lateral_tolerance=geometry.lateral_tolerance,
                            wall_stop=(
                                None
                                if self.options.wall_stop_distance is None
                                else WallStopSettings(
                                    self.options.wall_stop_distance,
                                    tolerance=geometry.position_tolerance,
                                )
                            ),
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
                        f"GISKARD: submitting {'PassDoorAction (native staging correction, latched doorway)' if self.options.pass_door_action else ('lidar-updated longitudinal target' if self.options.wall_stop_distance is not None else 'fixed inside target')}; nominal_distance_from_staging={geometry.distance:g} m"
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
                    if (
                        self.policy.phase is EntryPhase.SUCCEEDED
                        and self.options.turn_after_entry
                    ):
                        return self.turn_inside()
                    return self.policy.phase
                time.sleep(0.1)

    def turn_inside(self) -> EntryPhase:
        """Authorize a separate pure turn from fresh longitudinal acceptance and monitor its arrest."""
        current = self.navigation.poll()
        observation = self.inputs.observe(
            current and self.navigation.idle and self.navigation.routing_ready
        )
        authorization = CabinTurnAuthorization(
            self.policy.phase is EntryPhase.SUCCEEDED
            and self.motion is not None
            and self.motion.done()
            and self.motion.exception() is None,
            observation,
            self.inputs.last_assessment,
        )
        if (
            self.stop.requested
            or not authorization.allowed
            or self.options.wall_stop_distance is None
            or self.inputs.last_assessment.wall_reference
            != self.options.wall_stop_distance
            or not self.inputs.feedback.ready()
        ):
            self.events.report(
                "TURN BLOCKED: fresh longitudinal arrival, measured rest or entry success not certified; no turn goal sent",
                problem=True,
            )
            return EntryPhase.CANCELED if self.stop.requested else EntryPhase.FAILED
        wheel = self.inputs.pose_feedback
        if wheel is None:
            self.events.report(
                "TURN BLOCKED: wheel pose unavailable; no turn goal sent", problem=True
            )
            return EntryPhase.FAILED
        start = PlanarPose.from_ros(wheel.pose.pose)
        heading = start.yaw + math.pi
        target = Pose.from_xyz_rpy(
            x=start.x,
            y=start.y,
            yaw=heading,
            reference_frame=self.inputs.context.world.root,
        )
        measurement = CabinTurnMeasurement(
            start,
            PlanarPose(start.x, start.y, heading),
            wheel.header.stamp.sec + wheel.header.stamp.nanosec / 1e9,
            self.inputs.node.get_clock().now().nanoseconds / 1e9,
            self.options.position_tolerance,
            self.options.orientation_tolerance_deg,
        )
        if not measurement.fresh:
            self.events.report(
                "TURN BLOCKED: wheel pose stale; no turn goal sent", problem=True
            )
            return EntryPhase.FAILED
        self.events.report(
            f"TURN AUTHORIZED: lidar distance={self.inputs.last_assessment.wall_distance:.4f} m; "
            f"reference={self.options.wall_stop_distance:.4f} m; wheels stopped; entry succeeded"
        )
        plan = execute_single(
            RobocatCabinTurnMotion(
                target, orientation_tolerance_deg=self.options.orientation_tolerance_deg
            ),
            context=self.inputs.context,
        )
        self.motion = self.pool.submit(plan.perform)
        self.cancellation = GoalCancellation(
            self.inputs.context.giskard_wrapper, self.motion
        )
        deadline = time.monotonic() + self.options.motion_timeout
        stopping_deadline = math.inf
        failure: EntryPhase | None = None
        self.events.report(
            f"TURN: rotating toward doorway with zero forward command; target_yaw={math.degrees(heading):.3f} deg; tolerance={self.options.orientation_tolerance_deg:g} deg"
        )
        while True:
            current = self.navigation.poll()
            reasons = self.inputs.feedback.blocking_reasons(require_free_mux=False)
            if (
                not current
                or not self.navigation.idle
                or not self.navigation.routing_ready
            ):
                reasons.append("Native navigation is unavailable or owns the base")
            if (
                self.inputs.scene is None
                or self.inputs.scene.fingerprint != self.options.map_fingerprint
            ):
                reasons.append("Map changed or is unavailable")
            wheel = self.inputs.pose_feedback
            if wheel is None:
                reasons.append("Wheel pose unavailable")
            else:
                measurement = replace(
                    measurement,
                    measured=PlanarPose.from_ros(wheel.pose.pose),
                    stamp=wheel.header.stamp.sec + wheel.header.stamp.nanosec / 1e9,
                    observed_at=self.inputs.node.get_clock().now().nanoseconds / 1e9,
                )
                if not measurement.fresh:
                    reasons.append("Wheel pose stale or invalid")
                elif measurement.displacement > measurement.position_tolerance:
                    reasons.append(
                        "Base displaced beyond the pure-turn position tolerance"
                    )
            terminal = self.motion.done()
            if terminal and self.motion.exception() is not None:
                reasons.append(f"Giskard turn failed: {self.motion.exception()!r}")
            if time.monotonic() > deadline:
                reasons.append("Turn deadline exceeded")
            settled = self.inputs.feedback.wheels_settled()
            if (
                terminal
                and self.motion.exception() is None
                and settled
                and not measurement.reached
            ):
                reasons.append(
                    "Turn action succeeded but measured final heading or position is outside tolerance"
                )
            if failure is None and (reasons or self.stop.requested):
                failure = (
                    EntryPhase.CANCELED if self.stop.requested else EntryPhase.FAILED
                )
                stopping_deadline = time.monotonic() + 10
                self.events.report(
                    "TURN STOPPING: "
                    + (
                        "Caller canceled" if self.stop.requested else "; ".join(reasons)
                    ),
                    problem=True,
                )
            if failure is not None:
                if not terminal:
                    self.cancellation.cancel()
                if terminal and settled:
                    self.events.report(f"TURN RESULT: {failure}", problem=True)
                    return failure
                if time.monotonic() > stopping_deadline:
                    raise EntryError(
                        "Turn cancellation/arrest not confirmed; inspect robot before a new request"
                    )
            elif terminal and settled and measurement.reached:
                self.events.report(
                    f"TURN FINAL: yaw_error={math.degrees(measurement.yaw_error):.3f} deg; displacement={measurement.displacement:.4f} m; wheels stopped; result=succeeded"
                )
                return EntryPhase.SUCCEEDED
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
    if options.profile is not None:
        print(
            f"ENTRY PROFILE: {options.profile.value}; explicit CLI overrides applied",
            flush=True,
        )
    RobocatEnvironment().validate()
    staging = options.load_staging()
    if (
        options.doorway_distance is not None
        and not options.handover_only
        and options.inside_waypoint is None
    ):
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
        if options.wall_stop_distance is not None:
            inputs.wall_settings = WallStopSettings(
                options.wall_stop_distance, tolerance=options.position_tolerance
            )
        inputs.visibility.allow_isolated_gaps = options.allow_isolated_scan_gaps
        parameters = AsyncParameterClient(node, "giskard")
        parameter_names = ["base_feedback_topic", "base_command_topic"]
        if options.wall_stop_distance is not None:
            parameter_names.append("wall_stop_supported")
        settings = parameters.get_parameters(parameter_names)
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
        if len(values) != len(parameter_names):
            raise EntryError("Giskard feedback and command parameters are missing")
        if options.wall_stop_distance is not None and not values[2].bool_value:
            raise EntryError(
                "Restart updated Giskard: lidar wall stopping is not supported by the running server"
            )
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
        inside = None
        if options.inside_waypoint is not None:
            staging_document = yaml.safe_load(options.staging.read_text())
            inside = WaypointConnection().read(
                options.inside_waypoint, staging_document["parent_key"]
            )
            print(
                f"INSIDE WAYPOINT: {options.inside_waypoint}; map pose={inside}",
                flush=True,
            )
            if options.doorway_distance is not None:
                options.entry_geometry(
                    staging, options.doorway_distance, inside=inside
                ).validate()
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
        outcome = mission.run(
            options.entry_geometry(staging, options.doorway_distance, inside=inside)
        )
        print(f"ENTRY RESULT: {outcome}", flush=True)
        if outcome is not EntryPhase.SUCCEEDED:
            raise EntryError(f"Entry terminated with {outcome}; no automatic restart")
