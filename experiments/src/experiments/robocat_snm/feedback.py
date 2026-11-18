"""
Fresh robot feedback, map alignment and entry-corridor observations.
"""

# %% imports
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field, replace
from functools import partial

import numpy as np
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from coraplex.datastructures.dataclasses import Context
from experiments.robocat_safety import RobocatFeedback, FeedbackSource
from experiments.robocat_wall import (
    WallStopSettings,
    WallStopController,
    WallTelemetry,
    WallDiagnostic,
)
from experiments.robocat_snm.exceptions import EntryError
from experiments.robocat_snm.geometry import (
    EntryAssessment,
    EntryGeometry,
    SegmentVisibility,
    StagingAlignment,
)
from experiments.robocat_snm.policy import EntryObservation
from experiments.robocat_snm.verbose import EntryTrace, SectionTrace
from experiments.robocat_snm.odometry_history import OdometryHistory
from experiments.robocat_snm.map_alignment import MapPoseAlignment
from experiments.robocat_snm.pose_comparison import PoseComparison, PoseDiagnostic
from experiments.tiago_goal_logging import PlanarPose
from experiments.robocat_waypoints.model import WaypointMap, WaypointSnapshot
from giskardpy.middleware.ros2.scripts.iai_robots.robocat import RobocatLidarSource
from semantic_digital_twin.spatial_types import (
    Point3,
    Pose,
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.datastructures.lidar_reading import LidarReading
from semantic_digital_twin.adapters.ros.ros2_to_semdt_converters import (
    PoseToSemDTConverter,
)


# %% motor and operator interlocks
@dataclass
class EntryFeedback(RobocatFeedback):
    """
    Include wheel-driver faults in the existing operator interlocks.
    """

    motor_faults: dict[str, str] = field(default_factory=dict, init=False)
    """
    Last observed driver error for each reported wheel actuator.
    """

    laser_history: OdometryHistory = field(default_factory=OdometryHistory, init=False)
    """
    Recent laser odometry samples retained independently of real wheel feedback.
    """

    def receive_odometry(self, message: Odometry) -> None:
        super().receive_odometry(message)
        self.laser_history.samples.append(message)

    def receive_diagnostics(self, message: DiagnosticArray) -> None:
        super().receive_diagnostics(message)
        for status in message.status:
            if "wheel" in status.name.lower() and "actuator" in status.name.lower():
                self.motor_faults[status.name] = (
                    status.message
                    if status.level in (DiagnosticStatus.ERROR, DiagnosticStatus.STALE)
                    else ""
                )

    def blocking_reasons(self, require_free_mux: bool = True) -> list[str]:
        reasons = super().blocking_reasons(require_free_mux) + [
            f"{actuator}: {reason}"
            for actuator, reason in self.motor_faults.items()
            if reason
        ]
        if self.wheel_odometry is not None:
            pose = self.wheel_odometry.pose.pose
            rotation = pose.orientation
            values = (
                pose.position.x,
                pose.position.y,
                pose.position.z,
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            )
            if (
                not all(math.isfinite(value) for value in values)
                or abs(sum(value * value for value in values[3:]) - 1) > 0.05
                or self.wheel_odometry.header.frame_id != "odom"
                or self.wheel_odometry.child_frame_id != "base_footprint"
            ):
                reasons.append("Invalid wheel odometry pose, quaternion or frames")
        return reasons


# %% current scene and frozen entry frame
@dataclass
class EntryInputs:
    """
    Observe the real robot with a fixed reference in the selected odometry.
    """

    node: Node
    """
    Spinning local ROS node.
    """

    context: Context
    """
    Native CRAM context synchronized with the selected-feedback Giskard server.
    """

    feedback_source: FeedbackSource = FeedbackSource.WHEEL
    """
    Pose coordinates shared by the controller, target and geometric checks.
    """

    @property
    def pose_feedback(self) -> Odometry | None:
        """
        Return the selected pose while retaining separate wheel safety feedback.
        """
        return (
            self.feedback.wheel_odometry
            if self.feedback_source is FeedbackSource.WHEEL
            else self.feedback.odometry
        )

    @property
    def pose_history(self) -> OdometryHistory:
        """
        Return the timestamp history belonging to the selected pose source.
        """
        return (
            self.feedback.wheel_history
            if self.feedback_source is FeedbackSource.WHEEL
            else self.feedback.laser_history
        )

    feedback: EntryFeedback = field(init=False)
    """
    Fresh measured base and operator interlocks.
    """
    transforms: Buffer = field(init=False)
    """
    Native robot TF cache for the initial map alignment.
    """
    listener: TransformListener = field(init=False)
    """
    Subscriptions maintaining the TF cache.
    """
    subscription: object = field(init=False)
    """
    Map subscription retained for this request.
    """
    scene: WaypointMap | None = field(default=None, init=False)
    """
    Current map, including an identity independent of publication time.
    """
    source: RobocatLidarSource = field(init=False)
    """
    Real best-effort lidar source; no simulated sensor is used.
    """
    frozen_geometry: EntryGeometry | None = field(default=None, init=False)
    """
    Entry configuration expressed in the captured selected-odometry coordinates.
    """
    snapshot: WaypointSnapshot | None = field(default=None, init=False)
    """
    Map, wheel and selected-odometry alignment captured after native navigation stops.
    """
    fingerprint: str | None = field(default=None, init=False)
    """
    Identity of the active map at handover.
    """
    visibility: SegmentVisibility = field(
        default_factory=partial(SegmentVisibility, no_return_range=25.0)
    )
    """
    PAL-filtered clearance evidence capped at the configured 25 m range-filter limit.
    """
    maximum_age: float = 0.5
    """
    Maximum age of scan and selected pose measurements during entry, in seconds.
    """
    maximum_skew: float = 0.25
    """
    Maximum difference between scan and matched selected-odometry timestamps.
    """
    maximum_pose_age: float = 2.0
    """
    Maximum age of the initial map transform, in seconds.
    """
    pose_blocker: str | None = field(default=None, init=False)
    """
    Latest missing or unsynchronized map-pose evidence.
    """
    last_reasons: list[str] = field(default_factory=list, init=False)
    """
    Current blockers reported without retaining sensor histories.
    """
    last_assessment: EntryAssessment | None = field(default=None, init=False)
    """
    Final-pose evidence from the current healthy observation, if available.
    """
    comparison_publisher: Publisher | None = field(default=None, init=False)
    """
    On-demand numeric diagnostics sampled in the existing observation loop.
    """
    model_position_limit: float = 0.03
    """
    Existing planar model/feedback disagreement limit in metres.
    """
    model_yaw_limit: float = 0.05
    """
    Existing model/feedback disagreement limit in radians.
    """
    wall_settings: WallStopSettings | None = None
    """
    Optional calibrated lidar stopping reference.
    """
    wall_controller: WallStopController | None = field(default=None, init=False)
    """
    Wheel-frame wall estimator initialized after verified handover.
    """
    wall_publisher: Publisher | None = field(default=None, init=False)
    """
    Recorded continuous wall target and measurement evidence.
    """
    trace: EntryTrace = field(default_factory=EntryTrace, init=False)
    """
    Evidence from the latest observation, independent of the acceptance policy.
    """

    def __post_init__(self) -> None:
        self.feedback = EntryFeedback(self.node, active_priorities=("0", "10", "20"))
        self.transforms = Buffer()
        self.listener = TransformListener(self.transforms, self.node)
        self.subscription = self.node.create_subscription(
            OccupancyGrid,
            "/map",
            self.receive_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.source = RobocatLidarSource(node=self.node, topic_name="/scan")
        self.context.robot.mobile_base.lidar.source = self.source
        self.wall_publisher = self.node.create_publisher(
            DiagnosticArray, WallDiagnostic.TOPIC, QoSProfile(depth=10)
        )
        self.comparison_publisher = self.node.create_publisher(
            DiagnosticArray, PoseDiagnostic.TOPIC, QoSProfile(depth=10)
        )

    def receive_map(self, message: OccupancyGrid) -> None:
        """
        Replace a map only when its contents or geometry have changed.
        """
        candidate = WaypointMap(message, clearance=0.30)
        if self.scene is None or candidate.fingerprint != self.scene.fingerprint:
            self.scene = candidate

    map_alignment: MapPoseAlignment | None = field(default=None, init=False)
    """
    Map pose and exact current feedback retained for the handover snapshot.
    """

    def map_pose(self) -> Pose | None:
        """
        Align map TF with historical feedback and apply measured displacement to the
        current sample.
        """
        self.map_alignment = None
        odometry = self.pose_feedback
        if odometry is None:
            self.pose_blocker = f"{self.feedback_source.value} odometry missing"
            return None
        if not self.transforms.can_transform("map", "base_footprint", Time()):
            self.pose_blocker = "map to base_footprint TF unavailable"
            return None
        transform = self.transforms.lookup_transform("map", "base_footprint", Time())
        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
        feedback_stamp = odometry.header.stamp.sec + odometry.header.stamp.nanosec / 1e9
        now = self.node.get_clock().now().nanoseconds / 1e9
        if (
            abs(now - stamp) > self.maximum_pose_age
            or abs(now - feedback_stamp) > self.maximum_age
        ):
            self.pose_blocker = (
                f"TF age={now-stamp:.3f} s (limit={self.maximum_pose_age:.3f}); "
                f"{self.feedback_source.value} feedback age={now-feedback_stamp:.3f} s (limit={self.maximum_age:.3f})"
            )
            return None
        matched = (
            odometry
            if abs(feedback_stamp - stamp) < 1e-6
            else self.pose_history.nearest(stamp, self.maximum_skew)
        )
        if matched is None:
            self.pose_blocker = (
                f"No {self.feedback_source.value} sample matches TF acquisition: "
                f"latest feedback/TF skew={feedback_stamp-stamp:.3f} s; matching limit={self.maximum_skew:.3f} s"
            )
            return None
        self.pose_blocker = None
        translation, rotation = (
            transform.transform.translation,
            transform.transform.rotation,
        )
        map_at_tf = Pose.from_xyz_quaternion(
            translation.x,
            translation.y,
            translation.z,
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )
        self.map_alignment = MapPoseAlignment.from_samples(
            map_at_tf, matched, odometry, self.context.world, stamp
        )
        return self.map_alignment.pose

    def confirm_handover(
        self, alignment: StagingAlignment | EntryGeometry, fingerprint: str
    ) -> Pose:
        """
        Return the reached staging pose only with matching map and a stopped free base.
        """
        start = self.map_pose()
        deadline = time.monotonic() + self.maximum_age
        while (
            start is None
            and time.monotonic() < deadline
            and self.feedback.ready()
            and self.feedback.wheels_settled()
        ):
            time.sleep(0.05)
            start = self.map_pose()
        bounds = (
            alignment
            if isinstance(alignment, StagingAlignment)
            else StagingAlignment(
                alignment.staging,
                alignment.staging_tolerance,
                alignment.staging_orientation_tolerance,
            )
        )
        reasons = []
        if start is None:
            reasons.append(
                "Map pose missing, stale or unsynchronized"
                + (f": {self.pose_blocker}" if self.pose_blocker else "")
            )
        else:
            residual = bounds.measure(start)
            details = (
                f"position error={residual.distance:.4f} m (limit={bounds.position_tolerance:.4f} m); "
                f"yaw error={math.degrees(residual.yaw):.3f} deg (limit={math.degrees(bounds.orientation_tolerance):.3f} deg)"
            )
            print(
                f"STAGING ERROR: longitudinal={residual.longitudinal:.4f} m; lateral={residual.lateral:.4f} m; {details}",
                flush=True,
            )
            if not alignment.aligned(start):
                reasons.append("Staging alignment failed: " + details)
        if not self.feedback.ready():
            reasons.extend(
                self.feedback.blocking_reasons()
                or ["Interlocks changed before details could be collected"]
            )
        if not self.feedback.wheels_settled():
            reasons.append("Base wheels are not confirmed settled")
        if self.scene is None:
            reasons.append("Map unavailable")
        elif self.scene.fingerprint != fingerprint:
            reasons.append(
                f"Map fingerprint mismatch: current={self.scene.fingerprint}; required={fingerprint}"
            )
        if reasons:
            raise EntryError("Handover blocked: " + "; ".join(reasons))
        return start

    def freeze(self, geometry: EntryGeometry, fingerprint: str) -> None:
        """
        Capture selected-odometry targets after verified staging arrival and arrest.
        """
        geometry.validate()
        start = self.confirm_handover(geometry, fingerprint)
        blockers = self.scene.blockers(start, geometry.target)
        if blockers:
            raise EntryError("; ".join(blockers))
        wheel_pose = PoseToSemDTConverter.convert(
            self.feedback.wheel_odometry.pose.pose, self.context.world
        )
        selected_message = (
            self.pose_feedback
            if self.map_alignment is None
            else self.map_alignment.feedback
        )
        selected_pose = PoseToSemDTConverter.convert(
            selected_message.pose.pose, self.context.world
        )
        if self.map_alignment is not None:
            print(
                f"MAP ALIGNMENT: TF stamp={self.map_alignment.evidence.tf_stamp:.6f}; "
                f"associated {self.feedback_source.value} stamp={self.map_alignment.evidence.association_stamp:.6f}; "
                f"current feedback stamp={self.map_alignment.evidence.feedback_stamp:.6f}",
                flush=True,
            )
            wheel_message = (
                selected_message
                if self.feedback_source is FeedbackSource.WHEEL
                else self.feedback.wheel_history.nearest(
                    self.map_alignment.evidence.feedback_stamp, self.maximum_skew
                )
            )
            if wheel_message is None:
                raise EntryError("No wheel sample matches the selected handover pose")
            wheel_pose = PoseToSemDTConverter.convert(
                wheel_message.pose.pose, self.context.world
            )
        self.snapshot = WaypointSnapshot(
            start, wheel_pose, geometry.target, feedback_start=selected_pose
        )
        selected_staging = WaypointSnapshot(
            start, wheel_pose, geometry.staging, feedback_start=selected_pose
        ).feedback_target
        self.frozen_geometry = replace(geometry, staging=selected_staging)
        self.fingerprint = fingerprint
        if self.wall_settings is not None:
            self.wall_controller = WallStopController(
                self.wall_settings,
                PoseComparison.planar(self.snapshot.feedback_target),
                self.context.world.compute_forward_kinematics(
                    self.context.robot.root, self.context.robot.mobile_base.lidar.root
                ).to_np(),
            )

    maximum_observation_attempts: int = 2
    """
    Bounded evaluations before an expired observation must block entry.
    """

    def observe(self, navigation_idle: bool) -> EntryObservation:
        """
        Discard expired computations and require fresh evidence before returning a
        verdict.
        """
        for attempt in range(self.maximum_observation_attempts):
            observation = self.assess_feedback(navigation_idle)
            checked_at = self.node.get_clock().now().nanoseconds / 1e9
            self.trace.evaluated_at = checked_at
            self.trace.attempts = attempt + 1
            if not observation.healthy:
                return observation
            expired = [
                f"{source} age={checked_at-stamp:.3f} s (limit={self.maximum_age:.3f} s)"
                for source, stamp in (
                    ("scan", self.trace.scan_stamp),
                    (self.feedback_source.value, self.trace.wheel_stamp),
                )
                if math.isfinite(stamp) and abs(checked_at - stamp) > self.maximum_age
            ]
            if expired and attempt + 1 < self.maximum_observation_attempts:
                continue
            if expired:
                self.last_assessment = None
                self.last_reasons = self.last_reasons + [
                    "Observation expired during evaluation: " + "; ".join(expired)
                ]
                return replace(
                    observation,
                    healthy=False,
                    door_open=None,
                    passage_clear=None,
                    target_reached=False,
                    fully_inside=False,
                )
            return observation
        raise EntryError("The observation evaluation limit must be positive")

    def assess_feedback(self, navigation_idle: bool) -> EntryObservation:
        """
        Assess only the remaining entry corridor and directly relevant interlocks.
        """
        self.last_assessment = None
        self.trace = EntryTrace(
            observed_at=self.node.get_clock().now().nanoseconds / 1e9,
            maximum_age=self.maximum_age,
            maximum_skew=self.maximum_skew,
            wall_settings=self.wall_settings,
        )
        geometry = self.frozen_geometry
        reasons = self.feedback.blocking_reasons(require_free_mux=False)
        if self.scene is None or self.scene.fingerprint != self.fingerprint:
            reasons.append("Map changed or is unavailable")
        odometry = self.pose_feedback
        if geometry is None or odometry is None or not self.source.has_reading:
            self.last_reasons = reasons + [
                f"Missing entry geometry, {self.feedback_source.value} feedback or scan"
            ]
            return EntryObservation(
                False, self.feedback.wheels_settled(), navigation_idle, None, None, 0
            )
        wheel_odometry = (
            odometry
            if self.feedback_source is FeedbackSource.WHEEL
            else self.feedback.wheel_odometry
        )
        scan = self.source.received_scan
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec / 1e9
        feedback_stamp = odometry.header.stamp.sec + odometry.header.stamp.nanosec / 1e9
        now = self.node.get_clock().now().nanoseconds / 1e9
        self.trace.observed_at = now
        self.trace.scan_stamp = stamp
        self.trace.wheel_stamp = feedback_stamp
        if abs(now - stamp) > self.maximum_age:
            reasons.append(
                f"Scan stale: age={now-stamp:.3f} s (limit={self.maximum_age:.3f} s)"
            )
        if abs(now - feedback_stamp) > self.maximum_age:
            reasons.append(
                f"{self.feedback_source.value} feedback stale: age={now-feedback_stamp:.3f} s "
                f"(limit={self.maximum_age:.3f} s)"
            )
        measured_pose = PoseToSemDTConverter.convert(
            odometry.pose.pose, self.context.world
        )
        local = geometry.local_pose(measured_pose)
        model = self.context.robot.root.global_transform
        model_yaw = float(
            Pose(position=model.position, orientation=model.quaternion).yaw
        )
        assessment = geometry.assess(measured_pose)
        comparison = PoseComparison(
            model=PlanarPose(
                float(model.position.x), float(model.position.y), model_yaw
            ),
            feedback=PoseComparison.planar(measured_pose),
            geometry_target=PoseComparison.planar(geometry.target),
            submitted_target=PoseComparison.planar(
                self.snapshot.feedback_target
                if self.snapshot is not None
                else geometry.target
            ),
            source=self.feedback_source,
            frame_id=str(self.context.world.root.name),
            model_observed_at=now,
            feedback_stamp=feedback_stamp,
            scan_stamp=stamp,
            position_limit_m=self.model_position_limit,
            yaw_limit_rad=self.model_yaw_limit,
            assessment=assessment,
            map_alignment=(
                None if self.map_alignment is None else self.map_alignment.evidence
            ),
        )
        if comparison.disagrees:
            reasons.append(comparison.describe())
        if abs(float(local.y)) > geometry.half_width - 0.21 or abs(
            float(local.yaw)
        ) > math.radians(15):
            reasons.append("Robot left the configured straight entry corridor")
        self.last_reasons = reasons
        sample = scan.header.stamp.sec * 10**9 + scan.header.stamp.nanosec
        if reasons:
            self.publish_comparison(comparison)
            return EntryObservation(
                False,
                self.feedback.wheels_settled(),
                navigation_idle,
                None,
                None,
                sample,
            )
        matched = (
            odometry
            if stamp == feedback_stamp
            else self.pose_history.nearest(stamp, self.maximum_skew)
        )
        if matched is None:
            self.publish_comparison(comparison)
            self.last_reasons = [
                f"No {self.feedback_source.value} sample matches scan: latest odometry/scan skew="
                f"{feedback_stamp-stamp:.3f} s (matching limit={self.maximum_skew:.3f} s)"
            ]
            return EntryObservation(
                False,
                self.feedback.wheels_settled(),
                navigation_idle,
                None,
                None,
                sample,
            )
        self.trace.association_stamp = (
            matched.header.stamp.sec + matched.header.stamp.nanosec / 1e9
        )
        acquisition_pose = PoseToSemDTConverter.convert(
            matched.pose.pose, self.context.world
        )
        if self.wall_controller is not None:
            wall = self.wall_controller.observe(
                scan, wheel_odometry, self.feedback.wheel_history, now
            )
            self.trace.wall = wall
            if self.wall_publisher is not None:
                self.wall_publisher.publish(
                    WallTelemetry(wall, self.wall_settings).to_message(now)
                )
            if not wall.valid:
                self.publish_comparison(comparison)
                self.last_reasons = ["Lidar stop feedback unavailable: " + wall.reason]
                return EntryObservation(
                    False,
                    self.feedback.wheels_settled(),
                    navigation_idle,
                    None,
                    None,
                    sample,
                )
            nominal = self.wall_controller.nominal
            remaining = (
                wall.target_progress
                - math.cos(nominal.yaw) * float(measured_pose.x)
                - math.sin(nominal.yaw) * float(measured_pose.y)
            )
            target = Pose.from_xyz_rpy(
                x=float(measured_pose.x) + math.cos(nominal.yaw) * remaining,
                y=float(measured_pose.y) + math.sin(nominal.yaw) * remaining,
                yaw=nominal.yaw,
            )
            if self.snapshot is None:
                self.last_reasons = [
                    "Wall stopping requires a frozen handover alignment"
                ]
                return EntryObservation(
                    False,
                    self.feedback.wheels_settled(),
                    navigation_idle,
                    None,
                    None,
                    sample,
                )
            map_T_target = (
                self.snapshot.map_start.homogeneous_matrix
                @ self.snapshot.feedback_start.homogeneous_matrix.inverse()
                @ target.homogeneous_matrix
            )
            map_target = Pose(
                position=map_T_target.position, orientation=map_T_target.quaternion
            )
            map_blockers = self.scene.blockers(self.snapshot.map_start, map_target)
            if map_blockers:
                self.last_reasons = map_blockers
                return EntryObservation(
                    False,
                    self.feedback.wheels_settled(),
                    navigation_idle,
                    None,
                    None,
                    sample,
                )
            residual = geometry.local_pose(target)
            geometry = replace(
                geometry, distance=float(residual.x), target_lateral=float(residual.y)
            )
            self.frozen_geometry = geometry
            assessment = replace(
                geometry.assess(measured_pose),
                wall_distance=wall.distance,
                wall_reference=self.wall_settings.reference_distance,
                wall_tolerance=self.wall_settings.tolerance,
            )
        self.publish_comparison(
            replace(
                comparison,
                assessment=assessment,
                geometry_target=PoseComparison.planar(geometry.target),
            )
        )
        scan_to_current = model @ acquisition_pose.homogeneous_matrix.inverse()
        reading = self.source.get_lidar_reading(
            self.context.robot.mobile_base.lidar, scan=scan
        )
        sensor_T_staging = (
            self.context.world.compute_forward_kinematics(
                root=reading.reference_frame, tip=self.context.world.root
            ).to_np()
            @ scan_to_current.to_np()
            @ geometry.staging.homogeneous_matrix.to_np()
        )
        door = self.section_clear(
            reading, geometry.doorway_distance, sensor_T_staging=sensor_T_staging
        )
        front = float(local.x) + geometry.front_extent + geometry.margin
        end = (
            geometry.distance
            + geometry.front_extent
            + geometry.margin
            + geometry.position_tolerance
        )
        sections = np.linspace(
            front,
            max(front + 0.05, end),
            max(2, math.ceil(max(0, end - front) / geometry.section_spacing) + 1),
        )
        observations = [
            self.section_clear(
                reading, float(section), sensor_T_staging=sensor_T_staging
            )
            for section in sections
        ]
        self.trace.sections = [
            SectionTrace(float(section), verdict)
            for section, verdict in zip(sections, observations)
        ]
        passage = (
            False if False in observations else None if None in observations else True
        )
        self.last_assessment = assessment
        return EntryObservation(
            True,
            self.feedback.wheels_settled(),
            navigation_idle,
            door,
            passage,
            sample,
            fully_inside=self.last_assessment.fully_inside,
            target_reached=self.last_assessment.target_reached,
            threshold_crossing=float(local.x) + geometry.front_extent
            >= geometry.doorway_distance,
        )

    def publish_comparison(self, comparison: PoseComparison) -> None:
        """
        Record the exact pose and final criterion used by this observation.
        """
        if (
            self.comparison_publisher is not None
            and self.comparison_publisher.get_subscription_count() > 0
        ):
            self.comparison_publisher.publish(comparison.to_message())

    def section_clear(
        self,
        reading: LidarReading,
        distance: float,
        scan_to_current: HomogeneousTransformationMatrix | None = None,
        *,
        sensor_T_staging: np.ndarray | None = None,
    ) -> bool | None:
        """
        Assess one section with native visibility and sensor-frame endpoints.

        :param sensor_T_staging: Common acquisition-compensated transform for all
            sections.
        """
        geometry = self.frozen_geometry
        if sensor_T_staging is None:
            sensor_T_root = self.context.world.compute_forward_kinematics(
                root=reading.reference_frame, tip=self.context.world.root
            ).to_np()
            sensor_T_staging = (
                sensor_T_root
                @ (np.eye(4) if scan_to_current is None else scan_to_current.to_np())
                @ geometry.staging.homogeneous_matrix.to_np()
            )
        endpoints = (
            np.array(
                [
                    [distance, -geometry.half_width, 0, 1],
                    [distance, geometry.half_width, 0, 1],
                ]
            )
            @ sensor_T_staging.T
        )
        start, end = [
            Point3(*point[:3], reference_frame=reading.reference_frame)
            for point in endpoints
        ]
        return self.visibility.evaluate(reading, start, end)
