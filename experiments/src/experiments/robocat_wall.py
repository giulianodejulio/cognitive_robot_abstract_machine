"""Cabin-relative lidar measurements and bounded wheel-frame stopping targets."""

# %% imports
from __future__ import annotations

from dataclasses import dataclass, field
import math
from dataclasses import asdict
from enum import StrEnum
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from builtin_interfaces.msg import Time
from std_msgs.msg import Header

import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from experiments.robocat_snm.odometry_history import OdometryHistory
from experiments.robocat_snm.exceptions import EntryError
from experiments.tiago_goal_logging import PlanarPose


# %% observable wall geometry
@dataclass(frozen=True)
class WallFit:
    """A normalized forward-facing wall plane in base coordinates."""

    normal_x: float
    """Forward component of the wall normal."""
    normal_y: float
    """Left component of the wall normal."""
    distance: float
    """Perpendicular distance from base origin to the plane, in metres."""
    beams: int
    """Number of supporting returns."""
    span: float
    """Observed transverse span in metres."""
    residual: float
    """Root mean square perpendicular fit residual in metres."""


@dataclass
class CabinWallEstimator:
    """Fit a broad front wall while rejecting sparse, nonplanar or oblique returns."""

    half_angle: float = math.radians(15)
    """Half angle of the forward scan sector."""
    minimum_beams: int = 30
    """Minimum number of supporting rays."""
    minimum_span: float = 0.30
    """Minimum observed lateral wall span in metres."""
    inlier_distance: float = 0.015
    """Maximum perpendicular support error in metres."""
    maximum_residual: float = 0.01
    """Maximum accepted root mean square fit residual."""
    maximum_angle: float = math.radians(20)
    """Maximum angle of the wall normal from base forward."""
    maximum_range: float = 8.0
    """Maximum physical wall measurement distance in metres."""
    bin_width: float = 0.05
    """Depth clustering bin width for initial plane support, in metres."""
    seed_window: float = 0.08
    """Initial depth support radius around the dominant cluster, in metres."""
    trim_iterations: int = 4
    """Number of robust support refinement steps."""
    reason: str = field(default="", init=False)
    """Explanation when the latest wall estimate was unavailable."""

    def estimate(self, scan: LaserScan, base_T_laser: np.ndarray) -> WallFit | None:
        """Measure a plane from finite unmasked returns with the actual sensor offset."""
        self.reason = "Insufficient broad front-wall support"
        ranges = np.asarray(scan.ranges, dtype=float)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        valid = (
            np.isfinite(ranges)
            & (ranges >= scan.range_min)
            & (ranges < min(scan.range_max, self.maximum_range))
            & (np.abs(angles) <= self.half_angle)
        )
        if np.count_nonzero(valid) < self.minimum_beams:
            return None
        points = np.column_stack(
            (
                ranges[valid] * np.cos(angles[valid]),
                ranges[valid] * np.sin(angles[valid]),
                np.zeros(np.count_nonzero(valid)),
            )
        )
        points = (points @ base_T_laser[:3, :3].T + base_T_laser[:3, 3])[:, :2]
        bins, counts = np.unique(
            np.round(points[:, 0] / self.bin_width), return_counts=True
        )
        centre = bins[np.argmax(counts)] * self.bin_width
        support = points[np.abs(points[:, 0] - centre) <= self.seed_window]
        for _ in range(self.trim_iterations):
            if len(support) < self.minimum_beams:
                return None
            intercept, slope = np.linalg.lstsq(
                np.column_stack((np.ones(len(support)), support[:, 1])),
                support[:, 0],
                rcond=None,
            )[0]
            support = points[
                np.abs(points[:, 0] - intercept - slope * points[:, 1])
                / math.sqrt(1 + slope * slope)
                <= self.inlier_distance
            ]
        if len(support) < self.minimum_beams:
            return None
        normalizer = math.sqrt(1 + slope * slope)
        span = float(np.ptp(support[:, 1]))
        residual = float(
            np.sqrt(
                np.mean(
                    ((support[:, 0] - intercept - slope * support[:, 1]) / normalizer)
                    ** 2
                )
            )
        )
        if (
            span < self.minimum_span
            or residual > self.maximum_residual
            or abs(math.atan(slope)) > self.maximum_angle
            or intercept <= 0
        ):
            self.reason = "Front-wall span, angle or fit quality outside limits"
            return None
        self.reason = ""
        return WallFit(
            float(1 / normalizer),
            float(-slope / normalizer),
            float(intercept / normalizer),
            len(support),
            span,
            residual,
        )


# %% bounded stop reference
@dataclass(frozen=True)
class WallStopSettings:
    """Distance acceptance and bounded target adaptation for a calibrated cabin."""

    reference_distance: float
    """Validated base-to-wall perpendicular distance in metres."""
    tolerance: float = 0.05
    """Accepted absolute distance error at measured rest, in metres."""
    maximum_age: float = 0.5
    """Maximum absolute scan and wheel timestamp age in seconds."""
    maximum_skew: float = 0.05
    """Maximum scan-to-associated-wheel timestamp separation in seconds."""
    maximum_correction: float = 0.40
    """Maximum shift of the nominal target along its heading, in metres."""
    maximum_target_speed: float = 0.10
    """Maximum adaptation speed after initial acquisition, in metres per second."""
    filter_seconds: float = 0.2
    """Time constant of the endpoint update filter."""
    maximum_plane_jump: float = 0.08
    """Maximum unexplained plane displacement between scans, in metres."""
    stalled_seconds: float = 1.0
    """Maximum settled interval outside the stop band after motion has begun."""
    maximum_normal_change: float = math.radians(5)
    """Maximum unexplained wall normal rotation between scans."""

    def __post_init__(self) -> None:
        """Reject invalid distance bands or adaptation bounds before control starts."""
        if (
            not all(
                math.isfinite(value) and value > 0 for value in asdict(self).values()
            )
            or self.reference_distance <= self.tolerance
        ):
            raise EntryError(
                "Wall stopping settings must be finite, positive and leave a positive reference distance band"
            )


@dataclass(frozen=True)
class WallSample:
    """Timestamped wall evidence and the corresponding wheel-frame endpoint."""

    valid: bool
    """Whether measurement, association and continuity checks succeeded."""
    reason: str
    """Unavailable-feedback explanation, empty for valid observations."""
    scan_stamp: float
    """Original scan acquisition timestamp."""
    wheel_stamp: float
    """Current measured wheel pose timestamp."""
    distance: float = math.nan
    """Wall distance corrected to the current wheel pose."""
    error: float = math.nan
    """Measured distance minus the calibrated stop distance."""
    target_progress: float = math.nan
    """Updated endpoint projection onto the nominal heading in wheel coordinates."""
    correction: float = math.nan
    """Endpoint shift from the nominal map-derived goal."""
    fit: WallFit | None = None
    """Supporting raw wall fit in the scan-time base frame."""
    association_stamp: float = math.nan
    """Wheel pose timestamp actually used to transform the accepted plane."""

    distance_jump: float = math.nan
    """Absolute candidate plane displacement from the last accepted plane."""
    normal_change: float = math.nan
    """Candidate normal rotation from the last accepted plane, in radians."""


@dataclass
class WallStopController:
    """Associate scans with wheel poses and update a smooth bounded longitudinal goal."""

    settings: WallStopSettings
    """Calibrated stop distance and acceptance bounds."""
    nominal: PlanarPose
    """Original wheel-frame target and retained travel heading."""
    base_T_laser: np.ndarray
    """Robot-model transform from the laser frame to base_footprint."""
    estimator: CabinWallEstimator = field(default_factory=CabinWallEstimator)
    """Broad front-wall fitter."""
    normal: np.ndarray | None = field(default=None, init=False)
    """Accepted wall normal in wheel coordinates."""
    offset: float = field(default=math.nan, init=False)
    """Accepted plane offset in wheel coordinates."""
    stamp: float | None = field(default=None, init=False)
    """Acquisition time of the accepted plane."""
    progress: float | None = field(default=None, init=False)
    """Smoothed endpoint coordinate."""
    fit: WallFit | None = field(default=None, init=False)
    """Latest accepted scan-time fit."""

    association_stamp: float = field(default=math.nan, init=False)
    """Timestamp of the acquisition pose used for the current plane."""

    def observe(
        self, scan: LaserScan, wheel: Odometry, history: OdometryHistory, now: float
    ) -> WallSample:
        """Fail closed on unavailable measurements instead of reusing a stale target."""
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec / 1e9
        wheel_stamp = wheel.header.stamp.sec + wheel.header.stamp.nanosec / 1e9
        if (
            abs(now - stamp) > self.settings.maximum_age
            or abs(now - wheel_stamp) > self.settings.maximum_age
        ):
            return WallSample(
                False, "Wall scan or wheel feedback stale", stamp, wheel_stamp
            )
        if (
            scan.header.frame_id.lstrip("/") != "laser_link"
            or wheel.header.frame_id != "odom"
            or wheel.child_frame_id != "base_footprint"
        ):
            return WallSample(
                False,
                "Unexpected wall scan or wheel reference frame",
                stamp,
                wheel_stamp,
            )
        matched = history.nearest(stamp, self.settings.maximum_skew)
        if matched is None:
            return WallSample(
                False, "No wheel pose matches wall scan acquisition", stamp, wheel_stamp
            )
        for message in (matched, wheel):
            pose, rotation = message.pose.pose.position, message.pose.pose.orientation
            values = (
                pose.x,
                pose.y,
                pose.z,
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            )
            if (
                not all(math.isfinite(value) for value in values)
                or abs(sum(value * value for value in values[3:]) - 1) > 1e-4
            ):
                return WallSample(
                    False, "Invalid wheel pose for wall association", stamp, wheel_stamp
                )
        acquisition = PlanarPose.from_ros(matched.pose.pose)
        current = PlanarPose.from_ros(wheel.pose.pose)
        position = np.array([current.x, current.y])
        direction = np.array([math.cos(self.nominal.yaw), math.sin(self.nominal.yaw)])
        nominal_progress = float(direction @ np.array([self.nominal.x, self.nominal.y]))
        distance_jump, normal_change = 0.0, 0.0
        if self.stamp != stamp:
            fit = self.estimator.estimate(scan, self.base_T_laser)
            if fit is None:
                return WallSample(False, self.estimator.reason, stamp, wheel_stamp)
            rotation = np.array(
                [
                    [math.cos(acquisition.yaw), -math.sin(acquisition.yaw)],
                    [math.sin(acquisition.yaw), math.cos(acquisition.yaw)],
                ]
            )
            normal = rotation @ np.array([fit.normal_x, fit.normal_y])
            offset = fit.distance + float(
                normal @ np.array([acquisition.x, acquisition.y])
            )
            distance = offset - float(normal @ position)
            projection = float(normal @ direction)
            if projection < math.cos(math.radians(25)):
                return WallSample(
                    False,
                    "Wall normal incompatible with entry heading",
                    stamp,
                    wheel_stamp,
                )
            candidate = (
                float(direction @ position)
                + (distance - self.settings.reference_distance) / projection
            )
            if abs(candidate - nominal_progress) > self.settings.maximum_correction:
                return WallSample(
                    False,
                    "Lidar target correction exceeds configured bound",
                    stamp,
                    wheel_stamp,
                )
            if self.normal is not None:
                old_distance = self.offset - float(self.normal @ position)
                angle = math.acos(float(np.clip(normal @ self.normal, -1, 1)))
                distance_jump, normal_change = abs(distance - old_distance), angle
                if (
                    abs(distance - old_distance) > self.settings.maximum_plane_jump
                    or angle > self.settings.maximum_normal_change
                ):
                    return WallSample(
                        False,
                        "Front wall changed discontinuously",
                        stamp,
                        wheel_stamp,
                        distance=distance,
                        error=distance - self.settings.reference_distance,
                        target_progress=self.progress,
                        correction=candidate - nominal_progress,
                        fit=fit,
                        association_stamp=matched.header.stamp.sec
                        + matched.header.stamp.nanosec / 1e9,
                        distance_jump=distance_jump,
                        normal_change=normal_change,
                    )
            if self.progress is None:
                self.progress = candidate
            else:
                elapsed = stamp - self.stamp
                if elapsed <= 0:
                    return WallSample(
                        False, "Wall scan timestamp moved backwards", stamp, wheel_stamp
                    )
                alpha = elapsed / (self.settings.filter_seconds + elapsed)
                step = float(
                    np.clip(
                        alpha * (candidate - self.progress),
                        -self.settings.maximum_target_speed * elapsed,
                        self.settings.maximum_target_speed * elapsed,
                    )
                )
                self.progress += step
            self.normal, self.offset, self.stamp, self.fit = normal, offset, stamp, fit
            self.association_stamp = (
                matched.header.stamp.sec + matched.header.stamp.nanosec / 1e9
            )
        distance = self.offset - float(self.normal @ position)
        return WallSample(
            True,
            "",
            stamp,
            wheel_stamp,
            distance,
            distance - self.settings.reference_distance,
            self.progress,
            self.progress - nominal_progress,
            self.fit,
            self.association_stamp,
            distance_jump,
            normal_change,
        )


# %% recorded wall acceptance
class WallDiagnostic(StrEnum):
    """Recorded identity of the cabin-relative stop evidence."""

    TOPIC = "/snm/wall_tracking"
    STATUS = "snm/cabin_wall_stop"


@dataclass
class WallTelemetry:
    """Distance evidence and calibrated reference for rosbag reconstruction."""

    sample: WallSample
    """Exact measurement used for this observation."""
    settings: WallStopSettings
    """Reference and bounds used by this trial."""

    def to_message(self, now: float) -> DiagnosticArray:
        """Publish fit quality, timing, target adaptation and distance acceptance."""
        values = []
        for group, data in (
            ("sample", asdict(self.sample)),
            ("settings", asdict(self.settings)),
        ):
            for name, value in data.items():
                if isinstance(value, dict):
                    values.extend(
                        KeyValue(key=f"{group}.{name}.{axis}", value=str(number))
                        for axis, number in value.items()
                    )
                else:
                    values.append(KeyValue(key=f"{group}.{name}", value=str(value)))
        nanos = round(now * 1e9)
        return DiagnosticArray(
            header=Header(
                stamp=Time(sec=nanos // 10**9, nanosec=nanos % 10**9), frame_id="odom"
            ),
            status=[
                DiagnosticStatus(
                    level=(
                        DiagnosticStatus.OK
                        if self.sample.valid
                        else DiagnosticStatus.ERROR
                    ),
                    name=WallDiagnostic.STATUS,
                    message=(
                        "Wall feedback valid"
                        if self.sample.valid
                        else self.sample.reason
                    ),
                    values=values,
                )
            ],
        )


# %% bounded stationary recovery
@dataclass
class WallMotionProgress:
    """Detect a stopped entry that is still trying to reach the wall reference."""

    limit: float = 1.0
    """Maximum stationary interval outside the distance band, in seconds."""
    moved: bool = field(default=False, init=False)
    """Whether measured wheels have left their settled state."""
    stopped_since: float | None = field(default=None, init=False)
    """Start of the current settled interval outside acceptance."""

    def blocked(self, now: float, stationary: bool, reached: bool) -> bool:
        """Allow startup but bound subsequent stationary precision attempts."""
        if not stationary:
            self.moved = True
        if not self.moved or not stationary or reached:
            self.stopped_since = None
            return False
        if self.stopped_since is None:
            self.stopped_since = now
        return now - self.stopped_since >= self.limit
