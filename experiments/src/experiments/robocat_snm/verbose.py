"""Bounded terminal diagnostics for supervised entry experiments."""

# %% imports
from __future__ import annotations

from dataclasses import dataclass, field
import math

from experiments.robocat_wall import WallSample, WallStopSettings


# %% sampled diagnostic evidence
@dataclass(frozen=True)
class SectionTrace:
    """Clearance observation at one longitudinal corridor section."""

    distance: float
    """Section position along the staging axis, in metres."""
    clear: bool | None
    """True for measured clearance, false for blockage, None for missing coverage."""


@dataclass
class EntryTrace:
    """Read-only snapshot of the quantities used by the existing guards."""

    observed_at: float = math.nan
    """ROS observation timestamp in seconds."""
    evaluated_at: float = math.nan
    """ROS timestamp at which the computed observation is returned."""
    attempts: int = 1
    """Number of evaluations needed for the current returned verdict."""
    scan_stamp: float = math.nan
    """Source scan timestamp in seconds."""
    wheel_stamp: float = math.nan
    """Current selected odometry timestamp in seconds."""
    association_stamp: float = math.nan
    """Odometry timestamp matched to scan acquisition."""
    maximum_age: float = 0.5
    """Configured scan and feedback freshness bound."""
    maximum_skew: float = 0.25
    """Configured passage scan association bound."""
    wall: WallSample | None = None
    """Latest accepted or rejected wall candidate."""
    wall_settings: WallStopSettings | None = None
    """Reference and guard bounds associated with the wall sample."""
    sections: list[SectionTrace] = field(default_factory=list)
    """Section verdicts retained without a second geometry evaluation."""

    def describe(self) -> str:
        """Explain timing, wall guards and unavailable sections without changing verdicts."""
        lines = [
            f"VERBOSE EVALUATION: observed_at={self.observed_at:.6f}; evaluated_at={self.evaluated_at:.6f}; "
            f"duration={self.evaluated_at-self.observed_at:.3f} s; attempts={self.attempts}; "
            f"final_scan_age={self.evaluated_at-self.scan_stamp:.3f} s; final_feedback_age={self.evaluated_at-self.wheel_stamp:.3f} s",
            f"VERBOSE TIMING: time={self.observed_at:.6f}; scan_age={self.observed_at-self.scan_stamp:.3f} s; "
            f"feedback_age={self.observed_at-self.wheel_stamp:.3f} s (limit={self.maximum_age:.3f}); "
            f"scan/latest_feedback_skew={self.wheel_stamp-self.scan_stamp:.3f} s; "
            f"scan/matched_feedback_skew={self.association_stamp-self.scan_stamp:.3f} s (limit={self.maximum_skew:.3f})",
        ]
        if self.wall is not None and self.wall_settings is not None:
            wall, settings = self.wall, self.wall_settings
            lines.append(
                f"VERBOSE WALL: valid={wall.valid}; reason={wall.reason or 'none'}; distance={wall.distance:.4f} m; "
                f"reference={settings.reference_distance:.4f} m; error={wall.error:.4f} m (limit={settings.tolerance:.4f}); "
                f"target_progress={wall.target_progress:.4f} m; correction={wall.correction:.4f} m (limit={settings.maximum_correction:.4f})"
            )
            lines.append(
                f"VERBOSE WALL CHECK: distance_jump={wall.distance_jump:.4f} m (limit={settings.maximum_plane_jump:.4f}); "
                f"normal_change={math.degrees(wall.normal_change):.3f} deg (limit={math.degrees(settings.maximum_normal_change):.3f}); "
                f"scan/matched_wheel_skew={wall.association_stamp-wall.scan_stamp:.3f} s (limit={settings.maximum_skew:.3f})"
            )
            if wall.fit is not None:
                lines.append(
                    f"VERBOSE WALL FIT: beams={wall.fit.beams}; span={wall.fit.span:.4f} m; rms={wall.fit.residual:.4f} m; "
                    f"normal=({wall.fit.normal_x:.4f}, {wall.fit.normal_y:.4f})"
                )
        unknown = ", ".join(
            f"{section.distance:.3f}"
            for section in self.sections
            if section.clear is None
        )
        blocked = ", ".join(
            f"{section.distance:.3f}"
            for section in self.sections
            if section.clear is False
        )
        lines.append(
            f"VERBOSE PASSAGE: evaluated={bool(self.sections)}; unknown_sections=[{unknown}]; blocked_sections=[{blocked}]; positions in staging metres"
        )
        return "\n".join(lines)


# %% opt-in bounded console output
@dataclass
class TraceLogger:
    """Print periodic snapshots and immediately report changed failure conditions."""

    interval: float = 1.0
    """Minimum interval between unchanged diagnostic snapshots, in seconds."""
    printed_at: float = field(default=-math.inf, init=False)
    """Monotonic time of the last snapshot."""
    problem: str = field(default="", init=False)
    """Last failure signature, independent of the regular state logger."""

    def emit(
        self, trace: EntryTrace, now: float, problem: str = "", force: bool = False
    ) -> None:
        """Emit at most once per interval unless a failure changes or a final dump is requested."""
        if (
            not force
            and now - self.printed_at < self.interval
            and problem == self.problem
        ):
            return
        print(trace.describe(), flush=True)
        self.printed_at, self.problem = now, problem
