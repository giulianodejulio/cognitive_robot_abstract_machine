"""Decide when a stationary robot may start and finish an elevator crossing."""

# %% imports
from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum


# %% entry observations
class EntryPhase(StrEnum):
    """Observable lifecycle of one non-resumable entry request."""

    WAITING = "waiting_for_entry_conditions"
    ENTERING = "entering"
    STOPPING = "stopping"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELED = "canceled"


@dataclass(frozen=True)
class EntryObservation:
    """Measured conditions and the owned motion's terminal result."""

    healthy: bool
    """Fresh feedback, released interlocks and exclusive command ownership."""
    stationary: bool
    """Several wheel observations confirm the base has settled."""
    navigation_idle: bool
    """No native navigation goal owns the base."""
    door_open: bool | None
    """Measured doorway clearance; unavailable observations remain unknown."""
    passage_clear: bool | None
    """Clearance of the remaining swept corridor, not unrelated people motion."""
    sample: int
    """Identity of the scan; repeated evaluations cannot create new evidence."""
    fully_inside: bool = False
    """The robot's rear has cleared the threshold with its configured margin."""
    target_reached: bool = False
    """Measured pose meets the target's position and orientation tolerances."""
    motion_succeeded: bool = False
    """The owned Giskard action has returned success."""
    motion_terminal: bool = False
    """The owned action has completed, failed or acknowledged cancellation."""
    motion_failed: bool = False
    """The motion ended without a successful result."""
    threshold_crossing: bool = False
    """The chassis has started crossing with doors held open for this trial."""
    native_doorway_monitor: bool = False
    """The submitted native door action owns its latched opening gate."""


# %% deterministic entry policy
@dataclass
class EntryPolicy:
    """Authorize one crossing and require arrest after every interruption."""

    stability_seconds: float = 1.0
    """Continuous clear interval required before starting, in seconds."""
    minimum_scans: int = 3
    """Distinct clear scans required during that interval."""
    retain_door_clearance: bool = False
    """Retain stable opening authorization during a supervised held-open crossing."""
    phase: EntryPhase = field(default=EntryPhase.WAITING, init=False)
    """Current request state."""
    reason: str = field(
        default="Waiting for a stopped base and clear doorway", init=False
    )
    """Human-readable explanation of the current state."""
    clear_since: float | None = field(default=None, init=False)
    """Monotonic start of continuous clearance."""
    clear_samples: set[int] = field(default_factory=set, init=False)
    """Bounded evidence accumulated only until entry begins."""
    interrupted: bool = field(default=False, init=False)
    """Whether the caller explicitly canceled this request."""

    def cancel(self) -> None:
        """Request stopping without ever scheduling an automatic retry."""
        if self.phase in (EntryPhase.SUCCEEDED, EntryPhase.FAILED, EntryPhase.CANCELED):
            return
        self.interrupted = True
        self.phase = EntryPhase.STOPPING
        self.reason = "Caller canceled entry; waiting for motion termination and arrest"

    def update(self, observation: EntryObservation, now: float) -> None:
        """Advance only from current observations and the owned action result."""
        if self.phase in (EntryPhase.SUCCEEDED, EntryPhase.FAILED, EntryPhase.CANCELED):
            return
        if self.phase is EntryPhase.STOPPING:
            if observation.motion_terminal and observation.stationary:
                self.phase = (
                    EntryPhase.CANCELED if self.interrupted else EntryPhase.FAILED
                )
            return
        if self.phase is EntryPhase.WAITING:
            clear = (
                observation.healthy
                and observation.stationary
                and observation.navigation_idle
                and observation.door_open is True
                and observation.passage_clear is True
            )
            if not clear:
                self.clear_since = None
                self.clear_samples.clear()
                return
            if self.clear_since is None:
                self.clear_since = now
            if len(self.clear_samples) < self.minimum_scans:
                self.clear_samples.add(observation.sample)
            if (
                len(self.clear_samples) >= self.minimum_scans
                and now - self.clear_since >= self.stability_seconds
            ):
                self.phase = EntryPhase.ENTERING
                self.reason = "Entry conditions stable; executing the configured target"
            return
        unsafe = (
            not observation.healthy
            or not observation.navigation_idle
            or observation.passage_clear is not True
            or (
                not observation.fully_inside
                and not observation.native_doorway_monitor
                and not self.retain_door_clearance
                and (
                    observation.door_open is False
                    or (
                        observation.door_open is None
                        and not observation.threshold_crossing
                    )
                )
            )
        )
        if unsafe or observation.motion_failed:
            self.phase = EntryPhase.STOPPING
            self.reason = (
                "Entry interrupted: unsafe/unavailable feedback or motion failure"
            )
            return
        if observation.motion_succeeded:
            if (
                observation.stationary
                and observation.fully_inside
                and observation.target_reached
            ):
                self.phase = EntryPhase.SUCCEEDED
                self.reason = "Entry complete; target verified and base stopped"
            elif observation.stationary:
                self.phase = EntryPhase.STOPPING
                self.reason = "Motion succeeded but the measured final pose does not certify entry"
