# %% imports

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

# %% observed streams


class StreamCondition(StrEnum):
    """
    Whether a received timestamp can represent the current robot.
    """

    WAITING = "waiting"
    CURRENT = "current"
    STALE = "stale"
    CLOCK_SKEW = "clock_skew"


@dataclass
class StreamStatus:
    """
    The most recent sample time and freshness of one input stream.
    """

    topic: str
    """
    The topic or transform identifying this stream.
    """

    sample_time: float | None = None
    """
    The timestamp supplied by the source, in seconds.
    """

    age: float | None = None
    """
    The signed age of the sample at the last update.
    """

    condition: str = StreamCondition.WAITING.value
    """
    The condition evaluated at the last update.
    """

    def update(self, current_time: float, maximum_age: float) -> None:
        """
        Classify a sample against the supplied clock and tolerated age.
        """
        if self.sample_time is None:
            self.age = None
            self.condition = StreamCondition.WAITING.value
            return
        self.age = current_time - self.sample_time
        if self.age < -maximum_age:
            self.condition = StreamCondition.CLOCK_SKEW.value
            return
        if self.age > maximum_age:
            self.condition = StreamCondition.STALE.value
            return
        self.condition = StreamCondition.CURRENT.value


# %% browser snapshots


@dataclass
class MeshPose:
    """
    A visual mesh transform in the viewer's world frame.
    """

    node_name: str
    """
    The name of the node in the exported scene.
    """

    matrix: list[float]
    """
    The homogeneous transform flattened in column-major order.
    """


@dataclass
class MonitorSnapshot:
    """
    The state shown by the browser after one observation cycle.
    """

    generated_at: float
    """
    The ROS clock time of this snapshot, in seconds.
    """

    maximum_age: float
    """
    The maximum tolerated age of an input sample, in seconds.
    """

    streams: list[StreamStatus]
    """
    The freshness of the required input streams.
    """

    poses: list[MeshPose]
    """
    The transforms of all visual mesh nodes.
    """

    lidar_translation_error: float | None
    """
    The translation difference between model and latest lidar TF, in metres.
    """

    penetrating_contacts: int | None
    """
    The configured collision contacts at non-positive distance, if evaluated.
    """
