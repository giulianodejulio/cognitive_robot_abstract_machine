"""
Bounded timestamp association for delayed lidar observations.
"""

# %% imports
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from functools import partial

from nav_msgs.msg import Odometry


# %% timestamp association
@dataclass
class OdometryHistory:
    """
    Retain wheel samples for association with sensor acquisition times.
    """

    samples: deque[Odometry] = field(default_factory=partial(deque, maxlen=200))
    """
    Bounded recent wheel messages in receipt order.
    """

    def nearest(self, timestamp: float, maximum_skew: float) -> Odometry | None:
        """
        Return the closest measured sample within the permitted timestamp distance.
        """
        samples = tuple(self.samples)
        if not samples:
            return None
        candidate = min(
            samples,
            key=lambda message: abs(
                message.header.stamp.sec
                + message.header.stamp.nanosec / 1e9
                - timestamp
            ),
        )
        stamp = candidate.header.stamp.sec + candidate.header.stamp.nanosec / 1e9
        return candidate if abs(stamp - timestamp) <= maximum_skew else None
