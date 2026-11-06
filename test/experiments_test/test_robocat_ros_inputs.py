# %% imports
from __future__ import annotations

import pytest
from rclpy.qos import (
    DurabilityPolicy,
    LivelinessPolicy,
    QoSCompatibility,
    QoSProfile,
    ReliabilityPolicy,
    qos_check_compatible,
)

from giskardpy.middleware.ros2.scripts.iai_robots.robocat import (
    RobocatJointStateSource,
    RobocatLidarSource,
    RobocatOdometrySource,
)


# %% bridge sensor QoS interoperability
@pytest.mark.parametrize(
    "source", [RobocatJointStateSource, RobocatLidarSource, RobocatOdometrySource]
)
def test_bridge_sensor_publishers_are_compatible(source: type) -> None:
    """
    Accept best-effort sensor publishers exposed by the DDS bridge.
    """
    publisher = QoSProfile(
        depth=20,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        liveliness=LivelinessPolicy.AUTOMATIC,
    )
    compatibility, reason = qos_check_compatible(publisher, source.qos_profile)
    assert compatibility == QoSCompatibility.OK
