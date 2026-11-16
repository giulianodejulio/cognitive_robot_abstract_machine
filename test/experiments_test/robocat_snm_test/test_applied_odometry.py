"""
Server input telemetry identifies the sample actually written into the model.
"""

# %% imports
from unittest.mock import Mock

from nav_msgs.msg import Odometry

from giskardpy.middleware.ros2.scripts.iai_robots.robocat import RobocatOdometrySource


# %% application timestamp ownership
def test_base_source_retains_applied_sample_independently_of_latest() -> None:
    """
    A later received message does not change which sample built the decision pose.
    """
    source = object.__new__(RobocatOdometrySource)
    source.connection = Mock()
    applied = Odometry()
    applied.pose.pose.orientation.w = 1
    applied.header.stamp.sec = 100
    source.apply_message(applied)
    source.latest_message = Odometry()
    source.latest_message.header.stamp.sec = 101
    assert source.applied_message is applied
    assert source.applied_message.header.stamp.sec == 100
