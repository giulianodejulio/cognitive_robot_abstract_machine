"""
Verify calibrated doorway frames without connecting to the robot.
"""

# %% imports
from __future__ import annotations

from pathlib import Path
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

from experiments.robocat_doorway_view import CalibratedDoorway, DoorwayView, ViewTopic


# %% fixed calibration and independent wheel feedback
def test_calibrated_doorway_is_fixed_in_odometry() -> None:
    """
    Keep the doorway pose independent of subsequent robot poses.
    """
    frame = CalibratedDoorway(x=1.2, y=2.3, yaw=0.0).transform()
    assert frame.header.frame_id == "odom"
    assert frame.child_frame_id == "elevator_doorway"
    assert frame.transform.translation.x == 1.2
    assert frame.transform.translation.y == 2.3
    assert frame.transform.rotation.w == 1.0


def test_wheel_feedback_places_the_robot_without_moving_the_doorway() -> None:
    """
    Publish the measured wheel pose while retaining the calibrated static frame.
    """
    view = DoorwayView(Mock(), Mock(), CalibratedDoorway(1.2, 2.3, 0.0))
    message = Odometry()
    message.header.frame_id = "odom"
    message.child_frame_id = "base_footprint"
    message.pose.pose.position.x = 4.0
    message.pose.pose.orientation.w = 1.0
    view.receive_wheels(message)
    transform = view.publishers[ViewTopic.TRANSFORMS].publish.call_args.args[0]
    assert transform.transforms[0].transform.translation.x == 4.0
    assert view.static_frames["elevator_doorway"].transform.translation.x == 1.2


def test_native_odometry_transform_is_not_forwarded() -> None:
    """
    Exclude DLO and map localization from the wheel-based visualization.
    """
    view = DoorwayView(Mock(), Mock(), CalibratedDoorway(1.2, 2.3, 0.0))
    native = TransformStamped()
    native.header.frame_id = "odom"
    native.child_frame_id = "base_footprint"
    joint = TransformStamped()
    joint.header.frame_id = "base_link"
    joint.child_frame_id = "head_link"
    view.receive_transforms(TFMessage(transforms=[native, joint]))
    published = view.publishers[ViewTopic.TRANSFORMS].publish.call_args.args[0]
    assert published.transforms == [joint]


def test_invalid_wheel_frame_is_ignored() -> None:
    """
    Reject wheel feedback expressed in an unexpected reference frame.
    """
    view = DoorwayView(Mock(), Mock(), CalibratedDoorway(1.2, 2.3, 0.0))
    view.receive_wheels(Odometry())
    view.publishers[ViewTopic.TRANSFORMS].publish.assert_not_called()
