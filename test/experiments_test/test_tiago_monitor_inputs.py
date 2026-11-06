# %% imports

from __future__ import annotations

import math
from pathlib import Path

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("sensor_msgs.msg")

from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, JointState

from experiments.tiago_monitor.observer import RobotObserver
from experiments.tiago_monitor.status import StreamCondition
from experiments.tiago_monitor.view import RobotView
from semantic_digital_twin.robots.tiago_single_arm import TiagoSingleArmJoint

# %% mocked observations


@pytest.fixture
def observer(rclpy_node: Node) -> RobotObserver:
    """
    Create subscriptions without spinning or contacting a real robot.
    """
    packages = pytest.importorskip("ament_index_python.packages")
    required = {
        "tiago_description",
        "pmb2_description",
        "pal_gripper_description",
        "pal_urdf_utils",
    }
    if not required.issubset(packages.get_packages_with_prefixes()):
        pytest.skip("PAL description packages are not installed")
    return RobotObserver(node=rclpy_node)


def test_observer_applies_mocked_joint_positions(observer: RobotObserver) -> None:
    """
    Received positions update the twin while hardware interfaces stay disabled.
    """
    message = JointState()
    message.name = [TiagoSingleArmJoint.ARM_1, TiagoSingleArmJoint.TORSO_LIFT]
    message.position = [0.25, 0.15]
    message.header.stamp = observer.node.get_clock().now().to_msg()
    observer.observed_inputs[0].subscriber.buffer_message(message)

    observer.synchronize()

    assert [
        observer.world.state[
            observer.world.get_connection_by_name(joint_name).raw_dof.id
        ].position
        for joint_name in message.name
    ] == list(message.position)
    assert observer.robot.degrees_of_freedom_with_hardware_interface == []


def test_consumed_joint_sample_keeps_its_freshness_record(
    observer: RobotObserver,
) -> None:
    """
    A cycle without a new message retains the last joint timestamp.
    """
    message = JointState()
    message.header.stamp = observer.node.get_clock().now().to_msg()
    observer.observed_inputs[0].subscriber.buffer_message(message)
    observer.synchronize()
    sample_time = observer.observed_inputs[0].status.sample_time

    observer.synchronize()

    assert observer.observed_inputs[0].status.sample_time == sample_time
    assert observer.observed_inputs[0].status.condition == StreamCondition.CURRENT


def test_camera_uses_received_intrinsics_and_optical_frame(
    observer: RobotObserver,
) -> None:
    """
    Camera parameters come from the mocked message rather than inherited defaults.
    """
    information = CameraInfo()
    information.header.frame_id = "head_front_camera_color_optical_frame"
    information.width = 640
    information.height = 480
    information.k = [500.0, 0.0, 319.5, 0.0, 500.0, 239.5, 0.0, 0.0, 1.0]
    observer.camera.buffer_message(information)

    observer.synchronize()

    camera = observer.robot.get_default_camera()
    assert camera.root.name.name == information.header.frame_id
    assert camera.field_of_view.horizontal_angle == 2 * math.atan(
        information.width / (2 * information.k[0])
    )
    assert camera.field_of_view.vertical_angle == 2 * math.atan(
        information.height / (2 * information.k[4])
    )


def test_observer_starts_with_waiting_streams(
    observer: RobotObserver, tmp_path: Path
) -> None:
    """
    A monitor can publish its waiting state before the first robot message.
    """
    snapshot = observer.observe(RobotView(world=observer.world, directory=tmp_path))

    assert [stream.condition for stream in snapshot.streams] == [
        StreamCondition.WAITING
    ] * len(snapshot.streams)
    assert snapshot.penetrating_contacts is None
