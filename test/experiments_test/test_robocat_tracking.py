"""
Check timestamped tracker telemetry without ROS execution or robot motion.
"""

# %% imports
from types import SimpleNamespace
from unittest.mock import Mock
import math

import numpy as np
import pytest

from experiments.robocat_tracking import TrackingSample, TrackingPublisher
from giskardpy.motion_statechart.trajectory_tracking import TrajectoryTrackingState
from giskardpy.motion_statechart.trajectory_tracking import TrajectoryTrackingContext
from giskardpy.motion_statechart.tasks.cartesian_tasks import (
    CartesianPositionTrajectory,
)
from semantic_digital_twin.spatial_types import Pose


# %% measured geometry
@pytest.fixture
def trajectory() -> SimpleNamespace:
    """
    Provide a planar native task's state independently of a robot model.
    """
    return SimpleNamespace(
        _goal_points_np=np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]),
        current_index=0,
        threshold=0.02,
        control_heading=True,
        goal_reference_frame=Mock(),
        tip_link=Mock(),
        remaining_distance=Mock(),
    )


def test_tracking_sample_reports_signed_errors(trajectory: SimpleNamespace) -> None:
    """
    Use the actual moving target, signed lateral offset and wrapped yaw error.
    """
    state = TrajectoryTrackingState(
        np.array([0.4, 0.1, 0.0]), np.array([0.5, 0.0, 0.0])
    )
    sample = TrackingSample.from_state(trajectory, state, actual_yaw=0.2, remaining=0.7)
    assert sample.desired_yaw == pytest.approx(-math.pi / 4)
    assert sample.actual_yaw == 0.2
    assert sample.heading_error == pytest.approx(-math.pi / 4 - 0.2)
    assert sample.cross_track_error == pytest.approx(0.1)
    assert sample.distance_to_goal == pytest.approx(math.hypot(0.6, 0.1))
    assert sample.path_index == trajectory.current_index
    assert sample.remaining_distance == 0.7


def test_arrival_telemetry_matches_heading_hold(trajectory: SimpleNamespace) -> None:
    """
    Do not report atan2 of an almost-zero target vector as commanded yaw.
    """
    state = TrajectoryTrackingState(
        np.array([1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])
    )
    sample = TrackingSample.from_state(trajectory, state, actual_yaw=0.7, remaining=0.0)
    assert sample.desired_yaw == sample.actual_yaw == 0.7
    assert sample.heading_error == 0.0


def test_yaw_error_wraps_across_pi(trajectory: SimpleNamespace) -> None:
    """
    A wrap of the plotted yaw is not reported as a full-turn tracking error.
    """
    state = TrajectoryTrackingState(
        np.zeros(3), np.array([-math.cos(0.1), -math.sin(0.1), 0.0])
    )
    sample = TrackingSample.from_state(trajectory, state, math.pi - 0.1, 1.0)
    assert sample.heading_error == pytest.approx(0.2)


def test_native_tick_reports_the_target_it_actually_selected() -> None:
    """
    The observer receives the native same-cycle target, not a reconstructed one.
    """
    task = object.__new__(CartesianPositionTrajectory)
    task._goal_points_np = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    task.current_index = 0
    task.look_ahead_distance = 0.1
    task._compiled_goal_reference_frame_P_tip = Mock()
    task._compiled_goal_reference_frame_P_tip.evaluate.return_value = np.array(
        [0.4, 0.0, 0.0]
    )
    task.remaining_distance = Mock()
    task.goal_reference_frame_P_current_target_point = Mock()
    context, observer = Mock(), Mock()
    context.extensions = {
        TrajectoryTrackingContext: TrajectoryTrackingContext(observer)
    }
    task.on_tick(context)
    observer.observe.assert_called_once()
    observed_task, observed_context, sample = observer.observe.call_args.args
    assert observed_task is task
    assert observed_context is context
    np.testing.assert_allclose(sample.position, [0.4, 0.0, 0.0])
    np.testing.assert_allclose(sample.target, [0.5, 0.0, 0.0])


# %% publication budget
def test_no_subscribers_skip_geometry_and_publication(
    trajectory: SimpleNamespace,
) -> None:
    """
    An unused telemetry topic does not request additional forward kinematics.
    """
    node, context = Mock(), Mock()
    publisher = TrackingPublisher(node)
    publisher.publisher.get_subscription_count.return_value = 0
    publisher.observe(trajectory, context, Mock())
    context.world.compute_forward_kinematics.assert_not_called()
    publisher.publisher.publish.assert_not_called()


def test_publication_is_timestamped_and_throttled(
    trajectory: SimpleNamespace, monkeypatch: pytest.MonkeyPatch
) -> None:
    """
    Emit at most the configured rate, using numeric samples for PlotJuggler.
    """
    node, context = Mock(), Mock()
    publisher = TrackingPublisher(node, frequency=10.0)
    publisher.publisher.get_subscription_count.return_value = 1
    context.world.compute_forward_kinematics.return_value = Pose.from_xyz_rpy(
        yaw=0.2
    ).homogeneous_matrix
    context.float_variable_data.get_value.return_value = 0.7
    stamp = node.get_clock.return_value.now.return_value.to_msg.return_value
    stamp.sec, stamp.nanosec = 123, 456
    instant = [0.0]
    monkeypatch.setattr(
        "experiments.robocat_tracking.time.monotonic", lambda: instant[0]
    )
    state = TrajectoryTrackingState(
        np.array([0.4, 0.1, 0.0]), np.array([0.5, 0.0, 0.0])
    )
    publisher.observe(trajectory, context, state)
    publisher.observe(trajectory, context, state)
    publisher.publisher.publish.assert_called_once()
    message = publisher.publisher.publish.call_args.args[0]
    values = dict(zip(message.layout.dim[0].label.split(","), message.data))
    assert values["stamp_sec"] == stamp.sec
    assert values["stamp_nanosec"] == stamp.nanosec
    assert values["actual_yaw"] == pytest.approx(0.2)
    assert values["sequence"] == 1.0
    instant[0] = 0.11
    publisher.observe(trajectory, context, state)
    assert publisher.publisher.publish.call_count == 2
