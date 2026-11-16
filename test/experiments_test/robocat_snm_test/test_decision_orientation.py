"""
Server diagnostic orientation goals retain their bound reference frame.
"""

# %% imports
from unittest.mock import Mock

import pytest
from nav_msgs.msg import Odometry

from experiments.robocat_goal_diagnostics import GoalDecisionObserver, DecisionStage
from giskardpy.executor import Executor
from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianOrientation
from semantic_digital_twin.adapters.ros.input_synchronization import (
    OdometrySynchronizer,
)
from semantic_digital_twin.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import DifferentialDrive


# %% bound heading and held odometry
def test_bound_heading_and_sensor_stamp_are_recorded(
    cylinder_bot_diff_world: World,
) -> None:
    """
    The event distinguishes server decision time from source acquisition time.
    """
    world = cylinder_bot_diff_world
    drive = world.get_connections_by_type(DifferentialDrive)[0]
    task = CartesianOrientation(
        name="entry_heading",
        root_link=world.root,
        tip_link=drive.child,
        goal_orientation=Pose.from_xyz_rpy(
            yaw=0.3, reference_frame=world.root
        ).rotation_matrix,
        threshold=0.0872665,
    )
    chart = MotionStatechart()
    chart.add_node(task)
    executor = Executor(MotionStatechartContext(world=world))
    executor.compile(chart)
    measured = Odometry()
    measured.header.stamp.sec = 100
    measured.header.stamp.nanosec = 1234
    measured.header.frame_id = "odom"
    measured.child_frame_id = "base_footprint"
    measured.pose.pose.position.x = 0.25
    source = Mock(
        spec=OdometrySynchronizer, latest_message=measured, topic_name="/wheel"
    )
    node = Mock()
    node.resolve_topic_name.return_value = "/mobile_base_controller/odom"
    observer = GoalDecisionObserver(
        node, executor, Mock(goal_id=4), Mock(synchronizers=[source])
    )
    observer.capture(DecisionStage.COMPILED)
    message = observer.publisher.publish.call_args.args[0]
    values = {item.key: item.value for item in message.status[0].values}
    assert float(values["target.yaw"]) == pytest.approx(0.3)
    assert float(values["task_error"]) == pytest.approx(0.3)
    assert values["feedback_stamp_sec"] == "100"
    assert values["feedback_stamp_nanosec"] == "1234"
    assert values["feedback_topic"] == "/mobile_base_controller/odom"
    assert values["feedback_child_frame"] == measured.child_frame_id
    assert values["feedback.x"] == "0.25"
