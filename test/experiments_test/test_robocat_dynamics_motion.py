"""
A compiled straight motion must produce PAL-compatible dynamic commands.
"""

# %% imports
import numpy as np

from giskardpy.executor import Executor
from giskardpy.middleware.ros2.scripts.iai_robots.robocat import RobocatMotionLimits
from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPositionStraight
from semantic_digital_twin.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import DifferentialDrive


# %% generated command envelope
def test_straight_qp_respects_pal_dynamics(cylinder_bot_diff_world: World) -> None:
    """
    Actual optimized commands obey the real base's acceleration and jerk bounds.
    """
    world = cylinder_bot_diff_world
    drive = world.get_connections_by_type(DifferentialDrive)[0]
    limits = RobocatMotionLimits(maximum_forward_velocity=0.3, maximum_yaw_velocity=0.2)
    limits.apply_to(drive)
    chart = MotionStatechart()
    position = CartesianPositionStraight(
        root_link=world.root,
        tip_link=drive.child,
        goal_point=Pose.from_xyz_rpy(x=0.5, reference_frame=world.root).position,
        threshold=0.02,
    )
    chart.add_node(position)
    chart.add_node(EndMotion.when_true(position))
    executor = Executor(
        MotionStatechartContext(
            world=world, qp_controller_config=limits.controller_configuration()
        )
    )
    executor.compile(chart)
    samples = [0.0]
    for _ in range(200):
        executor.tick()
        samples.append(world.state[drive.x_velocity.id].velocity)
        if chart.is_end_motion():
            break
    assert chart.is_end_motion()
    interval = executor.context.qp_controller_config.control_time_step.total_seconds()
    acceleration = np.diff(samples) / interval
    jerk = np.diff(np.r_[0.0, acceleration]) / interval
    assert max(abs(acceleration)) <= limits.linear.acceleration + 1e-5
    assert max(abs(jerk)) <= limits.linear.jerk + 1e-5
    assert max(samples) > 0.1
    assert (
        abs(world.compute_forward_kinematics(world.root, drive.child).position.x - 0.5)
        < 0.02
    )
