"""
Robocat solver bounds must respect the measured PAL controller configuration.
"""

# %% imports
import pytest

from giskardpy.middleware.ros2.scripts.iai_robots.robocat import RobocatMotionLimits
from giskardpy.qp.dof_limits import DegreeOfFreedomLimitProfiler
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import DifferentialDrive


# %% PAL-compatible bounds
@pytest.mark.parametrize("speed", [0.1, 0.3])
def test_base_dynamics_and_horizon_respect_pal(
    cylinder_bot_diff_world: World,
    speed: float,
) -> None:
    """
    Both drive axes can brake within the selected horizon without exceeding PAL.
    """
    drive = cylinder_bot_diff_world.get_connections_by_type(DifferentialDrive)[0]
    limits = RobocatMotionLimits(
        maximum_forward_velocity=speed, maximum_yaw_velocity=0.2
    )
    limits.apply_to(drive)
    assert drive.x_velocity.limits.upper.acceleration == 0.6
    assert drive.x_velocity.limits.lower.acceleration == -0.6
    assert drive.x_velocity.limits.upper.jerk == 2.0
    assert drive.x_velocity.limits.lower.jerk == -2.0
    assert drive.yaw.limits.upper.acceleration == 1.0
    assert drive.yaw.limits.lower.acceleration == -1.0
    assert drive.yaw.limits.upper.jerk == 5.0
    assert drive.yaw.limits.lower.jerk == -5.0
    configuration = limits.controller_configuration()
    assert configuration.target_frequency == 20
    profiler = DegreeOfFreedomLimitProfiler(configuration)
    for degree in (drive.x_velocity, drive.yaw):
        profiler.compute(degree)
        resolved = profiler.resolve_limits(degree)
        assert resolved.upper.jerk <= degree.limits.upper.jerk
    assert configuration.prediction_horizon > 7
