"""
Regression checks for differential base alignment and planar goals.
"""

# %% imports
from __future__ import annotations

import math
from types import SimpleNamespace

import numpy as np
import pytest

from giskardpy.motion_statechart.data_types import DefaultWeights
from giskardpy.motion_statechart.goals.cartesian_goals import DifferentialDriveBaseGoal
from giskardpy.motion_statechart.goals.templates import Parallel
from giskardpy.motion_statechart.tasks.cartesian_tasks import (
    CartesianPositionVelocityLimit,
)
from semantic_digital_twin.spatial_types import Pose
from .test_tiago_differential_trial import expansion_context


# %% alignment and planar targets
def test_initial_alignment_limits_translation_at_maximum_priority(
    expansion_context: SimpleNamespace,
) -> None:
    """
    A heading correction cannot be traded for reversing towards the goal.
    """
    goal = DifferentialDriveBaseGoal(
        goal_pose=expansion_context.world.transform.return_value,
        diff_drive_connection=expansion_context.drive,
    )
    goal.expand(expansion_context)
    assert type(goal.nodes[0]) is Parallel
    limiter = goal.nodes[0].nodes[1]
    assert type(limiter) is CartesianPositionVelocityLimit
    assert limiter.max_linear_velocity == goal.translation_velocity_while_turning
    assert limiter.weight == DefaultWeights.WEIGHT_MAXIMUM


def test_planar_goal_retains_the_current_base_height(
    expansion_context: SimpleNamespace,
) -> None:
    """
    An unreachable goal height must not prevent a base-only motion completing.
    """
    root = expansion_context.world.root
    target = Pose.from_xyz_rpy(x=0.5, z=0.8, reference_frame=root)
    current = Pose.from_xyz_rpy(z=0.1, reference_frame=root)
    expansion_context.world.transform.return_value = target
    expansion_context.world.compose_forward_kinematics_expression.return_value = current
    goal = DifferentialDriveBaseGoal(
        goal_pose=target, diff_drive_connection=expansion_context.drive
    )
    goal.expand(expansion_context)
    for phase in goal.nodes[1:]:
        assert float(phase.goal_pose.z) == pytest.approx(float(current.z))


@pytest.mark.parametrize("x,y", [(0.0, 0.0), (0.004, 0.003)])
def test_nearby_goal_uses_final_heading_instead_of_bearing(
    expansion_context: SimpleNamespace, x: float, y: float
) -> None:
    """
    At the position threshold, heading must remain finite and face the final yaw.
    """
    target = Pose.from_xyz_rpy(
        x=x, y=y, yaw=-math.pi / 2, reference_frame=expansion_context.world.root
    )
    expansion_context.world.transform.return_value = target
    goal = DifferentialDriveBaseGoal(
        goal_pose=target, diff_drive_connection=expansion_context.drive
    )
    goal.expand(expansion_context)
    orientation = goal.nodes[0].nodes[0].goal_orientation
    np.testing.assert_allclose(orientation.to_np(), target.rotation_matrix.to_np())
