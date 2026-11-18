"""
Orientation progress measures the shortest rotation across angular branches.
"""

# %% imports
import math

import pytest

from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianOrientation
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types import (
    HomogeneousTransformationMatrix,
    RotationMatrix,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.world_entity import Body


# %% measured orientation residual
@pytest.mark.parametrize(
    "current_yaw,target_yaw",
    [
        (2.1426351199754463, -0.9995333643961807),
        (3.0494822938926567, -0.9995333643961807),
        (math.radians(179), math.radians(-179)),
        (math.radians(-179), math.radians(179)),
        (0.7, 0.7 + math.pi),
        (-2.8, -2.8 + math.pi),
    ],
)
def test_orientation_error_is_shortest_distance(
    current_yaw: float, target_yaw: float
) -> None:
    """
    The native task exposes an error in [0, pi] for either equivalent angular branch.
    """
    world = World.create_with_root_body("odom")
    body = Body(name=PrefixedName("base"))
    with world.modify_world():
        world.add_connection(
            FixedConnection(
                parent=world.root,
                child=body,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    yaw=current_yaw
                ),
            )
        )
    task = CartesianOrientation(
        root_link=world.root,
        tip_link=body,
        goal_orientation=RotationMatrix.from_rpy(
            yaw=target_yaw, reference_frame=world.root
        ),
    )
    artifacts = task.build(MotionStatechartContext(world=world))
    expected = abs(
        math.atan2(
            math.sin(target_yaw - current_yaw), math.cos(target_yaw - current_yaw)
        )
    )
    assert float(artifacts.error.evaluate()[0]) == pytest.approx(expected)
