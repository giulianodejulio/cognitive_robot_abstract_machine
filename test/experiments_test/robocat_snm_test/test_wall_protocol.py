"""
The wall controller settings survive native transport without runtime inputs.
"""

# %% imports
import json
from experiments.robocat_wall import WallStopSettings
from experiments.robocat_snm.lidar_entry import LidarLongitudinalTask
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from semantic_digital_twin.world import World
from semantic_digital_twin.adapters.world_entity_kwargs_tracker import (
    WorldEntityWithIDKwargsTracker,
)
from semantic_digital_twin.spatial_types import Pose


# %% native action payload
def test_wall_task_and_calibration_survive_native_json() -> None:
    world = World.create_with_root_body("odom")
    chart = MotionStatechart()
    chart.add_node(
        LidarLongitudinalTask(
            name="wall",
            root_link=world.root,
            tip_link=world.root,
            goal_pose=Pose.from_xyz_rpy(x=1.3, reference_frame=world.root),
            settings=WallStopSettings(0.936),
            threshold=0.05,
        )
    )
    restored = MotionStatechart.from_json(
        json.loads(json.dumps(chart.to_json())),
        **WorldEntityWithIDKwargsTracker.from_world(world).create_kwargs()
    )
    assert isinstance(restored.nodes[0], LidarLongitudinalTask)
    assert restored.nodes[0].settings == WallStopSettings(0.936)
