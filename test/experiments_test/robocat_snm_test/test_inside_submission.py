"""
The resolved waypoint reaches the existing native entry motion.
"""

# %% imports
from unittest.mock import Mock

from experiments.robocat_snm import application
from experiments.robocat_snm.policy import EntryPhase
from semantic_digital_twin.spatial_types import Pose

from .test_application import mission


# %% configured target submission
def test_entry_submits_frozen_waypoint_with_explicit_relaxed_tolerance(mission) -> None:
    """
    A waypoint request passes its transformed target and acceptance to Giskard.
    """
    mission.options.inside_waypoint = "elevator_inside"
    mission.options.position_tolerance = 0.05
    inside = Pose.from_xyz_rpy(x=1.4, y=-0.04, yaw=0.01)
    geometry = mission.options.entry_geometry(Pose.from_xyz_rpy(), 0.86, inside=inside)
    selected_target = Pose.from_xyz_rpy(x=5, y=6, yaw=0.2)
    mission.inputs.snapshot.feedback_target = selected_target
    assert mission.run(geometry) is EntryPhase.SUCCEEDED
    call = application.RobocatStraightMotion.call_args
    assert call.args[0] is selected_target
    assert call.kwargs["tolerances"].position == geometry.position_tolerance
    assert call.kwargs["lateral_tolerance"] == geometry.lateral_tolerance
    mission.pool.submit.assert_called_once()
