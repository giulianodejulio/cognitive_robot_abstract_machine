"""
Dispatch the CLI handover mode without constructing an elevator entry goal.
"""

# %% imports
from unittest.mock import Mock

import pytest
from rcl_interfaces.msg import ParameterValue

from experiments.robocat_snm import application
from experiments.robocat_snm.application import EntryOptions
from semantic_digital_twin.spatial_types import Pose


# %% executable CLI boundary
def test_handover_cli_never_dispatches_entry(monkeypatch: pytest.MonkeyPatch) -> None:
    """
    An authorized staging-only request calls handover without an entry geometry.
    """
    options = EntryOptions(
        execute=True, handover_only=True, map_fingerprint="checked-map"
    )
    staging = Pose.from_xyz_rpy()
    monkeypatch.setattr(EntryOptions, "parse", Mock(return_value=options))
    monkeypatch.setattr(EntryOptions, "load_staging", Mock(return_value=staging))
    monkeypatch.setattr(application, "RobocatEnvironment", Mock())
    node, world, context, inputs = Mock(), Mock(), Mock(), Mock()
    world.get_semantic_annotations_by_type.return_value = [Mock()]
    inputs.scene.fingerprint = options.map_fingerprint
    inputs.map_pose.return_value = staging
    inputs.feedback.ready.return_value = True
    inputs.feedback.wheels_settled.return_value = True
    monkeypatch.setattr(application.rospy, "init_node", Mock())
    monkeypatch.setattr(application.rospy, "shutdown", Mock())
    monkeypatch.setattr(application.rospy, "get_node", Mock(return_value=node))
    monkeypatch.setattr(
        application, "fetch_world_from_service", Mock(return_value=world)
    )
    monkeypatch.setattr(application, "WorldSynchronizer", Mock())
    monkeypatch.setattr(application, "Context", Mock(return_value=context))
    monkeypatch.setattr(application, "EntryInputs", Mock(return_value=inputs))
    settings = Mock()
    settings.done.return_value = True
    settings.exception.return_value = None
    settings.result.return_value.values = [
        ParameterValue(string_value="/mobile_base_controller/odom"),
        ParameterValue(string_value="/snm_vel"),
    ]
    parameters = Mock()
    parameters.get_parameters.return_value = settings
    monkeypatch.setattr(
        application, "AsyncParameterClient", Mock(return_value=parameters)
    )
    navigation = Mock()
    navigation_factory = Mock(return_value=navigation)
    monkeypatch.setattr(application, "NavigationConnection", navigation_factory)
    mission = Mock()
    monkeypatch.setattr(application, "EntryMission", Mock(return_value=mission))
    geometry = Mock()
    motion = Mock()
    monkeypatch.setattr(application, "EntryGeometry", geometry)
    monkeypatch.setattr(application, "execute_single", motion)
    application.main()
    mission.handover.assert_called_once_with(staging)
    mission.run.assert_not_called()
    geometry.assert_not_called()
    motion.assert_not_called()
    assert navigation_factory.call_args.kwargs["execute"] is True
    navigation.close.assert_called_once_with()
