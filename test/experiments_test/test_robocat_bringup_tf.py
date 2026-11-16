"""
Keep model transforms distinct from the physical robot TF tree.
"""

# %% imports
from unittest.mock import Mock, MagicMock

from scripts import robocat_giskard_bringup as bringup


# %% model visualization isolation
def test_bringup_prefixes_model_tf_without_replacing_native_base(monkeypatch) -> None:
    """
    The controller must not publish a second unprefixed base_footprint.
    """
    server = Mock()
    original = server.tf_publisher
    publisher = Mock()
    monkeypatch.setattr(bringup, "TFPublisher", publisher, raising=False)
    monkeypatch.setattr(bringup, "Giskard", Mock(return_value=server))
    monkeypatch.setattr(bringup, "RobocatEnvironment", Mock())
    monkeypatch.setattr(bringup, "WorldWithRobocatConfig", Mock())
    monkeypatch.setattr(bringup, "RobocatBaseInterface", Mock())
    monkeypatch.setattr(bringup, "GracefulShutdownSignals", MagicMock())
    monkeypatch.setattr(bringup, "rospy", Mock())
    monkeypatch.setattr(bringup.time, "sleep", Mock())
    monkeypatch.setattr(bringup, "remove_ros_args", lambda: ["bringup"])
    bringup.rospy.get_node.return_value.get_service_names_and_types.return_value = []
    bringup.main()
    original.stop.assert_called_once_with()
    assert publisher.call_args.kwargs["frame_names"].prefix == "giskard/"
    assert server.tf_publisher is publisher.return_value
