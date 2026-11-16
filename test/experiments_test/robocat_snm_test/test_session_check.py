"""
Protect a pre-existing Giskard session from controller preparation.
"""

# %% imports
from unittest.mock import Mock

import pytest

from experiments.robocat_snm import session_check


# %% read-only graph check
@pytest.mark.parametrize(
    "names, services, blocked",
    [
        (["giskard"], [], True),
        (["different_node"], [("/giskard/command/_action/send_goal", [])], True),
        (["unrelated_node"], [], False),
    ],
)
def test_existing_giskard_blocks_preparation_without_publishing(
    monkeypatch: pytest.MonkeyPatch,
    names: list[str],
    services: list[tuple[str, list[str]]],
    blocked: bool,
) -> None:
    """
    An existing server is rejected by node or action service, never stopped.
    """
    runtime = Mock()
    node = runtime.create_node.return_value
    node.get_node_names.return_value = names
    node.get_service_names_and_types.return_value = services
    monkeypatch.setattr(session_check, "rclpy", runtime)
    monkeypatch.setattr(session_check, "time", Mock(monotonic=Mock(side_effect=[0, 3])))
    if blocked:
        with pytest.raises(SystemExit):
            session_check.main()
    else:
        assert session_check.main() is None
    node.create_publisher.assert_not_called()
    node.destroy_node.assert_called_once_with()
