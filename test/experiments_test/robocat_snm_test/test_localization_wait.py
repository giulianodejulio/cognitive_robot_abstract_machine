"""
Wait for localization without requesting robot movement.
"""

# %% imports
from unittest.mock import Mock

from experiments.robocat_snm.application import EntryLocalization, EntryStop


# %% initial map pose
def test_localization_wait_resumes_when_pose_arrives() -> None:
    """
    Missing initial TF keeps the check waiting until a pose is available.
    """
    inputs = Mock()
    inputs.map_pose.side_effect = [None, object()]
    assert EntryLocalization(inputs, EntryStop(), poll_seconds=0).wait() is True
    assert inputs.map_pose.call_count == 2
    inputs.context.giskard_wrapper.send_goal.assert_not_called()


def test_localization_wait_stops_on_interrupt() -> None:
    """
    An interrupted startup cannot continue into navigation checks.
    """
    inputs = Mock()
    assert EntryLocalization(inputs, EntryStop(requested=True)).wait() is False
    inputs.map_pose.assert_not_called()
