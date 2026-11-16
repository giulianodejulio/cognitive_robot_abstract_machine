"""
Report the concrete feedback blocker that interrupts native navigation.
"""

# %% imports
import pytest

from experiments.robocat_snm.application import EntryMission
from experiments.robocat_snm.exceptions import EntryError

from .test_application import mission


# %% interrupted feedback
def test_native_approach_error_contains_feedback_blockers(
    mission: EntryMission,
) -> None:
    """
    An expired safety stream appears in the raised error without submitting Giskard.
    """
    mission.inputs.feedback.ready.return_value = False
    reasons = ["/diagnostics: safety stream missing or stale", "wheel odometry stale"]
    mission.inputs.feedback.blocking_reasons.return_value = reasons
    with pytest.raises(EntryError) as caught:
        mission.approach()
    assert all(reason in caught.value.reason for reason in reasons)
    mission.pool.submit.assert_not_called()
