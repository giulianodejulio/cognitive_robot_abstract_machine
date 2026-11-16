"""
SNM entry bags retain the exact final acceptance quantities.
"""

# %% imports
from unittest.mock import Mock

from experiments.robocat_snm.feedback import EntryInputs

from .test_feedback import entry_inputs


# %% numerical final conditions
def test_pose_diagnostic_includes_entry_assessment(entry_inputs: EntryInputs) -> None:
    """
    Goal precision and rear clearance remain distinct in the recorded data.
    """
    entry_inputs.comparison_publisher = Mock()
    entry_inputs.comparison_publisher.get_subscription_count.return_value = 1
    entry_inputs.observe(navigation_idle=True)
    message = entry_inputs.comparison_publisher.publish.call_args.args[0]
    values = {item.key: item.value for item in message.status[0].values}
    assessment = entry_inputs.last_assessment
    assert float(values["assessment.position_error"]) == assessment.position_error
    assert float(values["assessment.rear_clearance"]) == assessment.rear_clearance
    assert (
        float(values["assessment.position_tolerance"]) == assessment.position_tolerance
    )
    assert (
        float(values["assessment.orientation_tolerance"])
        == assessment.orientation_tolerance
    )
