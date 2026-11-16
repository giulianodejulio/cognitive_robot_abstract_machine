"""
Independent explanations of target precision and threshold clearance.
"""

# %% imports
import math
from dataclasses import replace
from unittest.mock import Mock, call

import pytest

from experiments.robocat_snm.geometry import EntryGeometry
from experiments.robocat_snm.policy import EntryObservation, EntryPhase
from semantic_digital_twin.spatial_types import Pose

from .test_application import mission
from .test_feedback import entry_inputs


# %% geometric assessment
def test_final_assessment_distinguishes_inside_from_target_precision():
    """
    Being fully inside does not imply meeting the configured target tolerance.
    """
    geometry = EntryGeometry(Pose.from_xyz_rpy(), doorway_distance=0.86, distance=1.30)
    assessment = geometry.assess(Pose.from_xyz_rpy(x=1.33))
    assert assessment.position_error == pytest.approx(0.03)
    assert assessment.yaw_error == pytest.approx(0.0)
    assert assessment.rear_clearance == pytest.approx(0.07)
    assert assessment.fully_inside is True
    assert assessment.target_reached is False


def test_final_assessment_accounts_for_heading_in_rear_clearance():
    """
    A pose within target tolerances can still lack the required rear clearance.
    """
    geometry = EntryGeometry(Pose.from_xyz_rpy(), doorway_distance=0.86, distance=1.30)
    heading = geometry.orientation_tolerance
    assessment = geometry.assess(Pose.from_xyz_rpy(x=1.281, yaw=heading))
    expected = (
        1.281
        - geometry.rear_extent * abs(math.cos(heading))
        - geometry.half_width * abs(math.sin(heading))
        - geometry.doorway_distance
        - geometry.margin
    )
    assert assessment.rear_clearance == pytest.approx(expected)
    assert assessment.fully_inside is False
    assert assessment.target_reached is True


def test_observer_retains_assessment_of_the_measured_pose(entry_inputs):
    """
    The diagnostic comes from the same wheel pose used by the decision.
    """
    entry_inputs.observe(navigation_idle=True)
    assessment = entry_inputs.last_assessment
    assert assessment.position_error == entry_inputs.frozen_geometry.distance
    assert assessment.fully_inside is False
    assert assessment.target_reached is False


# %% terminal request diagnostic
def test_terminal_failure_reports_measured_assessment(mission):
    """
    A successful motion with rejected final precision prints the numeric evidence.
    """
    geometry = EntryGeometry(Pose.from_xyz_rpy(), doorway_distance=0.86, distance=1.30)
    assessment = geometry.assess(Pose.from_xyz_rpy(x=1.33))
    mission.inputs.last_assessment = assessment
    clear = EntryObservation(True, True, True, True, True, 1)
    rejected = replace(
        clear,
        fully_inside=assessment.fully_inside,
        target_reached=assessment.target_reached,
    )
    mission.inputs.observe.side_effect = [clear, rejected, rejected]
    mission.events.report = Mock()
    assert mission.run(geometry) is EntryPhase.FAILED
    assert call(assessment.describe()) in mission.events.report.call_args_list
