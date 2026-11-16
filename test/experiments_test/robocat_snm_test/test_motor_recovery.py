"""
Motor recovery requires current driver evidence, not controller activity alone.
"""

# %% imports
from unittest.mock import Mock

import pytest
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from experiments.robocat_snm.motor_recovery import RecoverySafety


# %% actuator evidence
@pytest.mark.parametrize("failure", ["missing", "disabled", "fault", "stale"])
def test_recovery_rejects_incomplete_or_faulted_drivers(failure, monkeypatch) -> None:
    """
    Recovery never succeeds from missing, disabled, erroneous or old feedback.
    """
    safety = RecoverySafety(Mock())
    monkeypatch.setattr(
        "experiments.robocat_snm.motor_recovery.time.monotonic", lambda: 10.0
    )
    for side in ("left", "right"):
        if failure == "missing" and side == "right":
            continue
        status = DiagnosticStatus(
            name=f"/Hardware/Actuators/wheel_{side}_actuator: ",
            level=(
                DiagnosticStatus.ERROR
                if failure == "fault" and side == "right"
                else DiagnosticStatus.OK
            ),
            values=[
                KeyValue(key="Driver state", value="MotorOperational"),
                KeyValue(
                    key="Motor status",
                    value=(
                        "Disabled"
                        if failure == "disabled" and side == "right"
                        else "Enabled"
                    ),
                ),
                KeyValue(key="Motor errors", value="No errors"),
                KeyValue(key="ESM", value="OP"),
            ],
        )
        safety.receive_diagnostics(DiagnosticArray(status=[status]))
    if failure == "stale":
        monkeypatch.setattr(
            "experiments.robocat_snm.motor_recovery.time.monotonic", lambda: 20.0
        )
    assert safety.drivers_ready() is False


def test_recovery_accepts_both_operational_drivers_but_retains_fault_evidence() -> None:
    """
    Faulted motors may be recovered while their fault remains independently visible.
    """
    safety = RecoverySafety(Mock())
    statuses = []
    for side in ("left", "right"):
        statuses.append(
            DiagnosticStatus(
                name=f"/Hardware/Actuators/wheel_{side}_actuator: ",
                level=DiagnosticStatus.OK,
                values=[
                    KeyValue(key=key, value=value)
                    for key, value in {
                        "Driver state": "MotorOperational",
                        "Motor status": "Enabled",
                        "Motor errors": "No errors",
                        "ESM": "OP",
                    }.items()
                ],
            )
        )
    safety.receive_diagnostics(DiagnosticArray(status=statuses))
    assert safety.drivers_ready() is True
    assert len(safety.drivers) == 2


def test_recovery_permits_existing_fault_but_preserves_other_interlocks(
    monkeypatch,
) -> None:
    """
    Driver recovery never removes a motion-source, emergency or charger blocker.
    """
    safety = RecoverySafety(Mock())
    reasons = [
        "Wheel driver fault: right",
        "Teleop active or unknown",
        "Emergency active, unknown or stale",
    ]
    monkeypatch.setattr(
        "experiments.robocat_snm.native_preparation.PreparationSafety.blockers",
        Mock(return_value=reasons),
    )
    assert safety.blockers(require_wheels=True) == reasons[1:]
