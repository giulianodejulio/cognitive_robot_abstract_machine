"""Native read-only recovery evidence and preservation of navigation tolerances."""

# %% imports
from __future__ import annotations

import argparse
import math
import socket
import time
from contextlib import ExitStack
from dataclasses import dataclass, field
from enum import Enum

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from experiments.robocat_snm.native_preparation import PreparationSafety


# %% recovery evidence
class RecoveryCheck(str, Enum):
    """Evidence required at each operator-controlled recovery step."""

    STOPPED = "stopped"
    PRESSED = "pressed"
    REARMED = "rearmed"
    DRIVERS = "drivers"
    SETTINGS = "settings"
    RESTORE = "restore"


@dataclass
class DriverEvidence:
    """One actuator's independently timestamped diagnostic evidence."""

    values: dict[str, str]
    """Fields published by the native actuator diagnostic."""
    level: int
    """Diagnostic severity."""
    received_at: float
    """Monotonic receipt time."""

    def operational(self, now: float, maximum_age: float) -> bool:
        """Require valid communication, enabled motor and no reported driver fault."""
        return (
            now - self.received_at <= maximum_age
            and self.level == DiagnosticStatus.OK
            and self.values.get("Driver state") == "MotorOperational"
            and self.values.get("Motor status") == "Enabled"
            and self.values.get("Motor errors") == "No errors"
            and self.values.get("ESM") == "OP"
        )


@dataclass
class RecoverySafety(PreparationSafety):
    """Allow existing motor faults during recovery while retaining other interlocks."""

    drivers: dict[str, DriverEvidence] = field(default_factory=dict, init=False)
    """Latest wheel driver evidence, including faults."""
    wheel_names: tuple[str, str] = ("wheel_left_actuator", "wheel_right_actuator")
    """Both drive actuators required for recovery."""

    def __post_init__(self) -> None:
        super().__post_init__()
        self.subscriptions.append(
            self.node.create_subscription(
                DiagnosticArray,
                "/diagnostics_agg",
                self.receive_diagnostics,
                qos_profile_sensor_data,
            )
        )

    def receive_diagnostics(self, message: DiagnosticArray) -> None:
        """Retain faults for final verification even when recovery permits them."""
        super().receive_diagnostics(message)
        for status in message.status:
            for wheel in self.wheel_names:
                if "Actuators/" + wheel in status.name:
                    self.drivers[wheel] = DriverEvidence(
                        {item.key: item.value for item in status.values},
                        status.level,
                        time.monotonic(),
                    )

    def blockers(self, require_wheels: bool) -> list[str]:
        """Existing wheel faults do not prevent recovery; every other guard remains."""
        return [
            reason
            for reason in super().blockers(require_wheels)
            if not reason.startswith("Wheel driver fault: ")
        ]

    def drivers_ready(self) -> bool:
        """Both wheel drivers must independently report current operational states."""
        now = time.monotonic()
        return all(
            wheel in self.drivers
            and self.drivers[wheel].operational(now, self.maximum_age)
            for wheel in self.wheel_names
        )

    def verify(self, check: RecoveryCheck, timeout: float = 10.0) -> bool:
        """Wait a bounded interval for the requested measured recovery condition."""
        if check in (RecoveryCheck.STOPPED, RecoveryCheck.REARMED):
            return self.wait(
                require_wheels=check is RecoveryCheck.STOPPED, timeout=timeout
            )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if check is RecoveryCheck.PRESSED:
                if (
                    self.emergency is True
                    and time.monotonic() - self.emergency_at <= self.maximum_age
                ):
                    return True
            elif check is RecoveryCheck.DRIVERS and self.drivers_ready():
                return True
        print(
            f"RECOVERY CHECK FAILED: {check.value}; drivers={self.drivers}", flush=True
        )
        return False


# %% navigation settings
@dataclass
class RecoverySettings:
    """Preserve the two native goal-checker tolerances across recovery."""

    node: Node
    """Native ROS node owning parameter requests."""
    names: tuple[str, str] = (
        "general_goal_checker.xy_goal_tolerance",
        "general_goal_checker.yaw_goal_tolerance",
    )
    """Position and orientation tolerances, in metres and radians."""
    timeout: float = 5.0
    """Maximum wait for each parameter service operation."""

    def read(self) -> list[float] | None:
        """Read finite positive double tolerances, refusing unavailable settings."""
        client = self.node.create_client(
            GetParameters, "/controller_server/get_parameters"
        )
        if not client.wait_for_service(timeout_sec=self.timeout):
            return None
        future = client.call_async(GetParameters.Request(names=list(self.names)))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.timeout)
        if not future.done() or future.exception() is not None:
            return None
        values = future.result().values
        if len(values) != len(self.names) or any(
            item.type != ParameterType.PARAMETER_DOUBLE
            or not math.isfinite(item.double_value)
            or item.double_value <= 0
            for item in values
        ):
            return None
        return [item.double_value for item in values]

    def restore(self, position: float, orientation: float) -> bool:
        """Restore both saved tolerances atomically and verify actual readback."""
        values = [position, orientation]
        if any(not math.isfinite(value) or value <= 0 for value in values):
            return False
        if self.read() == values:
            return True
        client = self.node.create_client(
            SetParametersAtomically, "/controller_server/set_parameters_atomically"
        )
        if not client.wait_for_service(timeout_sec=self.timeout):
            return False
        request = SetParametersAtomically.Request(
            parameters=[
                Parameter(
                    name=name,
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_DOUBLE, double_value=value
                    ),
                )
                for name, value in zip(self.names, values)
            ]
        )
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.timeout)
        return (
            future.done()
            and future.exception() is None
            and future.result().result.successful
            and self.read() == values
        )


# %% native executable
def main() -> None:
    """Run a single explicit recovery check without publishing motion commands."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("check", choices=[item.value for item in RecoveryCheck])
    parser.add_argument("--position", type=float)
    parser.add_argument("--orientation", type=float)
    options = parser.parse_args()
    if not socket.gethostname().startswith("robocat"):
        parser.error("Recovery requires the native Robocat host")
    check = RecoveryCheck(options.check)
    if check is RecoveryCheck.RESTORE and (
        options.position is None or options.orientation is None
    ):
        parser.error("Restore requires both saved tolerances")
    with ExitStack() as cleanup:
        rclpy.init()
        cleanup.callback(rclpy.shutdown)
        node = rclpy.create_node("robocat_motor_recovery")
        cleanup.callback(node.destroy_node)
        if check is RecoveryCheck.SETTINGS:
            values = RecoverySettings(node).read()
            if values is None:
                raise SystemExit("Cannot preserve Nav2 tolerances; recovery stopped")
            print(
                "NAV2_SETTINGS: " + " ".join(str(value) for value in values), flush=True
            )
            return
        if check is RecoveryCheck.RESTORE:
            if not RecoverySettings(node).restore(
                options.position, options.orientation
            ):
                raise SystemExit("Cannot restore and verify Nav2 tolerances")
            return
        if not RecoverySafety(node, navigation_idle_confirmed=True).verify(check):
            raise SystemExit(1)
        print(f"RECOVERY CHECK OK: {check.value}", flush=True)


if __name__ == "__main__":
    main()
