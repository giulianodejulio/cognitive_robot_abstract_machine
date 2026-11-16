"""Read or explicitly reconfigure the native Robocat wheel controller."""

# %% imports
from __future__ import annotations

import argparse
import math
import socket
import subprocess
import time
from contextlib import ExitStack
from dataclasses import dataclass, field
from enum import Enum
from functools import partial

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from controller_manager_msgs.srv import ListControllers
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.subscription import Subscription
from std_msgs.msg import Bool

from experiments.robocat_snm.native_navigation import WheelCalibration


# %% preparation interlocks
@dataclass
class PreparationSafety:
    """Observe native stop conditions without publishing any velocity or goal."""

    node: Node
    """Native Humble node."""
    navigation_idle_confirmed: bool = False
    """Operator explicitly confirms that no native navigation goal is active."""
    emergency: bool | None = field(default=None, init=False)
    """Last physical emergency state."""
    emergency_at: float = field(default=0, init=False)
    """Monotonic emergency reception time."""
    charging: bool | None = field(default=None, init=False)
    """Last charger state."""
    charging_at: float = field(default=0, init=False)
    """Monotonic charger reception time."""
    joystick: bool | None = field(default=None, init=False)
    """Latched teleoperation priority."""
    wheel_at: float = field(default=0, init=False)
    """Last finite wheel velocity reception time."""
    stopped_since: float | None = field(default=None, init=False)
    """Start of the current stationary interval."""
    active: dict[str, bool] = field(default_factory=dict, init=False)
    """Observed active/canceling states of native base actions."""
    commands: dict[str, float] = field(default_factory=dict, init=False)
    """Last nonzero command time for each observed base command topic."""
    motor_faults: dict[str, bool] = field(default_factory=dict, init=False)
    """Current reported wheel actuator faults."""
    mux: dict[str, str] = field(default_factory=dict, init=False)
    """Native mux priority and active lock diagnostics."""
    mux_at: float = field(default=0, init=False)
    """Monotonic receipt time of native mux diagnostics."""
    subscriptions: list[Subscription] = field(default_factory=list, init=False)
    """Subscriptions retained throughout controller preparation."""
    maximum_age: float = 2.0
    """Maximum periodic feedback age in seconds."""
    stationary_seconds: float = 1.0
    """Measured stop and command-quiet interval in seconds."""

    def __post_init__(self) -> None:
        self.subscriptions = [
            self.node.create_subscription(
                Bool,
                "/power/is_emergency",
                self.receive_emergency,
                qos_profile_sensor_data,
            ),
            self.node.create_subscription(
                Bool,
                "/power/is_charging",
                self.receive_charging,
                qos_profile_sensor_data,
            ),
            self.node.create_subscription(
                Bool,
                "/joy_priority",
                self.receive_joystick,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            ),
            self.node.create_subscription(
                Odometry,
                "/mobile_base_controller/odom",
                self.receive_wheels,
                qos_profile_sensor_data,
            ),
            self.node.create_subscription(
                DiagnosticArray,
                "/diagnostics",
                self.receive_diagnostics,
                qos_profile_sensor_data,
            ),
        ]
        for action in (
            "/navigate_to_pose",
            "/navigate_through_poses",
            "/follow_waypoints",
            "/follow_path",
            "/backup",
            "/drive_on_heading",
            "/spin",
        ):
            self.subscriptions.append(
                self.node.create_subscription(
                    GoalStatusArray,
                    action + "/_action/status",
                    partial(self.receive_status, action),
                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                )
            )
        for topic in (
            "/cmd_vel",
            "/snm_vel",
            "/input_joy/cmd_vel",
            "/mobile_base_controller/cmd_vel_unstamped",
        ):
            self.subscriptions.append(
                self.node.create_subscription(
                    Twist,
                    topic,
                    partial(self.receive_command, topic),
                    qos_profile_sensor_data,
                )
            )
        self.subscriptions.append(
            self.node.create_subscription(
                TwistStamped,
                "/mobile_base_controller/cmd_vel",
                self.receive_stamped_command,
                qos_profile_sensor_data,
            )
        )

    def receive_emergency(self, message: Bool) -> None:
        """Retain emergency state and receipt time."""
        self.emergency, self.emergency_at = message.data, time.monotonic()

    def receive_charging(self, message: Bool) -> None:
        """Retain charger state and receipt time."""
        self.charging, self.charging_at = message.data, time.monotonic()

    def receive_joystick(self, message: Bool) -> None:
        """Retain the latched joystick interlock."""
        self.joystick = message.data

    def receive_wheels(self, message: Odometry) -> None:
        """Accumulate only finite, current wheel samples below stop thresholds."""
        now = time.monotonic()
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        clock = self.node.get_clock().now().nanoseconds / 1e9
        values = (message.twist.twist.linear.x, message.twist.twist.angular.z)
        valid = (
            all(math.isfinite(value) for value in values)
            and abs(clock - stamp) <= self.maximum_age
        )
        if now - self.wheel_at > self.maximum_age:
            self.stopped_since = None
        if valid:
            self.wheel_at = now
        if valid and abs(values[0]) < 0.005 and abs(values[1]) < 0.01:
            if self.stopped_since is None:
                self.stopped_since = now
        else:
            self.stopped_since = None

    def receive_status(self, action: str, message: GoalStatusArray) -> None:
        """Retain base-action ownership without canceling other clients."""
        self.active[action] = any(
            item.status
            in (
                GoalStatus.STATUS_ACCEPTED,
                GoalStatus.STATUS_EXECUTING,
                GoalStatus.STATUS_CANCELING,
            )
            for item in message.status_list
        )

    def receive_command(self, topic: str, message: Twist) -> None:
        """Record nonzero or invalid base commands as incompatible with preparation."""
        values = (
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )
        if any(not math.isfinite(value) or abs(value) > 0.001 for value in values):
            self.commands[topic] = time.monotonic()

    def receive_stamped_command(self, message: TwistStamped) -> None:
        """Apply the same velocity checks to the mux's stamped output."""
        self.receive_command("/mobile_base_controller/cmd_vel", message.twist)

    def receive_diagnostics(self, message: DiagnosticArray) -> None:
        """Retain wheel driver faults independently of controller lifecycle state."""
        for status in message.status:
            if "twist_mux" in status.name:
                self.mux = {item.key: item.value for item in status.values}
                self.mux_at = time.monotonic()
            if "wheel" in status.name.lower() and "actuator" in status.name.lower():
                self.motor_faults[status.name] = status.level in (
                    DiagnosticStatus.ERROR,
                    DiagnosticStatus.STALE,
                )

    def blockers(self, require_wheels: bool) -> list[str]:
        """Return unmet conditions without releasing any interlock."""
        now = time.monotonic()
        reasons = []
        if self.emergency is not False or now - self.emergency_at > self.maximum_age:
            reasons.append("Emergency active, unknown or stale")
        if self.charging is not False or now - self.charging_at > self.maximum_age:
            reasons.append("Charger connected, unknown or stale")
        if self.joystick is not False:
            reasons.append("Teleop active or unknown")
        if (
            now - self.mux_at > self.maximum_age
            or self.mux.get("current priority") != "0"
        ):
            reasons.append("Mux unavailable, stale or owned by a command source")
        reasons.extend(
            f"Active mux lock: {key}"
            for key, value in self.mux.items()
            if key.startswith("lock ") and value.lstrip().startswith("locked")
        )
        if "/navigate_to_pose" not in self.active and not (
            self.navigation_idle_confirmed
            and self.node.count_publishers("/navigate_to_pose/_action/status") > 0
        ):
            reasons.append("Native navigation status unavailable")
        reasons.extend(
            f"Active base action: {action}"
            for action, active in self.active.items()
            if active
        )
        reasons.extend(
            f"Nonzero/invalid command: {topic}"
            for topic, received in self.commands.items()
            if now - received < self.stationary_seconds
        )
        reasons.extend(
            f"Wheel driver fault: {actuator}"
            for actuator, fault in self.motor_faults.items()
            if fault
        )
        if require_wheels and (
            now - self.wheel_at > self.maximum_age
            or self.stopped_since is None
            or now - self.stopped_since < self.stationary_seconds
        ):
            reasons.append("Fresh stationary wheel feedback not confirmed")
        return reasons

    def wait(self, require_wheels: bool, timeout: float = 5.0) -> bool:
        """Refresh observations before approving a bounded preparation step."""
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            reasons = self.blockers(require_wheels)
            if not reasons and time.monotonic() - start >= 0.25:
                return True
        print(
            "PREPARATION BLOCKED: " + "; ".join(self.blockers(require_wheels)),
            flush=True,
        )
        return False


# %% controller transitions
class ControllerState(str, Enum):
    """Native controller lifecycle states required by the preparation sequence."""

    ACTIVE = "active"
    INACTIVE = "inactive"
    UNCONFIGURED = "unconfigured"


@dataclass
class ControllerConfiguration:
    """Observed base controller lifecycle and readable parameter values."""

    state: ControllerState
    """Observed base controller state."""
    values: list[ParameterValue]
    """Read-back values in calibration setting order."""


@dataclass
class ControllerPreparation:
    """Apply validated multipliers only through an observed cleanup/configure cycle."""

    node: Node
    """Native node owning configuration requests."""
    safety: PreparationSafety
    """Current stop and ownership evidence."""
    calibration: WheelCalibration = field(init=False)
    """Single source of validated wheel parameter names and values."""
    command_timeout: float = 15.0
    """Maximum duration of one controller CLI transition, in seconds."""

    def __post_init__(self) -> None:
        self.calibration = WheelCalibration(self.node)

    def read_configuration(
        self, timeout: float = 10.0
    ) -> ControllerConfiguration | None:
        """Wait for controller discovery and parameters within a bounded startup interval."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = self.controller_state()
            values = self.calibration.read()
            if (
                state is not None
                and values is not None
                and len(values) == len(self.calibration.settings)
            ):
                return ControllerConfiguration(state, values)
            time.sleep(0.1)
        return None

    def controller_state(self) -> ControllerState | None:
        """Read actual controller state instead of trusting CLI output alone."""
        client = self.node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        if not client.wait_for_service(timeout_sec=3):
            return None
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3)
        if not future.done() or future.exception() is not None:
            return None
        for controller in future.result().controller:
            if controller.name == "mobile_base_controller" and controller.state in {
                state.value for state in ControllerState
            }:
                return ControllerState(controller.state)
        return None

    def transition(self, target: ControllerState) -> bool:
        """Check interlocks and verify one native controller state transition."""
        if not self.safety.wait(require_wheels=False):
            return False
        if self.controller_state() is target:
            return True
        print(f"CONTROLLER: request {target.value}", flush=True)
        result = subprocess.run(
            [
                "ros2",
                "control",
                "set_controller_state",
                "mobile_base_controller",
                target.value,
            ],
            capture_output=True,
            text=True,
            timeout=self.command_timeout,
        )
        print(result.stdout, end="", flush=True)
        print(result.stderr, end="", flush=True)
        return result.returncode == 0 and self.controller_state() is target

    def write_values(self) -> bool:
        """Set all three multipliers atomically while the controller is unconfigured."""
        if (
            self.controller_state() is not ControllerState.UNCONFIGURED
            or not self.safety.wait(require_wheels=False)
        ):
            return False
        client = self.node.create_client(
            SetParametersAtomically, "/mobile_base_controller/set_parameters_atomically"
        )
        if not client.wait_for_service(timeout_sec=3):
            return False
        request = SetParametersAtomically.Request(
            parameters=[
                Parameter(
                    name=item.parameter,
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_DOUBLE,
                        double_value=item.multiplier,
                    ),
                )
                for item in self.calibration.settings
            ]
        )
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3)
        if not future.done() or future.exception() is not None:
            return False
        result = future.result().result
        print(f"PARAMETERS: success={result.successful}; {result.reason}", flush=True)
        return result.successful

    def apply(self) -> bool:
        """Stop at the first failed step without attempting recovery or motion."""
        if not self.safety.wait(require_wheels=True):
            return False
        for state in (ControllerState.INACTIVE, ControllerState.UNCONFIGURED):
            if not self.transition(state):
                return False
        if not self.write_values():
            return False
        for state in (ControllerState.INACTIVE, ControllerState.ACTIVE):
            if not self.transition(state):
                return False
        return (
            self.calibration.verify()
            and self.controller_state() is ControllerState.ACTIVE
            and self.safety.wait(require_wheels=True)
        )


# %% native executable
def main() -> None:
    """Read by default; change native controller state only with explicit apply."""
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--check", action="store_true")
    modes.add_argument("--apply", action="store_true")
    parser.add_argument("--confirm-navigation-idle", action="store_true")
    options = parser.parse_args()
    if not socket.gethostname().startswith("robocat"):
        parser.error(
            "Native preparation requires a Robocat host; robot identity did not match"
        )
    with ExitStack() as cleanup:
        rclpy.init()
        cleanup.callback(rclpy.shutdown)
        node = rclpy.create_node("robocat_controller_preparation")
        cleanup.callback(node.destroy_node)
        preparation = ControllerPreparation(
            node,
            PreparationSafety(
                node, navigation_idle_confirmed=options.confirm_navigation_idle
            ),
        )
        configuration = preparation.read_configuration()
        if configuration is None:
            print(
                "Controller parameters or lifecycle state unavailable after startup wait",
                flush=True,
            )
            raise SystemExit(1)
        state, values = configuration.state, configuration.values
        print(
            f"CONTROLLER STATE: {state.value if state is not None else 'unavailable'}",
            flush=True,
        )
        if (
            values is None
            or len(values) != len(preparation.calibration.settings)
            or state is None
        ):
            print("Controller parameters or lifecycle state unavailable", flush=True)
            raise SystemExit(1)
        for setting, value in zip(preparation.calibration.settings, values):
            print(
                f"{setting.parameter}: current={value.double_value:g}; required={setting.multiplier:g}",
                flush=True,
            )
        if not options.apply:
            print("READ ONLY: no controller state or parameter was changed", flush=True)
            return
        if not preparation.apply():
            print(
                "PREPARATION FAILED: no automatic recovery; inspect controller state before retrying",
                flush=True,
            )
            raise SystemExit(1)
        print(
            "PREPARATION VERIFIED: configured and active with 2.0 / 2.0 / 1.22; no goal sent",
            flush=True,
        )


if __name__ == "__main__":
    main()
