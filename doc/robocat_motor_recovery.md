# Robocat wheel-driver recovery, 2026-11-09

## Observed fault

Both MC23 wheel drivers reported `MotorHighDiscrepancyError`, driver state Error,
and motor status Disabled. Giskard commands reached the native base controller,
but wheel feedback stayed near zero. Charging from about 10% to 39% did not clear
the fault. This does not establish that low battery caused the original fault.

The recorded first wheel-driver errors were at ROS timestamps
1794238062.626581826 (right) and 1794238098.166544725 (left).

## Recovery performed

1. Stopped the dedicated Robocat waypoint client and Giskard server with SIGINT;
   checked zero base command output.
2. The operator pressed the physical emergency button. PB27 confirmed motor power
   and safety relay off; the ROS motion controllers became inactive.
3. The operator released the emergency button and pressed physical Start/On-Off.
   PB27 confirmed emergency false, no rearm pending and motor power on. The
   discrepancy error disappeared, but driver state remained Error with EtherCAT
   ESM UNKNOWN. Physical rearm alone did not finish recovery.
4. Restarted only PAL `controller_manager`. Both wheel drivers then reported
   `MotorOperational`, ESM OP, motors Enabled and no motor or gate-driver errors.
5. Restarted the finished PAL modules `emergency_stop_controller` and
   `default_controllers` to restore the controllers unloaded by step 4. The module
   command printed an error stopping the already-finished modules, then started
   them; controller loading completed a few seconds later.
6. The newly loaded base controller had all three multipliers reset to 1.0.
   Deactivated and cleaned up the base controller, applied 2.0/2.0/1.22,
   configured and activated it again.

Commands below require native Humble/PAL, domain 102 and CycloneDDS. They change
hardware/controller state: use only after stopping goal sources, verifying zero
commands and confirming the operator and free area. Never force-start modules or
publish guessed actuator engagement values.

```bash
pal module restart controller_manager
pal module restart emergency_stop_controller default_controllers
ros2 control list_controllers

ros2 control set_controller_state mobile_base_controller inactive
ros2 control set_controller_state mobile_base_controller unconfigured
ros2 param set /mobile_base_controller left_wheel_radius_multiplier 2.0
ros2 param set /mobile_base_controller right_wheel_radius_multiplier 2.0
ros2 param set /mobile_base_controller wheel_separation_multiplier 1.22
ros2 control set_controller_state mobile_base_controller inactive
ros2 control set_controller_state mobile_base_controller active
```

Execute transitions sequentially and stop if any step fails. Reading parameter
values alone is insufficient: the controller must configure again to apply them.
These multipliers are runtime settings, not a persistent modification of PAL's
factory configuration.

## Validation and limits

Final checks cover actuator diagnostics, controller state, runtime calibration,
zero commanded velocity and stationary wheel feedback. No movement goal was sent
during recovery. Physical navigation remains to be checked by the operator; the
underlying cause of the discrepancy fault is not established. Restart the stopped
Giskard server and waypoint client before the next operator-run trial.

Evidence is saved outside temporary directories at
`/home/gdj/workspaces/robocat_experiments/motor_recovery_20261109/`.

## Guided host command

With `robocat-infra` already running, use a **laptop host terminal**:

```bash
cd /home/gdj/workspaces/tiago_ws/cognitive_robot_abstract_machine
bash scripts/robocat-recover-motors
```

The command does not send motion goals. Cancel/finish all navigation requests
first. Keep the robot stationary, teleop off, charger disconnected and the area
clear, with an operator at the emergency button. Type `STOPPED` to confirm these
conditions. Existing observed active base actions, recent nonzero commands,
interlocks or missing stationary feedback block recovery. This confirmation
covers native action status that has never published; it does not override an
observed active goal. The container's SNM request lock stays held during recovery.

The script preserves the running Giskard linear/angular limits and the current
Nav2 position/orientation tolerances. It stops only the infrastructure-owned
Giskard server and leaves DDS running. It then prompts:

1. Press the physical emergency mushroom; type `PRESSED`. ROS must confirm it.
2. Release the mushroom and press/release physical Start/On-Off for about one
   second. Wait for a steady green light, then type `OK`. Software interlocks
   must independently confirm rearm.

If both wheel drivers are operational, a PAL restart is unnecessary. Otherwise
it restarts `controller_manager` once, verifies actual driver recovery and starts
`emergency_stop_controller` and `default_controllers`. It reapplies and verifies
2.0 / 2.0 / 1.22, restores the saved Nav2 tolerances, checks stopped wheels and
operational drivers, then respawns only the Giskard window with its saved limits.
Success is printed as `RECOVERY OK`. No navigation is resumed automatically.

After a controller restart, **check scan/map alignment and the robot pose in
RViz**: wheel odometry may have reset. Set a new initial pose only if alignment
is incorrect. The script does not change localization or assert that AMCL is
correct.

Failures and interruption stop the procedure; there is no repeated automatic
reset and no forced process termination. Giskard can remain stopped. Inspect the
reported `/tmp/robocat-motor-recovery.*/recovery.log` and the failed step before
restoring the server or attempting another trial. A new guided invocation
requires an existing owned server so that its velocity limits can be preserved;
resolve an incomplete recovery before restarting the command. Native diagnostic
checks use temporary files on the robot and remove the extracted source on exit.

The new procedure has automated tests with substituted SSH/Docker/Byobu commands;
it has not yet been validated end-to-end on the real robot.
