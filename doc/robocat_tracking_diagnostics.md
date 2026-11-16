# Robocat tracker diagnostics

Restart the Giskard server with the usual command. The bringup now advertises
`/robocat_waypoint/tracking` in the isolated Jazzy domain 103. No goal or hardware
controller is started by enabling diagnostics. Add `--no-tracking` to the Python
bringup command to disable the publisher and observer entirely.

Select the topic in PlotJuggler's ROS2 streaming subscriber **before** starting
an operator-run trial. Use **Best Effort / Volatile** QoS. Samples appear only
while `CartesianPositionTrajectory` with heading tracking is running; initial
alignment, braking, final turn and idle do not emit trajectory samples. This is
expected and does not indicate a feedback outage.

The message type is the existing `std_msgs/msg/Float64MultiArray`. All signals
are numeric and directly plottable without installing or building another ROS
message package. `layout.dim[0].label` contains the comma-separated channel names.
PlotJuggler may show the channels as indexed `data` entries; use the table below
to select or rename them in your saved layout.

| data index | Name | Units / meaning |
|---|---|---|
| 0 | stamp_sec | Server ROS timestamp, whole seconds |
| 1 | stamp_nanosec | Server ROS timestamp, nanoseconds within that second |
| 2 | sequence | Local counter distinguishing sampled trajectory tasks |
| 3 | desired_yaw | Heading requested towards the current lookahead target, rad |
| 4 | actual_yaw | Synchronized feedback/model yaw, rad |
| 5 | heading_error | Desired minus feedback yaw, wrapped to [-pi, pi], rad |
| 6 | path_index | Native tracker index |
| 7 | cross_track_error | Signed lateral error relative to the current segment, m; left positive |
| 8 | distance_to_goal | Euclidean endpoint distance, m |
| 9 | remaining_distance | Native path completion/progress metric, m |
| 10 | target_x | Lookahead target x, m |
| 11 | target_y | Lookahead target y, m |
| 12 | base_x | Synchronized feedback/model x, m |
| 13 | base_y | Synchronized feedback/model y, m |

Coordinates and yaw use the trajectory reference frame, currently `odom`.
Feedback/model values are estimates from the server's selected odometry, not
physical ground truth. At arrival, `desired_yaw` follows the same hold-current-yaw
branch as the controller rather than reporting an undefined direction at a zero
target vector. Yaw signals can wrap at pi; use `heading_error` to distinguish a
wrap from a real full-turn error. Cross-track error uses the native current
segment (the last segment at the final index), not a separate global projection.

The observer runs synchronously after the native index and target update and
before the QP computes/applies the next command. This avoids confusing predicted
post-command state with measured input state. Timestamps are publication times
in that control cycle, not the original odometry header timestamp. PlotJuggler
uses reception time for this headerless message, as it does for `/cmd_vel`.

Publication is capped at 10 Hz using monotonic elapsed time. Without subscribers,
the additional forward-kinematics and diagnostic geometry calculations are
skipped. QoS is best effort, volatile, depth one. No worker, timer, terminal
printing, full debug plotting or bag recorder is started. Actual loop overhead
has not yet been benchmarked on a physical trial.

To retain a trial, in another container terminal with the same ROS environment:

```bash
ros2 bag record -o /workspace/cram/tmp/robocat-tracking-$(date +%Y%m%d-%H%M%S) \
  /robocat_waypoint/tracking /cmd_vel /mobile_base_controller/odom
```

Start recording before your motion request and stop it with Ctrl+C afterwards.
The diagnostic topic is local to the container domain; no DDS router change is
needed for PlotJuggler in that domain. No physical motion was executed during
implementation. Tests cover signed/wrapped errors, arrival hold, no-subscriber
behavior, timestamps/rate limiting and the native same-cycle observer hook.
