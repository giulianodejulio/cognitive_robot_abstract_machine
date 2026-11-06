# Physical displacement comparison — 6 November 2026

Operator-authorized real Robocat trial, a single Cartesian straight goal
0.50 m relative to the initial base pose, constant heading, wheel feedback.
Command limits: 0.05 m/s, 0.1 rad/s. Tolerances: 0.02 m and 5 degrees.
The operator measured the same chassis point against floor marks and
reported **1.00 m actual displacement**. Measurement uncertainty was not
quantified; do not interpret the reported metre as sub-centimetre ground truth.

The initial preflight detected a wall about 1.24 m from the base, and no
goal was sent then. The operator identified the wall, reiterated the
request and explicitly took responsibility for intervention using teleop.
One goal was subsequently sent through the existing guarded persistent client.
No additional obstacle-distance cancellation observer was added. The goal
returned SUCCESS; stable wheel rest was confirmed. No second goal was sent.

Stationary windows in original bag receive times: 1793975305–5308 before,
1793975323–5326 after. Wheel samples 150/150, DLO samples 30/22.

| Source | Displacement |
| --- | ---: |
| Physical operator measurement | 1.00 m |
| Wheel odometry | 0.494868 m |
| Laser odometry (DLO) | 0.981978 m |

Physical/wheel ratio approximately 2.02074. DLO differs from the physical
measurement by about −1.8 cm. DLO lateral displacement approximately
0.98 cm; wheel heading change +0.0844 degrees, DLO +0.5205 degrees.
Wheel joint increments: left 4.946841 rad, right 4.950525 rad.
With radius 0.10 m, their average predicts the wheel odometry displacement.

This trial identifies the wheel-based metric estimate as under-reporting
relative to a physical measurement. It overturns an interpretation that DLO
must be erroneous merely because it reports twice the wheel distance.
It does not establish the exact faulty encoder, reduction, geometry or
hardware-interface parameter. Rotational accuracy is still unvalidated.
Do not apply a blanket x2 correction to translation and rotation.

Client result elapsed 11.798 s; request-to-wheel-feedback onset 1.593 s.
The rosbag was closed with SIGINT and checked with `ros2 bag info`:
26.4 MiB, 759 scans, 658 DLO messages, 3795 wheel messages, 3795 joint states,
201 commanded velocities, TF/static TF, URDF, interlocks, action feedback/status.
Logs and `physical-comparison.json` are retained alongside it.

The persistent server/client remain running with the reduced limits above.
No hardware configuration, encoder calibration or Giskard source was modified.
