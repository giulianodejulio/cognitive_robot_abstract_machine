# Cabin frame calibration — 18 November 2026

The current-floor calibration is saved in
`config/robocat_entry/cabin_labora_floor.yaml`. It is associated with Eulero
`labora_demo/labora_floor/elevator_v2_map` and the exact published grid fingerprint
`14cae3e62b38495028d71f384a7bea50e41025b26b866dfbcf6107edb8961fda`.
The Eulero database and existing waypoints have not been changed.

## Physical definition

`elevator_cabin_labora_floor` has its origin at the back-left wall intersection,
viewed from the open doorway toward the back wall, projected onto the floor.
X points toward the doorway, perpendicular to the back wall. Y points along the
back wall into the cabin (right when viewed from the doorway). Z points upward.
This is a right-handed frame. Height is assigned to the floor; a horizontal
lidar scan does not measure the floor height.

The floor-qualified child name avoids confusing this calibration with the one
that will be acquired on another floor. This is a floor-specific pose of the
cabin, not a continuously tracked frame while the elevator travels.

## Acquisition and validation

The operator confirmed the robot stopped on the existing inside tape reference,
facing the back wall, with the cabin empty and door open. Nineteen scan/TF pairs
were collected before the operator moved the robot outside. The acquisition
checked fresh scan/wheel feedback, measured rest, and available localized TF.
The actual lidar extrinsic was included.

The existing `CabinWallEstimator` fitted the back, left and right walls. All
nineteen observations produced valid back/left fits. Median perpendicular
base-to-wall distances were 0.932 m (back), 0.518 m (left), and 0.565 m (right).
Back/left fit RMS values were 5.4/7.1 mm. The back/left angle ranged from
89.49 to 90.05 degrees. The corner's map-coordinate spread was 9.7 mm in X and
4.4 mm in Y; yaw spread was 0.82 degrees. These repeatability values do not
include AMCL error or guarantee absolute calibration accuracy.

The saved transform passed right-handedness and map/cabin/base round-trip checks.
A scan-to-occupied-cell check gave median/p90 distances of 2.4/5.0 cm on the
5 cm occupancy grid. It was not an independent survey of the cabin.

The transform in map is x=20.0757446 m, y=-33.7921663 m, yaw=38.0813 degrees.
The captured base pose in cabin coordinates is x=0.9316363 m,
y=0.5146729 m, yaw=-179.3434 degrees. This stores the manually positioned tape
reference independently of the existing nominal `elevator_inside` waypoint.
The captured map pose differs from that existing waypoint by approximately
16.4 cm; the calibration does not attribute that difference to physical tape
placement versus localization error, and does not overwrite the waypoint.

Raw capture, fit results and active Eulero context:
`/home/gdj/workspaces/robocat_experiments/cabin_frame_20261118/`.

## Current RViz preview

The stock native `tf2_ros/static_transform_publisher` runs in Byobu window
`robocat-elevator:cabin-frame`, without publishing velocity or navigation goals.
Its transform was independently received through the native TF buffer.
In RViz enable `TF -> Frames -> elevator_cabin_labora_floor`.

This preview is only valid on the calibrated current map, with the cabin at that
floor. Stop the preview before a floor/map change:

```bash
byobu-tmux kill-window -t robocat-elevator:cabin-frame
```

Static TF can remain cached in existing listeners; do not use the old frame for
another floor. Future map-switch logic must select the matching calibration
by floor and map fingerprint. The preview is not integrated into automatic
infrastructure startup or map switching.

While the same map and floor are active, the stock publisher can be started
in a native robot terminal after sourcing ROS Humble and PAL Alum on domain 102.
The values below reproduce `config/robocat_entry/cabin_labora_floor.yaml`:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 20.075744582908843 --y -33.79216625757321 --z 0 \
  --qx 0 --qy 0 --qz 0.3262389338455842 --qw 0.9452873415228285 \
  --frame-id map --child-frame-id elevator_cabin_labora_floor
```

Stop that terminal with Ctrl+C before changing the map. This command only
publishes geometry; it neither registers an Eulero anchor nor moves the robot.
