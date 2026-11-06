# Robocat provisional elevator mapping — 6 November 2026

The operator placed the stationary robot at the desired final map origin.
`origin-reference.json` records both odometries and the native odom/base TF.
The native TF matches DLO, not wheel odometry. The previous wheel/laser
metric discrepancy remains unresolved; this map is provisional.

SLAM Toolbox was already installed on Robocat. Existing AMCL/map-server
processes refer to an old map. PAL module-management service requests failed
to respond; no PAL stack restart or hardware-controller change was made.

An independent native Humble SLAM process was started with the saved
`robocat_elevator_slam.yaml`, namespace `/elevator_mapping`, map topic
`/elevator_mapping/map`, map frame `elevator_mapping_map`. TF publication
is disabled to avoid adding a competing parent transform for `odom`.
It consumes `/scan` and the native DLO odom/base TF. It sends no base commands.

The first grid was verified nonempty: 307 x 349 cells, 0.05 m resolution.
Its raster origin was approximately (22.72004, 11.66713), not the selected
robot origin. The final saved map must be rigidly reframed using the saved
initial robot pose; do not claim its coordinates are already zeroed.
Do not reuse the earlier wheel-anchored doorway calibration in this DLO map.
Doorway and staging frames will be measured in the new map after acquisition.

The initial SLAM scan queue of one produced frequent TF filter drops. Queue
size was set to 20 and only this SLAM process was restarted. Some timestamp
filter drops persist (approximately one logged event every 6–7 seconds while
stationary). A first short manual movement must be checked for actual growth
of the map/pose graph before treating the setup as ready for a full mapping
tour. Native SLAM CPU was approximately 5.5%, RSS 64 MiB.

## Active recording locations

**Session completed:** the locations below describe the acquisition. Both
recorders and the dedicated SLAM process have now been stopped cleanly.
Persistent copies are in `inputs/` and `slam-segment-01/02/03/` here.

- Main laptop container `aicor-cram-demo`:
  `/tmp/robocat-elevator-mapping-20261106/inputs` (MCAP): scan, native TF,
  static TF, robot description, joints, both odometries, joystick, emergency,
  commanded velocity. Recorder PID file:
  `/tmp/robocat-elevator-mapping-recorder-20261106.pid`.
- Robot: `/tmp/robocat-elevator-map-output-20261106` (SQLite): SLAM map,
  pose and graph visualization. Recorder PID file:
  `/tmp/robocat-elevator-map-output-recorder.pid`.
- Robot SLAM PID and log:
  `/tmp/robocat-elevator-slam-20261106.pid` and corresponding `.log`.

These bags remain open during mapping. At completion, close recorders with
SIGINT, verify metadata/topic counts, copy both bags and the SLAM graph/map
to this persistent laptop directory. Temporary paths alone are not a backup.
RViz rendering is off. All future visualization labels should be in English.

No autonomous movement was sent during setup. The operator is to use teleop
for acquisition; a stationary final pose and door state must be confirmed
before calibrating the doorway or sending any later navigation goal.

Implementation checks used the upstream Humble sources:
https://github.com/SteveMacenski/slam_toolbox/blob/humble/src/slam_toolbox_common.cpp
and `slam_toolbox_sync.cpp`. In particular, `map_name` defaults to absolute
`/map`, so a namespace alone is insufficient to isolate the output topic.

## First manual rotation check

The first output bag was cleanly closed, copied here as `slam-segment-01`,
and inspected in the main container. It contains 81 map messages and 80
graph messages, but no new pose messages; the graph retains one node.
Grid extents remain 307 x 349 cells. A second output bag is now recording
on the robot at `/tmp/robocat-elevator-map-output-20261106-02`; its PID
uses the same output-recorder PID file. Input recording remains continuous.

The upstream Humble `shouldProcessScan` rejects scans when translational
distance squared is below 0.8 times minimum travel distance squared, even
for a rotation. Therefore the requested pure rotation was insufficient
to validate acquisition with the configured 0.10 m threshold. The earlier
attribution of the unchanged graph to TF alone was not established.
A short operator-controlled translation is needed before a full mapping tour.

A stationary TF check in the laptop container found exact-time odom/laser
transforms available for 15/15 tested recent scans. It also found Giskard
`robocat_drive_odom` transforms on the laptop topic; a short native TF
capture did not show those names. Do not equate the laptop combined TF
tree with the exact native SLAM input. Native timestamp drops persist.
Since the saved origin, wheel orientation changed by +136.6 degrees modulo
360 and DLO by −9.25 degrees, another unresolved odometry discrepancy;
these estimates do not determine the physical turn angle.

## First manual translation check

After the operator-confirmed short translation, `slam-segment-02` was closed,
copied here, and inspected in the container: 2 new pose messages, 38 maps,
37 graph messages. The graph now has 3 nodes and 2 edges. Grid dimensions
grew from 307 x 349 to 326 x 349 at 0.05 m resolution; latest known-cell
count is 20226. This verifies acquisition of additional poses, not metric
accuracy or readiness for autonomous navigation. Output recording continues
on the robot as `/tmp/robocat-elevator-map-output-20261106-03`, using the
same PID-file location. Input recording remains continuous.

## Final map and frame files

The final graph contains 55 nodes and 60 edges; segment 03 records 52 new
poses. The raw grid is 375 x 357 at 0.05 m/cell. Map saver and graph
serialization returned success. `elevator_raw.yaml/.pgm` and
`elevator_graph.data/.posegraph` preserve the native result; the robot copy
is in `/home/pal/robocat_elevator_mapping_20261106/`. Existing PAL maps were
not replaced and AMCL has not been switched to this map.

`elevator.yaml/.pgm` is a separate, rigidly reframed raster: the saved
initial DLO base pose becomes coordinate (0,0), heading 0. Pixel centres
are transformed to the original raster and sampled with nearest neighbours;
outside cells become unknown (205). Final raster origin is (−9.35,−10.8),
dimensions 512 x 507, YAML yaw zero. The raster origin is the lower-left
corner, not the selected coordinate-system origin. Round-trip coordinate
invariance was checked numerically. Resampling cell-centre displacement
is at most 0.05/sqrt(2) m. This numerical check does not validate odometry,
SLAM accuracy or the physical initial pose. The native pose graph remains
in the original coordinate frame; it is not the reframed raster graph.
`mapping-summary.json` preserves the raw/final transform.

`elevator_frames.yaml` defines provisional frames only:

- `map -> elevator_doorway`: x=6.07125, y=−0.70247 m,
  yaw=−1.591163 rad, x axis pointing into the cabin.
- `elevator_doorway -> elevator_staging`: x=−1.0 m, y=0, yaw=0.
  Staging is one metre outside the reference door plane, facing inward.

Calibration uses 32 open and 21 closed scans, stationary robot, independently
acquired after mapping. Open reveals indicate 0.891 m opening; their centre
agrees approximately with the closed telescopic leaf junction. The reference
plane is halfway between the two measured leaf depths (0.786 and 0.835 m
from the rotated base reference); it is not an independently measured floor
gap location. Orientation follows the measured cabin back wall.

The final SLAM pose was synchronized with DLO exactly by timestamp to form
`final-localization-snapshot.json`. At the later doorway measurement, aligning
one open scan to occupied map cells required about 0.391 m and 7.08 degrees
of correction relative to that snapshot propagated through DLO. The fit has
median occupied-cell distance 1.95 cm and all selected points within 10 cm,
but it is a single local alignment with possible association ambiguity, not
active AMCL or independent ground truth. Frame precision is not established.
Raw open/closed samples and `doorway-calibration-analysis.json` are retained.

The frames have **not** been published into the control TF tree. Next steps
are independent scan/map validation and localization against the saved map,
then publication of the map-anchored frames. No autonomous navigation was
performed during this mapping/calibration session.

Input bag closure and metadata were verified: 212.9 MiB, 771.6 seconds,
7706 scans, 7518 DLO samples, 38527 wheel-odometry samples, native TF/static
TF and one robot description. `/cmd_vel` has zero samples: no Giskard goals
were sent, and the feedback bridge does not forward native joystick velocity
commands. This does not imply the robot was stationary during the manual tour.
The laptop `/tf` also contains Giskard drive frames; raw replay must select
the native DLO branch and reconstruct the new map correction from SLAM poses.
Do not replay command topics onto a live control domain.

## Native map selection and visualization follow-up

SSH shells sourcing Humble/PAL had RMW_IMPLEMENTATION unset. Explicit
`ROS_DOMAIN_ID=102 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` made the native
map load service respond successfully (`result=0`). AMCL is active,
subscribes to `/map`, and `first_map_only=false`; pose initialization and
physical scan/map alignment still need validation. This corrects the earlier
interpretation of unavailable PAL services; no stack restart was needed.

The map is copied to `/home/pal/robocat_elevator_mapping_20261106/elevator.yaml`
and `.pgm`. The separate PAL override
`/home/pal/.pal/config/99_zz_robocat_elevator_map.yaml` selects it persistently.
`pal configuration get map_server` confirmed the effective YAML path.
The old `99_my_map_server_config.yaml` was retained.

Corrected `elevator.yaml` free threshold from 0.25 to 0.196: PGM value 205
represents unknown and must not be interpreted as free. The original
map-saver metadata used 0.25. With 0.196, the 205-valued pixels lie strictly
between free/occupied thresholds; the native load response confirms unknown
cells. Raster levels are 0 (2907 cells), 205 (214702), 254 (41975).
The raw original map metadata is preserved for provenance and has not been
silently rewritten; use the corrected final `elevator.yaml` for loading.

At the operator's request the two provisional static frames are now published
on native domain 102 by separate owned processes, not by robot controller
changes. PID/log paths: `/tmp/robocat-elevator-doorway-tf.pid/.log` and
`/tmp/robocat-elevator-staging-tf.pid/.log`. They are not configured for
automatic startup. Coordinate precision remains unvalidated.
`/home/pal/robocat_elevator_mapping_20261106/elevator.rviz` contains only the
saved-map display and the two selected TF frames, hiding labora_demo displays.
A native RViz was opened using X11 forwarding, explicit CycloneDDS, and
software rendering. Its map dimensions match the final raster (512 x 507).
