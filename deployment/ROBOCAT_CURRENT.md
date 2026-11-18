# Robocat elevator test setup — 2026-11-18

This branch preserves the current Nav2 approach, supervised SNM entry, Giskard
wheel tracking with lidar-referenced longitudinal stopping, and optional 180-degree
cabin turn. Real trials completed entry and turn with software success and no
motor fault in the last recorded trial. This does not establish fault-free operation
in all conditions. HPM/person perception, autonomous elevator travel, destination
floor/map switching and exit are not implemented.

The earlier transient-attainment/overshoot investigation remains in Git history.
Current completion retains control through deceleration and requires measured rest;
longitudinal arrival is assessed against the validated cabin back-wall reference.

## Required environment

- Laptop: Docker, Byobu/tmux, SSH alias `robocat` for `pal@10.68.0.1`, and dedicated
  Ethernet address 10.68.0.206/24. Configure authentication locally; no credentials
  or authenticated SSH sockets are supplied.
- Robot: existing PAL Alum/ROS Humble stack on domain 102, native Nav2/AMCL,
  Chrony, and separate twist-mux inputs: navigation `/cmd_vel` priority 10 and SNM
  `/snm_vel` priority 20. Keep native safety/joystick/charging locks. The fragment
  `doc/robocat_mux_snm_override.yaml` records the desired input; it must be merged
  into the PAL launch configuration actually consumed by the installed stack.
  Copying an unused override file alone does not enable the input.
- Laptop container `aicor-cram-demo`, host networking, repository bind-mounted at
  `/workspace/cram`, Jazzy overlay `/opt/ros/overlay_ws/install/setup.bash`, Python
  `/opt/ros/cram-env/bin/python`. The current local image is
  `tiago-cram-real:local`; its ID and installed Python versions are recorded in
  `deployment/container/*20261116*`. Image contents are not stored in Git.
- Existing image reconstruction instructions are in `README_REAL_ROBOTS.md` and
  `scripts/setup_tiago_laptop.sh`; that historical script builds a TIAGo-named
  base image shared by these Robocat tests. Source installation uses `uv.lock`
  and the workspace instructions, followed by `scripts/regenerate_all_orm.py`.
  Generated ORM files must not be committed. The dependency inventory documents
  the observed runtime, not a claim that an unpinned upstream Docker build is
  byte-identical to the recorded local image.
- DDS Router image `robocat-dds-router:3.5.4`: build with
  `docker build -t robocat-dds-router:3.5.4 .github/docker/robocat-dds-router`.
  Its versioned sources and configuration are included. Domain 103 is isolated
  for Giskard; the bridge forwards required robot streams and `/snm_vel`.

## Robot configuration and map

Use `deployment/maps/elevator_v2/elevator_v2.yaml` and its PGM. Register/select it
on the same PAL/Eulero building/floor as the recorded staging waypoint, checking
localization in RViz. Merely copying the map to the laptop does not activate it
on the robot. Current staging is `(21.69610789414912, -31.945390716277654)`, yaw
`-2.460608174041` rad in that map. The SNM client reads the YAML pose directly;
PAL/Eulero waypoint registration is separate.

The Byobu RViz window expects the native path
`/home/pal/robocat_elevator_mapping_20261109/waypoints.rviz`. If rebuilding another
robot account, create that file using `doc/robocat_elevator_localization.rviz`
as a starting configuration, or adjust `robocat_snm/rviz_remote.sh` to its actual
location. Set the initial AMCL pose manually and verify the scan/map alignment.

Prepare applies the experimentally validated wheel multipliers **2.0 / 2.0 /
1.22**, with explicit APPLY authorization while stationary. Nav2 approach tests
used its normal 0.20 m XY tolerance; the elevator profile requires staging position/orientation errors within 0.10 m / 10 degrees.
Read-only preparation and SNM checks must pass before a motion request.

## Normal startup (host)

```bash
bash scripts/robocat-infra start --feedback wheel --max-linear 0.3 --max-angular 0.2
byobu-tmux attach-session -t robocat-infra
```

In prepare, confirm APPLY only with the robot stopped and conditions ready.
Clock synchronization is integrated: Robocat serves NTP on Ethernet, the laptop
gets a temporary `/run` override, and startup requires a measured NTP offset
below 50 ms. Sudo may request robot/laptop passwords. Permanent laptop time
configuration is not installed. See `doc/robocat_clock.md`.

After preparation/DDS/Giskard are ready, on the host:

```bash
bash scripts/robocat-byobu
```

Set/check localization in RViz. In the container SNM terminal:

```bash
bash scripts/robocat-entry --profile elevator
```

This is read-only. Handover and motion are explicitly authorized separately.
The following operator-run test assumes doors are held open, clear space,
operator presence, teleop off and immediately reachable emergency stop:

```bash
bash scripts/robocat-entry --profile elevator \
  --execute --doors-held-open --bootstrap-navigation --turn-after-entry --verbose
```

## Current calibration and behavior

- `config/robocat_entry/elevator.yaml` contains the stable map fingerprint,
  staging file, `elevator_inside` name, wheel feedback, 0.936 m base-to-back-wall
  reference, 0.05 m longitudinal tolerance, and retained doorway authorization.
  Execution, held-open doors and bootstrap authorization remain explicit CLI flags.
- `elevator_inside` must exist in the native Eulero database on the configured
  floor. `doc/robocat_elevator_inside.yaml` is a read-only export of its map pose;
  it is evidence for restoration, not automatically imported or consumed by the
  client. Register the waypoint in the correctly aligned floor before running.
- Giskard uses PAL-compatible acceleration/jerk bounds: linear 0.6 m/s² and
  2 m/s³, angular 1 rad/s² and 5 rad/s³; control is 20 Hz. Wheel multipliers remain
  2.0 / 2.0 / 1.22. The calibrated lidar endpoint can adapt the nominal target by
  at most 0.40 m. See `doc/robocat_snm.md` for validity and geometric bounds.
- SNM associates scan/TF with historical odometry and evaluates one consistent
  snapshot. Result freshness is checked after computation, with one bounded
  re-evaluation; the 500 ms freshness limit is unchanged. Cached map distance
  fields and sensor transforms reduce processing cost without weakening checks.
- `--turn-after-entry` submits a separate native `CartesianOrientation` task,
  with zero commanded linear velocity, only after verified longitudinal arrival
  and rest. Its target is measured initial yaw plus pi. The progress/arrival error
  is the shortest quaternion distance, independent of absolute heading.
- The current-floor cabin-corner calibration is in
  `config/robocat_entry/cabin_labora_floor.yaml`. Read `doc/robocat_cabin_frame.md`
  before publishing it. The static preview is not an automatically tracked cabin
  frame; stop it before changing floors/maps. The upper-floor calibration is pending.

Motor discrepancy faults occurred in earlier trials and their hardware cause has
not been established. The recorded latest successful trial is not a general motor
reliability guarantee. Door retention is a held-open-door test assumption; there is
no HPM-based passenger handling. The two odometry estimates can disagree, and the
laser wall estimate is not an independent physical survey.

## Recording and shutdown

Before the request, in another container terminal:

```bash
bash scripts/robocat-record-entry
```

Verify the server prints `DIAGNOSTICS: /giskard/goal_diagnostics`. The bag contains
same-cycle native task decisions, applied odometry stamps, targets/errors,
SNM final-assessment values, odometries, scan, commands, map and TF. Stop the
recorder about five seconds after the robot stops. No recordings are committed.

Stop an active request with Ctrl+C and verify physical arrest. On the host:

```bash
byobu-tmux kill-session -t robocat-elevator
bash scripts/robocat-infra stop
```

Confirm STOP as requested. Native shutdown restores normal laptop time sources.
Do not kill the infra session directly; see the clock and lifecycle guides for
recovery after abnormal termination. No sudoers/password-free helper is installed
by this snapshot. Existing motor recovery remains an explicit guided operation.

## Validation of this snapshot

On 2026-11-18, 72 focused offline tests passed for profiles, timing, map/scan
association, cached observation geometry, wall stopping/guards, recorder coverage,
waypoint export and PAL dynamics. The copied runtime/test files were compared
byte-for-byte with the tested live workspace; 54 copied Python files also passed
syntax parsing. Earlier the same day, 40 orientation/turn-focused checks passed,
including native QP half-turns from multiple initial headings. These are focused
checks, not a full-workspace test run or a clean Docker image rebuild.
