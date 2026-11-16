# Robocat elevator test setup — 2026-11-16

This branch preserves the working setup used for Nav2 approach, SNM handover,
Giskard straight entry, and the diagnostic bags. It does **not** fix the observed
final overshoot: Giskard can report success after transient target attainment,
while SNM rejects the later stationary pose. No HPM/person-perception integration
is active. The known fault must remain visible when comparing future fixes.

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
used its normal 0.20 m XY tolerance; exact SNM handover limits are CLI arguments.
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
bash scripts/robocat-entry
```

This is read-only. Handover and motion are explicitly authorized separately.
The following operator-run test assumes doors are held open, clear space,
operator presence, teleop off and immediately reachable emergency stop:

```bash
bash scripts/robocat-entry --execute --bootstrap-navigation \
  --feedback wheel --retain-door-clearance \
  --doorway-distance 0.86 --distance 1.30 \
  --staging-position-tolerance 0.10 \
  --staging-orientation-tolerance-deg 10 --doors-held-open \
  --map-fingerprint 14cae3e62b38495028d71f384a7bea50e41025b26b866dfbcf6107edb8961fda
```

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
