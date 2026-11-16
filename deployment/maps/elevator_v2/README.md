# Active elevator map

Map acquired on 2026-11-09, with the two component-42 spurious occupied cells
cleared on 2026-11-13. This is the map used in the 2026-11-16 entry tests, not
the older merged map or the 2026-11-06 map retained elsewhere in this repository.

- Eulero building/floor: `labora_demo/labora_floor`.
- Map name: `elevator_v2_map`.
- SNM occupancy-grid fingerprint:
  `14cae3e62b38495028d71f384a7bea50e41025b26b866dfbcf6107edb8961fda`.
- Resolution 0.05 m, image 733 x 888.
- Register/select the map through the installed PAL/Eulero tools, then verify
  the live map fingerprint printed by `robocat-entry`; image file SHA256 is a
  different quantity. Frame names alone do not identify map alignment.
- Current staging pose is in `doc/robocat_elevator_staging.yaml`.
- `doc/robocat_localization_start.yaml` was captured before the cell cleanup;
  its old fingerprint is historical. Its pose may serve as an initial estimate
  only after physically matching the starting location and checking RViz.

`SHA256SUMS` checks the saved PGM/YAML. No SLAM graph, recordings, screenshots,
previous map image or per-test artifacts are required to load this occupancy map.
