# Capture a waypoint at the current Robocat pose

Run on the laptop host from this repository, with Robocat connected and SSH
configured. No Giskard server or DDS router is needed: acquisition uses the
native robot ROS environment.

```bash
bash scripts/robocat-save-waypoint elevator_inside --check
bash scripts/robocat-save-waypoint elevator_inside
```

The first command only previews. The second prints the loaded map YAML,
Eulero map identity, floor key and pose, then requires `SAVE` to persist it.
The robot must be stationary and correctly localized; visually check the scan
against the map in RViz. The script checks fresh TF and wheel feedback but
cannot certify that AMCL has localized at the correct physical location.
It never sends a movement or initial-pose request.

The live `/map` geometry and occupancy cells are compared with the active enabled
Eulero map files. The `yaml_filename` parameter is not used as proof of map identity:
it may remain stale after a service-based map load. An ambiguous or absent match blocks saving. The
current `map_server` frame to `base_footprint` pose is transformed through the
selected map's Eulero transform into floor coordinates. Eulero waypoints belong
to a floor, not to occupancy-grid pixels. Names already present on that floor
are refused; there is no overwrite mode. A changed published map, changed pose,
stale feedback or moving wheels prevents saving. Creation uses
`/eulero_manager/create_waypoint`, followed by listing verification. The data
persists in the native Eulero database. Temporary script copies are removed
from the robot after the command exits.

For staging/inside calibration, capture staging first, drive inside using
teleop, stop, verify clearance from the doors and cabin walls, then capture
inside with a different name. Existing SNM YAML configuration is not changed
by this capture command; consuming the new waypoint is a separate step.
