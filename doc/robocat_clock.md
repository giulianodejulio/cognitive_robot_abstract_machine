# Clock synchronization during Robocat tests

Use `bash scripts/robocat-infra start` and attach normally. After APPLY confirms
that the robot is stopped, prepare configures Robocat's existing Chrony server
if needed and selects it as the laptop's temporary NTP source. Robot and laptop
sudo passwords can be requested in prepare; no password is stored.

Preparation waits up to 60 seconds for a received NTP measurement from
10.68.0.1 with an absolute offset below 50 ms. Failure restores normal laptop
sources and blocks dependent startup. Successful synchronization prints CLOCK
READY, after which hardware preparation and DDS/Giskard startup continue.
The existing systemd-timesyncd service keeps polling every 16–32 seconds.
This initial check does not guarantee a permanent bound during operation.

The robot server configuration is persistent, restricted to the dedicated
Ethernet subnet. Local stratum 10 provides a common reference offline without
certifying accurate UTC. The laptop configuration is temporary, exclusively in
/run/systemd/timesyncd.conf.d. Existing laptop files are unchanged, and the
override disappears on reboot. Containers share the laptop clock.

Stop active clients and close the test session, then run:

```bash
bash scripts/robocat-infra stop
```

After owned Giskard and DDS stop, normal laptop time sources are restored.
Closing only the test session leaves infrastructure synchronization active.
Do not kill the infrastructure session directly: that bypasses orderly cleanup.
If it was killed abnormally, verify all motion is stopped before running:

```bash
bash scripts/robocat-clock stop
```

Cable loss alone does not remove the override. Stop infrastructure before
unplugging Ethernet. Clock-source changes can step time and must not happen
during movements. Missing Ethernet or an unreachable NTP server blocks startup.

Read-only verification:

```bash
timedatectl timesync-status
ssh robocat 'chronyc tracking; chronyc clients'
```

Other per-interface NTP sources can affect server selection. If a different
server is selected the readiness check blocks; inspect
`timedatectl show-timesync --all`. No new dependency is installed. Real Ethernet
synchronization still needs verification after the sudo-authorized first setup.
