#!/usr/bin/env bash
cat >/dev/null
printf '1794841512.972826 [102] ros2: wlan0: optional interface was not found.\r\n'
printf 'CONTROLLER STATE: active\n'
printf '1794841512.972826 [102] python3: wlan0: optional interface was not found.\n' >&2
printf 'DDS error: connection lost\n' >&2
exit "${ROBOCAT_PEER_EXIT:-0}"
