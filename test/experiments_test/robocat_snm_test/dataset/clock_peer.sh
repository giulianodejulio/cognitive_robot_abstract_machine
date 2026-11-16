#!/usr/bin/env bash
set -eu
case "$(basename "$0"):$*" in
    systemctl:*) exit 0 ;;
    timedatectl:timesync-status)
        if [[ $CLOCK_SYNCHRONIZED == 1 ]]; then
            printf 'Server: 10.68.0.1\nOffset: +2.1ms\nPacket count: 3\n'
        else
            printf 'Server: 10.68.0.1\nOffset: +1.5s\nPacket count: 1\n'
        fi ;;
    *) exit 1 ;;
esac
