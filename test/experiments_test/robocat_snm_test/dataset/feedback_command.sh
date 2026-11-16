#!/usr/bin/env bash
printf '%s\0' "$@" > "$ROBOCAT_FEEDBACK_ARGUMENTS"
