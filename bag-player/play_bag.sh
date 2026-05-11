#!/usr/bin/env bash
# Entry-point for the bag-player container.
# Plays a ros2bag folder and loops until stopped (or until BAG_LOOP=false).
#
# Environment variables:
#   BAG_PATH  — absolute path inside the container to the rosbag2 folder (default: /rosbags)
#   BAG_LOOP  — set to "false" to play once and exit (default: true)
#   BAG_RATE  — playback rate multiplier, e.g. 0.5 for half speed (default: 1.0)

set -euo pipefail

# ROS 2 setup scripts may reference unset variables; disable nounset around them.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

BAG_PATH="${BAG_PATH:-/rosbags}"
BAG_LOOP="${BAG_LOOP:-true}"
BAG_RATE="${BAG_RATE:-1.0}"

if [[ ! -d "${BAG_PATH}" ]]; then
    echo "[bag-player] ERROR: BAG_PATH '${BAG_PATH}' does not exist or is not a directory." >&2
    exit 1
fi

EXTRA_ARGS=()
if [[ "${BAG_LOOP}" == "true" ]]; then
    EXTRA_ARGS+=(--loop)
fi

echo "[bag-player] Playing ros2bag '${BAG_PATH}' at rate ${BAG_RATE} (loop=${BAG_LOOP}) ..."
exec ros2 bag play "${BAG_PATH}" \
    --clock \
    --rate "${BAG_RATE}" \
    "${EXTRA_ARGS[@]}"
