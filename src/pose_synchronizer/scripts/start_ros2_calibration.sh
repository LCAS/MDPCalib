#!/usr/bin/env bash

set -euo pipefail

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS2_DISTRO="${ROS2_DISTRO:-humble}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export MDPCALIB_RUNTIME_LOG_DIR="${MDPCALIB_RUNTIME_LOG_DIR:-/data/runtime_logs}"
export MDPCALIB_CLEAN_PREVIOUS_RUNS="${MDPCALIB_CLEAN_PREVIOUS_RUNS:-true}"
export MDPCALIB_REBUILD_WORKSPACE="${MDPCALIB_REBUILD_WORKSPACE:-true}"

mkdir -p "${MDPCALIB_RUNTIME_LOG_DIR}"
mkdir -p /data/cache /data/experiments

rm -rf /data/cache/*
if [[ "${MDPCALIB_CLEAN_PREVIOUS_RUNS}" != "false" ]]; then
    rm -rf /data/experiments/*
fi

ROSCORE_PID=""
BRIDGE_PID=""

cleanup() {
    local exit_code=$?
    trap - EXIT INT TERM
    if [[ -n "${BRIDGE_PID}" ]] && kill -0 "${BRIDGE_PID}" 2>/dev/null; then
        kill "${BRIDGE_PID}" 2>/dev/null || true
        wait "${BRIDGE_PID}" 2>/dev/null || true
    fi
    if [[ -n "${ROSCORE_PID}" ]] && kill -0 "${ROSCORE_PID}" 2>/dev/null; then
        kill "${ROSCORE_PID}" 2>/dev/null || true
        wait "${ROSCORE_PID}" 2>/dev/null || true
    fi
    exit "${exit_code}"
}
trap cleanup EXIT INT TERM

source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

if [[ "${MDPCALIB_REBUILD_WORKSPACE}" != "false" ]]; then
    echo "[mdpcalib] Rebuilding catkin workspace..."
    cd /root/catkin_ws
    catkin build -cs
    source /root/catkin_ws/devel/setup.bash
fi

echo "[mdpcalib] Starting roscore..."
roscore >"${MDPCALIB_RUNTIME_LOG_DIR}/roscore.log" 2>&1 &
ROSCORE_PID=$!

until rostopic list >/dev/null 2>&1; do
    sleep 1
done

if [[ "${ENABLE_ROS2_BRIDGE:-true}" != "false" ]]; then
    if [[ ! -f "/opt/ros/${ROS2_DISTRO}/setup.bash" ]]; then
        echo "[mdpcalib] ROS 2 setup file /opt/ros/${ROS2_DISTRO}/setup.bash does not exist." >&2
        exit 1
    fi

    echo "[mdpcalib] Starting ros1_bridge dynamic bridge..."
    source /opt/ros/noetic/setup.bash
    # ROS 2 setup scripts reference variables without defaults; temporarily
    # disable nounset so sourcing them does not abort under set -euo pipefail.
    set +u
    source "/opt/ros/${ROS2_DISTRO}/setup.bash"
    set -u
    ros2 run ros1_bridge dynamic_bridge --bridge-all-topics >"${MDPCALIB_RUNTIME_LOG_DIR}/ros1_bridge.log" 2>&1 &
    BRIDGE_PID=$!
    sleep 2
fi

echo "[mdpcalib] Launching live ROS 2 calibration stack..."

# ---------------------------------------------------------------------------
# CMRNext model weights — download automatically if not already present.
# All three weights must share the same directory (CMRNEXT_WEIGHTS_DIR).
# ---------------------------------------------------------------------------
CMRNEXT_WEIGHTS_DIR="${CMRNEXT_WEIGHTS_DIR:-/data/cmrnext}"
CMRNEXT_WEIGHTS_URL="${CMRNEXT_WEIGHTS_URL:-https://calibration.cs.uni-freiburg.de/downloads/cmrnext_weights.zip}"

_weight1="${CMRNEXT_WEIGHT_1:-${CMRNEXT_WEIGHTS_DIR}/cmrnext-calib-LEnc-iter1.tar}"
_weight2="${CMRNEXT_WEIGHT_2:-${CMRNEXT_WEIGHTS_DIR}/cmrnext-calib-LEnc-iter5.tar}"
_weight3="${CMRNEXT_WEIGHT_3:-${CMRNEXT_WEIGHTS_DIR}/cmrnext-calib-LEnc-iter6.tar}"

if [[ ! -f "${_weight1}" ]] || [[ ! -f "${_weight2}" ]] || [[ ! -f "${_weight3}" ]]; then
    echo "[mdpcalib] CMRNext model weights not found in '${CMRNEXT_WEIGHTS_DIR}'."
    echo "[mdpcalib] Downloading from ${CMRNEXT_WEIGHTS_URL} ..."
    mkdir -p "${CMRNEXT_WEIGHTS_DIR}"
    _tmp_zip="$(mktemp /tmp/cmrnext_weights_XXXXXX.zip)"
    if curl -fSL --retry 3 --retry-delay 5 -o "${_tmp_zip}" "${CMRNEXT_WEIGHTS_URL}"; then
        if ! unzip -o -j -d "${CMRNEXT_WEIGHTS_DIR}" "${_tmp_zip}" '*.tar'; then
            rm -f "${_tmp_zip}"
            echo "[mdpcalib] ERROR: Failed to unzip model weights archive from ${CMRNEXT_WEIGHTS_URL}." >&2
            exit 1
        fi
        rm -f "${_tmp_zip}"
        # Verify the expected files were actually extracted
        _missing=()
        [[ ! -f "${_weight1}" ]] && _missing+=("${_weight1}")
        [[ ! -f "${_weight2}" ]] && _missing+=("${_weight2}")
        [[ ! -f "${_weight3}" ]] && _missing+=("${_weight3}")
        if [[ ${#_missing[@]} -gt 0 ]]; then
            echo "[mdpcalib] ERROR: Download succeeded but the following weight files are missing after extraction:" >&2
            printf '[mdpcalib]   %s\n' "${_missing[@]}" >&2
            echo "[mdpcalib] Expected files: cmrnext-calib-LEnc-iter1.tar, cmrnext-calib-LEnc-iter5.tar, cmrnext-calib-LEnc-iter6.tar" >&2
            echo "[mdpcalib] The archive may not contain the expected *.tar files." >&2
            exit 1
        fi
        echo "[mdpcalib] CMRNext model weights downloaded to '${CMRNEXT_WEIGHTS_DIR}'."
    else
        rm -f "${_tmp_zip}"
        echo "[mdpcalib] ERROR: Failed to download model weights from ${CMRNEXT_WEIGHTS_URL}." >&2
        echo "[mdpcalib] Please download manually and place *.tar files in '${CMRNEXT_WEIGHTS_DIR}'." >&2
        echo "[mdpcalib] Download URL: ${CMRNEXT_WEIGHTS_URL}" >&2
        exit 1
    fi
fi
# ---------------------------------------------------------------------------

source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
roslaunch pose_synchronizer ros2_live_calibration.launch \
  camera_input_image_topic:="${ROS2_CAMERA_IMAGE_TOPIC:-/camera/image_raw}" \
  camera_input_camera_info_topic:="${ROS2_CAMERA_INFO_TOPIC:-/camera/camera_info}" \
  camera_output_image_topic:="${ROS2_INTERNAL_CAMERA_IMAGE_TOPIC:-/camera_undistorted/image}" \
  camera_output_camera_info_topic:="${ROS2_INTERNAL_CAMERA_INFO_TOPIC:-/camera_undistorted/camera_info}" \
  lidar_points_topic:="${ROS2_LIDAR_POINTS_TOPIC:-/points_raw}" \
  imu_topic:="${ROS2_IMU_TOPIC:-/imu/data}" \
  fast_lo_config_file:="${FAST_LO_CONFIG_FILE:-$(rospack find lidar_imu_init)/config/ouster.yaml}" \
  fast_lo_disable_motion_compensation:="${FAST_LO_DISABLE_MOTION_COMPENSATION:-false}" \
  orb_slam_settings_file:="${ORB_SLAM3_SETTINGS_FILE:-$(rospack find orb_slam3_ros_wrapper)/config/kitti_camera_color_left_right.yaml}" \
  cmrnext_weight_1:="${CMRNEXT_WEIGHT_1:-/data/cmrnext/cmrnext-calib-LEnc-iter1.tar}" \
  cmrnext_weight_2:="${CMRNEXT_WEIGHT_2:-/data/cmrnext/cmrnext-calib-LEnc-iter5.tar}" \
  cmrnext_weight_3:="${CMRNEXT_WEIGHT_3:-/data/cmrnext/cmrnext-calib-LEnc-iter6.tar}" \
  cmrnext_image_height:="${CMRNEXT_IMAGE_HEIGHT:-376}" \
  cmrnext_image_width:="${CMRNEXT_IMAGE_WIDTH:-1241}" \
  disable_pose_synchronizer:="${POSE_SYNCHRONIZER_DISABLE:-false}" \
  lidar_frame_id:="${LIDAR_FRAME_ID:-lidar}" \
  camera_frame_id:="${CAMERA_FRAME_ID:-camera}" \
  ros2_export_yaml_path:="${ROS2_CALIBRATION_OUTPUT_PATH:-/data/calibration/ros2/extrinsics.yaml}" \
  ros2_export_parameter_root:="${ROS2_CALIBRATION_PARAMETER_ROOT:-/**}" \
  calibration_timeout_sec:="${CALIBRATION_TIMEOUT_SEC:-900}"
