#!/usr/bin/env bash
# KITTI mode for the bag-player container.
#
# Downloads the KITTI raw_synced dataset into a named Docker volume (/kitti),
# replaces the raw velodyne scans with the motion-compensated odometry scans,
# converts the data to a ROS 2 bag once, and then plays it indefinitely with
# ros2 bag play --clock.
#
# All downloaded archives are cached in /kitti/.download_tmp so subsequent
# container restarts skip the download step.
#
# Environment variables:
#   KITTI_DIR         — container path for the named kitti volume (default: /kitti)
#   KITTI_DATE        — recording date                            (default: 2011_10_03)
#   KITTI_DRIVE       — drive number (zero-padded 4 digits)       (default: 0027)
#   KITTI_CAMERA      — 'left' or 'right'                         (default: left)
#   KITTI_SEQUENCE    — odometry sequence for velodyne download    (default: 00)
#   BAG_RATE          — playback rate multiplier                   (default: 1.0)
#   BAG_LOOP          — set to "false" to play once               (default: true)
#   ROS2_CAMERA_IMAGE_TOPIC   (default: /camera/image_raw)
#   ROS2_CAMERA_INFO_TOPIC    (default: /camera/camera_info)
#   ROS2_LIDAR_POINTS_TOPIC   (default: /points_raw)
#   ROS2_IMU_TOPIC            (default: /imu/data)

set -euo pipefail

# ROS 2 setup scripts may reference unset variables; disable nounset around them.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

KITTI_DIR="${KITTI_DIR:-/kitti}"
KITTI_DATE="${KITTI_DATE:-2011_10_03}"
KITTI_DRIVE="${KITTI_DRIVE:-0027}"
KITTI_CAMERA="${KITTI_CAMERA:-left}"
KITTI_SEQUENCE="${KITTI_SEQUENCE:-00}"
BAG_RATE="${BAG_RATE:-1.0}"
BAG_LOOP="${BAG_LOOP:-true}"
ROS2_CAMERA_IMAGE_TOPIC="${ROS2_CAMERA_IMAGE_TOPIC:-/camera/image_raw}"
ROS2_CAMERA_INFO_TOPIC="${ROS2_CAMERA_INFO_TOPIC:-/camera/camera_info}"
ROS2_LIDAR_POINTS_TOPIC="${ROS2_LIDAR_POINTS_TOPIC:-/points_raw}"
ROS2_IMU_TOPIC="${ROS2_IMU_TOPIC:-/imu/data}"

DRIVE_DIR="${KITTI_DIR}/${KITTI_DATE}/${KITTI_DATE}_drive_${KITTI_DRIVE}_sync"
ROS2BAG_DIR="${KITTI_DIR}/ros2bag/kitti_${KITTI_DATE}_drive_${KITTI_DRIVE}_${KITTI_CAMERA}"
TMP_DIR="${KITTI_DIR}/.download_tmp"

mkdir -p "${KITTI_DIR}" "${TMP_DIR}"

# -------------------------------------------------------------------------
# Helper: download a file, verify it, and cache it.
# If the file already exists but fails integrity check (e.g. interrupted
# download), it is deleted and re-downloaded automatically.
# -------------------------------------------------------------------------
_download() {
    local url="$1" dest="$2"
    if [[ -f "${dest}" ]]; then
        # Verify the existing file is a valid zip before trusting the cache.
        if unzip -t "${dest}" >/dev/null 2>&1; then
            echo "[bag-player/kitti] Cached (verified): $(basename "${dest}")"
            return
        else
            echo "[bag-player/kitti] Cached file '$(basename "${dest}")' is corrupt or incomplete — re-downloading ..."
            rm -f "${dest}"
        fi
    fi
    echo "[bag-player/kitti] Downloading $(basename "${dest}") ..."
    # Download to a temp file first; only rename on success so a crash
    # during download does not leave a partial file that looks valid-named.
    local tmp_dest="${dest}.tmp"
    rm -f "${tmp_dest}"
    if ! curl -fSL --retry 3 --retry-delay 10 -o "${tmp_dest}" "${url}"; then
        rm -f "${tmp_dest}"
        echo "[bag-player/kitti] ERROR: curl download failed for '${url}'." >&2
        exit 1
    fi
    # Validate the downloaded file before committing it to the cache.
    if ! unzip -t "${tmp_dest}" >/dev/null 2>&1; then
        rm -f "${tmp_dest}"
        echo "[bag-player/kitti] ERROR: Downloaded file from '${url}' failed zip integrity check." >&2
        exit 1
    fi
    mv "${tmp_dest}" "${dest}"
}

# -------------------------------------------------------------------------
# 1. Download KITTI raw synced + calibration data
# -------------------------------------------------------------------------
if [[ ! -d "${DRIVE_DIR}/image_02" ]] || [[ ! -d "${DRIVE_DIR}/image_03" ]]; then
    echo "[bag-player/kitti] KITTI raw data not found — starting download (~4 GB)."
    echo "[bag-player/kitti] Data will be cached in the 'kitti_data' Docker volume."

    _sync_url="https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/${KITTI_DATE}_drive_${KITTI_DRIVE}/${KITTI_DATE}_drive_${KITTI_DRIVE}_sync.zip"
    _sync_zip="${TMP_DIR}/${KITTI_DATE}_drive_${KITTI_DRIVE}_sync.zip"
    _download "${_sync_url}" "${_sync_zip}"

    _calib_url="https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/${KITTI_DATE}_calib.zip"
    _calib_zip="${TMP_DIR}/${KITTI_DATE}_calib.zip"
    _download "${_calib_url}" "${_calib_zip}"

    echo "[bag-player/kitti] Extracting raw synced data ..."
    unzip -q -o "${_sync_zip}" -d "${KITTI_DIR}"
    echo "[bag-player/kitti] Extracting calibration data ..."
    unzip -q -o "${_calib_zip}" -d "${KITTI_DIR}"
    echo "[bag-player/kitti] Raw KITTI data extracted."
fi

# -------------------------------------------------------------------------
# 2. Replace velodyne with motion-compensated odometry velodyne
# -------------------------------------------------------------------------
_velo_flag="${KITTI_DIR}/.velo_seq${KITTI_SEQUENCE}_replaced"
if [[ ! -f "${_velo_flag}" ]]; then
    echo "[bag-player/kitti] Downloading odometry velodyne data (~5 GB, motion-compensated)."
    echo "[bag-player/kitti] This large download is cached and only runs once."

    _velo_url="https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_velodyne.zip"
    _velo_zip="${TMP_DIR}/data_odometry_velodyne.zip"
    _download "${_velo_url}" "${_velo_zip}"

    _velo_data_dir="${DRIVE_DIR}/velodyne_points/data"
    echo "[bag-player/kitti] Replacing velodyne data with motion-compensated scans ..."
    rm -rf "${_velo_data_dir}"
    mkdir -p "${_velo_data_dir}"
    if ! unzip -q -j -d "${_velo_data_dir}" "${_velo_zip}" \
            "dataset/sequences/${KITTI_SEQUENCE}/velodyne/*.bin"; then
        echo "[bag-player/kitti] ERROR: Failed to extract odometry velodyne data." >&2
        echo "[bag-player/kitti]   Verify KITTI_SEQUENCE='${KITTI_SEQUENCE}' is a valid sequence" >&2
        echo "[bag-player/kitti]   (00-10 for training, 11-21 for test) and that the downloaded" >&2
        echo "[bag-player/kitti]   archive is intact. Delete '${_velo_zip}' to re-download." >&2
        exit 1
    fi
    touch "${_velo_flag}"
    echo "[bag-player/kitti] Velodyne data replaced with motion-compensated version."
fi

# -------------------------------------------------------------------------
# 3. Convert KITTI raw data → ROS 2 bag (skip if already done)
# -------------------------------------------------------------------------
if [[ ! -d "${ROS2BAG_DIR}" ]]; then
    echo "[bag-player/kitti] Converting KITTI data to ROS 2 bag — this may take several minutes ..."
    python3 /workspace/kitti_to_ros2bag.py \
        --kitti-dir  "${KITTI_DIR}" \
        --date       "${KITTI_DATE}" \
        --drive      "${KITTI_DRIVE}" \
        --camera     "${KITTI_CAMERA}" \
        --output-bag "${ROS2BAG_DIR}" \
        --image-topic       "${ROS2_CAMERA_IMAGE_TOPIC}" \
        --camera-info-topic "${ROS2_CAMERA_INFO_TOPIC}" \
        --lidar-topic       "${ROS2_LIDAR_POINTS_TOPIC}" \
        --imu-topic         "${ROS2_IMU_TOPIC}"
else
    echo "[bag-player/kitti] ROS 2 bag already exists at '${ROS2BAG_DIR}', skipping conversion."
fi

# -------------------------------------------------------------------------
# 4. Play the converted ROS 2 bag
# -------------------------------------------------------------------------
EXTRA_ARGS=()
if [[ "${BAG_LOOP}" == "true" ]]; then
    EXTRA_ARGS+=(--loop)
fi

echo "[bag-player/kitti] Playing KITTI ros2bag '${ROS2BAG_DIR}' at rate ${BAG_RATE} (loop=${BAG_LOOP}) ..."
exec ros2 bag play "${ROS2BAG_DIR}" \
    --clock \
    --rate "${BAG_RATE}" \
    "${EXTRA_ARGS[@]}"
