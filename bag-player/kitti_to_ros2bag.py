#!/usr/bin/env python3
"""Convert KITTI raw_synced data to a ROS 2 bag (sqlite3 storage).

All four sensor streams (color camera, velodyne, IMU) are written in
timestamp order so the resulting bag behaves identically to a ros2bag
created with ros2 bag record.

Events are merged on-the-fly using heapq.merge so only one message
per stream is held in memory at a time, avoiding multi-GB buffers for
large sequences.

Usage (called automatically by play_kitti.sh):
  kitti_to_ros2bag.py \\
      --kitti-dir /kitti --date 2011_10_03 --drive 0027 \\
      --camera left --output-bag /kitti/ros2bag/kitti_seq00 \\
      --image-topic /camera/image_raw \\
      --camera-info-topic /camera/camera_info \\
      --lidar-topic /points_raw \\
      --imu-topic /imu/data

The published topics match the default mdpcalib Docker compose configuration
so no extra remapping is needed.
"""

import argparse
import calendar
import heapq
import os
from datetime import datetime
from typing import Generator, Tuple

import cv2
import numpy as np
import progressbar
import pykitti
import rosbag2_py
from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField

# KITTI colour camera indices: 2 = camera_color_left, 3 = camera_color_right
_CAMERA_IDX = {"left": 2, "right": 3}

# Type alias for a bag event tuple
_Event = Tuple[int, str, bytes]


# ---------------------------------------------------------------------------
# Timestamp helpers
# ---------------------------------------------------------------------------

def _dt_to_ns(dt: datetime) -> int:
    """Return nanoseconds since Unix epoch for a naive KITTI UTC datetime.

    Uses calendar.timegm so the result is always interpreted as UTC,
    regardless of the host system's local timezone.
    """
    # calendar.timegm treats the input tuple as UTC (no local-time adjustment)
    seconds = calendar.timegm(dt.timetuple())
    return seconds * 1_000_000_000 + dt.microsecond * 1_000


def _make_stamp(ts_ns: int) -> Time:
    t = Time()
    t.sec = ts_ns // 1_000_000_000
    t.nanosec = ts_ns % 1_000_000_000
    return t


# ---------------------------------------------------------------------------
# Message builders
# ---------------------------------------------------------------------------

def _make_image(path: str, frame_id: str, stamp: Time) -> Image:
    cv_img = cv2.imread(path)
    if cv_img is None:
        raise RuntimeError(
            f"[kitti2ros2bag] Could not read image: {path}\n"
            "  Verify the file exists, is a valid PNG/JPEG, and was fully "
            "extracted (re-run with a clean /kitti volume if the download was interrupted)."
        )
    msg = Image()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = cv_img.shape[0]
    msg.width = cv_img.shape[1]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = cv_img.shape[1] * 3
    msg.data = cv_img.tobytes()
    return msg


def _make_camera_info(util: dict, camera_pad: str, frame_id: str, stamp: Time) -> CameraInfo:
    """Build a CameraInfo message from a parsed calib_cam_to_cam.txt util dict."""
    s = util[f"S_rect_{camera_pad}"]
    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.width = int(s[0])
    msg.height = int(s[1])
    msg.distortion_model = "plumb_bob"
    # ROS 2 CameraInfo uses lower-case field names: k, d, r, p
    msg.k = util[f"K_{camera_pad}"].flatten().tolist()
    msg.r = util[f"R_rect_{camera_pad}"].flatten().tolist()
    msg.d = util[f"D_{camera_pad}"].flatten().tolist()
    msg.p = util[f"P_rect_{camera_pad}"].flatten().tolist()
    return msg


def _make_point_cloud(path: str, frame_id: str, stamp: Time) -> PointCloud2:
    """Build a PointCloud2 from a KITTI velodyne .bin file.

    Layout (matches kitti2bag.py): x,y,z,intensity(f32), ring(u16), _pad(u16), time(f32)
    → 24 bytes per point with ring at offset 16 and time at offset 20.
    """
    raw = np.fromfile(path, dtype=np.float32).reshape(-1, 4)

    # Compute per-point ring channel (Velodyne HDL-64E geometry)
    depth = np.linalg.norm(raw[:, :3], axis=1)
    depth = np.maximum(depth, 1e-9)
    pitch = np.arcsin(raw[:, 2] / depth)
    fov_down = -24.8 / 180.0 * np.pi
    fov = (24.8 + 2.0) / 180.0 * np.pi
    ring = np.clip(np.floor((pitch + abs(fov_down)) / fov * 64.0), 0, 63).astype(np.uint16)

    n = raw.shape[0]
    buf = np.zeros(n, dtype=[
        ("x",         np.float32),
        ("y",         np.float32),
        ("z",         np.float32),
        ("intensity", np.float32),
        ("ring",      np.uint16),
        ("_pad",      np.uint16),   # keeps 'time' at offset 20
        ("time",      np.float32),
    ])
    buf["x"] = raw[:, 0]
    buf["y"] = raw[:, 1]
    buf["z"] = raw[:, 2]
    buf["intensity"] = raw[:, 3]
    buf["ring"] = ring

    fields = [
        PointField(name="x",         offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name="y",         offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name="z",         offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="ring",      offset=16, datatype=PointField.UINT16,  count=1),
        PointField(name="time",      offset=20, datatype=PointField.FLOAT32, count=1),
    ]

    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = n
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = 24
    msg.row_step = 24 * n
    msg.data = buf.tobytes()
    msg.is_dense = True
    return msg


def _make_imu(path: str, frame_id: str, stamp: Time) -> Imu:
    """Build an Imu message from a KITTI OXTS text file.

    pykitti OxtsPacket field indices (0-based):
      14=af, 15=al, 16=au  (forward/left/up linear acceleration)
      20=wf, 21=wl, 22=wu  (forward/left/up angular rate)
    """
    with open(path, encoding="utf-8") as f:
        vals = list(map(float, f.read().split()))

    msg = Imu()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.linear_acceleration.x = vals[14]   # af
    msg.linear_acceleration.y = vals[15]   # al
    msg.linear_acceleration.z = vals[16]   # au
    msg.angular_velocity.x = vals[20]      # wf
    msg.angular_velocity.y = vals[21]      # wl
    msg.angular_velocity.z = vals[22]      # wu
    # Covariance unknown
    msg.linear_acceleration_covariance[0] = -1
    msg.angular_velocity_covariance[0] = -1
    msg.orientation_covariance[0] = -1
    return msg


# ---------------------------------------------------------------------------
# Timestamp file reader
# ---------------------------------------------------------------------------

def _read_timestamps(ts_file: str) -> list:
    """Read a KITTI timestamps.txt file into a list of datetime objects."""
    dts = []
    with open(ts_file, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            # KITTI timestamps are formatted as 'YYYY-MM-DD HH:MM:SS.ffffff'
            # Slice to 26 characters to keep exactly 6 sub-second digits so
            # strptime's %f directive (which requires 1-6 digits) always succeeds.
            dts.append(datetime.strptime(line[:26], "%Y-%m-%d %H:%M:%S.%f"))
    return dts


# ---------------------------------------------------------------------------
# Lazy event generators — one message at a time to keep memory bounded
# ---------------------------------------------------------------------------

def _camera_stream(
    img_dts, img_files, image_dir, util, camera_pad,
    frame_camera, image_topic, camera_info_topic
) -> Generator[_Event, None, None]:
    """Yield (ts_ns, topic, serialised_bytes) for camera images and infos."""
    for dt, fn in zip(img_dts, img_files):
        ts_ns = _dt_to_ns(dt)
        stamp = _make_stamp(ts_ns)
        img_msg = _make_image(os.path.join(image_dir, fn), frame_camera, stamp)
        ci_msg  = _make_camera_info(util, camera_pad, frame_camera, stamp)
        yield ts_ns, image_topic,       serialize_message(img_msg)
        yield ts_ns, camera_info_topic, serialize_message(ci_msg)


def _velo_stream(
    velo_dts, velo_files, velo_dir, frame_lidar, lidar_topic
) -> Generator[_Event, None, None]:
    """Yield (ts_ns, topic, serialised_bytes) for velodyne scans."""
    for dt, fn in zip(velo_dts, velo_files):
        ts_ns = _dt_to_ns(dt)
        stamp = _make_stamp(ts_ns)
        pcl_msg = _make_point_cloud(os.path.join(velo_dir, fn), frame_lidar, stamp)
        yield ts_ns, lidar_topic, serialize_message(pcl_msg)


def _imu_stream(
    imu_dts, imu_files, imu_dir, frame_imu, imu_topic
) -> Generator[_Event, None, None]:
    """Yield (ts_ns, topic, serialised_bytes) for IMU packets."""
    for dt, fn in zip(imu_dts, imu_files):
        ts_ns = _dt_to_ns(dt)
        stamp = _make_stamp(ts_ns)
        imu_msg = _make_imu(os.path.join(imu_dir, fn), frame_imu, stamp)
        yield ts_ns, imu_topic, serialize_message(imu_msg)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert KITTI raw_synced data to a ROS 2 bag (sqlite3)"
    )
    parser.add_argument("--kitti-dir", required=True,
                        help="Base KITTI directory (contains the date subfolder)")
    parser.add_argument("--date", required=True,
                        help="KITTI recording date, e.g. 2011_10_03")
    parser.add_argument("--drive", required=True,
                        help="KITTI drive number (zero-padded), e.g. 0027")
    parser.add_argument("--camera", choices=["left", "right"], default="left",
                        help="Which colour camera to include (default: left)")
    parser.add_argument("--output-bag", required=True,
                        help="Destination directory for the ros2bag")
    parser.add_argument("--image-topic",       default="/camera/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/camera_info")
    parser.add_argument("--lidar-topic",       default="/points_raw")
    parser.add_argument("--imu-topic",         default="/imu/data")
    args = parser.parse_args()

    camera_idx = _CAMERA_IDX[args.camera]
    camera_pad = f"{camera_idx:02}"
    frame_camera = "camera"
    frame_lidar = "lidar"
    frame_imu = "imu_link"

    # Use pykitti only for path resolution and calibration parsing
    kitti = pykitti.raw(args.kitti_dir, args.date, args.drive)
    util = pykitti.utils.read_calib_file(
        os.path.join(kitti.calib_path, "calib_cam_to_cam.txt")
    )

    data_path = kitti.data_path  # …/<date>/<date>_drive_<drive>_sync

    image_dir  = os.path.join(data_path, f"image_{camera_pad}", "data")
    image_ts   = os.path.join(data_path, f"image_{camera_pad}", "timestamps.txt")
    velo_dir   = os.path.join(data_path, "velodyne_points", "data")
    velo_ts    = os.path.join(data_path, "velodyne_points", "timestamps.txt")
    imu_dir    = os.path.join(data_path, "oxts", "data")
    imu_ts     = os.path.join(data_path, "oxts", "timestamps.txt")

    img_files  = sorted(os.listdir(image_dir))
    velo_files = sorted(os.listdir(velo_dir))
    imu_files  = sorted(os.listdir(imu_dir))

    img_dts    = _read_timestamps(image_ts)
    velo_dts   = _read_timestamps(velo_ts)
    imu_dts    = _read_timestamps(imu_ts)

    print(f"[kitti2ros2bag] Camera ({args.camera}) frames : {len(img_files)}")
    print(f"[kitti2ros2bag] Velodyne scans              : {len(velo_files)}")
    print(f"[kitti2ros2bag] IMU packets                 : {len(imu_files)}")

    # ------------------------------------------------------------------
    # Open the ros2bag writer
    # ------------------------------------------------------------------
    os.makedirs(args.output_bag, exist_ok=True)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=args.output_bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    def _add_topic(name: str, msg_type: str) -> None:
        writer.create_topic(rosbag2_py.TopicMetadata(
            name=name, type=msg_type, serialization_format="cdr"
        ))

    _add_topic(args.image_topic,       "sensor_msgs/msg/Image")
    _add_topic(args.camera_info_topic, "sensor_msgs/msg/CameraInfo")
    _add_topic(args.lidar_topic,       "sensor_msgs/msg/PointCloud2")
    _add_topic(args.imu_topic,         "sensor_msgs/msg/Imu")

    # ------------------------------------------------------------------
    # Merge all three streams in timestamp order and write on-the-fly.
    # Using generators + heapq.merge keeps memory bounded to O(1) per
    # stream — only one message per stream is held in memory at a time,
    # regardless of sequence length.  This avoids the >6 GB peak RAM that
    # would result from buffering all serialised camera frames at once.
    # ------------------------------------------------------------------
    total_events = len(img_files) * 2 + len(velo_files) + len(imu_files)
    print(f"[kitti2ros2bag] Writing {total_events} events to '{args.output_bag}' ...")

    cam_gen  = _camera_stream(
        img_dts, img_files, image_dir, util, camera_pad,
        frame_camera, args.image_topic, args.camera_info_topic,
    )
    velo_gen = _velo_stream(velo_dts, velo_files, velo_dir, frame_lidar, args.lidar_topic)
    imu_gen  = _imu_stream(imu_dts, imu_files, imu_dir, frame_imu, args.imu_topic)

    pbar = progressbar.ProgressBar(max_value=total_events)
    for count, (ts_ns, topic, data) in enumerate(
        heapq.merge(cam_gen, velo_gen, imu_gen, key=lambda e: e[0])
    ):
        writer.write(topic, data, ts_ns)
        pbar.update(count)
    pbar.finish()

    print(f"[kitti2ros2bag] Done → {args.output_bag}")


if __name__ == "__main__":
    main()

