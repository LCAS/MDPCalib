# MDPCalib
[**arXiv**](https://arxiv.org/abs/2404.17298) | [**IEEE Xplore**](https://ieeexplore.ieee.org/document/10694691) | [**Website**](https://calibration.cs.uni-freiburg.de/) | [**Video**](https://youtu.be/L1MwAenzd6g)

This repository is the official implementation of the paper:

> **Automatic Target-Less Camera-LiDAR Calibration from Motion and Deep Point Correspondences**
>
> [Kürsat Petek](http://www2.informatik.uni-freiburg.de/~petek/)&ast;, [Niclas Vödisch](https://vniclas.github.io/)&ast;, [Johannes Meyer](http://www2.informatik.uni-freiburg.de/~meyerjo/), [Daniele Cattaneo](https://rl.uni-freiburg.de/people/cattaneo), [Abhinav Valada](https://rl.uni-freiburg.de/people/valada), and [Wolfram Burgard](https://www.utn.de/person/wolfram-burgard/). <br>
> &ast;Equal contribution. <br>
>
> *IEEE Robotics and Automation Letters*, vol. 9, issue 11, pp. 9978-9985, November 2024.

<p align="center">
  <img src="./assets/mdpcalib_overview.png" alt="Overview of MDPCalib approach" width="800" />
</p>

If you find our work useful, please consider citing our paper:
```
@article{petek2024mdpcalib,
  author={Petek, Kürsat and Vödisch, Niclas and Meyer, Johannes and Cattaneo, Daniele and Valada, Abhinav and Burgard, Wolfram},
  journal={IEEE Robotics and Automation Letters},
  title={Automatic Target-Less Camera-LiDAR Calibration From Motion and Deep Point Correspondences},
  year={2024},
  volume={9},
  number={11},
  pages={9978-9985}
}
```


## 📔 Abstract

Sensor setups of robotic platforms commonly include both camera and LiDAR as they provide complementary information. However, fusing these two modalities typically requires a highly accurate calibration between them. In this paper, we propose MDPCalib which is a novel method for camera-LiDAR calibration that requires neither human supervision nor any specific target objects. Instead, we utilize sensor motion estimates from visual and LiDAR odometry as well as deep learning-based 2D-pixel-to-3D-point correspondences that are obtained without in-domain retraining. We represent the camera-LiDAR calibration as a graph optimization problem and minimize the costs induced by constraints from sensor motion and point correspondences. In extensive experiments, we demonstrate that our approach yields highly accurate extrinsic calibration parameters and is robust to random initialization. Additionally, our approach generalizes to a wide range of sensor setups, which we demonstrate by employing it on various robotic platforms including a self-driving perception car, a quadruped robot, and a UAV.


## 👩‍💻 Code

### 💻 Development

#### Docker 🐋

Tested with `Docker version 28.0.1` and `Docker Compose version v2.33.1`.

Pre-built images are published automatically to the GitHub Container Registry on every push to `main` and on every version tag:

```
ghcr.io/lcas/mdpcalib:latest       # latest build from main
ghcr.io/lcas/mdpcalib:<version>    # e.g. ghcr.io/lcas/mdpcalib:1.2.3
```

The compose stack includes a VNC service ([`lcas.lincoln.ac.uk/vnc`](https://github.com/LCAS/ros2_pkg_template)) so no local X11 display or `xhost` configuration is required. The VNC viewer is accessible at **http://localhost:5801**.

##### Data prerequisites

Two separate host directories are required — keep them distinct to avoid conflicts:

| Directory | Configured by | Contents |
|---|---|---|
| Calibration data | `CALIBRATION_DATA_PATH` | CMRNext model weights (auto-downloaded if absent), calibration output |
| ROS bags | `ROSBAG_PATH` | Your ros2bag folder(s) |

- **CMRNext model weights** are downloaded automatically on first run from  
  `https://calibration.cs.uni-freiburg.de/downloads/cmrnext_weights.zip`  
  and stored in `$CALIBRATION_DATA_PATH/cmrnext/`. No manual download needed.
- **ros2bag** — place your rosbag2 folder inside `$HOME/rosbags/` (or set `ROSBAG_PATH` in `.env`)

##### Quick start — calibrate from a ros2bag

The `bag-player` service plays a ros2bag folder automatically. The bag loops by default so the calibration stack has enough time to complete. CMRNext model weights are downloaded automatically on first startup.

1. Download the compose file and example environment:
   ```bash
   curl -O https://raw.githubusercontent.com/LCAS/MDPCalib/main/docker-compose.yaml
   curl -O https://raw.githubusercontent.com/LCAS/MDPCalib/main/.env.example
   cp .env.example .env
   ```
2. Edit `.env`:
   - Set `CALIBRATION_DATA_PATH` to an empty directory for calibration output (weights are downloaded there automatically).
   - Place your rosbag2 folder(s) in `$HOME/rosbags/` (or override `ROSBAG_PATH`).
   - Set the ROS 2 topic names to match what is recorded in your bag.
3. Start the full stack (VNC + bag player + calibration):
   ```bash
   docker compose up
   ```

The calibration starts automatically once the bag begins playing. Logs are written to
`$CALIBRATION_DATA_PATH/runtime_logs/`. The final calibration result is written to
`$CALIBRATION_DATA_PATH/calibration/ros2/extrinsics.yaml`.

Open **http://localhost:5801** in a browser to view the RViz / GUI output via VNC.

##### Quick start — calibrate from KITTI

The `bag-player` service also supports a built-in KITTI mode that downloads the KITTI raw_synced dataset automatically, converts it to a ROS 2 bag, and plays it. All data is stored in the `kitti_data` named Docker volume — no host bind mount or manual download is required.

> **⚠ Note:** The initial download is ~9 GB (raw sync + calibration + odometry velodyne). After the first run the data is cached in the Docker volume; subsequent `docker compose up` invocations skip both the download and the conversion step.

1. Copy and edit `.env` as above, then add / uncomment:
   ```ini
   PLAYBACK_MODE=kitti
   # Use the Velodyne HDL-64E FAST-LO config for KITTI:
   FAST_LO_CONFIG_FILE=/root/catkin_ws/src/mdpcalib/FAST_LO/config/velodyne.yaml
   ```
2. Start the stack:
   ```bash
   docker compose up
   ```
   The bag-player will download the KITTI data into the `kitti_data` volume, convert it to a
   ROS 2 bag, and start playback. The calibration stack starts automatically in `mdpcalib`.

To change the KITTI sequence, override `KITTI_DATE`, `KITTI_DRIVE`, and `KITTI_SEQUENCE` in `.env`
(see `.env.example` for details and the [KITTI mapping table](https://github.com/tomas789/kitti2bag/issues/10#issuecomment-352962278)).

The `.env` file controls the following variables (see [`.env.example`](.env.example) for full documentation):

| Variable | Default | Description |
|---|---|---|
| `CALIBRATION_DATA_PATH` | `./calibration-data` | Host path for calibration output + model weights (auto-downloaded) |
| `PLAYBACK_MODE` | `ros2bag` | `ros2bag` — play from file; `kitti` — download and play KITTI |
| `ROSBAG_PATH` | `$HOME/rosbags` | Host directory mounted as `/rosbags` in the bag-player (ros2bag mode) |
| `BAG_PATH` | `/rosbags` | Path inside container to the rosbag2 folder (ros2bag mode) |
| `BAG_LOOP` | `true` | Set to `false` to play once and exit |
| `BAG_RATE` | `1.0` | Playback rate multiplier |
| `KITTI_DATE` | `2011_10_03` | KITTI recording date (kitti mode) |
| `KITTI_DRIVE` | `0027` | KITTI drive number, zero-padded (kitti mode) |
| `KITTI_CAMERA` | `left` | Colour camera: `left` or `right` (kitti mode) |
| `KITTI_SEQUENCE` | `00` | Odometry sequence for motion-compensated velodyne (kitti mode) |
| `ROS2_CAMERA_IMAGE_TOPIC` | `/camera/image_raw` | Camera image topic |
| `ROS2_CAMERA_INFO_TOPIC` | `/camera/camera_info` | Camera info topic |
| `ROS2_LIDAR_POINTS_TOPIC` | `/points_raw` | LiDAR point cloud topic |
| `ROS2_IMU_TOPIC` | `/imu/data` | IMU topic |
| `LIDAR_FRAME_ID` | `lidar` | LiDAR frame ID in the exported YAML |
| `CAMERA_FRAME_ID` | `camera` | Camera frame ID in the exported YAML |

##### Building the images locally (for development)

The compose file includes `build:` contexts for both the `mdpcalib` and `bag-player` services, so you can build everything from source:

```bash
docker compose build
docker compose up
```

- Connect to a running container: `docker compose exec -it mdpcalib bash`


#### Githooks ✅

We used multiple githooks during the development of this code. You can set up them with the following steps:

1. Create a venv or conda environment. Make sure to source that before committing.
2. Install requirements: `pip install -r requirements.txt`
3. Install CMRNext requirements for pylint to work: `pip install -r src/CMRNext/rquirements.txt`
4. Install [pre-commit](https://pre-commit.com/) githook scripts: `pre-commit install`

Python formatter ([yapf](https://github.com/google/yapf), [iSort](https://github.com/PyCQA/isort)) settings can be set in [pyproject.toml](pyproject.toml). C++ formatter ([ClangFormat](https://clang.llvm.org/docs/ClangFormat.html)) settings are in [.clang-format](.clang-format).

This will automatically run the pre-commit hooks on every commit. You can skip this using the `--no-verify` flag, e.g., `commit -m "Update node" --no-verify`.
To run the githooks on all files independent of doing a commit, use `pre-commit run --all-files`


### 💾 Data

In the public release of our MDPCalib, we provide instructions for running camera-LiDAR calibration on the KITTI dataset.

#### Generating a KITTI rosbag 🐈

> &#x26a0;&#xfe0f; **Note:** If you created the rosbag before commit [19645fb](https://github.com/robot-learning-freiburg/MDPCalib/tree/19645fb5b224d45b6a56c6daa94ed873e911f10c) (Mar 13, 2025), please re-create it following the updated instructions.

- Install the provided `kitti2bag` package from within the package directory: `pip install -e .`
- Download the raw "synced+rectified" and "calibration" data for an odometry sequence. The mapping is available [here](https://github.com/tomas789/kitti2bag/issues/10#issuecomment-352962278). In the following, we will assume that the files will be downloaded to `/data/kitti/`.
- E.g., for sequence 00, download the residential data `2011_10_03_drive_0027`:
    - https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_10_03_drive_0027/2011_10_03_drive_0027_sync.zip
    - https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_10_03_calib.zip
- Unzip both files.
    - `unzip 2011_10_03_calib.zip`
    - `unzip 2011_10_03_drive_0027/2011_10_03_drive_0027_sync.zip`
- Now, we replace the raw velodyne data with the point clouds from the odometry benchmark as these have been motion compensated. Download the "velodyne laser data":
    - https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_velodyne.zip
- It is sufficient to unzip the same sequence as downloaded above, e.g.,
    - `rm -rf 2011_10_03/2011_10_03_drive_0027_sync/velodyne_points/data`
    - `unzip -j -d 2011_10_03/2011_10_03_drive_0027_sync/velodyne_points/data data_odometry_velodyne.zip 'dataset/sequences/00/velodyne/*.bin'`
- Then run: `kitti2bag -t 2011_10_03 -r 0027 raw_synced`.
- This will result in a rosbag: `kitti_2011_10_03_drive_0027_synced.bag`.
- For the following instructions, we will assume that the rosbag is located at `/data/kitti/`.
    - The folder can be changed in the launchers [play_bag_kitti_left.launch](src/pose_synchronizer/launch/play_bag_kitti_left.launch) and [play_bag_kitti_right.launch](src/pose_synchronizer/launch/play_bag_kitti_right.launch).

#### Downloading model weights 🏋️

When using the Docker compose stack, model weights are **downloaded automatically** on first run
from `https://calibration.cs.uni-freiburg.de/downloads/cmrnext_weights.zip` and stored in
`$CALIBRATION_DATA_PATH/cmrnext/`.

For manual / non-Docker runs, download the weights and store them under `/data/cmrnext`:
- Model weights: https://calibration.cs.uni-freiburg.de/downloads/cmrnext_weights.zip

### 🏃 Running the calibration

For changing the configuration settings, please consult the [config file](src/calib_cfg/config/config.yaml).
Note that there we also specify the name of the experiment that creates an output folder under `/root/catkin_ws/src/mdpcalib/experiments`.
To prevent overwriting previous results, names of experiments must be unique.

Please execute the following steps in separate terminals:

1. Start a roscore: `roscore`
2. [Optional] Start rviz: `roscd pose_synchronizer; rviz -d rviz/combined.rviz`
3. Launch CMRNext: `roslaunch cmrnext cmrnext_kitti.launch`
4. Launch the optimizer: `roslaunch optimization_utils optimizer.launch`
5. Launch the data synchronizer: `roslaunch pose_synchronizer pose_synchronizer_fastlo_kitti.launch`
6. Wait until the ORB vocabulary has been loaded.
7. Play the data (left camera): `roslaunch pose_synchronizer play_bag_kitti_left.launch`
    - Alternatively (right camera): `roslaunch pose_synchronizer play_bag_kitti_right.launch`
8. [Optional] Once the fine calibration is finished, you could stop playing the data (Ctrl + c).

### 🤖 Live ROS 2 Calibration

The repository now also contains a non-interactive workflow for calibrating against a live external ROS 2 system:

- `run_mdpcalib_ros2.sh` starts the container, mounts `/data`, launches `roscore`, starts `ros1_bridge`, runs the calibration stack, and exits once the final calibration has been produced.
- `src/pose_synchronizer/launch/ros2_live_calibration.launch` starts the full live pipeline inside ROS 1: camera passthrough adapter, ORB-SLAM3, FAST-LO, pose synchronizer, CMRNext, optimizer, and a completion monitor.
- The optimizer exports the final refined transform to a ROS 2 parameter YAML at `/data/calibration/ros2/extrinsics.yaml` by default and also publishes it on `/optimizer/refined_transform`, which the bridge can expose back to ROS 2.

The live pipeline expects the external ROS 2 system to publish at least:

- a camera image topic
- a matching camera info topic
- a LiDAR point cloud topic
- an IMU topic for FAST-LO

Common overrides are passed as environment variables before running `./run_mdpcalib_ros2.sh`:

- `ROS2_CAMERA_IMAGE_TOPIC` and `ROS2_CAMERA_INFO_TOPIC`: ROS 2 camera topics to bridge into the container
- `ROS2_LIDAR_POINTS_TOPIC` and `ROS2_IMU_TOPIC`: ROS 2 LiDAR and IMU topics
- `ORB_SLAM3_SETTINGS_FILE`: ORB-SLAM3 camera settings YAML for your robot camera
- `FAST_LO_CONFIG_FILE`: FAST-LO LiDAR configuration YAML matching your sensor
- `LIDAR_FRAME_ID` and `CAMERA_FRAME_ID`: frame IDs written into `/optimizer/refined_transform` and the exported ROS 2 YAML
- `ROS2_CALIBRATION_OUTPUT_PATH`: output path for the exported ROS 2 YAML inside the container
- `CALIBRATION_TIMEOUT_SEC`: watchdog for unattended runs
- `MDPCALIB_CLEAN_PREVIOUS_RUNS`: when not set to `false`, the live runner clears `/data/cache/*` and `/data/experiments/*` before starting, matching the legacy tmux workflow
- `MDPCALIB_REBUILD_WORKSPACE`: when not set to `false`, the live runner executes `catkin build -cs` before launching so the mounted `./src` tree and the container binaries stay aligned

Example:

```bash
export ROS2_CAMERA_IMAGE_TOPIC=/front_left/image_raw
export ROS2_CAMERA_INFO_TOPIC=/front_left/camera_info
export ROS2_LIDAR_POINTS_TOPIC=/os_cloud_node/points
export ROS2_IMU_TOPIC=/imu/data
export ORB_SLAM3_SETTINGS_FILE=/data/config/front_left_orbslam3.yaml
export FAST_LO_CONFIG_FILE=/data/config/ouster_fast_lo.yaml
export LIDAR_FRAME_ID=os_sensor
export CAMERA_FRAME_ID=front_left_camera_optical_frame
./run_mdpcalib_ros2.sh
```

The exported YAML is ROS 2 parameter-file compatible and includes:

- `mdpcalib_parent_frame`
- `mdpcalib_child_frame`
- `mdpcalib_translation_xyz`
- `mdpcalib_rotation_xyzw`
- `mdpcalib_transform_matrix_row_major`


## 👩‍⚖️  License

For academic usage, the code is released under the [GPLv3](https://www.gnu.org/licenses/gpl-3.0.en.html) license. Components of other works are released under their original license.
For any commercial purpose, please contact the authors.


## 🙏 Acknowledgment

In our work and experiments, we have used components from many other works. We thank the authors for open-sourcing their code. In no specific order, we list source repositories:
- CMRNext: https://github.com/robot-learning-freiburg/CMRNext
- ORB SLAM3 ROS Wrapper: https://github.com/thien94/orb_slam3_ros_wrapper
- kitti2bag: https://github.com/tomas789/kitti2bag
- FAST-LO: https://github.com/hku-mars/LiDAR_IMU_Init


This work was funded by the German Research Foundation (DFG) Emmy Noether Program grant No 468878300 and an academic grant from NVIDIA.
<br><br>
<p float="left">
  <a href="https://www.dfg.de/en/research_funding/programmes/individual/emmy_noether/index.html"><img src="./assets/dfg_logo.png" alt="DFG logo" height="100"/></a>
</p>
