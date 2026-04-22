# MDPCalib Developer Handoff

This document is for the next developer taking over the live ROS 2 calibration workflow in this repository.

It explains:

- what the current architecture actually is
- how the ROS 2 workflow is wired into a mostly ROS 1 codebase
- how the calibration algorithm works at a high level
- which files matter for each part of the system
- how data flows through the runtime
- what is configurable
- what is currently fragile or surprising

This is the document to read before making changes.


## 1. The Most Important Mental Model

The live "ROS 2 calibration workflow" is not a native ROS 2 calibration stack.

It is:

1. an external ROS 2 robot publishing sensor topics
2. a Docker container running mostly ROS 1 Noetic components
3. `ros1_bridge` connecting ROS 2 sensor topics into the ROS 1 computation graph
4. a ROS 1 calibration pipeline doing the actual work
5. the final result exported both as:
   - a ROS 1 `TransformStamped` on `/optimizer/refined_transform`
   - a ROS 2-compatible parameter YAML written to `/data/calibration/ros2/extrinsics.yaml` by default

So when modifying the live workflow, think of it as:

- ROS 2 at the edges
- ROS 1 in the middle
- file export as the durable output


## 2. What the System Is Trying to Do

The goal is to estimate the extrinsic transform between a LiDAR and a camera without a calibration target.

At a high level the algorithm uses two kinds of information:

- motion consistency between visual odometry and LiDAR odometry
- learned 2D-3D image-to-point-cloud correspondences from CMRNext

The rough sequence is:

1. ORB-SLAM3 estimates camera motion.
2. FAST-LO estimates LiDAR motion.
3. `pose_synchronizer` time-aligns camera and LiDAR data and writes synchronized mini-rosbags to disk.
4. `optimization_utils` uses synchronized pose bags to compute an initial camera-LiDAR rotation and a translation scale-like initialization via motion constraints.
5. That initial transform is published to CMRNext.
6. CMRNext uses the initial transform to project LiDAR data into the image and predict pixel-to-3D correspondences.
7. `optimization_utils` refines the extrinsic using both:
   - motion constraints
   - image reprojection constraints from the correspondences
8. The final refined extrinsic is published and exported to YAML.


## 3. Entry Points You Should Know First

If you only read a few files first, read these:

- `run_mdpcalib_ros2.sh`
- `src/pose_synchronizer/scripts/start_ros2_calibration.sh`
- `src/pose_synchronizer/launch/ros2_live_calibration.launch`
- `src/pose_synchronizer/src/pose_synchronizer.cpp`
- `src/CMRNext/src/cmrnext/cmrnext_ros_node.py`
- `src/optimization_utils/src/optimizer.cpp`
- `src/calib_cfg/config/config.yaml`
- `README.md`

These files define almost the entire live workflow.


## 4. Runtime Architecture

### 4.1 Host-side launcher

File:

- `run_mdpcalib_ros2.sh`

Purpose:

- starts the container with `/data` mounted
- forwards the important environment variables into the container
- executes the in-container live calibration script

Key point:

- this is a thin wrapper only
- all real orchestration happens inside the container


### 4.2 In-container orchestrator

File:

- `src/pose_synchronizer/scripts/start_ros2_calibration.sh`

Purpose:

- clears stale cache and experiment state
- optionally rebuilds the catkin workspace
- starts `roscore`
- starts `ros1_bridge`
- launches the full calibration graph

Important behavior:

- `MDPCALIB_CLEAN_PREVIOUS_RUNS=true` by default
  - clears `/data/cache/*`
  - clears `/data/experiments/*`
- `MDPCALIB_REBUILD_WORKSPACE=true` by default
  - runs `catkin build -cs`
  - this is important because the container bind-mounts `./src`, so the runtime source tree can drift from the image-built binaries

Why the rebuild matters:

- `docker-compose.yaml` mounts `./src:/root/catkin_ws/src/mdpcalib`
- if the host source changes after the image is built, the mounted source and compiled binaries may no longer match
- rebuilding inside the container keeps them aligned


### 4.3 ROS graph launcher

File:

- `src/pose_synchronizer/launch/ros2_live_calibration.launch`

This launch file starts the live graph:

- `camera_passthrough_adapter.py`
- ORB-SLAM3 mono node
- FAST-LO
- `pose_synchronizer`
- `optimization_utils`
- `cmrnext_ros_node.py`
- `calibration_result_monitor.py`

This launch file is the best single-file overview of the current live system.


## 5. End-to-End Dataflow

This is the actual runtime dataflow.

### Step 1. External ROS 2 robot publishes raw topics

Expected external inputs:

- camera image
- camera info
- LiDAR point cloud
- IMU

Configured via env vars:

- `ROS2_CAMERA_IMAGE_TOPIC`
- `ROS2_CAMERA_INFO_TOPIC`
- `ROS2_LIDAR_POINTS_TOPIC`
- `ROS2_IMU_TOPIC`


### Step 2. `ros1_bridge` exposes them inside the container

Started by:

- `start_ros2_calibration.sh`

Command:

- `ros2 run ros1_bridge dynamic_bridge --bridge-all-topics`

Important notes:

- the live pipeline depends on ROS message types that `ros1_bridge` can translate
- standard types like `sensor_msgs/Image`, `CameraInfo`, `PointCloud2`, and `Imu` are expected to bridge cleanly
- custom `calib_msgs/*` stay internal to ROS 1 and do not need ROS 2 equivalents


### Step 3. Camera passthrough adapter republishes camera topics

File:

- `src/pose_synchronizer/scripts/camera_passthrough_adapter.py`

Purpose:

- republishes input camera image and camera info to the legacy topic names expected by the rest of the stack

Default mapping:

- input image -> `/camera_undistorted/image`
- input camera info -> `/camera_undistorted/camera_info`

Important caveat:

- despite the topic name, this node does not undistort anything
- it is currently a republisher only
- if your camera needs true rectification/undistortion, that must be done upstream or replaced here


### Step 4. ORB-SLAM3 estimates camera motion

Files:

- `src/orb_slam3_ros_wrapper/launch/...`
- in live mode the node is launched directly in `ros2_live_calibration.launch`

Inputs:

- `/camera_undistorted/image`

Output used by the calibration stack:

- `/orb_slam3/camera_pose`

Developer note:

- the live launch currently uses the monocular ORB-SLAM3 wrapper
- camera intrinsics/settings come from `ORB_SLAM3_SETTINGS_FILE`


### Step 5. FAST-LO estimates LiDAR motion

Files:

- `src/FAST_LO/src/FAST_LO.cpp`
- sensor-specific config YAMLs under `src/FAST_LO/config/`

Inputs:

- LiDAR point cloud topic
- IMU topic

Outputs used by calibration:

- `/odom`
- `/cloud_registered_body`

Important note:

- FAST-LO topic names are injected through ROS params in `ros2_live_calibration.launch`
- sensor behavior depends heavily on the chosen FAST-LO config file


### Step 6. `pose_synchronizer` aligns camera and LiDAR streams

Files:

- `src/pose_synchronizer/src/node.cpp`
- `src/pose_synchronizer/src/pose_synchronizer.cpp`
- `src/pose_synchronizer/include/pose_synchronizer/pose_synchronizer.h`

Inputs:

- camera pose
- camera image
- LiDAR odometry
- LiDAR point cloud

Logic:

- camera timestamps are the reference
- LiDAR poses are interpolated to the image timestamps
- LiDAR clouds are optionally motion-compensated to the image timestamps
- synchronized tuples are written to temporary rosbag files in `/data/cache`

Outputs:

- `/synced_data/filename`
  - full synchronized bag with image, point cloud, and poses
- `/synced_data/filename_poses`
  - smaller bag containing only poses

Important thresholds from code:

- buffer duration: `4.0` seconds
- max camera/LiDAR sync delta: `0.025` seconds

Important switch:

- `disable_pose_synchronizer`
  - if `true`, the point cloud is not transformed to the camera timestamp
  - useful for datasets or sensors where external timing/compensation is already handled


### Step 7. `optimization_utils` computes the initial transform

Files:

- `src/optimization_utils/src/node.cpp`
- `src/optimization_utils/src/optimizer.cpp`

Inputs:

- synchronized pose bag filenames from `/synced_data/filename_poses`
- camera intrinsics from `/camera_undistorted/camera_info`
- optional `/gt_extrinsics`

High-level initialization logic:

- load sequential synchronized pose bags
- compute camera and LiDAR relative motions
- build a Ceres problem with:
  - rotation constraints
  - translation constraints
- solve for an initial extrinsic estimate

Important behavior:

- the first `starting_pose` frames are skipped
- ORB-SLAM3 origin poses are ignored
- after initial solve, translation is temporarily set to zero before sending the initial transform to CMRNext

Why translation is zeroed:

- the code comments state that CMRNext is robust enough to work with zero translation
- the initial transform is mainly used to give CMRNext a usable orientation prior

After the initial step:

- the optimizer publishes `/optimizer/initial_transform`
- it also publishes `/optimizer/initial_transform_meta` containing the first and last sequence IDs used for initialization

That metadata tells CMRNext which synchronized bags are relevant for refinement.


### Step 8. CMRNext generates 2D-3D correspondences

File:

- `src/CMRNext/src/cmrnext/cmrnext_ros_node.py`

Inputs:

- `/synced_data/filename`
- `/optimizer/initial_transform`
- `/optimizer/initial_transform_meta`
- `/camera_undistorted/camera_info`

Logic:

- cache only the synchronized bags that fall inside the optimizer’s initialization window
- subsample bag candidates according to `number_poses` and `number_image_pcl_pairs`
- discard high-yaw transitions
- use the initial transform to project LiDAR points into image space
- run the learned CMRNext model to estimate correspondences
- subsample correspondences according to `amount_correspondences`

Output:

- `/cmrnext/correspondences`

Recent hardening:

- zero-norm LiDAR quaternions are now detected and skipped instead of crashing the node

Developer note:

- CMRNext also prints an average predicted calibration for comparison, but that value is not what the optimizer exports as the final result
- the actual final calibration comes from `optimization_utils`


### Step 9. `optimization_utils` refines the transform

File:

- `src/optimization_utils/src/optimizer.cpp`

Refinement logic:

- reuse trajectory constraints from synchronized camera/LiDAR motion
- add image reprojection constraints from CMRNext correspondences
- solve a second Ceres problem
- invert the resulting pose matrix into the exported convention

Outputs:

- `/optimizer/refined_transform`
- results logs under `/data/experiments/<run_name>/results`
- ROS 2 parameter YAML under:
  - default: `/data/calibration/ros2/extrinsics.yaml`
  - or the configured `ROS2_CALIBRATION_OUTPUT_PATH`


### Step 10. Completion monitor stops the run

File:

- `src/pose_synchronizer/scripts/calibration_result_monitor.py`

Purpose:

- waits for `/optimizer/refined_transform`
- shuts down once the final result is seen
- fails if the timeout expires

This is what makes the live workflow non-interactive.


## 6. Transform Semantics

This is one of the easiest places for a new developer to get confused.

The exported final transform is intended to represent:

- camera pose in the LiDAR frame

The code path does several inversions internally:

- the initialization stage computes a form of LiDAR pose in the camera frame
- then inverts it so the rest of the stack can use camera-in-LiDAR convention
- the refinement stage also inverts the pose matrix before exporting

Final published/exported labels:

- parent frame = `LIDAR_FRAME_ID`
- child frame = `CAMERA_FRAME_ID`

These labels are written into:

- `/optimizer/refined_transform`
- the exported ROS 2 YAML

If results look transposed or backwards, this frame convention is the first place to inspect.


## 7. Output Files and Directories

### Runtime cache

Location:

- `/data/cache`

Contents:

- synchronized mini-rosbags

Lifecycle:

- created by `pose_synchronizer`
- deleted by `Optimizer::~Optimizer()` on normal shutdown
- proactively cleared at startup by `start_ros2_calibration.sh`


### Experiment outputs

Location:

- `/data/experiments/<run_name>/`

Subdirectories:

- `configs`
- `launch`
- `results`
- `visualizations`

Important file:

- `results/results.txt`

This contains timing and transform logs from the optimizer.


### Exported ROS 2 YAML

Default location:

- `/data/calibration/ros2/extrinsics.yaml`

Created by:

- `IOUtils::writeRos2CalibrationYaml()` in `src/optimization_utils/src/io_utils.cpp`

Schema:

- `mdpcalib_parent_frame`
- `mdpcalib_child_frame`
- `mdpcalib_translation_xyz`
- `mdpcalib_rotation_xyzw`
- `mdpcalib_transform_matrix_row_major`

This file is ROS 2 parameter-file compatible.


## 8. Key Configuration Surface

### 8.1 Global calibration config

File:

- `src/calib_cfg/config/config.yaml`

Important fields:

- `optimization.number_poses`
  - number of synchronized poses used in the motion stage
- `optimization.starting_pose`
  - number of initial synchronized poses skipped
- `optimization.number_image_pcl_pairs`
  - number of image/point-cloud pairs used for refinement
- `cmrnext.amount_correspondences`
  - percent of correspondences kept from each processed pair
- `cmrnext.rotation_threshold`
  - skip high-yaw pair candidates
- `io.cache_folder`
  - temporary synchronized rosbag cache
- `io.path_base`
  - base output path, normally `/data`
- `io.run_name`
  - experiment folder name under `/data/experiments`

Important gotcha:

- `run_name` must not collide with an existing experiment folder unless startup cleanup removes it first


### 8.2 Optimizer config

File:

- `src/optimization_utils/config/config.yaml`

This controls Ceres settings such as:

- max iterations
- tolerances
- linear solver type


### 8.3 Runtime environment variables

The live path is controlled mainly through environment variables passed by `run_mdpcalib_ros2.sh`.

Most important:

- sensor topics
- ORB-SLAM3 settings file
- FAST-LO config file
- frame IDs
- output path
- timeout
- whether to rebuild
- whether to clear old runs

See `README.md` for the current list and example.


## 9. Container and Build Model

### Docker behavior

Files:

- `Dockerfile`
- `docker-compose.yaml`

Important facts:

- base runtime is Ubuntu 20.04
- ROS 1 Noetic is installed
- ROS 2 Foxy and `ros1_bridge` are installed
- GPU-enabled image with CUDA and Torch is used for CMRNext
- host networking is enabled in `docker-compose.yaml`
- the repo `./src` directory is bind-mounted into `/root/catkin_ws/src/mdpcalib`

Why host networking matters:

- it simplifies ROS 1 and ROS 2 discovery between host and container
- it also means topic/network collisions are easier to create

Why bind-mounting matters:

- source changes are immediately visible in the container
- compiled binaries are not automatically updated
- that is why the live start script rebuilds by default


## 10. Debugging Guide

When the workflow fails, check in this order.

### No ROS 2 topics appear inside the container

Check:

- `ros1_bridge` started successfully
- topic types are bridgeable standard messages
- `ROS_DOMAIN_ID` matches the robot
- network discovery is working with host networking


### ORB-SLAM3 never initializes

Check:

- camera image is arriving on `/camera_undistorted/image`
- camera settings YAML matches the actual camera
- image encoding is compatible
- scene has enough visual texture and motion

Symptom:

- optimizer keeps ignoring origin poses


### FAST-LO produces bad or no odometry

Check:

- LiDAR topic and IMU topic are correct
- chosen FAST-LO config matches the sensor
- IMU scaling/frame assumptions are correct
- timestamps are sane


### `pose_synchronizer` never emits synchronized bags

Check:

- camera and LiDAR timestamps are within the `0.025` second sync threshold
- both odometry streams are actually publishing
- point cloud and odometry topics match the launch params


### CMRNext crashes or caches nothing useful

Check:

- model weights exist under `/data/cmrnext`
- initial transform was published
- yaw threshold is not filtering everything
- zero-norm LiDAR quaternion warnings are not happening repeatedly


### Optimizer exits without a useful final result

Check:

- camera intrinsics arrived
- enough synchronized pose bags were collected
- enough correspondence messages were collected
- `/data/experiments/<run_name>/results/results.txt` for timing/log clues


### YAML was not written

Check:

- output path directory exists or is creatable
- `ros2_export_yaml_path` parameter is not empty
- the optimizer actually reached `ComputeRefinedTransform()`


## 11. Important Non-Obvious Behavior

### The "undistorted" camera topic is currently just a rename

This is probably the single most important semantic footgun in the live path.

`camera_passthrough_adapter.py` republishes the camera topics but does not undistort them.

If the incoming images are not already rectified enough for the ORB-SLAM3 settings file, calibration quality can degrade badly.


### Ground truth is optional in live mode

The original research flow expected `/gt_extrinsics` for evaluation.

The live workflow was modified so that:

- `/gt_extrinsics` may be absent
- the optimizer skips final error evaluation if no ground truth arrives

This is expected for real robot operation.


### Temporary synchronized data is stored as rosbag files

The live pipeline still uses disk-backed intermediate rosbag files, even though the input is live topics.

That means:

- calibration is not purely stream-in-memory
- disk I/O matters
- cache cleanup matters
- stale files can cause confusing failures


### The BLT workflow is local history, not maintained repo API

There are logs in `logs/` showing BLT-specific experiments, and some developers may also have local scripts such as `run_mdpcalib_blt.sh`.

At the time of writing:

- `run_mdpcalib_blt.sh` is not part of the tracked repository
- treat BLT scripts as developer-local history unless they are explicitly added to git later


## 12. Where to Make Changes

### To change topic wiring

Edit:

- `src/pose_synchronizer/launch/ros2_live_calibration.launch`
- `src/pose_synchronizer/src/node.cpp`
- `src/optimization_utils/src/node.cpp`
- `src/CMRNext/src/cmrnext/cmrnext_ros_node.py`


### To change runtime orchestration

Edit:

- `run_mdpcalib_ros2.sh`
- `src/pose_synchronizer/scripts/start_ros2_calibration.sh`


### To change synchronization behavior

Edit:

- `src/pose_synchronizer/src/pose_synchronizer.cpp`
- `src/pose_synchronizer/include/pose_synchronizer/pose_synchronizer.h`


### To change the calibration math

Edit:

- `src/optimization_utils/src/optimizer.cpp`
- `src/optimization_utils/include/optimization_utils/cost_functors.h`
- `src/optimization_utils/include/optimization_utils/pose_graph_constraints.h`


### To change exported output format

Edit:

- `src/optimization_utils/src/io_utils.cpp`
- `src/optimization_utils/include/optimization_utils/io_utils.h`


### To change learned correspondence behavior

Edit:

- `src/CMRNext/src/cmrnext/cmrnext_ros_node.py`
- supporting code under `src/CMRNext/src/cmrnext/`


## 13. Recommended First Tasks for a New Developer

Before making major changes, the next developer should do these first:

1. Run one end-to-end live calibration with known-good topics and configs.
2. Inspect `/data/cache` and `/data/experiments/<run_name>/results/results.txt` after the run.
3. Echo these topics during a run:
   - `/orb_slam3/camera_pose`
   - `/odom`
   - `/synced_data/filename`
   - `/optimizer/initial_transform`
   - `/cmrnext/correspondences`
   - `/optimizer/refined_transform`
4. Confirm the exported YAML matches the expected frame convention on the robot.
5. Decide whether the camera adapter should remain a passthrough or become a real rectification stage.


## 14. Current Rough Edges

These are the things I would consider technical debt or follow-up items rather than finished design.

- The ROS 2 workflow is still a ROS 1 workflow internally.
- Camera "undistortion" is only a republish today.
- Intermediate synchronized rosbag files are disk-backed and fairly old-school for a live system.
- The optimizer still carries research-code assumptions and naming that are not always obvious.
- The transform convention requires careful reading because of internal inversion steps.
- `run_name` collisions can still be annoying if startup cleanup behavior changes.
- BLT-specific work exists in logs and local history but is not cleanly productized in the tracked repo.


## 15. Bottom Line

If you remember only five things, remember these:

1. The live workflow is ROS 1 computation wrapped around a ROS 2 robot via `ros1_bridge`.
2. `ros2_live_calibration.launch` is the single best file for understanding the live runtime graph.
3. The actual calibration logic lives in `pose_synchronizer.cpp`, `cmrnext_ros_node.py`, and `optimizer.cpp`.
4. The final exported transform is camera-in-LiDAR, labeled with `LIDAR_FRAME_ID` and `CAMERA_FRAME_ID`.
5. The current "camera_undistorted" stage is only a topic adapter, not real image rectification.
