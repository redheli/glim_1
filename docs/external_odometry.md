# External Odometry Integration (FAST-LIVO2 + GLIM)

Use pre-computed odometry from FAST-LIVO2 as input to GLIM's mapping pipeline (SubMapping + GlobalMapping with pose graph optimization and loop closure).

## Design

```
 FAST-LIVO2 (Offline)                      GLIM
┌──────────────────────┐    ┌─────────────────────────────────────────┐
│                      │    │                                         │
│  rosbag ──► pose.txt │    │  ExternalOdometryEstimation             │
│             (9 cols) │───►│    │  PoseFileLoader (parse pose.txt)   │
│                      │    │    │  PoseInterpolator (SLERP + lerp)   │
│                      │    │    │  CloudDeskewing (motion compensate) │
└──────────────────────┘    │    ▼                                    │
                            │  EstimationFrame                        │
        rosbag ────────────►│    │                                    │
     (point cloud + IMU)    │    ├──► SubMapping (local submaps)      │
                            │    │                                    │
                            │    └──► GlobalMapping (pose graph)      │
                            │              │                          │
                            │              ▼                          │
                            │         Optimized Map                   │
                            └─────────────────────────────────────────┘
```

### Module Architecture

| Component | File | Purpose |
|-----------|------|---------|
| `PoseFileLoader` | `src/glim/util/pose_file_loader.cpp` | Parse FAST-LIVO2 pose.txt format |
| `PoseInterpolator` | `src/glim/util/pose_interpolator.cpp` | SLERP rotation + linear translation interpolation with binary search |
| `ExternalOdometryEstimation` | `src/glim/odometry/external_odometry_estimation.cpp` | OdometryEstimationBase plugin: timestamp matching, deskewing, EstimationFrame creation |

### Key Design Decisions

- **No timestamp offset**: rosbag and pose.txt share the same time domain. FAST-LIVO2 skips initial frames during initialization, so early frames before the first pose use the boundary pose.
- **Pose-interpolated deskewing**: For each point cloud scan (~100ms), a mini-trajectory of 10 interpolated poses is generated and fed to GLIM's `CloudDeskewing` to motion-compensate every point. This is critical for Livox LiDARs with non-repetitive scan patterns.
- **Dynamic loading**: Built as `libexternal_odometry_estimation.so`, loaded via `so_name` in config -- no changes to GLIM core code needed.

## pose.txt Format

FAST-LIVO2 outputs 9 columns, space-separated:

```
tx ty tz qw qx qy qz lidar_ts img_ts
```

| Column | Description |
|--------|-------------|
| 1-3 | `tx, ty, tz` -- position in meters (world frame) |
| 4-7 | `qw, qx, qy, qz` -- quaternion, w-first |
| 8 | `lidar_ts` -- LiDAR timestamp (Unix epoch seconds) |
| 9 | `img_ts` -- image timestamp (Unix epoch seconds) |

Example:
```
-0.000590 -0.004577 -0.001811 0.999858 0.007535 0.014983 0.001732 1735690391.808028 1735690391.908022
```

## Configuration

Two config directories are involved:

### 1. `config_external/config.json` -- point odometry to external module

The key change is `config_odometry`:

```json
{
  "global": {
    "config_odometry": "config_odometry_external.json",
    "config_sub_mapping": "config_sub_mapping_gpu.json",
    "config_global_mapping": "config_global_mapping_gpu.json"
  }
}
```

### 2. `config_external/config_odometry_external.json` -- external odometry settings

```json
{
  "odometry_estimation": {
    "so_name": "libexternal_odometry_estimation.so",
    "pose_file_path": "/data/koide3/data/pose.txt",
    "timestamp_tolerance": 0.1,
    "estimate_velocity": true,
    "velocity_dt": 0.02
  }
}
```

| Parameter | Description | Default |
|-----------|-------------|---------|
| `so_name` | Shared library name (do not change) | `libexternal_odometry_estimation.so` |
| `pose_file_path` | Absolute path to FAST-LIVO2 pose.txt | `""` |
| `timestamp_tolerance` | Max time diff for nearest-pose lookup (sec) | `0.1` |
| `estimate_velocity` | Estimate velocity from pose finite differences | `true` |
| `velocity_dt` | Time delta for velocity estimation (sec) | `0.02` |

### 3. `config_external/config_ros.json` -- ROS topics

Update topic names to match your rosbag:

```json
{
  "glim_ros": {
    "imu_topic": "/livox/imu",
    "points_topic": "/livox/pc2",
    "acc_scale": 9.80665
  }
}
```

Set `acc_scale` to `9.80665` for Livox sensors (raw IMU outputs in g units).

### 4. `config_external/config_sensors.json` -- LiDAR-IMU extrinsics

Set `T_lidar_imu` in TUM format `[x, y, z, qx, qy, qz, qw]`:

```json
{
  "sensors": {
    "T_lidar_imu": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
  }
}
```

Common values:
- Livox Mid360: `[0, 0, 0, 0, 0, 0, 1]` (identity)
- Livox Avia: `[0.04165, 0.02326, -0.0284, 0, 0, 0, 1]`

## How to Run

Uses `scripts/run_glim_custom.sh` which builds the `glim_custom:humble_cuda12.2` Docker image
and runs the container with GPU support.

### Docker volume mounts (defined in run_glim_custom.sh)

| Host path | Container path | Purpose |
|-----------|---------------|---------|
| `/home/max/ground_map/koide3/glim_1/` | `/root/ros2_ws/src/glim` | GLIM source (for building + config access) |
| `/home/max/` | `/home_max/` | Access to rosbags, pose files, output dirs |

All paths in config files must use **container paths** (e.g. `/home_max/...`).

### Step 1: Generate pose.txt with FAST-LIVO2

```bash
# Start singamap-dev container
docker run --rm -it --privileged --net=host --ipc=host --gpus all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/home/max/.Xauthority \
  -e XAUTHORITY=/home/max/.Xauthority \
  -v /home/max:/home_max \
  -v /home/max/ground_map/catkin_ws/src/singamap/slam/FAST-LIVO2-OFFLINE/:/home/max/catkin_ws/src/FAST-LIVO2-OFFLINE \
  --name singamap-dev --entrypoint /bin/bash singamap-dev

# Inside container
source /opt/ros/noetic/setup.bash
source /home/max/catkin_ws/devel/setup.bash
roslaunch fast_livo mapping_offline.launch bag_file:=/path/to/your.bag
```

Output: `<output_dir>/YYYYMMDD_HHMMSS_laserMapping/pose.txt`

### Step 2: Update config (MUST check for each new bag)

Three parameters **must** be verified and updated before each run:

#### 2a. `config_external/config_odometry_external.json` -- pose file path

Set `pose_file_path` to the **container path** of your pose.txt file:

```json
"pose_file_path": "/home_max/ground_map/rosbag/<your-bag-dir>/pose.txt"
```

#### 2b. `config_external/config_ros.json` -- ROS topic names

The `points_topic` and `imu_topic` **must match the actual topic names in your rosbag**.
Different bags may use different topic names. Check with:

```bash
# From host (if rosbags python package installed):
python3 -c "
from rosbags.rosbag2 import Reader
with Reader('/path/to/your_ros2_bag') as r:
    for c in r.connections:
        print(f'{c.topic} ({c.msgtype}, {c.msgcount} msgs)')
"

# Or from inside the container:
ros2 bag info /home_max/path/to/your_ros2_bag
```

Update `config_ros.json` to match:

```json
"imu_topic": "/livox/imu",
"points_topic": "/livox/lidar"
```

Common topic names:
- Livox Mid360: `/livox/lidar` (PointCloud2), `/livox/imu`
- Converted bags: may use `/livox/pc2` if explicitly converted

#### 2c. `config_external/config_ros.json` -- dump path

Choose where to write output (container path):

```
/home_max/ground_map/koide3/data/glim_dump
```

#### Timestamp note

The pose.txt `lidar_ts` column uses **Livox hardware timestamps**, which may differ from
ROS bag recording time by a large offset. This is expected -- GLIM matches against the
PointCloud2 `header.stamp` which also uses the Livox hardware clock. No manual offset needed.

### Step 3: Build and run

#### Option A: Non-interactive (single command)

Build the Docker image (first time or after Dockerfile changes), build GLIM inside the
container, and run `glim_rosbag` -- all in one command:

```bash
# Build Docker image (skipped if already exists, use --build to force rebuild)
./scripts/run_glim_custom.sh --build

# Or run everything non-interactively:
docker rm -f glim 2>/dev/null
docker run --rm \
    --net=host --ipc=host --pid=host --gpus all \
    -e DISPLAY=$DISPLAY \
    -e ROS_IP=127.0.0.1 \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,graphics,utility,video,display \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev/dri:/dev/dri \
    -v /home/max/ground_map/koide3/glim_1/:/root/ros2_ws/src/glim \
    -v /home/max/:/home_max \
    --name glim \
    --entrypoint /bin/bash \
    glim_custom:humble_cuda12.2 \
    -c 'source /opt/ros/humble/setup.bash && \
cd /root/ros2_ws && \
source install/setup.bash && \
colcon build --packages-select glim --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo 2>&1 && \
source install/setup.bash && \
/root/ros2_ws/install/glim_ros/lib/glim_ros/glim_rosbag \
    /home_max/path/to/your_ros2_bag \
    --ros-args \
    -p config_path:=/root/ros2_ws/src/glim/config_external \
    -p auto_quit:=true \
    -p dump_path:=/home_max/ground_map/koide3/data/glim_dump'
```

#### Option B: Interactive

```bash
# Start interactive shell
./scripts/run_glim_custom.sh

# Inside container:
cd /root/ros2_ws
colcon build --packages-select glim --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash

/root/ros2_ws/install/glim_ros/lib/glim_ros/glim_rosbag \
    /home_max/path/to/your_ros2_bag \
    --ros-args \
    -p config_path:=/root/ros2_ws/src/glim/config_external \
    -p auto_quit:=true \
    -p dump_path:=/home_max/ground_map/koide3/data/glim_dump
```

### Step 4: View results

```bash
./scripts/run_glim_custom.sh offline_viewer /home_max/ground_map/koide3/data/glim_dump
```

## Output

GLIM produces the following in the dump directory:

| File | Description |
|------|-------------|
| `000000/` ... `NNNNNN/` | Submap data (point clouds + poses) |
| `graph.bin` | Serialized factor graph (binary) |
| `graph.txt` | Factor graph (text, for inspection) |
| `odom_lidar.txt` | Odometry trajectory (LiDAR frame, TUM format) |
| `odom_imu.txt` | Odometry trajectory (IMU frame, TUM format) |
| `traj_lidar.txt` | Optimized trajectory (LiDAR frame, TUM format) |
| `traj_imu.txt` | Optimized trajectory (IMU frame, TUM format) |
| `values.bin` | Optimized graph values (binary) |
