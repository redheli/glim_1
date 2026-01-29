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

### Step 2: Build GLIM with external odometry module

```bash
# Start GLIM container
docker run --rm -it --privileged --net=host --ipc=host --gpus all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -e XAUTHORITY=/root/.Xauthority \
  -v /home/max/ground_map/koide3/glim:/glim_src \
  -v /home/max/ground_map:/data \
  --name glim-dev koide3/glim_ros2:humble_cuda12.2 /bin/bash

# Inside container: copy new files into the pre-built workspace and rebuild
cp /glim_src/include/glim/util/pose_file_loader.hpp /root/ros2_ws/src/glim/include/glim/util/
cp /glim_src/include/glim/util/pose_interpolator.hpp /root/ros2_ws/src/glim/include/glim/util/
cp /glim_src/include/glim/odometry/external_odometry_estimation.hpp /root/ros2_ws/src/glim/include/glim/odometry/
cp /glim_src/src/glim/util/pose_file_loader.cpp /root/ros2_ws/src/glim/src/glim/util/
cp /glim_src/src/glim/util/pose_interpolator.cpp /root/ros2_ws/src/glim/src/glim/util/
cp /glim_src/src/glim/odometry/external_odometry_estimation.cpp /root/ros2_ws/src/glim/src/glim/odometry/
cp /glim_src/src/glim/odometry/external_odometry_estimation_create.cpp /root/ros2_ws/src/glim/src/glim/odometry/
cp /glim_src/config/config_odometry_external.json /root/ros2_ws/src/glim/config/
cp /glim_src/CMakeLists.txt /root/ros2_ws/src/glim/CMakeLists.txt

# Fix CMakeLists.txt for container's GLIM version (may not have viewer_ui files)
sed -i '/standard_viewer_ui.cpp/d' /root/ros2_ws/src/glim/CMakeLists.txt
sed -i '/standard_viewer_callbacks.cpp/d' /root/ros2_ws/src/glim/CMakeLists.txt

source /opt/ros/humble/setup.bash
cd /root/ros2_ws
colcon build --packages-select glim --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

### Step 3: Update config

Edit `config_external/config_odometry_external.json` with your pose.txt path:

```json
"pose_file_path": "/data/koide3/data/pose.txt"
```

Edit `config_external/config_ros.json` with your rosbag topics:

```json
"imu_topic": "/livox/imu",
"points_topic": "/livox/pc2"
```

### Step 4: Run GLIM mapping

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

/root/ros2_ws/install/glim_ros/lib/glim_ros/glim_rosbag \
  /data/rosbag/hdb_r_long_pc2_ros2 \
  --ros-args \
  -p config_path:=/glim_src/config_external \
  -p auto_quit:=true \
  -p dump_path:=/data/koide3/data/glim_dump
```

### Step 5: View results

```bash
/root/ros2_ws/install/glim_ros/lib/glim_ros/offline_viewer \
  /data/koide3/data/glim_dump
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
