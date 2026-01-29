# External Odometry Integration (FAST-LIVO2)

This module allows GLIM to use pre-computed odometry from external sources (e.g., FAST-LIVO2) instead of running its internal LiDAR-IMU odometry estimation.

## Overview

```
┌─────────────────────┐      ┌─────────────────────────────────────────────┐
│    FAST-LIVO2       │      │                   GLIM                       │
│    (Offline)        │      │                                              │
│                     │      │  ┌─────────┐   ┌──────────┐   ┌───────────┐ │
│  rosbag ──► pose.txt│ ─►   │  │External │──►│SubMapping│──►│ Global    │ │
│            (9 cols) │      │  │Odometry │   │          │   │ Mapping   │ │
│                     │      │  │Estimator│   │          │   │(PoseGraph)│ │
└─────────────────────┘      │  └─────────┘   └──────────┘   └───────────┘ │
                             └─────────────────────────────────────────────┘
```

## Workflow

### Step 1: Run FAST-LIVO2 to Generate Odometry

```bash
# In singamap-dev Docker container
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

This generates `pose.txt` in the output directory with format:
```
tx ty tz qw qx qy qz lidar_ts img_ts
```

### Step 2: Build GLIM with External Odometry

```bash
# Run GLIM Docker container
docker run --rm -it --privileged --net=host --ipc=host --gpus all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  -v /home/max/ground_map/koide3/glim:/glim \
  -v /home/max/ground_map:/data \
  --name glim-dev koide3/glim_ros2:humble_cuda12.2

# Inside container - build
cd /glim
colcon build --symlink-install
```

### Step 3: Configure GLIM

Edit `config/config.json` to use external odometry:
```json
{
  "global": {
    "config_odometry": "config_odometry_external.json",
    ...
  }
}
```

Edit `config/config_odometry_external.json`:
```json
{
  "odometry_estimation": {
    "so_name": "libexternal_odometry_estimation.so",
    "pose_file_path": "/data/annotator/one_txt/20250811_121516_laserMapping/pose.txt",
    "timestamp_tolerance": 0.1,
    "estimate_velocity": true,
    "velocity_dt": 0.02
  }
}
```

### Step 4: Run GLIM Mapping

```bash
# Inside GLIM container
source install/setup.bash
ros2 launch glim_ros glim.launch.py
# In another terminal, play the rosbag
ros2 bag play /data/path/to/your_bag
```

## Configuration Options

| Parameter | Description | Default |
|-----------|-------------|---------|
| `pose_file_path` | Path to FAST-LIVO2 pose.txt | "" |
| `timestamp_tolerance` | Max time diff for pose matching (sec) | 0.1 |
| `estimate_velocity` | Estimate velocity from poses | true |
| `velocity_dt` | Time delta for velocity estimation | 0.02 |

## pose.txt Format

FAST-LIVO2 outputs poses in this format (9 columns, space-separated):

| Column | Description |
|--------|-------------|
| 1-3 | tx, ty, tz (position in meters) |
| 4-7 | qw, qx, qy, qz (quaternion, w-first) |
| 8 | lidar_timestamp (Unix epoch seconds) |
| 9 | image_timestamp (Unix epoch seconds) |

## Notes

- The external odometry module automatically handles timestamp offset between rosbag and pose.txt
- Poses are interpolated using SLERP for rotation and linear interpolation for translation
- Velocity is estimated using finite differences from interpolated poses
- GLIM's SubMapping and GlobalMapping still perform loop closure and pose graph optimization
