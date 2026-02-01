# GLIM Project Notes

## What This Is

GLIM is a 3D LiDAR mapping framework running in Docker (`glim_custom:humble_cuda12.2`) with ROS2 Humble + CUDA 12.2. The main workflow uses pre-computed poses from FAST-LIVO2 (external odometry) to build optimized 3D maps from rosbag data.

## Quick Reference: Running GLIM with External Pose

### Prerequisites
- Run `xhost +local:root` on the host before any Docker GUI commands
- Docker image `glim_custom:humble_cuda12.2` must be built (auto-built on first run)

### Config Files to Update Per Run

All configs live in `config_external/`. Three things **must** be checked before each run:

1. **`config_external/config_odometry_external.json`** -- set `pose_file_path` to the pose.txt (use container path `/home_max/...` not host path `/home/max/...`)
2. **`config_external/config_ros.json`** -- set `imu_topic` and `points_topic` to match the rosbag topics
3. **Dump path** -- passed as `-p dump_path:=...` on the command line

### Volume Mounts (hardcoded in scripts/run_glim_custom.sh)

| Host | Container |
|------|-----------|
| `/home/max/ground_map/koide3/glim_1/` | `/root/ros2_ws/src/glim` |
| `/home/max/` | `/home_max/` |

All config paths must use **container paths** (e.g. `/home_max/...`).

### Run Commands

**Interactive shell:**
```bash
./scripts/run_glim_custom.sh
```

**Run glim_rosbag non-interactively (build + run):**
```bash
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

**View results (offline viewer):**
```bash
./scripts/run_glim_custom.sh offline_viewer /home_max/ground_map/koide3/data/glim_dump
```

### Common Gotchas

- `docker run -it` commands (like `run_glim_custom.sh`) require a real terminal -- they won't work from non-TTY contexts
- `acc_scale` must be `9.80665` for Livox sensors (raw IMU in g units)
- Livox Mid360 LiDAR-IMU extrinsics are identity: `[0,0,0,0,0,0,1]` in `config_sensors.json`
- Pose timestamps use Livox hardware clock -- no manual offset needed vs rosbag

### Full Documentation

See `docs/external_odometry.md` for the complete guide including FAST-LIVO2 pose generation, architecture details, and output file descriptions.
