# Render Colored Point Cloud from GLIM Dump

## Overview

`scripts/render_colored_pcd.py` colors GLIM submap point clouds using camera
images from a ROS2 bag. For each submap, it projects every 3D point into the
nearest-in-time camera frame, samples the pixel color, and writes the combined
result as a binary PCD file with XYZ + RGB fields.

## Architecture

```
ROS2 Bag (fisheye images)     GLIM Dump (submaps)
        │                            │
        ▼                            ▼
  Build image index            Parse data.txt
  (header timestamps)         (T_world_origin,
        │                     per-frame T_world_lidar)
        │                            │
        ▼                            ▼
  Batch-fetch images          Load points_compact.bin
        │                     (N×3 float32, origin frame)
        ▼                            │
  Rectify fisheye ◄─── OmniRadtan   │
  (cv2.remap)          remap tables  │
        │                            │
        └──────────┬─────────────────┘
                   ▼
         Per-frame projection loop
         (origin → lidar → camera → pixel)
                   │
                   ▼
         Sample colors from rectified image
                   │
                   ▼
         Transform points to world frame
                   │
                   ▼
         Write binary PCD (XYZRGB)
```

## Coordinate Frames and Transforms

### Frame definitions

| Frame | Description |
|-------|-------------|
| **origin** | The submap's reference frame (middle frame of the submap). Points in `points_compact.bin` are in this frame. |
| **world** | Global frame. `T_world_origin` maps origin → world. |
| **lidar** | LiDAR sensor frame at a specific timestamp. `T_world_lidar` maps lidar → world. |
| **camera** | Camera optical frame. `Rcl`, `Pcl` map lidar → camera. |

### Projection pipeline

To project a submap point into a camera image at a given frame's timestamp:

```
p_origin  (from points_compact.bin)
    │
    │  T_lidar_origin = inv(T_world_lidar) @ T_world_origin
    ▼
p_lidar = T_lidar_origin[:3,:3] @ p_origin + T_lidar_origin[:3,3]
    │
    │  Rcl, Pcl (LiDAR → Camera extrinsics)
    ▼
p_cam = Rcl @ p_lidar + Pcl
    │
    │  Pinhole projection (rectified intrinsics)
    ▼
u = fx * p_cam.x / p_cam.z + cx
v = fy * p_cam.y / p_cam.z + cy
```

Points are valid when `p_cam.z > 0.1` and `(u, v)` falls within `[0, 1920) × [0, 1200)`.

## Fisheye Rectification

The raw camera images use an OmniRadtan (Kalibr convention) distortion model.
The script builds rectification remap tables to produce pinhole-equivalent
images, following the same math as
`FAST-LIVO2-OFFLINE/include/FisheyeRectifier.h`:

1. For each output pixel `(j, i)`, unproject through the inverse of the output
   camera matrix to get a 3D ray `(x, y, w)`.
2. Normalize to unit sphere: `(Xs, Ys, Zs) = (x, y, w) / ‖(x, y, w)‖`.
3. Apply omni projection: `xu = Xs / (Zs + ξ)`, `yu = Ys / (Zs + ξ)`.
4. Apply radial-tangential distortion (k1, k2, p1, p2).
5. Map to raw pixel coordinates using the raw intrinsics.

This is computed once as vectorized numpy and applied per image via
`cv2.remap()`.

### Calibration parameters

**Raw camera (OmniRadtan, right camera)**:
| Parameter | Value |
|-----------|-------|
| ξ (xi) | 0.9524 |
| fx, fy | 874.94, 875.68 |
| cx, cy | 955.73, 627.12 |
| k1, k2 | -0.0810, -0.0206 |
| p1, p2 | 0.000392, 0.000744 |

**Rectified output** (pinhole, zero distortion):
| Parameter | Value |
|-----------|-------|
| Resolution | 1920 × 1200 |
| fx, fy | 960, 960 |
| cx, cy | 960, 600 |

**LiDAR → Camera extrinsics**:
```
Rcl = [[-0.7054, -0.7087,  0.0109],
       [ 0.2699, -0.2828, -0.9204],
       [ 0.6554, -0.6464,  0.3908]]

Pcl = [0.0001, -0.0401, -0.0321]
```

## Image Time Offset

Camera and LiDAR clocks may be offset. The `--img-time-offset` parameter
follows the FAST-LIVO2 convention:

```
corrected_image_time = raw_header_time + img_time_offset
```

When searching for the image matching a LiDAR timestamp, the script queries:

```
raw_header_time = lidar_stamp - img_time_offset
```

With the default offset of **-0.2s** (from `mid360_s1_right.yaml`), a LiDAR
frame at `t` matches an image whose raw header timestamp is at `t + 0.2s`.
This compensates for the camera clock running 0.2s ahead of the LiDAR clock
and produces sharper color alignment.

## Data Formats

### Input: GLIM dump

Each submap directory (e.g., `000000/`) contains:

| File | Format |
|------|--------|
| `data.txt` | Text file with `T_world_origin` (4×4 matrix), per-frame `stamp` and `T_world_lidar` (4×4 matrix) |
| `points_compact.bin` | Flat binary, `float32`, reshape to `(N, 3)` — points in origin frame |

### Input: ROS2 bag

Images on topic `/fisheye/right/image_raw`:
- Encoding: `bgr8`
- Resolution: 1920 × 1200
- Header timestamps match LiDAR hardware clock (not bag recording time)

### Output: PCD

Binary PCD v0.7 with fields `x y z rgb`:
- XYZ: `float32`, world frame coordinates
- RGB: packed as `uint32` viewed as `float32` (PCL convention: `R<<16 | G<<8 | B`)
- Uncolored points have RGB = 0 (black)

## Usage

```bash
# Color first 5 submaps (default)
python3 scripts/render_colored_pcd.py

# Color all 126 submaps
python3 scripts/render_colored_pcd.py --submaps 0-125 \
    --output /path/to/colored_all.pcd

# Specific submaps
python3 scripts/render_colored_pcd.py --submaps 0,10,50

# Custom time offset
python3 scripts/render_colored_pcd.py --img-time-offset -0.15
```

### All CLI arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `--submaps` | `0-4` | Submap range (`0-4`) or comma-separated (`0,1,2`) |
| `--output` | `colored_output.pcd` | Output PCD file path |
| `--dump-dir` | `/home/max/.../glim_dump` | GLIM dump directory |
| `--bag-dir` | `/home/max/.../hdb_r_long_pc2_ros2` | ROS2 bag directory |
| `--topic` | `/fisheye/right/image_raw` | Image topic name |
| `--img-time-offset` | `-0.2` | Image time offset in seconds (FAST-LIVO2 convention) |
| `--max-dt` | `0.2` | Max allowed time delta for image matching (seconds) |

## Performance

The script batch-fetches all needed images in a single pass through the ROS2
bag, then rectifies them upfront. This avoids repeated bag scans (which would
be O(frames × bag_size)).

Typical run on all 126 submaps (~6.3M points, ~1916 images):
- ~76 unique images needed for 5 submaps, ~1900 for all 126
- Output: ~97 MB PCD file
- Coverage: ~30% of points colored (single right-side fisheye camera vs 360° LiDAR)

## Tests

```bash
cd scripts
python3 -m pytest test_render_colored_pcd.py -v
```

Tests cover:
- **parse_submap_range**: Range and comma-separated inputs, edge cases
- **find_nearest_image_idx**: Exact match, interpolation, boundary cases, time offset shifts, empty input
- **parse_data_txt**: Matrix parsing, frame extraction, error on missing data
- **load_points**: Binary format, empty file
- **build_rectification_maps**: Output shape, dtype, center pixel sanity, value ranges
- **write_pcd_binary**: Roundtrip read-back, RGB packing, file size
- **Projection math**: Rotation matrix validity, identity/translation transforms, FOV checks
- **process_submap**: End-to-end with synthetic data (solid red image, identity transforms)
