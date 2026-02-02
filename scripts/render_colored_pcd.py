#!/usr/bin/env python3
"""
Render a colored point cloud from GLIM dump submaps + ROS2 bag camera images.

Colors submap point clouds by projecting each point into the nearest camera
frame, sampling the pixel color, and saving the combined result as a PCD file.

Usage:
    python3 scripts/render_colored_pcd.py [--submaps 0-4] [--output colored_output.pcd]
"""

import argparse
import bisect
import re
import sys
from pathlib import Path

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# Default paths
# ---------------------------------------------------------------------------
GLIM_DUMP_DIR = Path("/home/max/ground_map/koide3/data/glim_dump")
ROSBAG_DIR = Path("/home/max/ground_map/rosbag/hdb_r_long_pc2_ros2")
IMAGE_TOPIC = "/fisheye/right/image_raw"

# ---------------------------------------------------------------------------
# Right camera OmniRadtan calibration (from FisheyeRectifier.h)
# ---------------------------------------------------------------------------
OMNI_XI = 0.9524384178562157
OMNI_FX = 874.944410541234
OMNI_FY = 875.6750578201546
OMNI_CX = 955.7348543761232
OMNI_CY = 627.1241522186086
OMNI_K1 = -0.08102144586791893
OMNI_K2 = -0.02060671773807439
OMNI_P1 = 0.0003923220711579816
OMNI_P2 = 0.0007441945112059008

# Raw image size
RAW_W, RAW_H = 1920, 1200

# Rectified output intrinsics (from metadata.yaml / FisheyeRectifier.h with fov_scale=1.0)
RECT_FX = RAW_W / 2.0  # 960
RECT_FY = RAW_W / 2.0  # 960
RECT_CX = RAW_W / 2.0  # 960
RECT_CY = RAW_H / 2.0  # 600
RECT_W, RECT_H = RAW_W, RAW_H

# ---------------------------------------------------------------------------
# LiDAR → Camera extrinsics (from metadata.yaml)
# ---------------------------------------------------------------------------
RCL = np.array([
    -0.7054238, -0.7087025,  0.0108644,
     0.2699248, -0.282786,  -0.9204198,
     0.6553761, -0.6463535,  0.3907803,
], dtype=np.float64).reshape(3, 3)

PCL = np.array([0.0001417, -0.0400961, -0.0320987], dtype=np.float64).reshape(3, 1)


def build_rectification_maps():
    """Build fisheye rectification remap tables (vectorized numpy port of FisheyeRectifier.h)."""
    # Output camera matrix inverse (same logic as FisheyeRectifier.h)
    out_fx = RECT_FX
    out_fy = RECT_FY
    out_cx = RECT_CX
    out_cy = RECT_CY

    # P_square = [[out_fx, 0, out_cx], [0, out_fy, out_cy], [0, 0, 1]]
    P = np.array([
        [out_fx, 0, out_cx],
        [0, out_fy, out_cy],
        [0, 0, 1],
    ], dtype=np.float64)
    iKR = np.linalg.inv(P)

    # Grid of output pixel coordinates
    jj, ii = np.meshgrid(np.arange(RECT_W), np.arange(RECT_H))
    jj = jj.astype(np.float64)
    ii = ii.astype(np.float64)

    # Unproject to 3D direction
    x = jj * iKR[0, 0] + ii * iKR[0, 1] + iKR[0, 2]
    y = jj * iKR[1, 0] + ii * iKR[1, 1] + iKR[1, 2]
    w = jj * iKR[2, 0] + ii * iKR[2, 1] + iKR[2, 2]

    # Normalize to unit sphere
    r = np.sqrt(x * x + y * y + w * w)
    Xs = x / r
    Ys = y / r
    Zs = w / r

    # Omni projection
    denom = Zs + OMNI_XI
    xu = Xs / denom
    yu = Ys / denom

    # Radtan distortion
    r2 = xu * xu + yu * yu
    r4 = r2 * r2
    radial = 1.0 + OMNI_K1 * r2 + OMNI_K2 * r4
    xd = radial * xu + 2.0 * OMNI_P1 * xu * yu + OMNI_P2 * (r2 + 2.0 * xu * xu)
    yd = radial * yu + OMNI_P1 * (r2 + 2.0 * yu * yu) + 2.0 * OMNI_P2 * xu * yu

    # Pixel coordinates in original image
    map_x = (OMNI_FX * xd + OMNI_CX).astype(np.float32)
    map_y = (OMNI_FY * yd + OMNI_CY).astype(np.float32)

    return map_x, map_y


def parse_data_txt(path: Path):
    """Parse a GLIM dump data.txt file.

    Returns:
        T_world_origin: (4, 4) numpy array
        frames: list of dicts with keys 'stamp' and 'T_world_lidar' (4x4)
    """
    text = path.read_text()

    def parse_matrix(block: str) -> np.ndarray:
        """Parse a 4x4 matrix from whitespace-separated values (4 lines of 4 numbers)."""
        nums = []
        for line in block.strip().splitlines():
            nums.extend(float(x) for x in line.split())
        return np.array(nums, dtype=np.float64).reshape(4, 4)

    # Extract T_world_origin
    m = re.search(r"T_world_origin:\s*\n((?:\s*[-\d.e]+\s+[-\d.e]+\s+[-\d.e]+\s+[-\d.e]+\s*\n){4})", text)
    if not m:
        raise ValueError(f"Could not find T_world_origin in {path}")
    T_world_origin = parse_matrix(m.group(1))

    # Extract num_frames
    m = re.search(r"num_frames:\s*(\d+)", text)
    num_frames = int(m.group(1)) if m else 0

    # Extract per-frame data
    frames = []
    for fm in re.finditer(
        r"frame_\d+\n"
        r"id:\s*\d+\n"
        r"stamp:\s*([\d.]+)\n"
        r"T_odom_lidar:\s*\n(?:.*\n){4}"
        r"T_world_lidar:\s*\n((?:\s*[-\d.e]+\s+[-\d.e]+\s+[-\d.e]+\s+[-\d.e]+\s*\n){4})",
        text,
    ):
        stamp = float(fm.group(1))
        T_world_lidar = parse_matrix(fm.group(2))
        frames.append({"stamp": stamp, "T_world_lidar": T_world_lidar})

    print(f"  Parsed {len(frames)} frames (expected {num_frames})")
    return T_world_origin, frames


def load_points(submap_dir: Path) -> np.ndarray:
    """Load points_compact.bin as (N, 3) float32 array."""
    data = np.fromfile(submap_dir / "points_compact.bin", dtype=np.float32)
    return data.reshape(-1, 3)


def build_image_index(bag_dir: Path, topic: str):
    """Build sorted index of (header_timestamp, bag_timestamp) for an image topic.

    Returns:
        header_stamps: sorted numpy array of header timestamps (seconds, float64)
        bag_stamps: corresponding bag timestamps (nanoseconds, int) for message retrieval
    """
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    reader = Reader(bag_dir)
    reader.open()

    img_conns = [c for c in reader.connections if c.topic == topic]
    if not img_conns:
        raise ValueError(f"Topic {topic} not found in bag. Available: {[c.topic for c in reader.connections]}")

    header_stamps = []
    bag_stamps = []

    for conn, ts, data in reader.messages(img_conns):
        msg = typestore.deserialize_cdr(data, conn.msgtype)
        header_ts = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        header_stamps.append(header_ts)
        bag_stamps.append(ts)

    reader.close()

    header_stamps = np.array(header_stamps, dtype=np.float64)
    bag_stamps = np.array(bag_stamps, dtype=np.int64)

    # Sort by header timestamp (should already be sorted, but be safe)
    order = np.argsort(header_stamps)
    header_stamps = header_stamps[order]
    bag_stamps = bag_stamps[order]

    print(f"  Indexed {len(header_stamps)} images, time range: [{header_stamps[0]:.3f}, {header_stamps[-1]:.3f}]")
    return header_stamps, bag_stamps


def fetch_images_batch(bag_dir: Path, topic: str, target_bag_ts_set: set) -> dict:
    """Fetch multiple images from the bag in a single pass.

    Args:
        target_bag_ts_set: set of bag timestamps (int) to fetch

    Returns:
        dict mapping bag_ts -> numpy image (BGR, uint8)
    """
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    reader = Reader(bag_dir)
    reader.open()

    result = {}
    img_conns = [c for c in reader.connections if c.topic == topic]
    for conn, ts, data in reader.messages(img_conns):
        if ts in target_bag_ts_set:
            msg = typestore.deserialize_cdr(data, conn.msgtype)
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            result[ts] = img.copy()
            if len(result) == len(target_bag_ts_set):
                break

    reader.close()
    print(f"  Fetched {len(result)}/{len(target_bag_ts_set)} images from bag")
    return result


def find_nearest_image_idx(header_stamps: np.ndarray, query_stamp: float,
                           img_time_offset: float = 0.0, max_dt: float = 0.2) -> int:
    """Find the index of the nearest image by header timestamp. Returns -1 if too far.

    FAST-LIVO2 convention: corrected_img_time = header_time + img_time_offset.
    So to find the raw header_time matching a LiDAR stamp:
        header_time = lidar_stamp - img_time_offset
    """
    # Convert LiDAR stamp to raw image header time space
    query_in_img_space = query_stamp - img_time_offset
    idx = bisect.bisect_left(header_stamps, query_in_img_space)
    best_idx = -1
    best_dt = max_dt

    for candidate in [idx - 1, idx]:
        if 0 <= candidate < len(header_stamps):
            dt = abs(header_stamps[candidate] - query_in_img_space)
            if dt < best_dt:
                best_dt = dt
                best_idx = candidate

    return best_idx


def write_pcd_binary(path: Path, xyz: np.ndarray, rgb: np.ndarray):
    """Write a binary PCD file with XYZ + RGB fields.

    Args:
        xyz: (N, 3) float32
        rgb: (N, 3) uint8, in RGB order
    """
    n = xyz.shape[0]
    # Pack RGB into a single float32 (PCL convention: R<<16 | G<<8 | B packed as uint32, viewed as float32)
    r = rgb[:, 0].astype(np.uint32)
    g = rgb[:, 1].astype(np.uint32)
    b = rgb[:, 2].astype(np.uint32)
    rgb_packed = (r << 16) | (g << 8) | b
    # View as float32 without changing bits
    rgb_float = rgb_packed.view(np.float32)

    header = (
        f"# .PCD v0.7 - Point Cloud Data file format\n"
        f"VERSION 0.7\n"
        f"FIELDS x y z rgb\n"
        f"SIZE 4 4 4 4\n"
        f"TYPE F F F F\n"
        f"COUNT 1 1 1 1\n"
        f"WIDTH {n}\n"
        f"HEIGHT 1\n"
        f"VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        f"DATA binary\n"
    )

    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        # Interleave xyz and rgb_float
        combined = np.column_stack([xyz, rgb_float]).astype(np.float32)
        f.write(combined.tobytes())

    print(f"  Wrote {n} points to {path}")


def parse_submap_range(s: str) -> list[str]:
    """Parse a submap range like '0-4' or '0,2,5' into zero-padded directory names."""
    if "-" in s and "," not in s:
        parts = s.split("-")
        start, end = int(parts[0]), int(parts[1])
        return [f"{i:06d}" for i in range(start, end + 1)]
    else:
        return [f"{int(x):06d}" for x in s.split(",")]


def process_submap(submap_dir: Path, map_x, map_y, header_stamps, bag_stamps,
                   rectified_cache: dict, img_time_offset: float, max_dt: float):
    """Process a single submap: project points into camera frames, sample colors.

    Returns:
        p_world: (N, 3) float32 points in world frame
        colors: (N, 3) uint8 RGB colors
        n_colored: number of points that got colored
    """
    T_world_origin, frames = parse_data_txt(submap_dir / "data.txt")
    points_origin = load_points(submap_dir)
    n_points = points_origin.shape[0]
    print(f"  Loaded {n_points} points")

    colors = np.zeros((n_points, 3), dtype=np.uint8)
    colored_mask = np.zeros(n_points, dtype=bool)

    for i, frame in enumerate(frames):
        stamp = frame["stamp"]
        T_world_lidar = frame["T_world_lidar"]

        img_idx = find_nearest_image_idx(header_stamps, stamp, img_time_offset, max_dt)
        if img_idx < 0:
            print(f"    Frame {i}: stamp={stamp:.6f} - no matching image (dt > {max_dt}s)")
            continue

        dt = header_stamps[img_idx] - (stamp - img_time_offset)
        bag_ts = int(bag_stamps[img_idx])

        # Rectify on first use
        if bag_ts not in rectified_cache:
            raise KeyError(f"Image bag_ts={bag_ts} not pre-fetched")
        rectified = rectified_cache[bag_ts]

        # Transform: origin frame → lidar frame at this timestamp
        T_lidar_world = np.linalg.inv(T_world_lidar)
        T_lidar_origin = T_lidar_world @ T_world_origin
        p_lidar = (T_lidar_origin[:3, :3] @ points_origin.T + T_lidar_origin[:3, 3:]).T

        # Transform: lidar frame → camera frame
        p_cam = (RCL @ p_lidar.T + PCL).T

        # Pinhole projection with rectified intrinsics
        z = p_cam[:, 2]
        valid = z > 0.1

        u = np.full(n_points, -1.0)
        v = np.full(n_points, -1.0)
        u[valid] = RECT_FX * p_cam[valid, 0] / z[valid] + RECT_CX
        v[valid] = RECT_FY * p_cam[valid, 1] / z[valid] + RECT_CY

        in_fov = valid & (u >= 0) & (u < RECT_W) & (v >= 0) & (v < RECT_H)
        new_color = in_fov & ~colored_mask

        n_new = np.sum(new_color)
        if n_new > 0:
            ui = np.clip(u[new_color].astype(np.int32), 0, RECT_W - 1)
            vi = np.clip(v[new_color].astype(np.int32), 0, RECT_H - 1)
            bgr = rectified[vi, ui]
            colors[new_color] = bgr[:, ::-1]  # BGR -> RGB
            colored_mask |= new_color

        n_colored_total = np.sum(colored_mask)
        print(f"    Frame {i}: stamp={stamp:.6f}, dt={dt:+.4f}s, "
              f"new={n_new}, total_colored={n_colored_total}/{n_points} "
              f"({100*n_colored_total/n_points:.1f}%)")

    # Transform to world frame
    p_world = (T_world_origin[:3, :3] @ points_origin.T + T_world_origin[:3, 3:]).T
    p_world = p_world.astype(np.float32)

    n_colored = int(np.sum(colored_mask))
    return p_world, colors, n_colored


def main():
    parser = argparse.ArgumentParser(description="Color GLIM submap point clouds using camera images")
    parser.add_argument("--submaps", default="0-4",
                        help="Submap range '0-4' or comma-separated '0,1,2' (default: 0-4)")
    parser.add_argument("--output", default="colored_output.pcd", help="Output PCD file path")
    parser.add_argument("--dump-dir", default=str(GLIM_DUMP_DIR), help="Path to GLIM dump directory")
    parser.add_argument("--bag-dir", default=str(ROSBAG_DIR), help="Path to ROS2 bag directory")
    parser.add_argument("--topic", default=IMAGE_TOPIC, help="Image topic name")
    parser.add_argument("--img-time-offset", type=float, default=-0.2,
                        help="Image time offset (s), FAST-LIVO2 convention: corrected = header + offset (default: -0.2)")
    parser.add_argument("--max-dt", type=float, default=0.2, help="Max time delta (s) for image matching")
    args = parser.parse_args()

    dump_dir = Path(args.dump_dir)
    bag_dir = Path(args.bag_dir)
    submap_names = parse_submap_range(args.submaps)

    # Validate all submaps exist
    for name in submap_names:
        if not (dump_dir / name).exists():
            print(f"Error: submap directory {dump_dir / name} does not exist")
            sys.exit(1)

    img_time_offset = args.img_time_offset
    print(f"Will process {len(submap_names)} submaps: {', '.join(submap_names)}")
    print(f"Image time offset: {img_time_offset:+.3f}s")

    # Step 1: Build fisheye rectification maps
    print("\nBuilding fisheye rectification maps...")
    map_x, map_y = build_rectification_maps()

    # Step 2: Index camera images from bag
    print("Indexing camera images from ROS2 bag...")
    header_stamps, bag_stamps = build_image_index(bag_dir, args.topic)

    # Step 3: Collect all needed bag timestamps across all submaps
    print("Collecting needed image timestamps...")
    needed_bag_ts = set()
    for name in submap_names:
        _, frames = parse_data_txt(dump_dir / name / "data.txt")
        for frame in frames:
            img_idx = find_nearest_image_idx(header_stamps, frame["stamp"], img_time_offset, args.max_dt)
            if img_idx >= 0:
                needed_bag_ts.add(int(bag_stamps[img_idx]))
    print(f"  Need {len(needed_bag_ts)} unique images across all submaps")

    # Step 4: Batch-fetch all needed images in a single bag pass
    print("Fetching images from ROS2 bag...")
    raw_images = fetch_images_batch(bag_dir, args.topic, needed_bag_ts)

    # Rectify all fetched images
    print("Rectifying images...")
    rectified_cache = {}
    for bag_ts, raw_img in raw_images.items():
        rectified_cache[bag_ts] = cv2.remap(raw_img, map_x, map_y, cv2.INTER_LINEAR)
    del raw_images
    print(f"  Rectified {len(rectified_cache)} images")

    # Step 5: Process each submap
    all_points = []
    all_colors = []
    total_points = 0
    total_colored = 0

    for name in submap_names:
        print(f"\n--- Submap {name} ---")
        submap_dir = dump_dir / name
        p_world, colors, n_colored = process_submap(
            submap_dir, map_x, map_y, header_stamps, bag_stamps,
            rectified_cache, img_time_offset, args.max_dt,
        )
        all_points.append(p_world)
        all_colors.append(colors)
        total_points += p_world.shape[0]
        total_colored += n_colored
        print(f"  Submap {name}: {n_colored}/{p_world.shape[0]} colored "
              f"({100*n_colored/p_world.shape[0]:.1f}%)")

    # Step 6: Combine and save
    combined_xyz = np.concatenate(all_points, axis=0)
    combined_rgb = np.concatenate(all_colors, axis=0)

    output_path = Path(args.output)
    print(f"\nSaving combined colored point cloud to {output_path}...")
    write_pcd_binary(output_path, combined_xyz, combined_rgb)

    print(f"\nDone! {total_colored}/{total_points} points colored "
          f"({100*total_colored/total_points:.1f}%)")
    print(f"Output: {output_path}")


if __name__ == "__main__":
    main()
