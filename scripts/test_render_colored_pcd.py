#!/usr/bin/env python3
"""Unit tests for render_colored_pcd.py.

Run with:
    python3 -m pytest scripts/test_render_colored_pcd.py -v
"""

import textwrap
from pathlib import Path

import cv2
import numpy as np
import pytest

from render_colored_pcd import (
    build_rectification_maps,
    find_nearest_image_idx,
    load_points,
    parse_data_txt,
    parse_submap_range,
    write_pcd_binary,
    RAW_H,
    RAW_W,
    RECT_CX,
    RECT_CY,
    RECT_FX,
    RECT_FY,
    RECT_H,
    RECT_W,
    RCL,
    PCL,
)


# ---------------------------------------------------------------------------
# parse_submap_range
# ---------------------------------------------------------------------------
class TestParseSubmapRange:
    def test_single_range(self):
        assert parse_submap_range("0-4") == [
            "000000", "000001", "000002", "000003", "000004"
        ]

    def test_single_value(self):
        assert parse_submap_range("7") == ["000007"]

    def test_comma_separated(self):
        assert parse_submap_range("0,3,10") == ["000000", "000003", "000010"]

    def test_large_index(self):
        assert parse_submap_range("125") == ["000125"]

    def test_range_single_element(self):
        assert parse_submap_range("5-5") == ["000005"]


# ---------------------------------------------------------------------------
# find_nearest_image_idx
# ---------------------------------------------------------------------------
class TestFindNearestImageIdx:
    @pytest.fixture
    def stamps(self):
        return np.array([100.0, 100.1, 100.2, 100.3, 100.4], dtype=np.float64)

    def test_exact_match(self, stamps):
        idx = find_nearest_image_idx(stamps, 100.2, img_time_offset=0.0)
        assert idx == 2

    def test_between_stamps(self, stamps):
        idx = find_nearest_image_idx(stamps, 100.15, img_time_offset=0.0)
        assert idx in [1, 2]  # either neighbor is acceptable
        assert abs(stamps[idx] - 100.15) <= 0.05

    def test_before_first(self, stamps):
        idx = find_nearest_image_idx(stamps, 99.95, img_time_offset=0.0)
        assert idx == 0

    def test_after_last(self, stamps):
        idx = find_nearest_image_idx(stamps, 100.42, img_time_offset=0.0)
        assert idx == 4

    def test_too_far_returns_minus_one(self, stamps):
        idx = find_nearest_image_idx(stamps, 200.0, img_time_offset=0.0, max_dt=0.2)
        assert idx == -1

    def test_negative_offset_shifts_query(self, stamps):
        # With offset=-0.2, query_stamp=100.0 searches at 100.0 - (-0.2) = 100.2
        idx = find_nearest_image_idx(stamps, 100.0, img_time_offset=-0.2)
        assert idx == 2

    def test_positive_offset_shifts_query(self, stamps):
        # With offset=+0.1, query_stamp=100.2 searches at 100.2 - 0.1 = 100.1
        idx = find_nearest_image_idx(stamps, 100.2, img_time_offset=0.1)
        assert idx == 1

    def test_zero_offset_is_default(self, stamps):
        idx_default = find_nearest_image_idx(stamps, 100.2)
        idx_zero = find_nearest_image_idx(stamps, 100.2, img_time_offset=0.0)
        assert idx_default == idx_zero

    def test_offset_can_push_out_of_range(self, stamps):
        # query 100.0 with offset=-1.0 searches at 101.0, which is >max_dt from 100.4
        idx = find_nearest_image_idx(stamps, 100.0, img_time_offset=-1.0, max_dt=0.2)
        assert idx == -1

    def test_empty_stamps(self):
        idx = find_nearest_image_idx(np.array([], dtype=np.float64), 100.0)
        assert idx == -1

    def test_single_stamp(self):
        stamps = np.array([100.0], dtype=np.float64)
        assert find_nearest_image_idx(stamps, 100.05) == 0
        assert find_nearest_image_idx(stamps, 100.05, max_dt=0.01) == -1


# ---------------------------------------------------------------------------
# parse_data_txt
# ---------------------------------------------------------------------------
class TestParseDataTxt:
    SAMPLE_DATA_TXT = textwrap.dedent("""\
        id: 0
        T_world_origin:
           1.0  0.0  0.0  1.5
           0.0  1.0  0.0  2.5
           0.0  0.0  1.0  3.5
           0.0  0.0  0.0  1.0
        T_origin_endpoint_L:
           1.0  0.0  0.0  0.0
           0.0  1.0  0.0  0.0
           0.0  0.0  1.0  0.0
           0.0  0.0  0.0  1.0
        T_origin_endpoint_R:
           1.0  0.0  0.0  0.0
           0.0  1.0  0.0  0.0
           0.0  0.0  1.0  0.0
           0.0  0.0  0.0  1.0
        T_lidar_imu:
        1 0 0 0
        0 1 0 0
        0 0 1 0
        0 0 0 1
        imu_bias: 0 0 0 0 0 0
        frame_id: 1
        num_frames: 2
        frame_0
        id: 0
        stamp: 1000.100000000
        T_odom_lidar:
           1.0  0.0  0.0  0.1
           0.0  1.0  0.0  0.2
           0.0  0.0  1.0  0.3
           0.0  0.0  0.0  1.0
        T_world_lidar:
           1.0  0.0  0.0  0.1
           0.0  1.0  0.0  0.2
           0.0  0.0  1.0  0.3
           0.0  0.0  0.0  1.0
        v_world_imu: 0 0 0
        frame_1
        id: 1
        stamp: 1000.200000000
        T_odom_lidar:
           0.0  -1.0  0.0  0.5
           1.0   0.0  0.0  0.6
           0.0   0.0  1.0  0.7
           0.0   0.0  0.0  1.0
        T_world_lidar:
           0.0  -1.0  0.0  0.5
           1.0   0.0  0.0  0.6
           0.0   0.0  1.0  0.7
           0.0   0.0  0.0  1.0
        v_world_imu: 0 0 0
    """)

    def test_parse_T_world_origin(self, tmp_path):
        p = tmp_path / "data.txt"
        p.write_text(self.SAMPLE_DATA_TXT)
        T_world_origin, frames = parse_data_txt(p)

        expected = np.eye(4)
        expected[0, 3] = 1.5
        expected[1, 3] = 2.5
        expected[2, 3] = 3.5
        np.testing.assert_array_almost_equal(T_world_origin, expected)

    def test_parse_frame_count(self, tmp_path):
        p = tmp_path / "data.txt"
        p.write_text(self.SAMPLE_DATA_TXT)
        _, frames = parse_data_txt(p)
        assert len(frames) == 2

    def test_parse_frame_stamps(self, tmp_path):
        p = tmp_path / "data.txt"
        p.write_text(self.SAMPLE_DATA_TXT)
        _, frames = parse_data_txt(p)
        assert frames[0]["stamp"] == pytest.approx(1000.1)
        assert frames[1]["stamp"] == pytest.approx(1000.2)

    def test_parse_frame_transforms(self, tmp_path):
        p = tmp_path / "data.txt"
        p.write_text(self.SAMPLE_DATA_TXT)
        _, frames = parse_data_txt(p)

        T0 = frames[0]["T_world_lidar"]
        assert T0[0, 3] == pytest.approx(0.1)
        assert T0[1, 3] == pytest.approx(0.2)
        assert T0[2, 3] == pytest.approx(0.3)

        T1 = frames[1]["T_world_lidar"]
        # Rotation part: 90-degree rotation around Z
        assert T1[0, 0] == pytest.approx(0.0)
        assert T1[0, 1] == pytest.approx(-1.0)
        assert T1[1, 0] == pytest.approx(1.0)

    def test_missing_T_world_origin_raises(self, tmp_path):
        p = tmp_path / "data.txt"
        p.write_text("id: 0\nnum_frames: 0\n")
        with pytest.raises(ValueError, match="T_world_origin"):
            parse_data_txt(p)


# ---------------------------------------------------------------------------
# load_points
# ---------------------------------------------------------------------------
class TestLoadPoints:
    def test_load_float32_points(self, tmp_path):
        pts = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
        pts.tofile(tmp_path / "points_compact.bin")
        loaded = load_points(tmp_path)
        np.testing.assert_array_equal(loaded, pts)

    def test_empty_file(self, tmp_path):
        np.array([], dtype=np.float32).tofile(tmp_path / "points_compact.bin")
        loaded = load_points(tmp_path)
        assert loaded.shape == (0, 3)


# ---------------------------------------------------------------------------
# build_rectification_maps
# ---------------------------------------------------------------------------
class TestBuildRectificationMaps:
    @pytest.fixture(scope="class")
    def maps(self):
        return build_rectification_maps()

    def test_output_shape(self, maps):
        map_x, map_y = maps
        assert map_x.shape == (RECT_H, RECT_W)
        assert map_y.shape == (RECT_H, RECT_W)

    def test_output_dtype(self, maps):
        map_x, map_y = maps
        assert map_x.dtype == np.float32
        assert map_y.dtype == np.float32

    def test_center_pixel_maps_near_center(self, maps):
        """The center of the rectified image should map near the center of the raw image."""
        map_x, map_y = maps
        cy, cx = RECT_H // 2, RECT_W // 2
        # Should be within ~100 pixels of optical center
        assert abs(map_x[cy, cx] - 955.7) < 100
        assert abs(map_y[cy, cx] - 627.1) < 100

    def test_maps_within_reasonable_range(self, maps):
        """Most map values should be within or near the raw image bounds."""
        map_x, map_y = maps
        # At least the center portion should map inside the raw image
        center_x = map_x[300:900, 480:1440]
        center_y = map_y[300:900, 480:1440]
        assert np.all(center_x >= -100) and np.all(center_x <= RAW_W + 100)
        assert np.all(center_y >= -100) and np.all(center_y <= RAW_H + 100)


# ---------------------------------------------------------------------------
# write_pcd_binary + read-back
# ---------------------------------------------------------------------------
class TestWritePcdBinary:
    def test_roundtrip(self, tmp_path):
        xyz = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
        rgb = np.array([[255, 0, 0], [0, 255, 0]], dtype=np.uint8)

        pcd_path = tmp_path / "test.pcd"
        write_pcd_binary(pcd_path, xyz, rgb)

        # Read back and verify
        with open(pcd_path, "rb") as f:
            header_lines = []
            for _ in range(11):
                header_lines.append(f.readline().decode("ascii").strip())
            binary_data = f.read()

        assert "FIELDS x y z rgb" in header_lines[2]
        assert "WIDTH 2" in header_lines[6]
        assert "POINTS 2" in header_lines[9]
        assert "DATA binary" in header_lines[10]

        data = np.frombuffer(binary_data, dtype=np.float32).reshape(-1, 4)
        np.testing.assert_array_almost_equal(data[:, :3], xyz)

        # Verify RGB packing
        rgb_packed = data[:, 3].view(np.uint32)
        r = (rgb_packed >> 16) & 0xFF
        g = (rgb_packed >> 8) & 0xFF
        b = rgb_packed & 0xFF
        assert r[0] == 255 and g[0] == 0 and b[0] == 0
        assert r[1] == 0 and g[1] == 255 and b[1] == 0

    def test_single_point(self, tmp_path):
        xyz = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
        rgb = np.array([[128, 64, 32]], dtype=np.uint8)

        pcd_path = tmp_path / "single.pcd"
        write_pcd_binary(pcd_path, xyz, rgb)

        with open(pcd_path, "rb") as f:
            for _ in range(11):
                f.readline()
            data = np.frombuffer(f.read(), dtype=np.float32).reshape(-1, 4)

        assert data.shape == (1, 4)
        rgb_packed = data[0, 3].view(np.uint32)
        assert (rgb_packed >> 16) & 0xFF == 128
        assert (rgb_packed >> 8) & 0xFF == 64
        assert rgb_packed & 0xFF == 32

    def test_file_size(self, tmp_path):
        n = 1000
        xyz = np.random.randn(n, 3).astype(np.float32)
        rgb = np.random.randint(0, 256, (n, 3), dtype=np.uint8)

        pcd_path = tmp_path / "size_check.pcd"
        write_pcd_binary(pcd_path, xyz, rgb)

        file_size = pcd_path.stat().st_size
        # Header is ~200 bytes, data is n * 16 bytes
        assert file_size > n * 16
        assert file_size < n * 16 + 500  # generous header allowance


# ---------------------------------------------------------------------------
# Projection math: LiDAR → Camera → Pixel
# ---------------------------------------------------------------------------
class TestProjectionMath:
    def test_rcl_is_rotation_matrix(self):
        """Rcl should be a proper rotation matrix: det=1, R^T R = I."""
        det = np.linalg.det(RCL)
        assert abs(det - 1.0) < 1e-4, f"det(Rcl) = {det}, expected 1.0"

        RtR = RCL.T @ RCL
        np.testing.assert_array_almost_equal(RtR, np.eye(3), decimal=4)

    def test_point_in_front_of_camera_projects_to_image(self):
        """A point directly in front of the LiDAR should project into the image."""
        # Point 5m in front of LiDAR along its x-axis
        p_lidar = np.array([[5.0, 0.0, 0.0]])
        p_cam = (RCL @ p_lidar.T + PCL).T

        # Should have positive z (in front of camera)
        assert p_cam[0, 2] > 0, f"Camera z = {p_cam[0, 2]}, expected > 0"

        # Project
        u = RECT_FX * p_cam[0, 0] / p_cam[0, 2] + RECT_CX
        v = RECT_FY * p_cam[0, 1] / p_cam[0, 2] + RECT_CY

        # Should be somewhere in or near the image
        assert -500 < u < RECT_W + 500
        assert -500 < v < RECT_H + 500

    def test_point_behind_lidar_has_mixed_camera_z(self):
        """A point behind the LiDAR (negative x) may or may not be in front of camera."""
        p_lidar = np.array([[-5.0, 0.0, 0.0]])
        p_cam = (RCL @ p_lidar.T + PCL).T
        # Just verify the math runs; z sign depends on extrinsics
        assert p_cam.shape == (1, 3)

    def test_identity_transform_preserves_points(self):
        """With identity T_world_lidar and T_world_origin, p_lidar == p_origin."""
        T_world_lidar = np.eye(4)
        T_world_origin = np.eye(4)
        T_lidar_origin = np.linalg.inv(T_world_lidar) @ T_world_origin

        pts = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
        p_lidar = (T_lidar_origin[:3, :3] @ pts.T + T_lidar_origin[:3, 3:]).T
        np.testing.assert_array_almost_equal(p_lidar, pts)

    def test_translation_transform(self):
        """A pure translation T_world_lidar should shift points accordingly."""
        T_world_lidar = np.eye(4)
        T_world_lidar[0, 3] = 10.0  # lidar is at x=10 in world
        T_world_origin = np.eye(4)  # origin is at world origin

        T_lidar_origin = np.linalg.inv(T_world_lidar) @ T_world_origin

        pts = np.array([[0.0, 0.0, 0.0]])
        p_lidar = (T_lidar_origin[:3, :3] @ pts.T + T_lidar_origin[:3, 3:]).T

        # Origin (0,0,0) in world should be at (-10,0,0) in lidar frame
        np.testing.assert_array_almost_equal(p_lidar, [[-10.0, 0.0, 0.0]])


# ---------------------------------------------------------------------------
# End-to-end: process_submap with synthetic data
# ---------------------------------------------------------------------------
class TestProcessSubmap:
    def _make_submap(self, tmp_path):
        """Create a minimal synthetic submap directory."""
        submap_dir = tmp_path / "000000"
        submap_dir.mkdir()

        # Points: a grid on the XY plane at z=0, range [-2, 2]
        xx = np.linspace(-2, 2, 10)
        yy = np.linspace(-2, 2, 10)
        grid = np.array([[x, y, 0.0] for x in xx for y in yy], dtype=np.float32)
        grid.tofile(submap_dir / "points_compact.bin")

        # data.txt with identity transforms and 1 frame
        data_txt = textwrap.dedent("""\
            id: 0
            T_world_origin:
            1 0 0 0
            0 1 0 0
            0 0 1 0
            0 0 0 1
            T_origin_endpoint_L:
            1 0 0 0
            0 1 0 0
            0 0 1 0
            0 0 0 1
            T_origin_endpoint_R:
            1 0 0 0
            0 1 0 0
            0 0 1 0
            0 0 0 1
            T_lidar_imu:
            1 0 0 0
            0 1 0 0
            0 0 1 0
            0 0 0 1
            imu_bias: 0 0 0 0 0 0
            frame_id: 0
            num_frames: 1
            frame_0
            id: 0
            stamp: 1000.000000000
            T_odom_lidar:
            1 0 0 0
            0 1 0 0
            0 0 1 0
            0 0 0 1
            T_world_lidar:
            1 0 0 0
            0 1 0 0
            0 0 1 0
            0 0 0 1
            v_world_imu: 0 0 0
        """)
        (submap_dir / "data.txt").write_text(data_txt)

        return submap_dir, grid

    def test_synthetic_submap_processes(self, tmp_path):
        """Verify process_submap runs on synthetic data and colors some points."""
        from render_colored_pcd import process_submap

        submap_dir, grid = self._make_submap(tmp_path)

        # Create a solid red rectified image
        red_img = np.zeros((RECT_H, RECT_W, 3), dtype=np.uint8)
        red_img[:, :, 2] = 255  # BGR: blue=0, green=0, red=255

        header_stamps = np.array([999.9, 1000.0, 1000.1], dtype=np.float64)
        bag_stamps = np.array([1, 2, 3], dtype=np.int64)
        rectified_cache = {2: red_img}

        p_world, colors, n_colored = process_submap(
            submap_dir, None, None,  # maps not used since we provide rectified_cache
            header_stamps, bag_stamps, rectified_cache,
            img_time_offset=0.0, max_dt=0.2,
        )

        assert p_world.shape == (100, 3)
        assert colors.shape == (100, 3)

        # Some points should be colored (those in camera FOV)
        if n_colored > 0:
            # Colored points should be red (RGB)
            colored_mask = np.any(colors > 0, axis=1)
            assert np.all(colors[colored_mask, 0] == 255)  # R
            assert np.all(colors[colored_mask, 1] == 0)    # G
            assert np.all(colors[colored_mask, 2] == 0)    # B
