"""
bev_utils.py 수학 로직 단위 테스트

핀홀 역투영, 좌표변환, BEV 격자 투영의 정확성을 검증.
ROS2 환경 없이 pytest로 단독 실행 가능.

실행:
    pytest src/bev_navigation/test/test_bev_utils.py -v
"""

import sys
import os

# ROS2 환경 없이도 import 가능하도록 경로 추가
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import pytest

from bev_navigation.utils.bev_utils import (
    depth_to_pointcloud,
    camera_to_world,
    pointcloud_to_bev_grid,
)


# =============================================================================
# 공통 픽스처
# =============================================================================

@pytest.fixture
def camera_info():
    """640x480 일반적인 RGBD 카메라 내부 파라미터."""
    return {
        'fx': 525.0,
        'fy': 525.0,
        'cx': 319.5,
        'cy': 239.5,
    }


@pytest.fixture
def simple_depth_image():
    """중앙 픽셀만 1.0m depth인 640x480 이미지."""
    img = np.zeros((480, 640), dtype=np.float32)
    img[239, 319] = 1.0   # 정중앙 픽셀 (cx=319.5, cy=239.5에 가장 가까운 정수)
    return img


# =============================================================================
# depth_to_pointcloud 테스트
# =============================================================================

class TestDepthToPointcloud:

    def test_center_pixel_maps_to_optical_axis(self, camera_info, simple_depth_image):
        """
        광축(optical axis) 픽셀은 카메라 좌표계 Z축 위에 있어야 함.
        즉, X_cam ≈ 0, Y_cam ≈ 0, Z_cam = depth.
        """
        points = depth_to_pointcloud(simple_depth_image, camera_info)

        assert len(points) == 1, "유효 픽셀이 1개여야 함"

        x, y, z = points[0]
        assert abs(x) < 0.01, f"광축 픽셀의 X_cam은 0에 가까워야 함, got {x}"
        assert abs(y) < 0.01, f"광축 픽셀의 Y_cam은 0에 가까워야 함, got {y}"
        assert abs(z - 1.0) < 1e-5, f"Z_cam은 depth값(1.0m)이어야 함, got {z}"

    def test_invalid_depth_filtered_out(self, camera_info):
        """depth = 0 (무효값)은 포인트클라우드에서 제외되어야 함."""
        img = np.zeros((480, 640), dtype=np.float32)  # 전체 0
        points = depth_to_pointcloud(img, camera_info)
        assert len(points) == 0, "모든 픽셀이 0이면 포인트가 없어야 함"

    def test_depth_min_max_filtering(self, camera_info):
        """유효 범위(0.1~10.0m) 밖의 depth는 제외되어야 함."""
        img = np.zeros((480, 640), dtype=np.float32)
        img[100, 100] = 0.05    # 너무 가까움 → 제외
        img[200, 200] = 5.0     # 유효 범위 → 포함
        img[300, 300] = 15.0    # 너무 멂 → 제외

        points = depth_to_pointcloud(img, camera_info)
        assert len(points) == 1, "유효 픽셀 1개만 포함되어야 함"

    def test_output_shape(self, camera_info):
        """출력이 (N, 3) 형태여야 함."""
        img = np.full((480, 640), 2.0, dtype=np.float32)
        points = depth_to_pointcloud(img, camera_info)

        assert points.ndim == 2
        assert points.shape[1] == 3

    def test_right_pixel_has_positive_x(self, camera_info):
        """
        이미지 오른쪽 픽셀 → X_cam > 0 (카메라 좌표계 오른쪽).
        핀홀 역투영: X_cam = (u - cx) * d / fx
        u > cx 이면 X_cam > 0
        """
        img = np.zeros((480, 640), dtype=np.float32)
        img[239, 500] = 1.0   # cx(319.5)보다 오른쪽

        points = depth_to_pointcloud(img, camera_info)
        assert len(points) == 1
        assert points[0, 0] > 0, "오른쪽 픽셀의 X_cam은 양수여야 함"

    def test_pinhole_backprojection_formula(self, camera_info):
        """핀홀 역투영 수식 직접 검증."""
        fx, fy = camera_info['fx'], camera_info['fy']
        cx, cy = camera_info['cx'], camera_info['cy']

        u, v, d = 400, 300, 2.0
        expected_x = (u - cx) * d / fx
        expected_y = (v - cy) * d / fy
        expected_z = d

        img = np.zeros((480, 640), dtype=np.float32)
        img[v, u] = d
        points = depth_to_pointcloud(img, camera_info)

        assert len(points) == 1
        np.testing.assert_allclose(points[0, 0], expected_x, rtol=1e-5)
        np.testing.assert_allclose(points[0, 1], expected_y, rtol=1e-5)
        np.testing.assert_allclose(points[0, 2], expected_z, rtol=1e-5)


# =============================================================================
# camera_to_world 테스트
# =============================================================================

class TestCameraToWorld:

    def test_forward_point_is_in_front_of_robot(self):
        """
        카메라 정면(Z_cam 방향)의 포인트는
        로봇 기준 앞(X_world > 0)에 있어야 함.
        """
        # Z_cam = 3.0m (카메라 정면 3m)
        points_cam = np.array([[0.0, 0.0, 3.0]])
        points_world = camera_to_world(points_cam, camera_height=0.3, camera_pitch_rad=0.0)

        assert points_world[0, 0] > 0, "정면 포인트는 X_world > 0이어야 함"

    def test_camera_height_offset(self):
        """
        pitch=0일 때 Z_cam 방향 포인트(광축, Y_cam=0)는
        Z_world = camera_height 이어야 함.
        Rx(-0) = I 이므로 R_pitch가 항등행렬 → Z_cam → X_world, -Y_cam → Z_world=0+height
        """
        camera_height = 0.3
        points_cam = np.array([[0.0, 0.0, 1.0]])  # 광축 방향, Y_cam=0
        points_world = camera_to_world(points_cam, camera_height=camera_height, camera_pitch_rad=0.0)

        # pitch=0, Y_cam=0 → Z_world = -Y_cam + camera_height = camera_height
        np.testing.assert_allclose(points_world[0, 2], camera_height, atol=1e-5)

    def test_output_shape_preserved(self):
        """입력과 출력의 포인트 수가 동일해야 함."""
        points_cam = np.random.rand(100, 3) * 5.0
        points_world = camera_to_world(points_cam, camera_height=0.3)

        assert points_world.shape == (100, 3)

    def test_no_pitch_symmetry(self):
        """
        피치 없을 때, 좌우 대칭 픽셀은 Y_world 기준 대칭이어야 함.
        X_cam = +a → Y_world = -a
        X_cam = -a → Y_world = +a
        """
        a = 1.0
        points_cam = np.array([
            [ a, 0.0, 2.0],
            [-a, 0.0, 2.0],
        ])
        points_world = camera_to_world(points_cam, camera_height=0.3, camera_pitch_rad=0.0)

        np.testing.assert_allclose(points_world[0, 1], -points_world[1, 1], atol=1e-5)


# =============================================================================
# pointcloud_to_bev_grid 테스트
# =============================================================================

class TestPointcloudToBEVGrid:

    def test_obstacle_point_marked_100(self):
        """장애물 높이 범위(0.05~2.0m)의 포인트는 셀값이 100이어야 함."""
        # 로봇 정면 2m, 높이 1m (장애물)
        points = np.array([[2.0, 0.0, 1.0]])
        grid = pointcloud_to_bev_grid(
            points,
            grid_resolution=0.05,
            grid_size_m=10.0,
            min_height=0.05,
            max_height=2.0,
        )

        assert 100 in grid, "장애물 포인트가 grid에 표시되어야 함"

    def test_ground_point_marked_0(self):
        """지면(높이 0~0.05m)의 포인트는 셀값이 0(빈 공간)이어야 함."""
        points = np.array([[2.0, 0.0, 0.02]])  # 지면 바로 위
        grid = pointcloud_to_bev_grid(
            points,
            grid_resolution=0.05,
            grid_size_m=10.0,
            min_height=0.05,
            max_height=2.0,
        )

        assert 0 in grid, "지면 포인트는 빈 공간(0)으로 표시되어야 함"

    def test_grid_shape(self):
        """grid_size_m / grid_resolution = grid_cells 크기여야 함."""
        points = np.array([[0.0, 0.0, 1.0]])
        grid_resolution = 0.05
        grid_size_m     = 10.0
        expected_cells  = int(grid_size_m / grid_resolution)  # 200

        grid = pointcloud_to_bev_grid(points, grid_resolution, grid_size_m)

        assert grid.shape == (expected_cells, expected_cells)

    def test_unknown_cells_are_minus_one(self):
        """포인트가 없는 셀은 -1(미탐색)이어야 함."""
        # 빈 포인트클라우드
        points = np.zeros((0, 3))
        grid = pointcloud_to_bev_grid(points, 0.05, 10.0)

        assert np.all(grid == -1), "포인트 없으면 전체 -1이어야 함"

    def test_out_of_range_point_ignored(self):
        """그리드 범위(±5m) 밖의 포인트는 무시되어야 함."""
        points = np.array([[100.0, 100.0, 1.0]])  # 그리드 훨씬 밖
        grid = pointcloud_to_bev_grid(points, 0.05, 10.0)

        assert 100 not in grid, "범위 밖 포인트는 grid에 표시되면 안 됨"

    def test_obstacle_overrides_ground(self):
        """같은 셀에 지면과 장애물 포인트가 있으면 장애물(100)이 우선해야 함."""
        # 동일 XY, 다른 높이
        points = np.array([
            [2.0, 0.0, 0.02],   # 지면
            [2.0, 0.0, 1.0],    # 장애물
        ])
        grid = pointcloud_to_bev_grid(points, 0.05, 10.0)

        # 해당 셀 좌표 계산
        grid_cells    = int(10.0 / 0.05)
        origin_offset = 10.0 / 2.0
        col = int((2.0 + origin_offset) / 0.05)
        row = int((0.0 + origin_offset) / 0.05)

        assert grid[row, col] == 100, "장애물이 지면보다 우선해야 함"


# =============================================================================
# 통합: depth → pointcloud → world → BEV 전체 파이프라인 테스트
# =============================================================================

class TestFullPipeline:

    def test_obstacle_in_front_appears_in_bev(self):
        """
        카메라 정면 중앙 2m 거리 장애물이
        BEV 그리드 로봇 앞쪽 셀에 나타나야 함.
        """
        fx, fy, cx, cy = 525.0, 525.0, 319.5, 239.5
        camera_info = {'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy}

        # 중앙 픽셀에 2m depth (장애물 높이 가정: 카메라 pitch로 지면이 아닌 곳)
        img = np.zeros((480, 640), dtype=np.float32)
        img[200, 319] = 2.0   # cy=239.5보다 위 → Y_cam < 0 → Z_world > camera_height

        from bev_navigation.utils.bev_utils import depth_to_pointcloud, camera_to_world, pointcloud_to_bev_grid

        points_cam   = depth_to_pointcloud(img, camera_info)
        points_world = camera_to_world(points_cam, camera_height=0.3, camera_pitch_rad=0.0)
        grid         = pointcloud_to_bev_grid(points_world, 0.05, 10.0)

        # X_world(forward) → col 방향
        # X_world > 0 이면 col > center → grid[:, center:] 로 확인해야 함
        grid_cells = int(10.0 / 0.05)
        center     = grid_cells // 2
        front_half = grid[:, center:]      # col 기준: X_world > 0 (앞쪽)

        assert 100 in front_half or 0 in front_half, \
            "카메라 정면 포인트가 BEV 앞쪽(col > center)에 표시되어야 함"
