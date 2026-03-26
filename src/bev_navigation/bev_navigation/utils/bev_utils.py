"""
BEV (Bird's Eye View) 변환 유틸리티 함수 모음.

Depth 이미지를 BEV Occupancy Grid로 변환하는 핵심 수학 로직을 담당.
"""

import numpy as np


def depth_to_pointcloud(depth_image: np.ndarray, camera_info: dict) -> np.ndarray:
    """
    Depth 이미지를 카메라 좌표계 기준 3D 포인트클라우드로 변환.

    카메라 핀홀 모델 역투영(back-projection) 공식:
        X_cam = (u - cx) * d / fx
        Y_cam = (v - cy) * d / fy
        Z_cam = d

    Args:
        depth_image: (H, W) float32 배열, 단위 미터
        camera_info: fx, fy, cx, cy 포함 딕셔너리

    Returns:
        (N, 3) 배열 - 유효한 포인트들의 [X, Y, Z] (카메라 좌표계)
    """
    fx = camera_info['fx']
    fy = camera_info['fy']
    cx = camera_info['cx']
    cy = camera_info['cy']

    h, w = depth_image.shape
    u_coords, v_coords = np.meshgrid(np.arange(w), np.arange(h))

    valid_mask = (depth_image > 0.1) & (depth_image < 10.0)
    d = depth_image[valid_mask]
    u = u_coords[valid_mask]
    v = v_coords[valid_mask]

    x_cam = (u - cx) * d / fx
    y_cam = (v - cy) * d / fy
    z_cam = d

    return np.stack([x_cam, y_cam, z_cam], axis=1)


def camera_to_world(points_cam: np.ndarray, camera_height: float,
                    camera_pitch_rad: float = 0.0) -> np.ndarray:
    """
    카메라 좌표계 → 월드 좌표계 (로봇 기준 수평면) 변환.

    카메라가 지면으로부터 camera_height(m) 높이에 있고
    pitch 각도만큼 아래를 향한다고 가정.

    카메라 좌표계 (ROS 표준):
        X_cam: 오른쪽, Y_cam: 아래, Z_cam: 앞쪽

    월드 좌표계 (로봇 기준):
        X_world: 앞, Y_world: 왼쪽, Z_world: 위

    Args:
        points_cam: (N, 3) 카메라 좌표계 포인트
        camera_height: 카메라 지면 높이 (미터)
        camera_pitch_rad: 카메라 피치각 (라디안, 아래 방향 양수)

    Returns:
        (N, 3) 월드 좌표계 포인트 [X_forward, Y_left, Z_up]
    """
    # pitch 회전 행렬 (X_cam 축 기준, Rx(-p))
    # 카메라 pitch-down(p > 0): Z_cam이 +Y_cam(아래) 방향으로 기울어짐
    # Rx(-p) 적용 시: Z_cam=[0,0,1] → [0, sin_p, cos_p] ✓
    cos_p = np.cos(camera_pitch_rad)
    sin_p = np.sin(camera_pitch_rad)

    R_pitch = np.array([
        [1,      0,      0   ],
        [0,  cos_p,  sin_p   ],
        [0, -sin_p,  cos_p   ],
    ])

    # 카메라 좌표 → 로봇 좌표 축 변환 (카메라 X→-Y_world, 카메라 Y→-Z_world, 카메라 Z→X_world)
    R_axis = np.array([
        [0,  0, 1],
        [-1, 0, 0],
        [0, -1, 0],
    ])

    R = R_axis @ R_pitch
    points_world = (R @ points_cam.T).T
    points_world[:, 2] += camera_height  # 카메라 높이 오프셋

    return points_world


def pointcloud_to_bev_grid(points_world: np.ndarray,
                           grid_resolution: float,
                           grid_size_m: float,
                           min_height: float = 0.05,
                           max_height: float = 2.0) -> np.ndarray:
    """
    3D 포인트클라우드를 BEV Occupancy Grid로 투영.

    지면(min_height)과 천장(max_height) 사이의 포인트만 장애물로 판단.

    Args:
        points_world: (N, 3) 월드 좌표계 포인트
        grid_resolution: 그리드 셀 크기 (미터/셀), 예: 0.05
        grid_size_m: 그리드 전체 크기 (미터), 예: 10.0
        min_height: 장애물 최소 높이 (미터)
        max_height: 장애물 최대 높이 (미터)

    Returns:
        (grid_cells, grid_cells) int8 배열 - 0: 빈 공간, 100: 장애물, -1: 미탐색
    """
    grid_cells = int(grid_size_m / grid_resolution)
    grid = np.full((grid_cells, grid_cells), -1, dtype=np.int8)

    # 장애물 판단 범위 필터
    obstacle_mask = (points_world[:, 2] > min_height) & (points_world[:, 2] < max_height)
    ground_mask   = (points_world[:, 2] >= 0.0) & (points_world[:, 2] <= min_height)

    origin_offset = grid_size_m / 2.0  # 그리드 중심 = 로봇 위치

    def world_to_grid(pts):
        # astype(int)은 0 방향 truncate → 음수 경계값(-0.02 → 0)이 범위 안으로 잘못 들어옴
        # np.floor 사용 시: -0.02 → -1 → valid 체크에서 올바르게 걸러짐
        col = np.floor((pts[:, 0] + origin_offset) / grid_resolution).astype(int)
        row = np.floor((pts[:, 1] + origin_offset) / grid_resolution).astype(int)
        valid = (col >= 0) & (col < grid_cells) & (row >= 0) & (row < grid_cells)
        return row[valid], col[valid]

    # 지면 포인트 → 빈 공간 (0)
    rows, cols = world_to_grid(points_world[ground_mask])
    grid[rows, cols] = 0

    # 장애물 포인트 → 점유 (100)
    rows, cols = world_to_grid(points_world[obstacle_mask])
    grid[rows, cols] = 100

    return grid
