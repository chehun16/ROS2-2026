# 프로젝트 제안서

**과목**: 로보틱스 / ROS2 실습
**작성일**: 2026-03-26

---

## (1) 주제

**Camera 기반 Depth 정보를 활용한 BEV Occupancy Map 생성 및 자율 이동 로봇 시스템**

RGBD 카메라에서 획득한 Depth 이미지를 핀홀 카메라 모델 역투영(back-projection)으로 3D 포인트클라우드로 변환하고, 이를 Bird's Eye View(BEV) Occupancy Grid로 투영하여 ROS2 Nav2 기반 자율 이동에 활용하는 시스템을 구현한다.

---

## (2) 목표

### 최종 목표
Gazebo 시뮬레이터의 실내 환경(TurtleBot3 House)에서 RGBD 카메라만을 이용해 실시간 BEV 장애물 맵을 생성하고, 이를 기반으로 목표 지점까지 자율 이동하는 로봇 시스템을 완성한다.

### 세부 목표

| # | 목표 | 내용 |
|---|------|------|
| 1 | **센서 처리** | RGBD 카메라 RGB·Depth 스트림 시간 동기화 및 전처리 |
| 2 | **BEV 변환** | 핀홀 역투영 + 좌표계 변환으로 Depth → BEV Occupancy Grid 생성 |
| 3 | **자율 이동** | 생성한 BEV 맵을 Nav2 코스트맵에 연결하여 경로 계획 및 장애물 회피 |
| 4 | **시각화** | RViz2에서 BEV 맵, 포인트클라우드, 로봇 경로를 실시간으로 표시 |
| 5 | **검증** | 수학 변환 로직(역투영, 좌표변환, BEV 투영) 단위 테스트로 정확성 확인 |

### 필수 요구사항 충족

| 요구사항 | 달성 방법 |
|---------|----------|
| ROS2 기반 | ROS2 Humble + ament_python |
| 최소 3개 노드 | rgbd_processor / bev_occupancy / goal_sender |
| 센서 데이터 | RGBD 카메라 (Depth 이미지 + CameraInfo) |
| 시각화 | RViz2 — BEV OccupancyGrid + PointCloud2 + 경로 |
| 시뮬레이션 | Gazebo + TurtleBot3 Waffle |

---

## (3) 진행 방법

### 시스템 구조

```
[Gazebo 시뮬레이터]
  TurtleBot3 Waffle Pi + RGBD 카메라 + House 실내 환경
          │
          ▼  /camera/color/image_raw
             /camera/depth/image_raw
             /camera/color/camera_info
┌──────────────────────────────┐
│  Node 1: rgbd_processor_node │
│  - RGB + Depth 시간 동기화   │
│  - Depth 유효 범위 클리핑    │
│  - NaN / inf 마스킹          │
└──────────────────────────────┘
          │ /processed/depth
          │ /processed/camera_info
          ▼
┌──────────────────────────────┐
│  Node 2: bev_occupancy_node  │  ← 핵심 구현 노드
│                              │
│  Step 1. 핀홀 역투영         │
│    X = (u-cx)*d/fx           │
│    Y = (v-cy)*d/fy           │
│    Z = d                     │
│                              │
│  Step 2. 좌표계 변환          │
│    Rx(-pitch) + 축 재매핑    │
│    cam → robot base_link     │
│                              │
│  Step 3. BEV 투영            │
│    높이 필터링 → 2D 격자     │
│    0:빈공간 / 100:장애물     │
└──────────────────────────────┘
          │ /bev_map/occupancy   (OccupancyGrid)
          │ /bev_map/pointcloud  (PointCloud2)
          ▼
┌──────────────────────────────┐     ┌─────────────────────────┐
│  SLAM Toolbox                │     │  Node 3: goal_sender    │
│  (실시간 맵 생성 → /map)     │     │  - Nav2 Action Client   │
└──────────────────────────────┘     │  - 목표 지점 전송       │
          │                          │  - 이동 상태 모니터링   │
          ▼                          └─────────────────────────┘
┌──────────────────────────────┐               │
│  Nav2                        │◄──────────────┘
│  - Global Costmap (/map)     │
│  - Local Costmap             │
│    (/bev_map/pointcloud 활용)│
│  - 경로 계획 (A*)            │
│  - 속도 제어 (DWB)           │
└──────────────────────────────┘
          │
          ▼
       RViz2 시각화
```

### 핵심 수학: BEV 변환 파이프라인

```
① Depth 이미지 픽셀 (u, v, d)
       ↓ 핀홀 역투영 (카메라 내부 파라미터 K 사용)
② 3D 포인트 — 카메라 좌표계
   X_cam = (u - cx) * d / fx
   Y_cam = (v - cy) * d / fy
   Z_cam = d
       ↓ Rx(-pitch) 회전 + 축 재매핑 + 카메라 높이 보정
③ 3D 포인트 — 로봇 기준 월드 좌표계 (base_link)
   X_world: 앞,  Y_world: 왼쪽,  Z_world: 위
       ↓ 높이 필터링 (0.05m < Z < 2.0m → 장애물)
④ 2D BEV Occupancy Grid
   cell(row, col) = 100 (장애물) / 0 (빈 공간) / -1 (미탐색)
```

### 개발 환경

| 항목 | 내용 |
|------|------|
| OS | Ubuntu 22.04 (네이티브) / Docker (Mac·Windows) |
| ROS2 | Humble Hawksbill |
| 시뮬레이터 | Gazebo Classic |
| 로봇 | TurtleBot3 Waffle Pi |
| 언어 | Python 3.10 |
| 주요 패키지 | Nav2, slam_toolbox, cv_bridge, message_filters |

### 구현 단계

```
Week 1-2   환경 구축
           └─ ROS2 Humble + Gazebo + TurtleBot3 설치 및 동작 확인

Week 3-4   Node 1, 2 구현
           └─ RGBD 처리 + BEV 변환 수학 구현 및 단위 테스트

Week 5-6   Nav2 연동
           └─ BEV 맵 → Nav2 코스트맵 연결 + 경로 계획 동작 확인

Week 7-8   Node 3 + 통합 테스트
           └─ goal_sender 구현 + 전체 시스템 Launch + RViz2 시각화

Week 9-10  실험 및 발표 준비
           └─ 다양한 환경 테스트 + Demo 영상 촬영 + 발표 자료 준비
```

---

## (4) 참고자료

### ROS2 / 시뮬레이션

- ROS2 Humble 공식 문서
  https://docs.ros.org/en/humble/index.html

- Nav2 공식 문서 (Navigation2)
  https://navigation.ros.org

- TurtleBot3 e-Manual
  https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/

- SLAM Toolbox
  Macenski, S. et al., "SLAM Toolbox: SLAM for the dynamic world," *Journal of Open Source Software*, 2021.
  https://github.com/SteveMacenski/slam_toolbox

- Gazebo 공식 문서
  https://gazebosim.org/docs

### 컴퓨터 비전 / BEV

- Hartley, R. & Zisserman, A., *Multiple View Geometry in Computer Vision*, 2nd ed., Cambridge University Press, 2004.
  (핀홀 카메라 모델, 역투영 수식의 이론적 근거)

- Mallot, H. A. et al., "Inverse Perspective Mapping Simplifies Optical Flow Computation and Obstacle Detection," *Biological Cybernetics*, 1991.
  (IPM 기반 BEV 변환의 원조 논문)

- Li, Z. et al., "BEVDepth: Acquisition of Reliable Depth for Multi-view 3D Object Detection," *AAAI*, 2023.
  https://arxiv.org/abs/2206.10092
  (카메라 기반 BEV 인식 최신 동향 참고)

- Liu, Z. et al., "BEVFusion: Multi-Task Multi-Sensor Fusion with Unified Bird's-Eye View Representation," *ICRA*, 2023.
  https://arxiv.org/abs/2205.13542

### 포인트클라우드 / Occupancy

- ROS2 sensor_msgs/PointCloud2 메시지 정의
  https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html

- nav_msgs/OccupancyGrid 메시지 정의
  https://docs.ros2.org/latest/api/nav_msgs/msg/OccupancyGrid.html

- Nav2 Costmap 2D 플러그인 문서
  https://navigation.ros.org/configuration/packages/configuring-costmaps.html

### 카메라 캘리브레이션

- Zhang, Z., "A Flexible New Technique for Camera Calibration," *IEEE TPAMI*, 2000.
  (CameraInfo K 행렬 파라미터의 이론적 배경)

- OpenCV 카메라 캘리브레이션 문서
  https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
