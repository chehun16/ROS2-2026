# ROS2-2026: Camera 기반 BEV Occupancy Map + 자율이동 시스템

## 프로젝트 개요
RGBD 카메라의 Depth 이미지를 Bird's Eye View(BEV) Occupancy Grid로 변환하여
실내 환경에서 자율이동 로봇을 운용하는 ROS2 기반 시스템.

## 시스템 구조

```
[Gazebo 시뮬레이터]
  TurtleBot3 Waffle + RGBD 카메라 + 실내 환경(House)
          │
          ▼ /camera/color/image_raw
          ▼ /camera/depth/image_raw
          ▼ /camera/color/camera_info
┌─────────────────────────┐
│  Node 1: rgbd_processor │  RGBD 시간동기화 + Depth 전처리
└─────────────────────────┘
          │ /processed/depth
          │ /processed/camera_info
          ▼
┌─────────────────────────┐
│  Node 2: bev_occupancy  │  ★ 핵심: Depth → BEV Occupancy Grid
│                         │  (카메라 역투영 → 좌표변환 → 2D 투영)
└─────────────────────────┘
          │ /bev_map/occupancy   (nav_msgs/OccupancyGrid)
          │ /bev_map/pointcloud  (sensor_msgs/PointCloud2)
          ▼
    ┌──────────┐    ┌──────────────────────┐
    │   Nav2   │    │  Node 3: goal_sender │
    │ (기존    │◄───│  목표 지점 전송 +    │
    │  패키지) │    │  이동 상태 모니터링  │
    └──────────┘    └──────────────────────┘
          │
          ▼
       RViz2 시각화
       (BEV Map + PointCloud + 로봇 경로)
```

## BEV 변환 핵심 수학

```
Depth 이미지 픽셀 (u, v, d)
    ↓ 카메라 핀홀 역투영
3D 포인트 (카메라 좌표계)
    X_cam = (u - cx) * d / fx
    Y_cam = (v - cy) * d / fy
    Z_cam = d
    ↓ 좌표계 변환 + 카메라 높이 보정 (Rx(-pitch) → R_axis)
3D 포인트 (로봇 기준 월드 좌표계)
    ↓ 높이 필터링 (0.05m ~ 2.0m → 장애물)
2D BEV Occupancy Grid
    0: 빈 공간 / 100: 장애물 / -1: 미탐색
```

## 패키지 구조

```
ROS2-2026/
├── Dockerfile
├── docker-compose.yml
├── run.sh
└── src/
    └── bev_navigation/
        ├── bev_navigation/
        │   ├── rgbd_processor_node.py   # Node 1
        │   ├── bev_occupancy_node.py    # Node 2 (핵심)
        │   ├── goal_sender_node.py      # Node 3
        │   └── utils/
        │       └── bev_utils.py         # 수학 변환 로직
        ├── config/
        │   ├── params.yaml
        │   ├── nav2_params.yaml
        │   └── bev_navigation.rviz
        ├── launch/
        │   └── bev_navigation.launch.py
        ├── test/
        │   └── test_bev_utils.py
        ├── package.xml
        └── setup.py
```

## 필수 요구사항 충족

| 요구사항 | 충족 방법 |
|---------|----------|
| ROS2 기반 | ROS2 Humble, ament_python |
| 최소 3개 노드 | rgbd_processor / bev_occupancy / goal_sender |
| 센서 데이터 활용 | RGBD 카메라 (RGB + Depth 이미지) |
| 결과 시각화 | RViz2: BEV OccupancyGrid + PointCloud + 경로 |
| 시뮬레이션 | Gazebo + TurtleBot3 Waffle |
| Demo | 실내 House 환경에서 장애물 회피 자율이동 |

---

## 환경 설정 가이드

> **ROS2 버전 호환표**
> | OS | 권장 방식 | ROS2 버전 |
> |----|----------|----------|
> | Ubuntu 22.04 | 네이티브 설치 | Humble (공식 지원) |
> | Ubuntu 24.04 | Docker 권장 | Humble은 미지원 → Docker로 해결 |
> | macOS | Docker 필수 | - |
> | Windows | Docker 필수 | - |

---

## Ubuntu 22.04 (네이티브 — 권장)

ROS2 Humble을 직접 설치하는 방법입니다. 가장 안정적입니다.

### Step 1. ROS2 Humble 설치

```bash
# UTF-8 로케일 설정
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ROS2 apt 저장소 등록
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS2 Humble Desktop 설치 (RViz2 포함)
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-desktop
```

### Step 2. 의존 패키지 설치

```bash
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-simulations \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-depth-image-proc \
    ros-humble-gazebo-ros-pkgs

pip3 install numpy opencv-python-headless
```

### Step 3. rosdep 초기화

```bash
sudo rosdep init
rosdep update
```

### Step 4. 환경변수 설정

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=waffle"    >> ~/.bashrc
source ~/.bashrc
```

### Step 5. 워크스페이스 빌드

```bash
cd ~/ROS2-2026
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select bev_navigation
source install/setup.bash
```

### Step 6. 실행

```bash
ros2 launch bev_navigation bev_navigation.launch.py
```

---

## Ubuntu 24.04 (Docker 사용)

Ubuntu 24.04에서 ROS2 Humble은 공식 지원하지 않습니다.
Docker를 통해 Ubuntu 22.04 + Humble 환경을 구성합니다.

### Step 1. Docker 설치

```bash
sudo apt update
sudo apt install -y ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
    | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
    https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) stable" \
    | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

# sudo 없이 docker 사용
sudo usermod -aG docker $USER
newgrp docker
```

### Step 2. GUI 포워딩 설정 (RViz2, Gazebo 화면 출력)

```bash
# X11 로컬 접근 허용
xhost +local:docker
```

### Step 3. 워크스페이스로 이동 후 빌드 및 실행

```bash
cd ~/ROS2-2026

# 이미지 빌드 (최초 1회, 10~20분 소요)
docker compose build

# 전체 시스템 실행
DISPLAY=$DISPLAY docker compose up bev_full

# 개발용 쉘 접속
DISPLAY=$DISPLAY docker compose run --rm dev bash
```

### Step 4. 컨테이너 내부에서 빌드 (코드 수정 후)

```bash
# 컨테이너 내부
cd /ros2_ws
colcon build --packages-select bev_navigation
source install/setup.bash
ros2 launch bev_navigation bev_navigation.launch.py
```

---

## macOS

Mac은 ROS2를 네이티브로 지원하지 않습니다. Docker + XQuartz를 사용합니다.

### Step 1. Homebrew 설치 (없는 경우)

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

### Step 2. Docker Desktop 설치

```bash
brew install --cask docker
```

Docker Desktop 앱을 실행하고 상단 메뉴바에 고래 아이콘이 뜰 때까지 대기.

### Step 3. XQuartz 설치 (GUI 포워딩)

```bash
brew install --cask xquartz
```

설치 후 **반드시 로그아웃 → 재로그인** (또는 재시작).

### Step 4. XQuartz 설정

XQuartz 앱 실행 → 메뉴바 `XQuartz` → `Settings` → `Security` 탭
→ **"Allow connections from network clients" 체크** → XQuartz 재시작

### Step 5. X11 접근 허용

```bash
# 터미널에서 (매 세션마다 실행)
xhost +localhost
```

### Step 6. 이미지 빌드 및 실행

```bash
cd ~/ROS2-2026
chmod +x run.sh

# 이미지 빌드 (최초 1회, 10~20분 소요)
./run.sh build

# 전체 시스템 실행
./run.sh run

# 자주 쓰는 명령
./run.sh gazebo   # Gazebo만 실행
./run.sh rviz     # RViz2만 실행
./run.sh dev      # 개발용 쉘
./run.sh rebuild  # 코드 수정 후 재빌드
```

> **Apple Silicon (M1/M2/M3) 주의사항**
> Docker Desktop에서 `Settings` → `General` → **"Use Rosetta for x86/amd64 emulation"** 활성화.
> 또는 `docker-compose.yml`의 build 섹션에 `platform: linux/amd64` 추가.

---

## Windows

Windows에서는 WSL2 + Docker Desktop을 사용합니다.

### Step 1. WSL2 활성화

PowerShell을 **관리자 권한**으로 실행:

```powershell
wsl --install
wsl --set-default-version 2
```

재시작 후 Microsoft Store에서 **Ubuntu 22.04** 설치.

### Step 2. Docker Desktop 설치

[Docker Desktop for Windows](https://www.docker.com/products/docker-desktop/) 다운로드 후 설치.

설치 완료 후 Docker Desktop 실행 → `Settings` → `Resources` → `WSL Integration`
→ Ubuntu 22.04 토글 **활성화**.

### Step 3. VcXsrv 설치 (GUI 포워딩)

[VcXsrv](https://sourceforge.net/projects/vcxsrv/) 다운로드 후 설치.

VcXsrv 실행 시 설정:
- Display number: `0`
- **"Disable access control" 체크**
- `-ac` 옵션 추가

### Step 4. WSL2 Ubuntu 내부에서 환경 설정

WSL2 Ubuntu 터미널 열기:

```bash
# Windows 호스트 IP 확인 및 DISPLAY 설정
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
echo "export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0" >> ~/.bashrc

# X11 접근 허용
export LIBGL_ALWAYS_INDIRECT=1
```

### Step 5. 워크스페이스 클론 및 실행

```bash
# WSL2 Ubuntu 터미널
cd ~
git clone <repo-url> ROS2-2026
cd ROS2-2026

# 이미지 빌드 (최초 1회)
docker compose build

# 전체 시스템 실행
docker compose up bev_full
```

---

## 빠른 실행 요약

| 환경 | 실행 명령 |
|------|----------|
| Ubuntu 22.04 | `ros2 launch bev_navigation bev_navigation.launch.py` |
| Ubuntu 24.04 | `DISPLAY=$DISPLAY docker compose up bev_full` |
| macOS | `./run.sh run` |
| Windows (WSL2) | `docker compose up bev_full` |

## 테스트 실행

```bash
# ROS2 환경 없이 Mac/로컬에서 바로 실행 가능
pip install numpy pytest
pytest src/bev_navigation/test/test_bev_utils.py -v

# colcon test (ROS2 환경)
colcon test --packages-select bev_navigation
colcon test-result --verbose
```
