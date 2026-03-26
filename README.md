# ROS2-2026: Camera-based BEV Occupancy Map + Autonomous Navigation

## Overview
A ROS2-based system that converts depth images from an RGBD camera into a Bird's Eye View (BEV) Occupancy Grid and uses it for autonomous indoor robot navigation.

## System Architecture

```
[Gazebo Simulator]
  TurtleBot3 Waffle + RGBD Camera + Indoor Environment (House)
          │
          ▼ /camera/color/image_raw
          ▼ /camera/depth/image_raw
          ▼ /camera/color/camera_info
┌─────────────────────────┐
│  Node 1: rgbd_processor │  RGBD time sync + depth preprocessing
└─────────────────────────┘
          │ /processed/depth
          │ /processed/camera_info
          ▼
┌─────────────────────────┐
│  Node 2: bev_occupancy  │  ★ Core: Depth → BEV Occupancy Grid
│                         │  (back-projection → coord transform → 2D projection)
└─────────────────────────┘
          │ /bev_map/occupancy   (nav_msgs/OccupancyGrid)
          │ /bev_map/pointcloud  (sensor_msgs/PointCloud2)
          ▼
    ┌──────────┐    ┌──────────────────────┐
    │   Nav2   │    │  Node 3: goal_sender │
    │          │◄───│  Send navigation goal│
    │          │    │  + monitor status    │
    └──────────┘    └──────────────────────┘
          │
          ▼
       RViz2 Visualization
       (BEV Map + PointCloud + Robot Path)
```

## BEV Transformation Math

```
Depth image pixel (u, v, d)
    ↓ Pinhole back-projection
3D point in camera frame
    X_cam = (u - cx) * d / fx
    Y_cam = (v - cy) * d / fy
    Z_cam = d
    ↓ Coordinate transform + camera height offset (Rx(-pitch) → R_axis)
3D point in robot world frame (base_link)
    ↓ Height filtering (0.05m ~ 2.0m → obstacle)
2D BEV Occupancy Grid
    0: free space / 100: obstacle / -1: unknown
```

## Package Structure

```
ROS2-2026/
├── Dockerfile
├── docker-compose.yml
├── run.sh
└── src/
    └── bev_navigation/
        ├── bev_navigation/
        │   ├── rgbd_processor_node.py   # Node 1
        │   ├── bev_occupancy_node.py    # Node 2 (core)
        │   ├── goal_sender_node.py      # Node 3
        │   └── utils/
        │       └── bev_utils.py         # Math transformation logic
        ├── config/
        │   ├── params.yaml
        │   ├── nav2_params.yaml
        │   └── bev_navigation.rviz
        ├── launch/
        │   └── bev_navigation.launch.py
        ├── urdf/
        │   └── turtlebot3_waffle_depth.urdf.xacro
        ├── test/
        │   └── test_bev_utils.py
        ├── package.xml
        └── setup.py
```

## Requirements

| Requirement | Implementation |
|-------------|----------------|
| ROS2-based | ROS2 Humble, ament_python |
| Minimum 3 nodes | rgbd_processor / bev_occupancy / goal_sender |
| Sensor data | RGBD camera (RGB + Depth images) |
| Visualization | RViz2: BEV OccupancyGrid + PointCloud + path |
| Simulation | Gazebo + TurtleBot3 Waffle |
| Demo | Obstacle-avoiding autonomous navigation in indoor House environment |

---

## Setup Guide

> **ROS2 Compatibility**
> | OS | Recommended | ROS2 Version |
> |----|-------------|-------------|
> | Ubuntu 22.04 | Native install | Humble (officially supported) |
> | Ubuntu 24.04 | Docker | Humble not supported natively → use Docker |
> | macOS | Docker + XQuartz | - |
> | Windows | Docker + WSL2 | - |

---

## Ubuntu 22.04 (Native — Recommended)

### Step 1. Install ROS2 Humble

```bash
# Set UTF-8 locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 apt repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble Desktop (includes RViz2)
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-desktop
```

### Step 2. Install Dependencies

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
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-slam-toolbox

pip3 install numpy opencv-python-headless
```

### Step 3. Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

### Step 4. Set Environment Variables

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=waffle"    >> ~/.bashrc
source ~/.bashrc
```

### Step 5. Build Workspace

```bash
cd ~/ROS2-2026
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select bev_navigation
source install/setup.bash
```

### Step 6. Run

```bash
ros2 launch bev_navigation bev_navigation.launch.py
```

---

## Ubuntu 24.04 (Docker)

ROS2 Humble is not officially supported on Ubuntu 24.04. Use Docker to run an Ubuntu 22.04 + Humble environment.

### Step 1. Install Docker

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

# Run Docker without sudo
sudo usermod -aG docker $USER
newgrp docker
```

### Step 2. Allow GUI Forwarding (for RViz2, Gazebo)

```bash
xhost +local:docker
```

### Step 3. Build and Run

```bash
cd ~/ROS2-2026

# Build image (first time only, ~10-20 min)
docker compose build

# Run full system
DISPLAY=$DISPLAY docker compose up bev_full

# Open a development shell
DISPLAY=$DISPLAY docker compose run --rm dev bash
```

### Step 4. Rebuild Inside Container (after code changes)

```bash
# Inside the container
cd /ros2_ws
colcon build --packages-select bev_navigation
source install/setup.bash
ros2 launch bev_navigation bev_navigation.launch.py
```

---

## macOS

ROS2 does not run natively on macOS. Use Docker + XQuartz for GUI forwarding.

### Step 1. Install Homebrew (if not installed)

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

### Step 2. Install Docker Desktop

```bash
brew install --cask docker
```

Launch Docker Desktop and wait until the whale icon appears in the menu bar.

### Step 3. Install XQuartz (GUI forwarding)

```bash
brew install --cask xquartz
```

After installation, **log out and log back in** (or restart).

### Step 4. Configure XQuartz

Open XQuartz → menu bar `XQuartz` → `Settings` → `Security` tab
→ Check **"Allow connections from network clients"** → Restart XQuartz

### Step 5. Allow X11 Access

```bash
# Run this every session
xhost +localhost
```

### Step 6. Build and Run

```bash
cd ~/ROS2-2026
chmod +x run.sh

# Build image (first time only, ~10-20 min)
./run.sh build

# Run full system
./run.sh run

# Other commands
./run.sh gazebo   # Gazebo only
./run.sh rviz     # RViz2 only
./run.sh dev      # Development shell
./run.sh rebuild  # Rebuild after code changes
```

> **Apple Silicon (M1/M2/M3)**
> In Docker Desktop: `Settings` → `General` → enable **"Use Rosetta for x86/amd64 emulation"**.

---

## Windows

Use WSL2 + Docker Desktop on Windows.

### Step 1. Enable WSL2

Run PowerShell as **Administrator**:

```powershell
wsl --install
wsl --set-default-version 2
```

After restarting, install **Ubuntu 22.04** from the Microsoft Store.

### Step 2. Install Docker Desktop

Download and install [Docker Desktop for Windows](https://www.docker.com/products/docker-desktop/).

After installation: Docker Desktop → `Settings` → `Resources` → `WSL Integration`
→ Enable the Ubuntu 22.04 toggle.

### Step 3. Install VcXsrv (GUI forwarding)

Download and install [VcXsrv](https://sourceforge.net/projects/vcxsrv/).

When launching VcXsrv, use these settings:
- Display number: `0`
- Check **"Disable access control"**
- Check **"Native opengl"**

> VcXsrv must be running every time before you launch the container.

### Step 4. Configure Environment in WSL2

Open a WSL2 Ubuntu terminal:

```bash
# Point DISPLAY at the Windows host IP (where VcXsrv is listening)
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
echo "export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0" >> ~/.bashrc
export LIBGL_ALWAYS_INDIRECT=1
echo "export LIBGL_ALWAYS_INDIRECT=1" >> ~/.bashrc
source ~/.bashrc
```

### Step 5. Clone and Run

```bash
# In WSL2 Ubuntu terminal
cd ~
git clone <repo-url> ROS2-2026
cd ROS2-2026

# Build image (first time only, ~10-20 min)
docker compose build

# Run full system
docker compose up bev_full
```

> **Note**: `network_mode: host` is not used in this project because all ROS2 nodes
> run inside a single container. Docker's default bridge network is sufficient and
> is fully compatible with Windows Docker Desktop.

---

## Quick Start Summary

| Environment | Command |
|-------------|---------|
| Ubuntu 22.04 | `ros2 launch bev_navigation bev_navigation.launch.py` |
| Ubuntu 24.04 | `DISPLAY=$DISPLAY docker compose up bev_full` |
| macOS | `./run.sh run` |
| Windows (WSL2) | `docker compose up bev_full` |

## Running Tests

```bash
# Run locally without ROS2 (Mac/any OS)
pip install numpy pytest
pytest src/bev_navigation/test/test_bev_utils.py -v

# Run with colcon (requires ROS2 environment)
colcon test --packages-select bev_navigation
colcon test-result --verbose
```
