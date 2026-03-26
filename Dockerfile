# =============================================================================
# BEV Navigation - ROS2 Humble 개발/실행 환경
# Base: osrf/ros:humble-desktop-full (RViz2, Gazebo 포함)
# =============================================================================
FROM osrf/ros:humble-desktop-full

# 비대화형 설치 (apt 프롬프트 방지)
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV TURTLEBOT3_MODEL=waffle

# =============================================================================
# 시스템 패키지 설치
# =============================================================================
RUN apt-get update && apt-get install -y \
    # 빌드 도구
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    # ROS2 네비게이션 스택
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    ros-humble-navigation2 \
    # TurtleBot3 시뮬레이션
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-simulations \
    # 카메라 / 이미지 처리
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-pipeline \
    ros-humble-depth-image-proc \
    # 메시지 동기화
    ros-humble-message-filters \
    # 포인트클라우드
    ros-humble-sensor-msgs \
    # TF
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    # Gazebo ROS 연동
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    # URDF / 로봇 모델
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    # SLAM
    ros-humble-slam-toolbox \
    # 유틸
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    wget \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# Python 패키지 설치
# =============================================================================
RUN pip3 install --no-cache-dir \
    numpy \
    opencv-python-headless \
    scipy

# =============================================================================
# rosdep 초기화
# =============================================================================
RUN rosdep update

# =============================================================================
# 워크스페이스 설정
# =============================================================================
WORKDIR /ros2_ws

# 소스 코드 복사
COPY src/ src/

# 의존성 설치 (package.xml 기반)
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# colcon 빌드
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-select bev_navigation \
    --cmake-args -DCMAKE_BUILD_TYPE=Release"

# =============================================================================
# 환경 설정
# =============================================================================
RUN echo "source /opt/ros/humble/setup.bash"           >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash"           >> ~/.bashrc && \
    echo "export TURTLEBOT3_MODEL=waffle"               >> ~/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> ~/.bashrc

# GUI 관련 환경변수 (X11 포워딩)
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
