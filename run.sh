#!/bin/bash
# =============================================================================
# BEV Navigation 실행 스크립트 (Mac용)
# =============================================================================

set -e

print_usage() {
    echo "사용법: ./run.sh [명령]"
    echo ""
    echo "명령:"
    echo "  setup    - XQuartz X11 허용 설정 (최초 1회)"
    echo "  build    - Docker 이미지 빌드"
    echo "  run      - 전체 시스템 실행 (Gazebo + Nav2 + RViz2)"
    echo "  dev      - 개발용 쉘 접속"
    echo "  gazebo   - Gazebo 시뮬레이터만 실행"
    echo "  rviz     - RViz2만 실행"
    echo "  rebuild  - 소스 변경 후 재빌드"
    echo "  stop     - 실행 중인 컨테이너 종료"
    echo "  clean    - 컨테이너 및 이미지 삭제"
}

cmd_setup() {
    echo "[setup] X11 포워딩 설정 중..."
    if ! command -v xquartz &>/dev/null && [ ! -d "/Applications/Utilities/XQuartz.app" ]; then
        echo "XQuartz가 설치되어 있지 않습니다."
        echo "설치: brew install --cask xquartz"
        echo "설치 후 XQuartz 실행 → 설정 → 보안 → '네트워크 클라이언트 허용' 체크"
        exit 1
    fi
    xhost +localhost 2>/dev/null || xhost +127.0.0.1
    echo "[setup] X11 허용 완료."
}

cmd_build() {
    echo "[build] Docker 이미지 빌드 중... (최초 실행 시 10~20분 소요)"
    docker compose build
    echo "[build] 빌드 완료."
}

cmd_run() {
    cmd_setup
    echo "[run] 전체 시스템 실행 중..."
    docker compose up bev_full
}

cmd_dev() {
    cmd_setup
    echo "[dev] 개발 쉘 실행 중..."
    echo "컨테이너 내에서 빌드: colcon build --packages-select bev_navigation"
    docker compose run --rm dev bash
}

cmd_gazebo() {
    cmd_setup
    echo "[gazebo] Gazebo 시뮬레이터 실행 중..."
    docker compose up gazebo
}

cmd_rviz() {
    cmd_setup
    echo "[rviz] RViz2 실행 중..."
    docker compose up rviz
}

cmd_rebuild() {
    echo "[rebuild] 소스 코드 재빌드 중..."
    docker compose run --rm dev bash -c \
        "source /opt/ros/humble/setup.bash && \
         colcon build --packages-select bev_navigation && \
         source install/setup.bash && \
         echo '빌드 완료'"
}

cmd_stop() {
    echo "[stop] 컨테이너 종료 중..."
    docker compose down
}

cmd_clean() {
    echo "[clean] 컨테이너 및 이미지 삭제 중..."
    docker compose down --rmi local --volumes
}

# =============================================================================
# 명령 분기
# =============================================================================
case "${1:-}" in
    setup)   cmd_setup   ;;
    build)   cmd_build   ;;
    run)     cmd_run     ;;
    dev)     cmd_dev     ;;
    gazebo)  cmd_gazebo  ;;
    rviz)    cmd_rviz    ;;
    rebuild) cmd_rebuild ;;
    stop)    cmd_stop    ;;
    clean)   cmd_clean   ;;
    *)       print_usage ;;
esac
