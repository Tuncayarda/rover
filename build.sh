#!/usr/bin/env bash
set -e

ROS_WS_DIR="$HOME/project/uros_ws"
PICO_PROJECT_DIR="$HOME/project"        
PICO_BUILD_SUBDIR="build"

FIRMWARE_PACKAGES=(rover_interfaces)

usage() {
  cat <<EOF
Usage: $0 <mode>

Modes:
  clean        Remove all artifacts.
  build-all    Clean all artifacts, then run ROS + micro-ROS FW + PICO build.
  build-pico   Only run PICO build.
EOF
  exit 1
}

clean_all() {
  echo "=== CLEAN ALL: removing build artifacts ==="
  rm -rf "$PICO_PROJECT_DIR/$PICO_BUILD_SUBDIR"
  rm -rf "$ROS_WS_DIR"/build \
         "$ROS_WS_DIR"/firmware \
         "$ROS_WS_DIR"/install \
         "$ROS_WS_DIR"/log
}

build_all() {
  clean_all
  echo "=== INCREMENTAL: ROS + micro-ROS FW + PICO ==="

  cd "$ROS_WS_DIR"
  rosdep install --from-paths src --ignore-src -y

  colcon build

  source install/local_setup.bash

  ros2 run micro_ros_setup create_firmware_ws.sh generate_lib generic

  for pkg in "${FIRMWARE_PACKAGES[@]}"; do
    ros2 run "$pkg" create_fwws.sh || true
  done

  ros2 run micro_ros_setup build_firmware.sh \
    "$(pwd)/my_toolchain.cmake" \
    "$(pwd)/my_colcon.meta"

  build_pico
}

build_pico() {
  echo "- Running PICO make..."
  cd "$PICO_PROJECT_DIR"
  mkdir -p build
  cd "build"
  cmake ..
  make
}

if [ $# -ne 1 ]; then
  usage
fi

case "$1" in
  clean)
    clean_all
    ;;
  build-all)
    build_all
    ;;
  build-pico)
    build_pico
    ;;
  *)
    usage
    ;;
esac

echo "Done."