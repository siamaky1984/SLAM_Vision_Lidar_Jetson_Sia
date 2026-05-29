#!/usr/bin/env bash
# MIT License – see LICENSE in repo root
# ─────────────────────────────────────────────────────────────
# bringup.sh – Launch the full SLAM pipeline
#
# Usage:
#   chmod +x scripts/bringup.sh
#   ./scripts/bringup.sh
#   ./scripts/bringup.sh --rviz                  # also launch RViz
#   ./scripts/bringup.sh --camera /dev/video2     # custom camera device
# ─────────────────────────────────────────────────────────────
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$PROJECT_DIR/ros2_ws"

# ── Defaults ──
USE_RVIZ="false"
CAMERA_DEVICE="/dev/video0"
LIDAR_SERIAL="/dev/ttyUSB0"
LIDAR_BAUD="115200"

# ── Parse arguments ──
while [[ $# -gt 0 ]]; do
    case "$1" in
        --rviz)       USE_RVIZ="true"; shift ;;
        --camera)     CAMERA_DEVICE="$2"; shift 2 ;;
        --lidar)      LIDAR_SERIAL="$2"; shift 2 ;;
        --baud)       LIDAR_BAUD="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: $0 [--rviz] [--camera /dev/videoN] [--lidar /dev/ttyUSBN] [--baud RATE]"
            exit 0 ;;
        *)
            echo "[WARN] Unknown argument: $1"; shift ;;
    esac
done

# ── Source ROS 2 + workspace ──
# Set ROS environment variables to avoid unbound variable errors
export AMENT_TRACE_SETUP_FILES=""
export AMENT_PYTHON_EXECUTABLE=""
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

if [ -f "$WS_DIR/install/setup.bash" ]; then
    # Set ROS environment variables to avoid unbound variable errors
    export COLCON_TRACE=""
    export COLCON_PYTHON_EXECUTABLE=""
    # shellcheck disable=SC1091
    source "$WS_DIR/install/setup.bash"
else
    echo "[ERROR] Workspace not built. Run ./scripts/build_and_flash.sh first."
    exit 1
fi

# ── Set Cyclone DDS ──
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ── Optional: Cyclone DDS config ──
if [ -f "$HOME/cyclonedds.xml" ] && [ "${USE_CYCLONEDDS_CONFIG:-false}" = "true" ]; then
    export CYCLONEDDS_URI="file://$HOME/cyclonedds.xml"
    echo "[OK] Using Cyclone DDS config: $CYCLONEDDS_URI"
else
    if [ -f "$HOME/cyclonedds.xml" ]; then
        echo "[WARN] Detected $HOME/cyclonedds.xml but not using it (set USE_CYCLONEDDS_CONFIG=true to enable)."
    fi
fi

echo "═══════════════════════════════════════════════════"
echo "  slam_project – Bringup"
echo "  Camera : $CAMERA_DEVICE"
echo "  Lidar  : $LIDAR_SERIAL @ $LIDAR_BAUD"
echo "  RViz   : $USE_RVIZ"
echo "  DDS    : $RMW_IMPLEMENTATION"
echo "═══════════════════════════════════════════════════"
echo ""

ros2 launch slam_bringup slam_fusion.launch.py \
    camera_device:="$CAMERA_DEVICE" \
    lidar_serial:="$LIDAR_SERIAL" \
    lidar_baudrate:="$LIDAR_BAUD" \
    use_rviz:="$USE_RVIZ"
