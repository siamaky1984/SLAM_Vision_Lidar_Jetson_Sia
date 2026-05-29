#!/usr/bin/env bash
# MIT License – see LICENSE in repo root
# ─────────────────────────────────────────────────────────────
# record_bag.sh – Record key SLAM topics to a rosbag
#
# Usage:
#   chmod +x scripts/record_bag.sh
#   ./scripts/record_bag.sh                  # default output dir
#   ./scripts/record_bag.sh /path/to/output  # custom output dir
# ─────────────────────────────────────────────────────────────
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# ── Output directory ──
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="${1:-$PROJECT_DIR/bags/$TIMESTAMP}"
mkdir -p "$OUTPUT_DIR"

# ── Source ROS 2 ──
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ -f "$PROJECT_DIR/ros2_ws/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$PROJECT_DIR/ros2_ws/install/setup.bash"
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "═══════════════════════════════════════════════════"
echo "  slam_project – Bag Recording"
echo "  Output: $OUTPUT_DIR"
echo "  Press Ctrl+C to stop recording."
echo "═══════════════════════════════════════════════════"
echo ""

# ── Topics to record ──
ros2 bag record \
    -o "$OUTPUT_DIR" \
    /camera/image_raw \
    /camera/camera_info \
    /scan \
    /odom \
    /tf \
    /tf_static \
    /map
