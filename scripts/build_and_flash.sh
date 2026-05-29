#!/usr/bin/env bash
# MIT License – see LICENSE in repo root
# ─────────────────────────────────────────────────────────────
# build_and_flash.sh – One-shot workspace build for Jetson Orin Nano
#
# Usage:
#   chmod +x scripts/build_and_flash.sh
#   ./scripts/build_and_flash.sh
# ─────────────────────────────────────────────────────────────
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$PROJECT_DIR/ros2_ws"

echo "═══════════════════════════════════════════════════"
echo "  slam_project – Workspace Build"
echo "  Target: Jetson Orin Nano / Ubuntu 22.04 / ROS 2 Humble"
echo "═══════════════════════════════════════════════════"

# ── 1. Source ROS 2 ──
if [ -f /opt/ros/humble/setup.bash ]; then
    # Set ROS environment variables to avoid unbound variable errors
    export AMENT_TRACE_SETUP_FILES=""
    export AMENT_PYTHON_EXECUTABLE=""
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    echo "[OK] Sourced ROS 2 Humble"
else
    echo "[ERROR] /opt/ros/humble/setup.bash not found. Install ROS 2 Humble first."
    exit 1
fi

# ── 2. Set Cyclone DDS as default middleware ──
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "[OK] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"

# ── 3. Jetson performance mode (optional – requires sudo) ──
if command -v nvpmodel &>/dev/null; then
    echo "[INFO] Setting Jetson to MAXN power mode..."
    sudo nvpmodel -m 0 2>/dev/null || echo "[WARN] Could not set power mode (need sudo)"
    sudo jetson_clocks 2>/dev/null || echo "[WARN] Could not lock clocks (need sudo)"
fi

# ── 4. Install dependencies via rosdep ──
echo ""
echo "── Installing dependencies (rosdep) ──"
cd "$WS_DIR"
sudo rosdep init 2>/dev/null || true
rosdep update --include-eol-distros
rosdep install --from-paths src --ignore-src -r -y

# ── 5. Build with colcon ──
echo ""
echo "── Building workspace (colcon) ──"
cd "$WS_DIR"
colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers "$(nproc)"

# ── 6. Source the workspace ──
# shellcheck disable=SC1091
source "$WS_DIR/install/setup.bash"

echo ""
echo "═══════════════════════════════════════════════════"
echo "  Build complete!"
echo ""
echo "  Next steps:"
echo "    source $WS_DIR/install/setup.bash"
echo "    ros2 launch slam_bringup slam_fusion.launch.py"
echo ""
echo "  Or use:  ./scripts/bringup.sh"
echo "═══════════════════════════════════════════════════"
