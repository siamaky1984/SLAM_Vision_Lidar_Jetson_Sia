# slam_project

> **Monocular Camera + RPLIDAR A1 SLAM on NVIDIA Jetson Orin Nano**
> ROS 2 Humble · Ubuntu 22.04 · Cyclone DDS

A ready-to-build colcon workspace that fuses **ORB-SLAM3** (monocular visual odometry) with
**RTAB-Map** (graph-based SLAM) using a USB camera and RPLIDAR A1/R1 on the Jetson Orin Nano.

---

## Table of Contents

1. [Hardware Requirements](#hardware-requirements)
2. [Software Prerequisites](#software-prerequisites)
3. [Repository Layout](#repository-layout)
4. [Initial Setup](#initial-setup)
5. [Building ORB-SLAM3](#building-orb-slam3)
6. [Building RTAB-Map from Source](#building-rtab-map-from-source)
7. [Building the Workspace](#building-the-workspace)
8. [Running the SLAM Pipeline](#running-the-slam-pipeline)
9. [Recording Bags](#recording-bags)
10. [RViz Visualization](#rviz-visualization)
11. [Topic & Frame Reference](#topic--frame-reference)
12. [Tuning Tips](#tuning-tips)
13. [Troubleshooting](#troubleshooting)
14. [License](#license)

---

## Hardware Requirements

| Component | Tested Model | Notes |
|-----------|-------------|-------|
| SBC | NVIDIA Jetson Orin Nano (8 GB) | JetPack 5.x / Ubuntu 22.04 |
| Camera | Any USB UVC monocular camera | Appears at `/dev/video0` |
| Lidar | Slamtec RPLIDAR A1 or R1 | UART at `/dev/ttyUSB0`, 115200 baud |
| Storage | ≥ 64 GB microSD / NVMe | NVMe strongly recommended for build speed |
| Power | 5 V / 3 A barrel jack | Use the official Jetson power supply |

## Software Prerequisites

```bash
# 1. ROS 2 Humble (full desktop install recommended)
#    https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  ros-humble-v4l2-camera \
  ros-humble-image-transport \
  ros-humble-image-transport-plugins \
  ros-humble-diagnostic-updater \
  python3-colcon-common-extensions \
  python3-rosdep

# 2. Cyclone DDS (recommended for Jetson – lower latency than Fast-DDS)
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc

# 3. RPLIDAR driver (if not using the submodule)
#    Option A – apt:
sudo apt install -y ros-humble-rplidar-ros
#    Option B – build from source (submodule already configured):
#    See "Initial Setup" below.

# 4. Build tools for ORB-SLAM3
sudo apt install -y \
  cmake g++ git libgl1-mesa-dev libglew-dev \
  libopencv-dev libeigen3-dev libpangolin-dev \
  libboost-serialization-dev libssl-dev

# 5. RTAB-Map build dependencies
sudo apt install -y \
  libsqlite3-dev libpcl-dev liboctomap-dev \
  ros-humble-pcl-conversions ros-humble-tf2-geometry-msgs \
  ros-humble-nav-msgs ros-humble-octomap-msgs

# 6. rosdep
sudo rosdep init 2>/dev/null; rosdep update
```

> **Cyclone DDS tip:** Create `~/cyclonedds.xml` for fine-tuning:
> ```xml
> <CycloneDDS>
>   <Domain>
>     <General>
>       <NetworkInterfaceAddress>lo</NetworkInterfaceAddress>
>     </General>
>     <Internal>
>       <MinimumSocketReceiveBufferSize>10MB</MinimumSocketReceiveBufferSize>
>     </Internal>
>   </Domain>
> </CycloneDDS>
> ```
> Then `export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml`.

---

## Repository Layout

```
slam_project/
├── README.md
├── LICENSE                           # MIT
├── .gitignore
├── .gitmodules                       # rplidar_ros, orb_slam3_ros2 submodule refs
├── scripts/
│   ├── build_and_flash.sh            # One-shot workspace build
│   ├── bringup.sh                    # Launch the full pipeline
│   └── record_bag.sh                 # Record key topics to a rosbag
└── ros2_ws/
    └── src/
        ├── rplidar_ros/              # ← git submodule (Slamtec driver)
        ├── orb_slam3_ros2/           # ← git submodule (ORB-SLAM3 ROS 2 wrapper)
        ├── camera_launch/            # v4l2_camera launch + config
        │   ├── package.xml / CMakeLists.txt
        │   ├── launch/camera.launch.py
        │   └── config/camera.yaml
        ├── rplidar_launch/           # rplidar driver launch + config
        │   ├── package.xml / CMakeLists.txt
        │   ├── launch/rplidar.launch.py
        │   └── config/rplidar.yaml
        ├── orb_slam3_wrapper/        # Thin wrapper for ORB-SLAM3 mono node
        │   ├── package.xml / CMakeLists.txt
        │   ├── launch/orb_slam3_mono.launch.py
        │   └── config/orb_slam3.yaml
        ├── rtabmap_launch/           # RTAB-Map launch + tuned config
        │   ├── package.xml / CMakeLists.txt
        │   ├── launch/rtabmap.launch.py
        │   └── config/rtabmap.yaml
        ├── tf_broadcasters/          # Static TF: base_link → camera_link, laser
        │   ├── package.xml / CMakeLists.txt
        │   └── launch/static_tf.launch.py
        └── slam_bringup/            # Top-level orchestrator
            ├── package.xml / CMakeLists.txt
            ├── launch/
            │   ├── slam_fusion.launch.py
            │   └── rviz.launch.py
            └── rviz/slam_default.rviz
```

---

## Initial Setup

```bash
# Clone with submodules
git clone --recurse-submodules https://github.com/<your-user>/slam_project.git
cd slam_project

# If you already cloned without --recurse-submodules:
git submodule update --init --recursive

# Source ROS 2
source /opt/ros/humble/setup.bash
```

---

## Building ORB-SLAM3

ORB-SLAM3 must be built **outside** colcon first (it uses its own CMake build).

```bash
# 1. Clone (if not using the submodule)
cd ~/
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3

# 2. Build Thirdparty libraries
cd Thirdparty/DBoW2 && mkdir -p build && cd build && cmake .. && make -j$(nproc)
cd ../../g2o && mkdir -p build && cd build && cmake .. && make -j$(nproc)
cd ../../Sophus && mkdir -p build && cd build && cmake .. && make -j$(nproc)

# 3. Uncompress vocabulary
cd ~/ORB_SLAM3/Vocabulary
tar xf ORBvoc.txt.tar.gz

# 4. Build ORB-SLAM3
cd ~/ORB_SLAM3
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install   # Installs to /usr/local by default

# ── IMPORTANT ──
# Update the vocabulary path in:
#   ros2_ws/src/orb_slam3_wrapper/config/orb_slam3.yaml
# Set `vocabulary_file` to: /home/<user>/ORB_SLAM3/Vocabulary/ORBvoc.txt
```

> **Jetson tip:** If the build runs out of memory, reduce parallelism: `make -j2`.

---

## Building RTAB-Map from Source

```bash
# Option A: apt (if available for Humble on arm64)
sudo apt install -y ros-humble-rtabmap-ros

# Option B: Build from source
cd ~/
git clone https://github.com/introlab/rtabmap.git
cd rtabmap && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install

cd ~/
git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git \
  ~/slam_project/ros2_ws/src/rtabmap_ros
```

---

## Building the Workspace

```bash
cd ~/slam_project

# One-shot build script (sets Cyclone DDS, runs rosdep, builds)
chmod +x scripts/build_and_flash.sh
./scripts/build_and_flash.sh

# ── OR manually ──
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## Running the SLAM Pipeline

```bash
# Terminal 1 – full pipeline
chmod +x scripts/bringup.sh
./scripts/bringup.sh

# ── OR launch directly ──
source ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch slam_bringup slam_fusion.launch.py

# Terminal 2 – RViz (on the Jetson or over SSH with X forwarding)
ros2 launch slam_bringup rviz.launch.py
```

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `camera_device` | `/dev/video0` | V4L2 device path |
| `lidar_serial` | `/dev/ttyUSB0` | RPLIDAR serial port |
| `lidar_baudrate` | `115200` | RPLIDAR baud rate |
| `use_rviz` | `false` | Launch RViz alongside the pipeline |

Example:
```bash
ros2 launch slam_bringup slam_fusion.launch.py camera_device:=/dev/video2
```

---

## Recording Bags

```bash
chmod +x scripts/record_bag.sh
./scripts/record_bag.sh          # Saves to bags/<timestamp>/
```

Records: `/camera/image_raw`, `/camera/camera_info`, `/scan`, `/odom`, `/orb_slam3/odom`, `/odometry/filtered`, `/tf`, `/tf_static`, `/map`.

---

## RViz Visualization

An example RViz config is at `ros2_ws/src/slam_bringup/rviz/slam_default.rviz`.

Displays included:
- **TF** – all frames
- **LaserScan** – `/scan`
- **Image** – `/camera/image_raw`
- **Map** – `/map` (OccupancyGrid from RTAB-Map)
- **Odometry** – `/odometry/filtered`

---

## Topic & Frame Reference

### Topics

| Topic | Type | Source |
|-------|------|--------|
| `/camera/image_raw` | `sensor_msgs/Image` | v4l2_camera |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | v4l2_camera |
| `/scan` | `sensor_msgs/LaserScan` | rplidar_ros |
| `/odom` | `nav_msgs/Odometry` | Lidar ICP odometry (EKF input) |
| `/orb_slam3/odom` | `nav_msgs/Odometry` | ORB-SLAM3 mono (arbitrary scale) |
| `/orb_slam3/odom_scaled` | `nav_msgs/Odometry` | Mono VO rescaled to metres via ICP |
| `/orb_slam3/odom_scaled_twist` | `nav_msgs/Odometry` | Metric body-frame velocity (EKF input) |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF fused odom (RTAB-Map input) |
| `/map` | `nav_msgs/OccupancyGrid` | RTAB-Map |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | tf_broadcasters + ORB-SLAM3 |

### Frames

```
map
 └── odom            (published by robot_localization EKF)
      └── base_link
           ├── camera_link   (static TF)
           └── laser          (static TF)
```

---

## Tuning Tips

### Camera (640×480 @ 15 fps)
- Lower resolution reduces ORB-SLAM3 CPU load significantly on Jetson.
- If tracking is lost frequently, try 800×600 or increase `nFeatures`.

### ORB-SLAM3
- `nFeatures: 800` – reduce to 500 if CPU-bound; increase to 1200 for better accuracy.
- `scaleFactor: 1.2` and `nLevels: 8` – standard pyramid; reducing `nLevels` to 6 saves compute.
- `vocabulary_file` – **must** point to the uncompressed `ORBvoc.txt`.

### RTAB-Map
- `Mem/IncrementalMemory: "true"` – enables online mapping.
- `RGBD/LinearUpdate: "0.1"` – minimum 10 cm translation to add a node (increase for slower mapping).
- `RGBD/AngularUpdate: "0.05"` – minimum ~3° rotation to add a node.
- `Grid/RangeMax: "5.0"` – clip lidar range for indoor mapping.
- `Rtabmap/DetectionRate: "1.0"` – loop closure detection rate in Hz; lower to 0.5 on Jetson if CPU-bound.
- Set `approx_sync: true` if camera and lidar timestamps drift.

### Cyclone DDS
- Use `lo` interface for single-machine setups to avoid multicast overhead.
- Increase socket buffer for high-bandwidth image topics.

### Jetson Power Mode
```bash
sudo nvpmodel -m 0        # MAXN (max performance)
sudo jetson_clocks         # Lock clocks to max frequency
```

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `v4l2_camera` fails to open device | Check `ls /dev/video*`; try `sudo chmod 666 /dev/video0` or add user to `video` group |
| RPLIDAR not detected | Verify `ls /dev/ttyUSB0`; add udev rule (see below); check baud rate |
| ORB-SLAM3 "vocabulary not found" | Update `vocabulary_file` path in `orb_slam3.yaml` |
| ORB-SLAM3 tracking lost immediately | Ensure adequate lighting; increase `nFeatures`; check camera intrinsics |
| RTAB-Map "no odometry received" | Verify `/odometry/filtered` is published: `ros2 topic hz /odometry/filtered` (needs `/odom` from ICP) |
| High CPU / thermal throttling | Use `sudo nvpmodel -m 0 && sudo jetson_clocks`; reduce camera resolution |
| TF tree broken | Run `ros2 run tf2_tools view_frames` and check for missing links |
| DDS discovery slow | Set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and use loopback interface |

### RPLIDAR udev Rule

```bash
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  MODE="0666", SYMLINK+="rplidar"' | sudo tee /etc/udev/rules.d/99-rplidar.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
# Now use /dev/rplidar as the serial port
```

### Camera udev Rule

```bash
echo 'SUBSYSTEM=="video4linux", ATTRS{idVendor}=="<VID>", ATTRS{idProduct}=="<PID>", \
  MODE="0666", SYMLINK+="slam_camera"' | sudo tee /etc/udev/rules.d/99-slam-camera.rules
# Replace <VID> and <PID> with your camera's USB vendor/product ID (lsusb)
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## License

This project is licensed under the [MIT License](LICENSE).

> **Note:** ORB-SLAM3 is licensed under GPLv3. RTAB-Map is licensed under BSD.
> Check upstream licenses before commercial deployment.
