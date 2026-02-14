#!/bin/bash
# Quick README update script
# Run from: ~/robotics_projects/lidar_obstacle_detection_system_clean

echo "Backing up current README..."
cp README.md README_backup_$(date +%Y%m%d).md

echo "Downloading new README..."
# You'll paste the NEW_README.md content here

cat > README.md << 'NEWREADME'
# LiDAR Obstacle Detection & Tracking System

[![Ubuntu](https://img.shields.io/badge/OS-Ubuntu-informational?style=flat&logo=ubuntu&logoColor=white&color=2bbc8a)](https://ubuntu.com/)
[![ROS](https://img.shields.io/badge/Framework-ROS-informational?style=flat&logo=ROS&logoColor=white&color=2bbc8a)](https://www.ros.org/)
[![C++](https://img.shields.io/badge/Code-C++-informational?style=flat&logo=c%2B%2B&logoColor=white&color=2bbc8a)](https://isocpp.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

**3D LiDAR-based obstacle detection and multi-object tracking for autonomous vehicles**

![Demo 1](media/demo_1.gif)

---

## 🎯 Project Overview

This is a ROS-based system for **real-time 3D obstacle detection and tracking** using LiDAR point clouds. The pipeline combines classical point cloud processing (segmentation, clustering) with tracking algorithms to maintain object identities across frames.

**Use Cases:**
- Autonomous vehicle perception
- Mobile robot navigation
- Industrial safety systems
- Research and education in robotics

---

## 🚀 Key Features

### Detection Pipeline
- ✅ **Ground Plane Segmentation** - RANSAC-based separation of ground/obstacles
- ✅ **Configurable ROI** - Define regions of interest for detection
- ✅ **Ego Vehicle Filtering** - Remove robot's own points from cloud
- ✅ **Euclidean Clustering** - Group obstacle points into objects
- ✅ **PCA Bounding Boxes** - Generate 3D boxes around clusters

### Tracking
- ✅ **Multi-Object Tracking** - Hungarian algorithm for data association
- ✅ **IOU-based Matching** - Intersection-over-Union metric
- ✅ **Track Lifecycle Management** - New/confirmed/lost object states

### Developer Experience
- ✅ **Dynamic Reconfigure** - Tune parameters in real-time via rqt
- ✅ **ROS Integration** - Standard sensor_msgs, visualization_msgs
- ✅ **Well-Documented** - Architecture diagrams + inline comments

---

## 📊 Demo Results

![Demo 2](media/demo_2.gif)

---

## 🛠️ Installation

### Prerequisites
- Ubuntu 18.04+ / ROS Melodic or Noetic
- C++14 compatible compiler
- Dependencies: autoware-msgs, jsk-recognition-msgs

### Build Instructions

\`\`\`bash
# 1. Create catkin workspace (if you don't have one)
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 2. Clone this repository
git clone https://github.com/Althafsyed1/lidar_obstacle_detection_system_clean.git

# 3. Install dependencies
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. Build
catkin_make -DCMAKE_BUILD_TYPE=Release

# 5. Source the workspace
source devel/setup.bash
\`\`\`

---

## 🎮 Usage

### Option 1: Run with MAI City Dataset

![MAI City Demo](media/mai_city_00.gif)

**Step 1:** Download the [MAI City Dataset](https://www.ipb.uni-bonn.de/data/mai-city-dataset/)

**Step 2:** Launch the detector

\`\`\`bash
# This launches: detector node + RViz + parameter GUI
roslaunch lidar_obstacle_detector mai_city.launch
\`\`\`

**Step 3:** Play a rosbag

\`\`\`bash
cd /path/to/mai_city/bags
rosbag play 00.bag
\`\`\`

### Option 2: Run with LGSVL Simulator

**Requires:** [lgsvl_utils](https://github.com/SS47816/lgsvl_utils) package

\`\`\`bash
# Launch simulator + utils first (see lgsvl_utils README)

# Then launch detector
roslaunch lidar_obstacle_detector lgsvl.launch
\`\`\`

### Option 3: Run with Your Own LiDAR

Create a custom launch file:

\`\`\`xml
<!-- my_robot.launch -->
<launch>
  <node pkg="lidar_obstacle_detector" type="obstacle_detector_node" name="lidar_obstacle_detector">
    <!-- Remap to your LiDAR topic -->
    <remap from="/lidar_points" to="/your_lidar_topic"/>
    <rosparam command="load" file="$(find lidar_obstacle_detector)/cfg/params.yaml"/>
  </node>
</launch>
\`\`\`

---

## ⚙️ Configuration

Key parameters in \`cfg/LidarObstacleDetector.cfg\`:

| Parameter | Description | Default |
|-----------|-------------|---------|
| \`cluster_tolerance\` | Max distance between points in cluster (m) | 0.5 |
| \`cluster_min_size\` | Minimum points per cluster | 10 |
| \`cluster_max_size\` | Maximum points per cluster | 5000 |
| \`ground_threshold\` | RANSAC ground distance threshold (m) | 0.2 |
| \`roi_x_min/max\` | ROI boundaries in X (m) | -50 / 50 |

**Tune live:** Use \`rqt_reconfigure\` while the node is running!

---

## 🗺️ Roadmap

### Current Focus
- [ ] Add detailed evaluation metrics (precision, recall, MOTA)
- [ ] Improve tracking robustness (Kalman filter)
- [ ] Create reproducible benchmarks

### Future Work
- [ ] **ROS2 Port** - Migrate to ROS2 Jazzy/Humble
- [ ] **Motion Undistortion** - Compensate for ego-motion
- [ ] **L-Shape Fitting** - Refine vehicle bounding boxes
- [ ] **Advanced Tracking** - UKF/EKF with motion models
- [ ] **Drive Space Detection** - Curb/lane segmentation

See [TODO.md](TODO.md) and [GitHub Issues](https://github.com/Althafsyed1/lidar_obstacle_detection_system_clean/issues) for details.

---

## 📝 Attribution

This project builds upon the excellent work from:
- **Original Implementation:** [SS47816/lidar_obstacle_detector](https://github.com/SS47816/lidar_obstacle_detector)

See [NOTICE.md](NOTICE.md) for detailed attribution and [LICENSE](LICENSE) for terms.

**My Contributions:**
- Enhanced documentation and architecture guides
- Added evaluation tools and benchmarking
- Improved parameter organization
- Created reproducible demo instructions
- Ongoing: ROS2 port and tracking improvements

---

## 📄 License

MIT License - see [LICENSE](LICENSE) file.

---

## 🤝 Contributing

Contributions welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Open a Pull Request

**Code Style:** [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html), [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide)

---

## 📧 Contact

**Maintainer:** Althafsyed1  
**Issues:** [GitHub Issues](https://github.com/Althafsyed1/lidar_obstacle_detection_system_clean/issues)

---

**Status:** ⚠️ ROS1 project. ROS2 port planned (see roadmap).
NEWREADME

echo "README.md updated!"
echo ""
echo "Now run:"
echo "  git add README.md"
echo "  git commit -m 'docs: Professional README with attribution and roadmap'"
echo "  git push origin main"
