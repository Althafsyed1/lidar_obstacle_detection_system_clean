# LiDAR Obstacle Detection & Tracking System

[![Ubuntu](https://img.shields.io/badge/OS-Ubuntu-informational?style=flat&logo=ubuntu&logoColor=white&color=2bbc8a)](https://ubuntu.com/)
[![ROS](https://img.shields.io/badge/Framework-ROS-informational?style=flat&logo=ROS&logoColor=white&color=2bbc8a)](https://www.ros.org/)
[![C++](https://img.shields.io/badge/Language-C++-informational?style=flat&logo=c%2B%2B&logoColor=white&color=2bbc8a)](https://isocpp.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

**Real-time 3D obstacle detection and multi-object tracking using Euclidean clustering and Hungarian algorithm**

![Demo 1](media/demo_1.gif)

![Demo 2](media/demo_2.gif)

---

## 📋 Overview

This project implements a complete perception pipeline for autonomous vehicles using 3D LiDAR point clouds. The system performs ground segmentation, clusters obstacles, generates bounding boxes, and tracks objects across frames in real-time.

**Key Technologies:**
- RANSAC-based ground plane segmentation
- Euclidean clustering with KD-Tree acceleration
- PCA-based bounding box estimation
- Hungarian algorithm for data association
- IOU-based multi-object tracking

---

## ✨ Features

### Core Capabilities

- **Ground Plane Segmentation**  
  Separates ground from obstacles using RANSAC plane fitting, robust to uneven terrain and noise

- **Configurable Region of Interest (ROI)**  
  Define detection zones to focus on relevant areas and improve performance

- **Ego Vehicle Point Removal**  
  Filters out the robot's own body from the point cloud to prevent self-detection

- **Obstacle Clustering**  
  Groups obstacle points into distinct objects using Euclidean clustering with spatial proximity

- **Real-time Tracking**  
  Maintains object identities across frames using IOU gauge and Hungarian algorithm for optimal assignment

- **Live Parameter Tuning**  
  All critical algorithm parameters can be adjusted in real-time using ROS dynamic reconfigure - no need to restart nodes!

---

## 🎯 Planned Enhancements

### In Development

- [ ] **Motion Undistortion** - Compensate for LiDAR motion during scan acquisition
- [ ] **Drive Space Segmentation** - Detect curbs and drivable surface boundaries  
- [ ] **L-Shape Fitting** - Refine bounding boxes for vehicles with L-shape optimization
- [ ] **Advanced Tracking** - Integrate UKF/EKF for motion prediction and improved robustness

### Known Limitations

- PCA bounding boxes may not be optimal for L-shaped objects (vehicles)
- Tracking can lose IDs during extended occlusions (no motion model yet)
- Ground segmentation assumes relatively flat terrain

See [TODO.md](TODO.md) for complete roadmap.

---

## 🔧 System Requirements

### Dependencies

- **ROS:** Melodic or Noetic
- **Ubuntu:** 18.04 or 20.04
- **C++ Compiler:** C++14 or later
- **Required ROS Packages:**
  - `autoware-msgs`
  - `jsk-recognition-msgs`
  - PCL (Point Cloud Library)

---

## 🚀 Installation

### Step 1: Clone the Repository
```bash
cd ~/catkin_ws/src
git clone https://github.com/Althafsyed1/lidar_obstacle_detection_system_clean.git
```

### Step 2: Install Dependencies
```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Step 3: Build the Package
```bash
# Standard build
catkin_make

# OR for Python 3 compatibility
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

### Step 4: Source the Workspace
```bash
source devel/setup.bash
```

---

## 🎮 Usage Guide

### Option 1: Run with MAI City Dataset (Recommended for Testing)

This is the easiest way to see the system in action.

![MAI City Demo](media/mai_city_00.gif)

**Step 1:** Download the Dataset

Get the MAI City autonomous driving dataset from the [official website](https://www.ipb.uni-bonn.de/data/mai-city-dataset/)

**Step 2:** Launch the Detection System
```bash
# This launches:
# - Obstacle detector node
# - RViz visualization
# - Dynamic reconfigure GUI (rqt_reconfigure)
roslaunch lidar_obstacle_detector mai_city.launch
```

**Step 3:** Play a ROS Bag
```bash
cd /path/to/mai_city/bags
rosbag play 00.bag
```

You should now see:
- Point clouds in RViz
- Detected obstacles with bounding boxes
- Color-coded tracked objects with IDs
- Parameter controls in rqt_reconfigure window

---

### Option 2: Run with LGSVL Simulator

For testing in a controlled simulation environment.

![LGSVL Demo](media/lgsvl.gif)

**Prerequisites:**  
Install the [`lgsvl_utils`](https://github.com/SS47816/lgsvl_utils) package for simulator integration.

**Step 1:** Launch LGSVL Simulator and Utils

Follow the setup instructions in the [lgsvl_utils README](https://github.com/SS47816/lgsvl_utils)

**Step 2:** Launch the Detector
```bash
roslaunch lidar_obstacle_detector lgsvl.launch
```

---

### Option 3: Integrate with Your Own Robot

**Create a custom launch file:**
```xml
<!-- my_robot.launch -->
<launch>
  <!-- Obstacle Detector Node -->
  <node pkg="lidar_obstacle_detector" 
        type="obstacle_detector_node" 
        name="lidar_obstacle_detector"
        output="screen">
    
    <!-- Remap to your LiDAR topic -->
    <remap from="/lidar_points" to="/your_robot/velodyne_points"/>
    
    <!-- Load parameters -->
    <rosparam command="load" file="$(find lidar_obstacle_detector)/cfg/params.yaml"/>
  </node>
  
  <!-- Visualization -->
  <node pkg="rviz" type="rviz" name="rviz" 
        args="-d $(find lidar_obstacle_detector)/rviz/default.rviz"/>
</launch>
```

**Launch your custom setup:**
```bash
roslaunch lidar_obstacle_detector my_robot.launch
```

---

## ⚙️ Configuration

### Key Parameters

All parameters can be tuned via `rqt_reconfigure` or by editing `cfg/LidarObstacleDetector.cfg`:

#### Ground Segmentation
| Parameter | Description | Default | Range |
|-----------|-------------|---------|-------|
| `ground_threshold` | RANSAC distance threshold (m) | 0.2 | 0.1-0.5 |
| `ransac_iterations` | Number of plane fitting attempts | 100 | 50-200 |

#### Clustering
| Parameter | Description | Default | Range |
|-----------|-------------|---------|-------|
| `cluster_tolerance` | Max distance between points in cluster (m) | 0.5 | 0.2-1.0 |
| `cluster_min_size` | Minimum points per cluster | 10 | 5-50 |
| `cluster_max_size` | Maximum points per cluster | 5000 | 1000-10000 |

#### Region of Interest
| Parameter | Description | Default |
|-----------|-------------|---------|
| `roi_x_min` / `roi_x_max` | X-axis boundaries (m) | -50 / 50 |
| `roi_y_min` / `roi_y_max` | Y-axis boundaries (m) | -50 / 50 |
| `roi_z_min` / `roi_z_max` | Z-axis boundaries (m) | -2 / 5 |

#### Tracking
| Parameter | Description | Default |
|-----------|-------------|---------|
| `iou_threshold` | Min IOU for object association | 0.2 |
| `track_lost_threshold` | Frames before deleting lost track | 5 |

### Tuning Tips

- **Increase `cluster_tolerance`** if objects are being over-segmented (too many small clusters)
- **Decrease `cluster_tolerance`** if distinct objects are merging together
- **Adjust ROI** to focus on your area of interest and improve performance
- **Tune `ground_threshold`** based on terrain roughness (higher for rough terrain)

---

## 📐 Architecture

### Pipeline Overview
```
┌─────────────────┐
│  LiDAR Sensor   │
│ /lidar_points   │
└────────┬────────┘
         │
         ▼
┌─────────────────────────┐
│  1. Preprocessing       │
│  - Voxel downsampling   │
│  - ROI filtering        │
│  - Ego vehicle removal  │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│  2. Ground Segmentation │
│  - RANSAC plane fit     │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│  3. Clustering          │
│  - Euclidean clustering │
│  - KD-Tree search       │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│  4. Bounding Boxes      │
│  - PCA orientation      │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│  5. Tracking            │
│  - IOU calculation      │
│  - Hungarian matching   │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│  6. Visualization       │
│  - RViz markers         │
└─────────────────────────┘
```

See [docs/architecture.md](docs/architecture.md) for detailed algorithm explanations.

---

## 📊 Performance

### Typical Metrics

| Dataset | FPS | Latency | Detection Rate |
|---------|-----|---------|----------------|
| MAI City | 15-20 | 50-65ms | 95%+ |
| LGSVL Sim | 20-25 | 40-50ms | 92%+ |

*Results on Intel i7-8700K CPU, no GPU acceleration*

### Bottlenecks

- **Clustering stage** is the primary bottleneck on dense point clouds
- Use larger voxel grid size to improve FPS at cost of detail
- Reduce ROI range to process fewer points

See [docs/evaluation.md](docs/evaluation.md) for benchmarking tools.

---

## 🤝 Contributing

Contributions are welcome! This project follows standard open-source practices.

### Code Style

We follow these guidelines:
- [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
- [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#main)
- [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide)

### How to Contribute

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Test thoroughly
5. Commit (`git commit -m 'Add amazing feature'`)
6. Push to your fork (`git push origin feature/amazing-feature`)
7. Open a Pull Request

---

## 📝 Attribution & License

### License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

### Attribution

This implementation is based on the excellent work by [SS47816](https://github.com/SS47816):
- **Original Repository:** [SS47816/lidar_obstacle_detector](https://github.com/SS47816/lidar_obstacle_detector)

See [NOTICE.md](NOTICE.md) for detailed attribution requirements.

### My Contributions

This fork includes the following enhancements:

- ✅ **Comprehensive documentation** - Architecture guides, evaluation framework, and quick start
- ✅ **Benchmarking tools** - FPS measurement and latency profiling scripts  
- ✅ **Project organization** - Improved README structure and navigation
- ✅ **Results tracking** - Directory structure for performance metrics
- 🚧 **ROS2 port** - Migration to ROS2 (in progress - see TODO.md)
- 🚧 **Algorithm improvements** - Kalman filtering and L-shape fitting (planned)

---

## 📞 Contact & Support

- **Issues:** [GitHub Issues](https://github.com/Althafsyed1/lidar_obstacle_detection_system_clean/issues)
- **Discussions:** Use GitHub Discussions for questions
- **Pull Requests:** Contributions welcome!

---

## 📚 Additional Resources

- [Quick Start Guide](docs/quickstart.md) - Get running in 5 minutes
- [Architecture Documentation](docs/architecture.md) - Detailed pipeline explanation
- [Evaluation Guide](docs/evaluation.md) - Metrics and benchmarking
- [Project Roadmap](TODO.md) - Planned improvements

---

## 🙏 Acknowledgments

- Original implementation and algorithm design by [SS47816](https://github.com/SS47816)
- MAI City dataset team for excellent test data
- LGSVL simulator team for testing infrastructure
- ROS and PCL communities for foundational tools

---

**Note:** This is currently a **ROS1 package** (Melodic/Noetic). A ROS2 port is planned - see [TODO.md](TODO.md) for details.
