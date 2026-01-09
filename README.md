# BFM4523 Agricultural Rover Project

A ROS 2 simulation package for an autonomous agricultural rover. This project demonstrates differential drive mobile robotics, SLAM mapping, autonomous navigation (Nav2), and 3-DOF robotic arm manipulation using Gazebo Fortress and ROS 2.

## Key Features
* **Simulation:** Custom Gazebo Fortress worlds (Farm & Maze).
* **Mobile Robot:** Differential drive kinematics with simulated physics.
* **Mapping:** Lidar-based SLAM using `slam_toolbox`.
* **Autonomy:** Path planning and obstacle avoidance using `Nav2`.
* **Manipulation:** 3-DOF robotic arm with teleoperation control.
* **Interfacing:** Custom `ros_gz_bridge` configuration for sensor data fusion.

## Prerequisites
* **OS:** Ubuntu 22.04 LTS
* **ROS 2 Distro:** Humble
* **Simulator:** Gazebo Fortress (Ignition)
* **Dependencies:** `nav2`, `slam_toolbox`, `ros_gz_bridge`, `xacro`

## 📦 Installation

1.  **Clone the repository:**
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone [https://github.com/YOUR_USERNAME/BFM4523-Agricultural-Rover.git](https://github.com/YOUR_USERNAME/BFM4523-Agricultural-Rover.git)
    ```

2.  **Install dependencies:**
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  **Build the package:**
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select bfm4523_agricultural_rover
    source install/setup.bash
    ```

## Usage

### 1. Launch Simulation (Gazebo + RViz)
This launches the rover in the custom Maze/Farm world with the bridge active.
```bash
ros2 launch bfm4523_agricultural_rover gazebo.launch.py
```
## Team Members

* **PRAVIN THIRUCHELVAM**
* **LOO HUI KIE**
* **SITI NUR AIN BINTI ABDUL RAZAK**
* **MUHAMMAD DANISH BIN HAMDAN**
* **AHMAD MAHRUS ALI FARIS BIN SANUDDIN**

---
**Course:** BFM4523 - AUTONOMOUS ROBOTIC SYSTEM
**University:** UNIVERSITY MALAYSIA PAHANG AL-SULTAN ABDULAH (UMPSA)
