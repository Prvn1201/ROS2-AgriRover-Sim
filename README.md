# BFM4523 Agricultural Rover Project

A ROS 2 simulation package for an autonomous agricultural rover. This project demonstrates differential drive mobile robotics, SLAM mapping, autonomous navigation (Nav2), and 3-DOF robotic arm manipulation using Gazebo Classic and ROS 2.

## Key Features

* **Simulation:** Custom Gazebo Classic world (Pekan Map environment).
* **Mobile Robot:** Differential drive kinematics with simulated physics.
* **Mapping:** Lidar-based SLAM using `slam_toolbox`.
* **Autonomy:** Path planning and obstacle avoidance using `Nav2`.
* **Manipulation:** 3-DOF robotic arm with teleoperation control.
* **Interfacing:** ROS 2-based control, localization, and navigation stack.

## Prerequisites

* **OS:** Ubuntu 22.04 LTS
* **ROS 2 Distro:** Humble
* **Simulator:** Gazebo Classic
* **Dependencies:** `nav2`, `slam_toolbox`, `teleop_twist_keyboard`, `joy`, `xacro`

## 📦 Installation

1. **Clone the repository:**

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/YOUR_USERNAME/BFM4523-Agricultural-Rover.git
   ```

2. **Install dependencies:**

   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build and set up the workspace:**

   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

---

## ▶️ System Execution Instructions

This section outlines the step-by-step process to compile the code, launch the simulation, and execute the autonomous navigation stack.

### 1. Build and Setup the Workspace

Before running any commands, ensure the ROS 2 workspace is built so that all configuration files are correctly installed.

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

### 2. Launch the Simulation Environment

This step starts the Gazebo Classic simulator, loads the **Pekan Map** environment, and spawns the `Autonomous_Vin_V1` robot model.

1. Open a new terminal.
2. Source the workspace:

   ```bash
   source ~/ros2_ws/install/setup.bash
   ```
3. Set the Gazebo model path and launch the simulation:

   ```bash
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix Autonomous_Vin_V1_description)/share
   ros2 launch Autonomous_Vin_V1_description gazebo.launch.py
   ```

---

### 3. Control the Robot

#### Option A: Keyboard Teleoperation

1. Open a new terminal.
2. Source the workspace:

   ```bash
   source ~/ros2_ws/install/setup.bash
   ```
3. Run the teleoperation node:

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=cmd_vel_key
   ```

#### Option B: Joystick Control (PS4 Controller)

1. Ensure the PS4 controller is connected via Bluetooth or USB.
2. Open a new terminal.
3. Launch the joystick driver:

   ```bash
   ros2 run joy joy_node
   ```

To control the robotic arm joints, run the Python teleoperation script:

```bash
python3 arm_teleop.py
```

---

### 4. Launch the Navigation Stack

This step starts the Navigation 2 (Nav2) system, including SLAM, localization (AMCL), map server, and path planning.

1. Open a new terminal.

2. Source the workspace:

   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

3. Launch SLAM:

   ```bash
   ros2 launch Autonomous_Vin_V1_description slam.launch.py
   ```

4. In a new terminal, save the generated map:

   ```bash
   cd ~/ros2_ws/src/Autonomous_Vin_V1_description/maps/
   ros2 run nav2_map_server map_saver_cli -f my_map
   ```

5. Launch the navigation stack:

   ```bash
   ros2 launch Autonomous_Vin_V1_description navigation.launch.py
   ```

---

### 5. Operating the Robot in RViz

Once RViz is open, follow these steps to move the robot autonomously:

#### Initialize Robot Pose

* Click **"2D Pose Estimate"** in the RViz toolbar.
* Click and drag on the map at the robot’s current location to set its initial pose and orientation.
* Ensure the laser scan aligns correctly with the map walls.

#### Send a Navigation Goal

* Click **"Nav2 Goal"** (or **"2D Nav Goal"**) in the RViz toolbar.
* Click on a target location on the map.
* Drag the mouse to specify the desired final orientation.

#### Observation

* The global planner computes an optimal path.
* The rover navigates autonomously to the goal while avoiding obstacles.

---

## Team Members

* **PRAVIN THIRUCHELVAM**
* **LOO HUI KIE**
* **SITI NUR AIN BINTI ABDUL RAZAK**
* **MUHAMMAD DANISH BIN HAMDAN**
* **AHMAD MAHRUS ALI FARIS BIN SANUDDIN**

---

**Course:** BFM4523 – Autonomous Robotic System
**University:** Universiti Malaysia Pahang Al-Sultan Abdullah (UMPSA)
