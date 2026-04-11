# UR5e Dynamic Conveyor Tracking & Interception

This repository contains a ROS 2 Humble workspace for dynamically tracking and intercepting a moving object on a conveyor belt using a Universal Robots UR5e manipulator. The system relies entirely on overhead camera computer vision (no physical encoders) and dynamic inverse kinematics to predict and intercept a moving payload.

## 📂 Architecture & File Explanations

The workspace is divided into two main packages:

### 1. `conveyor_tracking` (The Brains & Eyes)
This Python-based package handles all the logic, math, and computer vision.
* **`conductor_node.py`**: The core orchestration node. It subscribes to the object's position, tracks its X/Y velocity using an exponential moving average, and calculates a dynamic ambush point. It interacts directly with MoveIt2 via the `/compute_ik` service to verify physical reachability before commanding the joint trajectory controllers and the Robotiq gripper to strike, hold, and release.
* **`perception_node.py`**: The vision system. It reads the raw `/conveyor_camera/image_raw` from Gazebo, applies HSV color masking to isolate the target, masks out the robot's base to prevent visual occlusion errors, and calculates the exact `(X, Y)` real-world coordinates using a calibrated pixel-to-meter scale factor.

### 2. `my_robot_workcell` (The Physical World)
This package contains all the URDFs, Xacro macros, and Launch files required to spawn the simulation.
* **`urdf/inverted_ur.urdf.xacro`**: The primary environment file. It mounts the UR5e upside down at Z=1.85m, spawns the 10kg conveyor belt, attaches the Robotiq 2F-85 gripper, and mounts the overhead Gazebo camera.
* **`urdf/smart_box.urdf`**: The target payload. It utilizes `libgazebo_ros_planar_move.so` to simulate conveyor belt motion without relying on complex friction physics, and is geometrically shaped with a mechanical interlock groove to ensure secure grasping.
* **`launch/sim_conveyor.launch.py`**: The world bring-up file. It launches the Gazebo simulator, spawns the robot description, and boots the `gripper_controller` spawner.
* **`launch/moveit.launch.py`**: The motion planning bring-up file. It loads the semantic descriptions (`.srdf`), establishes the kinematic solvers, and launches RViz2 alongside the OMPL planning pipelines.

---

## 🚀 How to Run the Simulation

### 1. Clone and Build
Open a terminal and create a new ROS 2 workspace:
```bash
mkdir -p ~/new_ur_ws/src
cd ~/new_ur_ws/src
git clone <YOUR_GITHUB_REPO_URL_HERE> .
cd ~/new_ur_ws
colcon build --symlink-install
source install/setup.bash
# Terminal 1
ros2 launch my_robot_workcell sim_conveyor.launch.py

# Terminal 2
ros2 launch my_robot_workcell moveit.launch.py
# Terminal 3
ros2 run conveyor_tracking conductor_node
# Terminal 4
ros2 topic pub -1 /box_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.08, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
