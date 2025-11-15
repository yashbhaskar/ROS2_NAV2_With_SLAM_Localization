# ROS2_NAV2_With_SLAM_Localization
This ROS 2 repository contains a complete robot simulation setup with URDF, Gazebo sensors, and plugins, enabling autonomous navigation using Nav2 and localization with SLAM. Includes custom launch files integrating navigation, SLAM Localization, map server, and visualization for real-time autonomous navigation.

---

## Concept of SLAM Toolbox Localization

**SLAM Toolbox localization** is a technique in ROS 2 that allows a robot to localize itself within a previously created map. Instead of building the map again, it uses the existing map along with sensor data such as LiDAR and odometry to continually estimate the robot’s position and orientation. SLAM Toolbox runs in localization mode, where it matches incoming laser scans against the stored map and corrects drift in odometry, enabling accurate pose estimation. This is essential for autonomous navigation, as Nav2 depends on a reliable robot pose to plan and execute paths. With loop closure, pose graph optimization, and serialization/deserialization capabilities, SLAM Toolbox makes localization robust for real-world and simulated environments.

<img width="1845" height="1052" alt="1" src="https://github.com/user-attachments/assets/c07baa74-8126-4673-a35c-7f0e3b72001e" />


---

## 🚀 Features

- Complete robot URDF/XACRO model with sensors and gazebo integration
- Autonomous Navigation using Nav2 with path planning and obstacle avoidance
- Custom bringup launch integrating navigation, SLAM localization and RViz
- SLAM Toolbox Localization Mode for pose estimation using a pre-built map
- Support for 2D LiDAR, IMU and differential drive odometry
- Map Server + Lifecycle Nodes for loading and managing maps
- RViz visualization with navigation plugins
- Custom Launch Files connecting navigation, map server, slam localization, RViz and controllers
- Support for serialization/deserialization to save and reuse localization sessions

---

## 🎓 Learning Objectives

- Understand robot modeling with URDF and sensor integration in ROS 2
- Configure and launch robot simulation in Gazebo with plugins
- Implement Nav2 for autonomous path planning and navigation
- Use SLAM Toolbox in localization mode to accurately estimate robot pose
- Operate Map Server, SLAM and other navigation lifecycle nodes
- Send navigation goals and observe autonomous movement to target points
- Optimize localization accuracy and overcome odometry drift issues

---

## Installation

### Make Workspace
```bash
mkdir robot_ws/
```

### Change Workspace
```bash
cd robot_ws
```

### Make src
```bash
mkdir src/
```

### Change Workspace
```bash
cd src
```

### Clone This Repository
```bash
git clone https://github.com/yashbhaskar/ROS2_NAV2_With_SLAM_Localization.git
```

### Clone SLAM Toolbox Repository
```bash
git clone -b humble https://github.com/SteveMacenski/slam_toolbox.git
```

### Install Nav2
```bash
sudo apt update
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

### (Optional recommended packages)
```bash
sudo apt install ros-humble-nav2-common
sudo apt install ros-humble-nav2-map-server
sudo apt install ros-humble-nav2-behavior-tree
```

### Change Config

- Open the file:
    slam_toolbox/config/mapper_params_localization.yaml
- Locate the parameter mode: and change it from:
```bash   
mode: mapping
```
to:
```bash   
mode: localization
```
- Add the following parameters below the mode line:
```bash   
map_file_name: /home/yash/robot_ws/src/robot_navigation/map/yash_map
map_start_at_dock: true
```
- Make sure the map_file_name path is correct and matches your saved map name.
- Refer to the example shown in the configuration screenshot and update lines 15, 17, and 18 accordingly.

<img width="1367" height="867" alt="Screenshot 2025-11-15 201212" src="https://github.com/user-attachments/assets/48114753-2bbc-48d1-bf8a-f4c83083847c" />


### Change Workspace
```bash
cd ..
```

### Build the Package
```bash
colcon build --packages-select my_bot robot_navigation
source install/setup.bash
```

---

## 🚀 How to Run

### 1st Terminal : Launch robot in gazebo
```bash
ros2 launch my_bot gazebo.launch.py
```

### 2nd Terminal : Start autonoumous navigation
```bash
ros2 launch robot_navigation slam_toolbox_localization.launch.py
```
- Now rviz is open and robot spawn in robot’s initial pose on the map and start SLAM localization.
- Then use Nav2 Goal / 2D Goal Pose to send a navigation goal. The robot will autonomously plan a path and move toward the target, avoiding obstacles and reaching the goal successfully.
- With SLAM Toolbox Localization, you will observe highly accurate pose estimation with minimal to no odometry drift, thanks to loop closure and pose graph optimization, which continuously refines the robot’s position on the map for precise localization.

<img width="1852" height="1050" alt="2" src="https://github.com/user-attachments/assets/292641ca-b53e-43dd-8032-0a027388214f" />


---

## ✉️ Contact

📧 Yash Bhaskar – ybbhaskar19@gmail.com

📌 GitHub: https://github.com/yashbhaskar

