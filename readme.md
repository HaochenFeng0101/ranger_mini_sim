# Ranger Mini V2 Simulation Package

This package provides a complete simulation environment for the Ranger Mini V2 robot with enhanced sensor capabilities including 3D LiDAR and camera sensors.

## Prerequisites

### 1. Clone and Build AWS Hospital World
```bash
cd ~/catkin_ws/src
git clone https://github.com/aws-robotics/aws-robomaker-hospital-world.git
```

### 2. Install Dependencies
```bash
sudo apt-get update
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
sudo apt-get install ros-noetic-velodyne-description
sudo apt-get install ros-noetic-cv-bridge
```

### 3. Build the Ranger Mini Simulation
```bash
cd ~/catkin_ws
catkin build 
source devel/setup.bash
```
You can catkin build or catkin_make

## Usage Examples

### Launch Robot in Hospital Environment
```bash
roslaunch ranger_mini_v2_gazebo hospital.launch
```

### Launch with Custom Parameters
```bash
roslaunch ranger_mini_v2_gazebo hospital.launch gui:=false paused:=true
```

### Launch Robot Control Only
```bash
roslaunch ranger_mini_control robot_control_v2.launch
```

### Display Robot Model
```bash
roslaunch ranger_mini_v2_gazebo display_xacro.launch
```

## Sensor Configuration

The simulation robot is equipped with:
- **3D LiDAR (Velodyne VLP-16)**: Topic `/mid/points`
- **Front Camera**: Topic `/front/image_raw`



## Package Structure

```
📁 catkin_ws/
├── 📁 src/
│   ├── 📁 aws-robomaker-hospital-world/     # 🏥 Hospital environment
│   └── 📁 ranger_mini_sim/                  # 🤖 Main simulation package
```