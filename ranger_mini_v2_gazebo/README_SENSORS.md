# Ranger Mini V2 with 3D LiDAR and Camera Sensors

This package has been enhanced with 3D LiDAR (Velodyne VLP-16) and front camera sensors for improved perception capabilities.

## New Features

### 3D LiDAR (Velodyne VLP-16)
- **Location**: Mounted on top of the robot at the center
- **Topic**: `/mid/points` (PointCloud2)
- **Range**: 100m
- **Field of View**: 360° horizontal, ±15° vertical
- **Update Rate**: 10 Hz

### Front Camera
- **Location**: Mounted on the front of the robot
- **Topic**: `/front/image_raw` (Image)
- **Resolution**: 640x480
- **Frame Rate**: 30 Hz
- **Field of View**: 80° horizontal

## Usage

### Launch with Sensors and RViz
```bash
roslaunch ranger_mini_v2_gazebo ranger_mini_v2_empty_world_with_rviz.launch
```

This will launch:
- Gazebo simulation with empty world
- Ranger Mini robot with 3D LiDAR and camera
- RViz with sensor visualization
- Robot control interface

### Launch without RViz
```bash
roslaunch ranger_mini_v2_gazebo ranger_mini_v2_empty_world_with_rviz.launch rviz:=false
```

### Test Sensor Data
```bash
rosrun ranger_mini_v2_gazebo test_sensors.py
```

This script will verify that both sensors are publishing data correctly.

## Environment Variables

You can customize sensor behavior using environment variables:

### 3D LiDAR Configuration
- `RANGER_LASER_3D`: Enable/disable 3D LiDAR (default: 1)
- `RANGER_LASER_3D_TOPIC`: PointCloud2 topic name (default: mid/points)
- `RANGER_LASER_3D_OFFSET`: Mounting position (default: 0 0 0.15)
- `RANGER_LASER_3D_RPY`: Mounting orientation (default: 0 0 0)

### Camera Configuration
- `RANGER_CAMERA`: Enable/disable camera (default: 1)
- `RANGER_CAMERA_TOPIC`: Image topic name (default: front/image_raw)
- `RANGER_CAMERA_OFFSET`: Mounting position (default: 0.2 0 0.1)
- `RANGER_CAMERA_RPY`: Mounting orientation (default: 0 0 0)
- `RANGER_CAMERA_TILT`: Camera tilt angle (default: 0)

## RViz Visualization

The improved RViz configuration includes:
- **Robot Model**: Complete robot with sensors
- **PointCloud2**: 3D LiDAR data visualization
- **Image**: Camera feed display
- **TF**: Transform tree visualization

## File Structure

```
ranger_mini_v2_gazebo/
├── xacro/
│   ├── ranger_mini_gazebo.xacro          # Main robot description
│   ├── accessories.urdf.xacro            # Sensor definitions
│   └── accessories/
│       ├── vlp16_mount.urdf.xacro        # VLP-16 mount
│       └── camera_mount.urdf.xacro       # Camera mount
├── launch/
│   ├── ranger_mini_v2_empty_world.launch           # Original launch
│   └── ranger_mini_v2_empty_world_with_rviz.launch # Enhanced launch
├── rviz/
│   └── ranger_mini_with_sensors.rviz     # RViz configuration
└── scripts/
    └── test_sensors.py                   # Sensor test script
```

## Troubleshooting

### No PointCloud2 Data
1. Check if the velodyne_description package is installed
2. Verify the topic is being published: `rostopic echo /mid/points`
3. Check TF transforms: `rosrun tf tf_echo base_link mid_vlp16`

### No Camera Image
1. Verify the topic is being published: `rostopic echo /front/image_raw`
2. Check TF transforms: `rosrun tf tf_echo base_link front_camera`
3. Ensure Gazebo camera plugin is loaded correctly

### RViz Issues
1. Make sure the RViz configuration file exists
2. Check that all required topics are being published
3. Verify TF tree is complete: `rosrun tf view_frames`

## Dependencies

- `velodyne_description`: For VLP-16 LiDAR model
- `gazebo_ros`: For Gazebo simulation
- `cv_bridge`: For image processing (in test script)

## Notes

- The sensors are mounted using a modular accessory system similar to Jackal
- All sensor parameters can be customized via environment variables
- The RViz configuration is optimized for sensor visualization
- The robot maintains its original four-wheel steering capabilities 