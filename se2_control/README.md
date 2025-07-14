# SE2 Controller for X-ray Source

This package provides an SE2 (Special Euclidean group in 2D) controller for the X-ray source model with four-wheel steering. The controller is similar to the SO3 controller used for UAVs but adapted for ground vehicles.

## Features

- **SE2 Control**: Handles position (x, y) and orientation (yaw) control
- **Four-wheel steering**: Supports both front and rear wheel steering
- **ROS Integration**: Publishes `FourWheelSteering` commands and `Twist` messages
- **Configurable**: Vehicle parameters and control gains can be tuned
- **Nodelet-based**: Efficient for high-frequency control

## Dependencies

- `roscpp`
- `nav_msgs`
- `geometry_msgs`
- `tf`
- `nodelet`
- `four_wheel_steering_msgs`
- `std_msgs`
- `sensor_msgs`
- `Eigen3`

## Usage

### Building

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Running the Controller

```bash
# Launch the SE2 controller
roslaunch se2_control se2_control.launch
```

### Topics

#### Subscribed Topics

- `/xray_source/odom` (nav_msgs/Odometry): Vehicle odometry
- `/xray_source/position_cmd` (geometry_msgs/Pose2D): Position and orientation commands
- `/xray_source/twist_cmd` (geometry_msgs/Twist): Velocity commands
- `/xray_source/enable_motors` (std_msgs/Bool): Enable/disable motors
- `/xray_source/imu` (sensor_msgs/Imu): IMU data (optional)

#### Published Topics

- `/xray_source/four_wheel_steering_command` (four_wheel_steering_msgs/FourWheelSteeringStamped): Steering commands
- `/xray_source/cmd_vel` (geometry_msgs/Twist): Velocity commands for compatibility

### Parameters

- `vehicle_name`: Name of the vehicle (default: "xray_source")
- `mass`: Vehicle mass in kg (default: 15.0)
- `wheel_base`: Distance between front and rear axles in m (default: 0.46)
- `track`: Distance between left and right wheels in m (default: 0.412)
- `wheel_radius`: Wheel radius in m (default: 0.08)
- `kx_x`: Position control gain for x-axis (default: 1.0)
- `kx_y`: Position control gain for y-axis (default: 1.0)
- `kyaw`: Yaw control gain (default: 1.0)
- `init_x`, `init_y`, `init_z`: Initial position (optional)
- `use_external_yaw`: Use external yaw from odometry (default: true)

### Control Modes

#### Position Control
Send a `geometry_msgs/Pose2D` message to `/xray_source/position_cmd`:
```python
from geometry_msgs.msg import Pose2D

cmd = Pose2D()
cmd.x = 5.0      # Target x position
cmd.y = 2.0      # Target y position
cmd.theta = 0.5  # Target yaw angle
pub.publish(cmd)
```

#### Velocity Control
Send a `geometry_msgs/Twist` message to `/xray_source/twist_cmd`:
```python
from geometry_msgs.msg import Twist

cmd = Twist()
cmd.linear.x = 1.0   # Forward velocity
cmd.linear.y = 0.0   # Lateral velocity
cmd.angular.z = 0.1  # Yaw rate
pub.publish(cmd)
```

## Integration with X-ray Source Model

The controller is designed to work with your X-ray source model that has four-wheel steering. The controller outputs steering angles and velocity commands that can be used by the four-wheel steering controller.

### Example Integration

1. Launch the X-ray source model in Gazebo
2. Launch the four-wheel steering controller
3. Launch the SE2 controller
4. Send position or velocity commands

```bash
# Terminal 1: Launch X-ray source in Gazebo
roslaunch ranger_mini_v2_gazebo ranger_mini_xray_source.launch

# Terminal 2: Launch four-wheel steering controller
roslaunch four_wheel_steering_controller four_wheel_steering_controller.launch

# Terminal 3: Launch SE2 controller
roslaunch se2_control se2_control.launch

# Terminal 4: Send commands
rostopic pub /xray_source/position_cmd geometry_msgs/Pose2D "x: 5.0
y: 2.0
theta: 0.5"
```

## Tuning

The control gains can be tuned based on your specific requirements:

- **kx_x, kx_y**: Position control gains. Higher values make the controller more aggressive
- **kyaw**: Yaw control gain. Higher values make the vehicle turn more quickly
- **Vehicle parameters**: Ensure these match your actual vehicle dimensions

## Troubleshooting

1. **Vehicle not moving**: Check that the four-wheel steering controller is running and the vehicle has proper odometry
2. **Oscillations**: Reduce control gains (kx_x, kx_y, kyaw)
3. **Slow response**: Increase control gains
4. **Wrong steering angles**: Verify vehicle parameters (wheel_base, track, wheel_radius)

## License

BSD License 