# Simple Three-Drone Encirclement System

This is a **simplified** implementation of a three-drone encirclement system designed to be easy to understand and use.

## Features

- ✅ **Simple Control**: Basic geometric control, no complex algorithms
- ✅ **Multi-Radius Formation**: Red drone at 1m, Green drone at 5m from target
- ✅ **Synchronized Circular Motion**: Both agents move in sync at different radii
- ✅ **Target Tracking**: Agents track the actual moving target drone (drone2)
- ✅ **Real-time Movement**: All drones move and respond in real-time
- ✅ **Easy Control**: Simple commands to move the target

## System Architecture

### Drones
- **drone0** (Red): Inner circle agent - 1m radius
- **drone1** (Green): Outer circle agent - 5m radius  
- **drone2** (Blue): Target drone to be encircled

### Controllers
- **Simple Encirclement Controller**: Maintains agents opposite each other around target
- **Target Manual Controller**: Allows manual control of target movement

## Quick Start

### 1. Launch the System
```bash
roslaunch se2_control simple_encirclement.launch
```

This starts:
- Three drone simulators
- Simple encirclement controller
- Target manual controller
- RViz visualization

### 2. Move the Target

Use the simple move command:

```bash
# Move target forward
rosrun se2_control move_target.py forward

# Move target left  
rosrun se2_control move_target.py left

# Move target in a circle
rosrun se2_control move_target.py circle

# Stop target
rosrun se2_control move_target.py stop

# Custom movement (vx, vy, vyaw)
rosrun se2_control move_target.py custom 0.5 0.2 0.1
```

### 3. Watch the Encirclement

- **Red drone (drone0)** and **Green drone (drone1)** will automatically move to maintain opposite positions around the **Blue target (drone2)**
- As you move the target, the encircling drones will follow and maintain formation
- In RViz, you can see all three drones and their trajectories

## How It Works

### Simple Algorithm

1. **Calculate Target Position**: Get current position of target drone (drone2)
2. **Calculate Desired Positions**: Place two agents on opposite sides of target
3. **Move to Positions**: Use simple proportional control to move agents to desired positions
4. **Maintain Formation**: Continuously update as target moves

### Key Parameters

```python
self.agent1_radius = 1.0   # Red drone: inner circle (meters)
self.agent2_radius = 5.0   # Green drone: outer circle (meters)
self.kp_radial = 0.5       # Control gain for positioning  
self.max_vel = 0.8         # Maximum velocity (m/s)
self.angular_speed = 0.15  # Circular motion speed (rad/s)
```

## Expected Behavior

1. **Initialization**: 
   - Red drone at (0, 0)
   - Green drone at (3, 0)
   - Blue target at (5, 2)

2. **Multi-Radius Formation**:
   - Red drone moves to 1m radius (inner circle)
   - Green drone moves to 5m radius (outer circle)
   - Both start synchronized circular motion

3. **Synchronized Circular Motion**:
   - Both agents move at same angular position but different radii
   - Red drone moves in tight 1m circle
   - Green drone moves in wide 5m circle
   - Both maintain same relative position (aligned radially)

4. **Target Movement**:
   - When target moves, both formations follow together
   - Inner and outer circles move as a unit
   - Formation maintains radial alignment while following target

## Manual Target Control

You can also control the target manually using ROS topics:

```bash
# Velocity control (recommended)
rostopic pub /target_manual_twist geometry_msgs/Twist "
linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0  
  y: 0.0
  z: 0.1"
```

## Monitoring

### Debug Information
The system prints status every 2 seconds:
- Target position
- Agent distances from target  
- Control commands being sent

### RViz Visualization
- **Red markers**: Agent 1 (drone0)
- **Green markers**: Agent 2 (drone1)
- **Blue markers**: Target (drone2)
- **Trails**: Show movement history

## Troubleshooting

### Drones Not Moving
1. Check if commands are being published:
   ```bash
   rostopic echo /drone0/se2_cmd
   rostopic echo /drone1/se2_cmd
   ```

2. Verify SE2 simulators are receiving commands:
   ```bash
   rostopic list | grep se2_cmd
   ```

3. Check debug output in terminal for status messages

### Formation Not Correct
- The agents will move to opposite sides of the target
- It may take a few seconds to reach the formation
- Try moving the target to see if agents follow

### Target Not Moving
1. Check target manual controller is running
2. Use the move_target.py script to test movement
3. Verify commands reach the target:
   ```bash
   rostopic echo /drone2/se2_cmd
   ```

## Advantages of Simple Version

- ✅ **Easy to Understand**: Basic geometric control
- ✅ **Reliable**: No complex algorithms that can fail
- ✅ **Fast Response**: Immediate reaction to target movement
- ✅ **Predictable**: Agents always try to be opposite each other
- ✅ **Debuggable**: Clear status messages and simple logic

## Files

```
se2_control/
├── scripts/
│   ├── simple_encirclement.py      # Main encirclement controller
│   ├── target_manual_controller.py # Target movement controller  
│   ├── move_target.py             # Easy target movement commands
│   └── se2_drone_simulator.py     # Updated drone simulator
├── launch/
│   └── simple_encirclement.launch # Main launch file
└── README_SIMPLE_ENCIRCLEMENT.md  # This documentation
```

This simplified system gives you exactly what you asked for:
- **Two robots maintaining opposite positions** around the target
- **Following the actual moving target** (drone2, not a fixed point)  
- **Simple, reliable control** without complex algorithms
