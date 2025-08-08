#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from se2_control.msg import SE2Command
from tf import transformations

class SE2DroneController:
    def __init__(self):
        rospy.init_node('se2_drone_controller')
        
        # Control parameters - reduced gains for smoother control
        self.kx = 0.3  # Further reduced for more stable control
        self.ky = 0.3  # Further reduced for more stable control
        self.kyaw = 0.8  # Reduced yaw gain
        self.max_linear_vel = 0.8  # Further reduced max velocity
        self.max_angular_vel = 0.4  # Further reduced max angular velocity
        
        # Fixed height for SE2 control
        self.fixed_height = 1.0
        
        # Current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Target state
        self.target_x = None
        self.target_y = None
        self.target_yaw = None
        self.has_target = False
        self.target_reached = False
        
        # Control tolerances - increased for more stable stopping
        self.position_tolerance = 0.3  # 30cm tolerance
        self.yaw_tolerance = 0.3  # ~17 degrees tolerance
        
        # Timing for target reached
        self.target_reached_time = None
        self.target_reached_delay = rospy.Duration(1.0)  # Increased delay to 1 second
        
        # Stop command counter to ensure multiple stop commands are sent
        self.stop_command_count = 0
        self.max_stop_commands = 5
        
        # Publishers and subscribers
        self.se2_cmd_pub = rospy.Publisher('/se2_cmd', SE2Command, queue_size=1)
        self.odom_sub = rospy.Subscriber('/sim/odom', Odometry, self.odom_callback)
        self.target_sub = rospy.Subscriber('/se2_position_cmd', Pose2D, self.target_callback)
        
        rospy.loginfo("SE2 Drone Controller initialized - 2D movement only")
        
    def odom_callback(self, msg):
        """Callback for odometry data"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                     msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.current_yaw = euler[2]
        
        # Only calculate control if we have a target and haven't reached it yet
        if self.has_target and not self.target_reached:
            self.calculate_control()
        elif self.target_reached and self.target_reached_time is not None:
            # Check if enough time has passed since reaching target
            if rospy.Time.now() - self.target_reached_time > self.target_reached_delay:
                # Send multiple stop commands to ensure stopping
                if self.stop_command_count < self.max_stop_commands:
                    self.send_stop_command()
                    self.stop_command_count += 1
                    rospy.loginfo(f"Sending stop command {self.stop_command_count}/{self.max_stop_commands}")
                else:
                    # Clear everything after sending multiple stop commands
                    self.has_target = False
                    self.target_reached = False
                    self.target_reached_time = None
                    self.stop_command_count = 0
                    rospy.loginfo("Target reached and stopped - ready for new commands")
    
    def target_callback(self, msg):
        """Callback for target pose commands"""
        self.target_x = msg.x
        self.target_y = msg.y
        self.target_yaw = msg.theta
        self.has_target = True
        self.target_reached = False
        self.target_reached_time = None
        self.stop_command_count = 0
        
        rospy.loginfo(f"Received SE2 target: x={msg.x}, y={msg.y}, theta={msg.theta}")
    
    def send_stop_command(self):
        """Send a stop command to the drone"""
        se2_cmd = SE2Command()
        se2_cmd.header.stamp = rospy.Time.now()
        se2_cmd.header.frame_id = "base_link"
        se2_cmd.linear_x = 0.0
        se2_cmd.linear_y = 0.0
        se2_cmd.angular_z = 0.0
        se2_cmd.kx = [self.kx, self.ky]
        se2_cmd.kyaw = self.kyaw
        se2_cmd.enable_motors = False
        se2_cmd.use_external_yaw = False
        se2_cmd.current_yaw = self.current_yaw
        
        self.se2_cmd_pub.publish(se2_cmd)
        rospy.loginfo("STOP command sent - motors disabled")
    
    def calculate_control(self):
        """Calculate and publish control commands for 2D movement"""
        # Check if target is valid
        if self.target_x is None or self.target_y is None or self.target_yaw is None:
            rospy.logwarn("Invalid target received, stopping")
            self.has_target = False
            self.target_reached = False
            self.target_reached_time = None
            self.stop_command_count = 0
            return
        
        # Calculate errors in world frame
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        dyaw = self.target_yaw - self.current_yaw
        
        # Normalize yaw error
        while dyaw > math.pi:
            dyaw -= 2 * math.pi
        while dyaw < -math.pi:
            dyaw += 2 * math.pi
        
        # Calculate position and yaw errors
        position_error = math.sqrt(dx*dx + dy*dy)
        yaw_error = abs(dyaw)
        
        # Calculate heading error (direction to target)
        target_heading = math.atan2(dy, dx)
        heading_error = target_heading - self.current_yaw
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        # Debug information
        rospy.loginfo(f"Errors - Position: {position_error:.3f}m, Yaw: {yaw_error:.3f}rad")
        rospy.loginfo(f"Target: ({self.target_x:.2f}, {self.target_y:.2f}, {self.target_yaw:.2f})")
        rospy.loginfo(f"Current: ({self.current_x:.2f}, {self.current_y:.2f}, {self.current_yaw:.2f})")
        rospy.loginfo(f"Raw errors - dx: {dx:.3f}, dy: {dy:.3f}, dyaw: {dyaw:.3f}")
        rospy.loginfo(f"Heading error: {heading_error:.3f}rad")
        
        # Check if we're close enough to target
        if position_error < self.position_tolerance:
            # Position reached, now check orientation
            if yaw_error < self.yaw_tolerance:
                # Both position and orientation reached
                if not self.target_reached:
                    self.target_reached = True
                    self.target_reached_time = rospy.Time.now()
                    rospy.loginfo(f"Target fully reached! Position error: {position_error:.3f}m, Yaw error: {yaw_error:.3f}rad")
                
                # Send stop command
                vx = 0.0
                vy = 0.0
                vyaw = 0.0
                motors_enabled = False
                rospy.loginfo("Sending stop command - target reached")
            else:
                # Position reached but orientation needs adjustment
                vx = 0.0
                vy = 0.0
                vyaw = self.kyaw * dyaw * 0.5  # Gentle orientation correction
                vyaw = max(-self.max_angular_vel, min(self.max_angular_vel, vyaw))
                motors_enabled = True
                rospy.loginfo(f"Position reached, adjusting orientation - yaw error: {yaw_error:.3f}rad")
        else:
            # Use simpler control approach: first turn towards target, then move forward
            if abs(heading_error) > 0.2:  # Reduced threshold to avoid oscillation
                # Turn towards target
                vx = 0.0
                vy = 0.0
                vyaw = self.kyaw * heading_error * 0.8  # Increased turning gain
                vyaw = max(-self.max_angular_vel, min(self.max_angular_vel, vyaw))
                rospy.loginfo(f"Turning towards target - heading error: {heading_error:.3f}rad")
            else:
                # Move towards target using simple proportional control
                # Calculate desired velocity magnitude (reduce as we get closer)
                max_velocity = min(self.max_linear_vel, position_error * 0.5)  # Increased velocity factor
                
                # Calculate velocity components in world frame
                if position_error > 0.1:  # Avoid division by zero
                    # Normalize the direction and apply velocity
                    vx_world = (dx / position_error) * max_velocity
                    vy_world = (dy / position_error) * max_velocity
                else:
                    vx_world = 0.0
                    vy_world = 0.0
                
                # Transform to robot frame (corrected transformation)
                cos_yaw = math.cos(self.current_yaw)
                sin_yaw = math.sin(self.current_yaw)
                # Robot frame: x is forward, y is left
                vx = vx_world * cos_yaw + vy_world * sin_yaw
                vy = -vx_world * sin_yaw + vy_world * cos_yaw
                
                # Add small turning correction
                vyaw = self.kyaw * heading_error * 0.1  # Reduced turning gain when moving
                
                # Limit velocities
                vx = max(-self.max_linear_vel, min(self.max_linear_vel, vx))
                vy = max(-self.max_linear_vel, min(self.max_linear_vel, vy))
                vyaw = max(-self.max_angular_vel, min(self.max_angular_vel, vyaw))
                
                rospy.loginfo(f"Moving towards target - position error: {position_error:.3f}m, max_velocity: {max_velocity:.3f}")
                rospy.loginfo(f"World velocities - vx_world: {vx_world:.3f}, vy_world: {vy_world:.3f}")
                rospy.loginfo(f"Robot frame velocities - vx: {vx:.3f}, vy: {vy:.3f}")
            
            motors_enabled = True
            
            rospy.loginfo(f"Moving to target - Pos error: {position_error:.3f}m, Yaw error: {yaw_error:.3f}rad")
        
        # Publish SE2 command
        se2_cmd = SE2Command()
        se2_cmd.header.stamp = rospy.Time.now()
        se2_cmd.header.frame_id = "base_link"
        se2_cmd.linear_x = vx
        se2_cmd.linear_y = vy
        se2_cmd.angular_z = vyaw
        se2_cmd.kx = [self.kx, self.ky]
        se2_cmd.kyaw = self.kyaw
        se2_cmd.enable_motors = motors_enabled
        se2_cmd.use_external_yaw = False
        se2_cmd.current_yaw = self.current_yaw
        
        self.se2_cmd_pub.publish(se2_cmd)
        
        if motors_enabled:
            rospy.loginfo(f"SE2 Command: vx={vx:.2f}, vy={vy:.2f}, vyaw={vyaw:.2f}")
        else:
            rospy.loginfo("SE2 Command: STOP (motors disabled)")

def main():
    try:
        controller = SE2DroneController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 