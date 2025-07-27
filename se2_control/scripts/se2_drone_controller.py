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
        
        # Control parameters
        self.kx = 1.0
        self.ky = 1.0
        self.kyaw = 1.0
        self.max_linear_vel = 2.0
        self.max_angular_vel = 1.0
        
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
        
        # Only calculate control if we have a target
        if self.has_target:
            self.calculate_control()
        # If no target, don't send any commands at all
    
    def target_callback(self, msg):
        """Callback for target pose commands"""
        self.target_x = msg.x
        self.target_y = msg.y
        self.target_yaw = msg.theta
        self.has_target = True
        
        rospy.loginfo(f"Received SE2 target: x={msg.x}, y={msg.y}, theta={msg.theta}")
    
    def calculate_control(self):
        """Calculate and publish control commands for 2D movement"""
        # This method is only called when we have a target
        
        # Check if target is valid
        if self.target_x is None or self.target_y is None or self.target_yaw is None:
            rospy.logwarn("Invalid target received, stopping")
            self.has_target = False
            return
        
        # Calculate errors
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        dyaw = self.target_yaw - self.current_yaw
        
        # Normalize yaw error
        while dyaw > math.pi:
            dyaw -= 2 * math.pi
        while dyaw < -math.pi:
            dyaw += 2 * math.pi
        
        # Transform errors to robot frame
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        error_x = dx * cos_yaw + dy * sin_yaw
        error_y = -dx * sin_yaw + dy * cos_yaw
        
        # Check if we're close enough to target
        position_error = math.sqrt(dx*dx + dy*dy)
        yaw_error = abs(dyaw)
        
        # Stop if we're close enough to target
        if position_error < 0.1 and yaw_error < 0.1:  # 10cm position, 0.1rad yaw
            vx = 0.0
            vy = 0.0
            vyaw = 0.0
            motors_enabled = False
            rospy.loginfo("Target reached! Stopping.")
        else:
            # Calculate control commands (2D only)
            vx = self.kx * error_x
            vy = self.ky * error_y
            vyaw = self.kyaw * dyaw
            
            # Limit velocities
            vx = max(-self.max_linear_vel, min(self.max_linear_vel, vx))
            vy = max(-self.max_linear_vel, min(self.max_linear_vel, vy))
            vyaw = max(-self.max_angular_vel, min(self.max_angular_vel, vyaw))
            motors_enabled = True
        
        # Publish SE2 command
        se2_cmd = SE2Command()
        se2_cmd.header.stamp = rospy.Time.now()
        se2_cmd.header.frame_id = "base_link"
        se2_cmd.linear_x = vx
        se2_cmd.linear_y = vy
        se2_cmd.angular_z = vyaw
        se2_cmd.kx = [self.kx, self.ky]
        se2_cmd.kyaw = self.kyaw
        se2_cmd.enable_motors = motors_enabled  # Only enable motors when moving
        se2_cmd.use_external_yaw = False
        se2_cmd.current_yaw = self.current_yaw
        
        self.se2_cmd_pub.publish(se2_cmd)
        
        rospy.loginfo(f"SE2 Command: vx={vx:.2f}, vy={vy:.2f}, vyaw={vyaw:.2f}")

def main():
    try:
        controller = SE2DroneController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 