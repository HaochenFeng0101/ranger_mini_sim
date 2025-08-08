#!/usr/bin/env python3
import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand
from tf import transformations

class SO3TargetController:
    """
    SO3 Target Controller for 3D movement capability.
    Receives target position commands and sends position commands to SO3 controller.
    """
    def __init__(self):
        rospy.init_node('so3_target_controller')
        
        # Current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 1.0
        self.current_yaw = 0.0
        
        # Target state
        self.target_x = 5.0
        self.target_y = 2.0
        self.target_z = 1.5
        self.target_yaw = 0.0
        
        # Control gains
        self.kx = [5.0, 3.0, 7.0]  # Position gains [kx, ky, kz]
        self.kv = [2.5, 2.5, 4.0]  # Velocity gains
        
        # Movement parameters
        self.max_velocity = 2.0
        self.max_acceleration = 1.0
        
        # Publishers and subscribers
        self.position_cmd_pub = rospy.Publisher('/drone2/position_cmd', PositionCommand, queue_size=1)
        self.odom_sub = rospy.Subscriber('/drone2/sim/odom', Odometry, self.odom_callback)
        self.target_sub = rospy.Subscriber('/target_position', PoseStamped, self.target_callback)
        
        # Control timer
        self.control_timer = rospy.Timer(rospy.Duration(0.05), self.control_callback)  # 20Hz
        
        rospy.loginfo("SO3 Target Controller initialized")
        rospy.loginfo("Target drone will move to commanded positions in 3D")
        
    def odom_callback(self, msg):
        """Update current drone state from odometry"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        
        # Extract yaw from quaternion
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler = transformations.euler_from_quaternion(quaternion)
        self.current_yaw = euler[2]
        
    def target_callback(self, msg):
        """Update target position from command"""
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.target_z = msg.pose.position.z
        
        # Extract target yaw if provided
        if msg.pose.orientation.w != 0 or msg.pose.orientation.z != 0:
            quaternion = (
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            )
            euler = transformations.euler_from_quaternion(quaternion)
            self.target_yaw = euler[2]
            
        rospy.loginfo(f"New target: ({self.target_x:.2f}, {self.target_y:.2f}, {self.target_z:.2f})")
        
    def control_callback(self, event):
        """Main control loop - generates position commands for SO3 controller"""
        
        # Calculate position errors
        pos_error_x = self.target_x - self.current_x
        pos_error_y = self.target_y - self.current_y
        pos_error_z = self.target_z - self.current_z
        
        # Calculate desired velocities (simple proportional control)
        des_vel_x = self.kx[0] * pos_error_x
        des_vel_y = self.kx[1] * pos_error_y
        des_vel_z = self.kx[2] * pos_error_z
        
        # Limit velocities
        vel_magnitude = math.sqrt(des_vel_x**2 + des_vel_y**2 + des_vel_z**2)
        if vel_magnitude > self.max_velocity:
            scale = self.max_velocity / vel_magnitude
            des_vel_x *= scale
            des_vel_y *= scale
            des_vel_z *= scale
            
        # Calculate desired accelerations (simple derivative control)
        des_acc_x = self.kv[0] * des_vel_x
        des_acc_y = self.kv[1] * des_vel_y  
        des_acc_z = self.kv[2] * des_vel_z
        
        # Limit accelerations
        acc_magnitude = math.sqrt(des_acc_x**2 + des_acc_y**2 + des_acc_z**2)
        if acc_magnitude > self.max_acceleration:
            scale = self.max_acceleration / acc_magnitude
            des_acc_x *= scale
            des_acc_y *= scale
            des_acc_z *= scale
            
        # Create position command for SO3 controller
        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "world"
        
        # Position
        cmd.position.x = self.target_x
        cmd.position.y = self.target_y
        cmd.position.z = self.target_z
        
        # Velocity
        cmd.velocity.x = des_vel_x
        cmd.velocity.y = des_vel_y
        cmd.velocity.z = des_vel_z
        
        # Acceleration
        cmd.acceleration.x = des_acc_x
        cmd.acceleration.y = des_acc_y
        cmd.acceleration.z = des_acc_z
        
        # Yaw
        cmd.yaw = self.target_yaw
        cmd.yaw_dot = 0.0
        
        # Control gains
        cmd.kx = self.kx
        cmd.kv = self.kv
        
        self.position_cmd_pub.publish(cmd)
        
    def set_target_position(self, x, y, z, yaw=0.0):
        """Programmatically set target position"""
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.target_yaw = yaw
        
    def move_in_pattern(self):
        """Example: Move target in a pattern for testing encirclement"""
        t = rospy.Time.now().to_sec()
        
        # Circular motion with vertical component
        radius = 2.0
        height_variation = 0.5
        frequency = 0.1  # Hz
        
        self.target_x = 5.0 + radius * math.cos(2 * math.pi * frequency * t)
        self.target_y = 2.0 + radius * math.sin(2 * math.pi * frequency * t)
        self.target_z = 1.5 + height_variation * math.sin(4 * math.pi * frequency * t)
        
def main():
    try:
        controller = SO3TargetController()
        
        # Example: Set a specific target after 5 seconds
        def delayed_target():
            rospy.sleep(5.0)
            controller.set_target_position(8.0, 4.0, 2.0, math.pi/4)
            rospy.loginfo("Target position updated!")
            
        # Start delayed target in a separate thread
        import threading
        target_thread = threading.Thread(target=delayed_target)
        target_thread.daemon = True
        target_thread.start()
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
