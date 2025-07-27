#!/usr/bin/env python3
import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Quaternion
from std_msgs.msg import Header
from se2_control.msg import SE2Command
from tf import transformations

class SE2DroneSimulator:
    def __init__(self):
        rospy.init_node('se2_drone_simulator')
        
        # Simulation parameters
        self.rate = 100.0  # Hz
        self.dt = 1.0 / self.rate
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.z = 1.0  # Fixed height
        self.yaw = 0.0
        
        # Velocity variables
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.vyaw = 0.0
        
        # Control flags
        self.has_received_command = False
        self.motors_enabled = False
        
        # Publishers and subscribers
        self.odom_pub = rospy.Publisher('/sim/odom', Odometry, queue_size=1)
        self.se2_cmd_sub = rospy.Subscriber('/se2_cmd', SE2Command, self.se2_cmd_callback)
        
        # Control gains
        self.kx = [1.0, 1.0]
        self.kyaw = 1.0
        
        rospy.loginfo("SE2 Drone Simulator initialized - waiting for commands")
        
    def se2_cmd_callback(self, msg):
        """Handle SE2 command messages"""
        # Extract velocities
        self.vx = msg.linear_x
        self.vy = msg.linear_y
        self.vyaw = msg.angular_z
        
        # Update gains if provided
        if len(msg.kx) >= 2:
            self.kx = msg.kx
        self.kyaw = msg.kyaw
        
        # Check motor enable status
        self.motors_enabled = msg.enable_motors
        self.has_received_command = True
        
        rospy.loginfo(f"SE2 Command: vx={self.vx:.2f}, vy={self.vy:.2f}, vyaw={self.vyaw:.2f}, motors={self.motors_enabled}")
    
    def update_state(self):
        """Update drone state based on velocities"""
        # Only move if motors are enabled
        if self.motors_enabled:
            # Update position (simple Euler integration)
            self.x += self.vx * self.dt
            self.y += self.vy * self.dt
            self.yaw += self.vyaw * self.dt
            
            # Normalize yaw
            while self.yaw > math.pi:
                self.yaw -= 2 * math.pi
            while self.yaw < -math.pi:
                self.yaw += 2 * math.pi
        # If motors disabled, don't move at all
    
    def publish_odometry(self):
        """Publish current odometry"""
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "world"
        odom.child_frame_id = "quadrotor"
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        
        # Set orientation (quaternion from yaw)
        q = transformations.quaternion_from_euler(0, 0, self.yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # Set velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.linear.z = self.vz
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.vyaw
        
        # Set covariance (identity matrix for simplicity)
        odom.pose.covariance = [0.01] * 36
        odom.twist.covariance = [0.01] * 36
        
        self.odom_pub.publish(odom)
    
    def run(self):
        """Main simulation loop"""
        rate = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            self.update_state()
            self.publish_odometry()
            rate.sleep()

def main():
    try:
        simulator = SE2DroneSimulator()
        simulator.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 