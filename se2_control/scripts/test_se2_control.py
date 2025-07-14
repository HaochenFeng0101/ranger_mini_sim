#!/usr/bin/env python

import rospy
import math
import time
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Bool

class SE2ControlTester:
    def __init__(self):
        rospy.init_node('se2_control_tester', anonymous=True)
        
        # Publishers
        self.position_pub = rospy.Publisher('/xray_source/position_cmd', Pose2D, queue_size=1)
        self.twist_pub = rospy.Publisher('/xray_source/twist_cmd', Twist, queue_size=1)
        self.enable_pub = rospy.Publisher('/xray_source/enable_motors', Bool, queue_size=1)
        
        # Wait for publishers to be ready
        rospy.sleep(1.0)
        
    def enable_motors(self):
        """Enable the motors"""
        enable_msg = Bool()
        enable_msg.data = True
        self.enable_pub.publish(enable_msg)
        rospy.loginfo("Motors enabled")
        
    def disable_motors(self):
        """Disable the motors"""
        enable_msg = Bool()
        enable_msg.data = False
        self.enable_pub.publish(enable_msg)
        rospy.loginfo("Motors disabled")
        
    def send_position_command(self, x, y, theta):
        """Send a position command"""
        cmd = Pose2D()
        cmd.x = x
        cmd.y = y
        cmd.theta = theta
        self.position_pub.publish(cmd)
        rospy.loginfo(f"Position command sent: x={x}, y={y}, theta={theta}")
        
    def send_velocity_command(self, vx, vy, omega):
        """Send a velocity command"""
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = omega
        self.twist_pub.publish(cmd)
        rospy.loginfo(f"Velocity command sent: vx={vx}, vy={vy}, omega={omega}")
        
    def test_position_control(self):
        """Test position control with a square trajectory"""
        rospy.loginfo("Starting position control test...")
        
        # Enable motors
        self.enable_motors()
        rospy.sleep(2.0)
        
        # Square trajectory
        positions = [
            (2.0, 0.0, 0.0),      # Forward
            (2.0, 2.0, math.pi/2), # Right
            (0.0, 2.0, math.pi),   # Backward
            (0.0, 0.0, -math.pi/2), # Left
            (0.0, 0.0, 0.0)        # Return to start
        ]
        
        for i, (x, y, theta) in enumerate(positions):
            rospy.loginfo(f"Moving to position {i+1}: ({x}, {y}, {theta})")
            self.send_position_command(x, y, theta)
            rospy.sleep(5.0)  # Wait for movement
            
        rospy.loginfo("Position control test completed")
        
    def test_velocity_control(self):
        """Test velocity control"""
        rospy.loginfo("Starting velocity control test...")
        
        # Enable motors
        self.enable_motors()
        rospy.sleep(2.0)
        
        # Forward motion
        rospy.loginfo("Moving forward...")
        self.send_velocity_command(1.0, 0.0, 0.0)
        rospy.sleep(3.0)
        
        # Turn left
        rospy.loginfo("Turning left...")
        self.send_velocity_command(0.5, 0.0, 0.5)
        rospy.sleep(3.0)
        
        # Stop
        rospy.loginfo("Stopping...")
        self.send_velocity_command(0.0, 0.0, 0.0)
        rospy.sleep(1.0)
        
        rospy.loginfo("Velocity control test completed")
        
    def run_tests(self):
        """Run all tests"""
        try:
            # Test position control
            self.test_position_control()
            rospy.sleep(2.0)
            
            # Test velocity control
            self.test_velocity_control()
            
        except rospy.ROSInterruptException:
            rospy.loginfo("Test interrupted")
        finally:
            # Disable motors
            self.disable_motors()

if __name__ == '__main__':
    try:
        tester = SE2ControlTester()
        tester.run_tests()
    except rospy.ROSInterruptException:
        pass 