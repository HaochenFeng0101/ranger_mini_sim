#!/usr/bin/env python3
import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Quaternion
from std_msgs.msg import Header
from se2_control.msg import SE2Command
from tf import transformations
import tf2_ros
import geometry_msgs.msg

class SE2DroneSimulator:
    def __init__(self):
        rospy.init_node('se2_drone_simulator')
        
        # Get namespace for this drone
        self.namespace = rospy.get_namespace().strip('/')
        if not self.namespace:
            self.namespace = "drone"
        
        # Simulation parameters
        self.rate = 100.0  # Hz
        self.dt = 1.0 / self.rate
        
        # Get initial position parameters
        self.x = rospy.get_param('~initial_x', 0.0)
        self.y = rospy.get_param('~initial_y', 0.0)
        self.z = 1.0  # Fixed height
        self.yaw = rospy.get_param('~initial_yaw', 0.0)
        
        # Velocity variables
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.vyaw = 0.0
        
        # Control flags
        self.has_received_command = False
        self.motors_enabled = False
        
        # Publishers and subscribers
        self.odom_pub = rospy.Publisher('sim/odom', Odometry, queue_size=1)  # Relative topic, will be namespaced
        self.se2_cmd_sub = rospy.Subscriber('se2_cmd', SE2Command, self.se2_cmd_callback)  # Relative topic, will be namespaced
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Control gains
        self.kx = [1.0, 1.0]
        self.kyaw = 1.0
        
        rospy.loginfo(f"SE2 Drone Simulator ({self.namespace}) initialized at ({self.x:.2f}, {self.y:.2f}, {self.yaw:.2f}) - waiting for commands")
        
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
        
        # If motors are disabled, set velocities to zero immediately
        if not self.motors_enabled:
            self.vx = 0.0
            self.vy = 0.0
            self.vyaw = 0.0
            rospy.logdebug("Motors disabled - setting velocities to zero")
        else:
            rospy.logdebug(f"Motors enabled - velocities: vx={self.vx:.2f}, vy={self.vy:.2f}, vyaw={self.vyaw:.2f}")
        
        self.has_received_command = True
        
        rospy.loginfo(f"SE2 Command received: vx={self.vx:.2f}, vy={self.vy:.2f}, vyaw={self.vyaw:.2f}, motors={self.motors_enabled}")
    
    def update_state(self):
        """Update drone state based on velocities"""
        # Only move if motors are enabled and velocities are non-zero
        if self.motors_enabled and (abs(self.vx) > 0.001 or abs(self.vy) > 0.001 or abs(self.vyaw) > 0.001):
            # Transform robot frame velocities to world frame
            cos_yaw = math.cos(self.yaw)
            sin_yaw = math.sin(self.yaw)
            
            # Robot frame to world frame transformation
            # vx_robot is forward, vy_robot is left
            vx_world = self.vx * cos_yaw - self.vy * sin_yaw
            vy_world = self.vx * sin_yaw + self.vy * cos_yaw
            
            # Update position (simple Euler integration) in world frame
            self.x += vx_world * self.dt
            self.y += vy_world * self.dt
            self.yaw += self.vyaw * self.dt
            
            # Normalize yaw
            while self.yaw > math.pi:
                self.yaw -= 2 * math.pi
            while self.yaw < -math.pi:
                self.yaw += 2 * math.pi
                
            # Debug logging
            rospy.logdebug(f"State update - Robot vel: vx={self.vx:.3f}, vy={self.vy:.3f}, vyaw={self.vyaw:.3f}")
            rospy.logdebug(f"State update - World vel: vx_world={vx_world:.3f}, vy_world={vy_world:.3f}")
            rospy.logdebug(f"State update - New pos: x={self.x:.3f}, y={self.y:.3f}, yaw={self.yaw:.3f}")
        # If motors disabled or velocities are very small, don't move at all
    
    def publish_tf(self):
        """Publish TF transform for visualization"""
        t = geometry_msgs.msg.TransformStamped()
        
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = f"{self.namespace}/base_link"
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        
        # Set orientation (quaternion from yaw)
        q = transformations.quaternion_from_euler(0, 0, self.yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_odometry(self):
        """Publish current odometry"""
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "world"
        odom.child_frame_id = f"{self.namespace}/base_link"
        
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
            self.publish_tf()  # Publish TF for visualization
            rate.sleep()

def main():
    try:
        simulator = SE2DroneSimulator()
        simulator.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 