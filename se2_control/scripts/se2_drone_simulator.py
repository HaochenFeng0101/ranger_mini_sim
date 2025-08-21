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
        
        # Manual control support (for target drone)
        self.manual_twist_sub = rospy.Subscriber('/target_manual_twist', Twist, self.manual_twist_callback)
        self.manual_control_active = False
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Control gains
        self.kx = [1.0, 1.0]
        self.kyaw = 1.0
        
        rospy.loginfo(f"SE2 Drone Simulator ({self.namespace}) initialized at ({self.x:.2f}, {self.y:.2f}, {self.yaw:.2f}) - waiting for commands")
        
    def se2_cmd_callback(self, msg):
        """Handle SE2 command messages with improved validation"""
        # Extract velocities with validation
        self.vx = max(-2.0, min(2.0, msg.linear_x))
        self.vy = max(-2.0, min(2.0, msg.linear_y))
        self.vyaw = max(-1.0, min(1.0, msg.angular_z))
        
        # Update gains if provided
        if len(msg.kx) >= 2:
            self.kx = [max(0.1, min(2.0, k)) for k in msg.kx]
        self.kyaw = max(0.1, min(2.0, msg.kyaw))
        
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
    
    def manual_twist_callback(self, msg):
        """Handle manual twist commands for target control with improved validation"""
        # Only apply manual control if this is the target drone (drone2)
        if self.namespace == "drone2":
            # Validate velocities before applying
            vx = max(-2.0, min(2.0, msg.linear.x))  # Limit to reasonable range
            vy = max(-2.0, min(2.0, msg.linear.y))
            vyaw = max(-1.0, min(1.0, msg.angular.z))
            
            self.vx = vx
            self.vy = vy
            self.vyaw = vyaw
            self.manual_control_active = True
            self.motors_enabled = True
            self.has_received_command = True
            
            rospy.logdebug(f"Manual control active: vx={self.vx:.2f}, vy={self.vy:.2f}, vyaw={self.vyaw:.2f}")
    
    def update_state(self):
        """Update drone state based on velocities with improved movement validation"""
        # Only move if motors are enabled, we have received commands, and velocities are non-zero
        if self.motors_enabled and self.has_received_command and (abs(self.vx) > 0.001 or abs(self.vy) > 0.001 or abs(self.vyaw) > 0.001):
            # Transform robot frame velocities to world frame
            cos_yaw = math.cos(self.yaw)
            sin_yaw = math.sin(self.yaw)
            
            # Robot frame to world frame transformation
            # vx_robot is forward, vy_robot is left
            vx_world = self.vx * cos_yaw - self.vy * sin_yaw
            vy_world = self.vx * sin_yaw + self.vy * cos_yaw
            
            # Calculate new position
            new_x = self.x + vx_world * self.dt
            new_y = self.y + vy_world * self.dt
            new_yaw = self.yaw + self.vyaw * self.dt
            
            # Normalize yaw
            while new_yaw > math.pi:
                new_yaw -= 2 * math.pi
            while new_yaw < -math.pi:
                new_yaw += 2 * math.pi
            
            # Validate movement (prevent getting stuck at boundaries)
            if self.is_valid_position(new_x, new_y):
                self.x = new_x
                self.y = new_y
                self.yaw = new_yaw
                
                # Debug logging
                rospy.logdebug(f"State update - Robot vel: vx={self.vx:.3f}, vy={self.vy:.3f}, vyaw={self.vyaw:.3f}")
                rospy.logdebug(f"State update - World vel: vx_world={vx_world:.3f}, vy_world={vy_world:.3f}")
                rospy.logdebug(f"State update - New pos: x={self.x:.3f}, y={self.y:.3f}, yaw={self.yaw:.3f}")
            else:
                rospy.logwarn(f"Invalid position detected: x={new_x:.3f}, y={new_y:.3f} - movement blocked")
                # Try to recover by moving in a different direction
                self.recover_from_stuck_position()
        else:
            # Debug why not moving
            if not self.motors_enabled:
                rospy.logdebug(f"{self.namespace}: Not moving - motors disabled")
            elif not self.has_received_command:
                rospy.logdebug(f"{self.namespace}: Not moving - no commands received")
            elif abs(self.vx) <= 0.001 and abs(self.vy) <= 0.001 and abs(self.vyaw) <= 0.001:
                rospy.logdebug(f"{self.namespace}: Not moving - velocities too small")
        # If motors disabled or velocities are very small, don't move at all
    
    def is_valid_position(self, x, y):
        """Check if a position is valid (within reasonable bounds)"""
        # Define world boundaries (adjust as needed)
        max_x = 50.0
        max_y = 50.0
        min_x = -50.0
        min_y = -50.0
        
        return min_x <= x <= max_x and min_y <= y <= max_y
    
    def recover_from_stuck_position(self):
        """Recover from stuck position by applying small random movement"""
        import random
        
        # Small random movement to escape stuck position
        recovery_vx = random.uniform(-0.1, 0.1)
        recovery_vy = random.uniform(-0.1, 0.1)
        recovery_vyaw = random.uniform(-0.1, 0.1)
        
        # Apply recovery movement
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        
        vx_world = recovery_vx * cos_yaw - recovery_vy * sin_yaw
        vy_world = recovery_vx * sin_yaw + recovery_vy * cos_yaw
        
        new_x = self.x + vx_world * self.dt
        new_y = self.y + vy_world * self.dt
        
        if self.is_valid_position(new_x, new_y):
            self.x = new_x
            self.y = new_y
            self.yaw += recovery_vyaw * self.dt
            rospy.loginfo(f"Recovered from stuck position: x={self.x:.3f}, y={self.y:.3f}")
        else:
            rospy.logwarn("Recovery movement also invalid - robot may be at boundary")
    
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