#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry
from se2_control.msg import SE2Command

class DroneMonitor:
    def __init__(self):
        rospy.init_node('drone_monitor')
        
        # Current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Command state
        self.last_cmd_vx = 0.0
        self.last_cmd_vy = 0.0
        self.last_cmd_vyaw = 0.0
        self.last_cmd_motors_enabled = False
        
        # Subscribers
        self.odom_sub = rospy.Subscriber('sim/odom', Odometry, self.odom_callback)  # Relative topic, will be namespaced
        self.cmd_sub = rospy.Subscriber('/se2_cmd', SE2Command, self.cmd_callback)
        
        # Timer for periodic status updates
        self.status_timer = rospy.Timer(rospy.Duration(1.0), self.status_callback)
        
        rospy.loginfo("Drone Monitor initialized")
    
    def odom_callback(self, msg):
        """Callback for odometry data"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        from tf import transformations
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                     msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.current_yaw = euler[2]
    
    def cmd_callback(self, msg):
        """Callback for SE2 command messages"""
        self.last_cmd_vx = msg.linear_x
        self.last_cmd_vy = msg.linear_y
        self.last_cmd_vyaw = msg.angular_z
        self.last_cmd_motors_enabled = msg.enable_motors
    
    def status_callback(self, event):
        """Periodic status update"""
        rospy.loginfo("=== DRONE STATUS ===")
        rospy.loginfo(f"Position: ({self.current_x:.3f}, {self.current_y:.3f}, {self.current_yaw:.3f})")
        rospy.loginfo(f"Last Command: vx={self.last_cmd_vx:.3f}, vy={self.last_cmd_vy:.3f}, vyaw={self.last_cmd_vyaw:.3f}")
        rospy.loginfo(f"Motors Enabled: {self.last_cmd_motors_enabled}")
        
        # Check if drone is moving
        velocity_magnitude = math.sqrt(self.last_cmd_vx**2 + self.last_cmd_vy**2 + self.last_cmd_vyaw**2)
        if velocity_magnitude > 0.001:
            rospy.loginfo(f"Status: MOVING (velocity magnitude: {velocity_magnitude:.3f})")
        else:
            rospy.loginfo("Status: STOPPED")
        rospy.loginfo("===================")

def main():
    try:
        monitor = DroneMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 