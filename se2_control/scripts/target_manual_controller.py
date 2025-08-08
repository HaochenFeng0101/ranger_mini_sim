#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose2D, Twist
from se2_control.msg import SE2Command

class TargetManualController:
    def __init__(self):
        rospy.init_node('target_manual_controller')
        
        # Control parameters
        self.max_linear_vel = 1.0
        self.max_angular_vel = 0.5
        
        # Publishers
        self.target_cmd_pub = rospy.Publisher('/drone2/se2_cmd', SE2Command, queue_size=1)
        
        # Subscribers for manual control
        self.pose_sub = rospy.Subscriber('/target_manual_cmd', Pose2D, self.pose_callback)
        self.twist_sub = rospy.Subscriber('/target_manual_twist', Twist, self.twist_callback)
        
        rospy.loginfo("Target Manual Controller initialized")
        rospy.loginfo("Control target drone using:")
        rospy.loginfo("  - Position: rostopic pub /target_manual_cmd geometry_msgs/Pose2D")
        rospy.loginfo("  - Velocity: rostopic pub /target_manual_twist geometry_msgs/Twist")
        rospy.loginfo("Example position: rostopic pub /target_manual_cmd geometry_msgs/Pose2D \"x: 8.0\\ny: 3.0\\ntheta: 0.0\"")
        rospy.loginfo("Example velocity: rostopic pub /target_manual_twist geometry_msgs/Twist \"linear: {x: 0.5, y: 0.0}\\nangular: {z: 0.2}\"")
    
    def pose_callback(self, msg):
        """Handle position commands"""
        # This is for waypoint-based control
        # For now, just log the command - the encirclement controller will handle movement
        rospy.loginfo(f"Target position command: ({msg.x}, {msg.y}, {msg.theta})")
        
        # Send a simple velocity command towards the target
        # This is a simple proportional controller
        # (In a real system, you'd use proper trajectory planning)
        cmd = SE2Command()
        cmd.header.stamp = rospy.Time.now()
        cmd.linear_x = 0.1  # Small constant velocity for demo
        cmd.linear_y = 0.0
        cmd.angular_z = 0.0
        cmd.enable_motors = True
        self.target_cmd_pub.publish(cmd)
    
    def twist_callback(self, msg):
        """Handle velocity commands"""
        cmd = SE2Command()
        cmd.header.stamp = rospy.Time.now()
        cmd.linear_x = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.x))
        cmd.linear_y = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.y))
        cmd.angular_z = max(-self.max_angular_vel, min(self.max_angular_vel, msg.angular.z))
        cmd.enable_motors = True
        cmd.use_external_yaw = False
        self.target_cmd_pub.publish(cmd)
        
        rospy.loginfo(f"Target velocity command: vx={cmd.linear_x:.2f}, vy={cmd.linear_y:.2f}, vyaw={cmd.angular_z:.2f}")

def main():
    try:
        controller = TargetManualController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 