#!/usr/bin/env python3
"""
SE2 to Velocity Converter - Converts SE2Command to Twist for four-wheel steering controller
"""
import rospy
from se2_control.msg import SE2Command
from geometry_msgs.msg import Twist

class SE2ToVelConverter:
    def __init__(self, robot_name):
        self.robot_name = robot_name
        
        # Subscribe to SE2 commands
        self.se2_sub = rospy.Subscriber(f'/{robot_name}/se2_cmd', SE2Command, self.se2_callback)
        
        # Publish to velocity controller
        self.vel_pub = rospy.Publisher(f'/{robot_name}/four_wheel_steering_controller/cmd_vel', Twist, queue_size=1)
        
        rospy.loginfo(f"SE2 to Velocity Converter initialized for {robot_name}")
    
    def se2_callback(self, msg):
        """Convert SE2Command to Twist"""
        if not msg.enable_motors:
            # Send stop command
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
        else:
            # Convert SE2 velocities to Twist
            twist = Twist()
            twist.linear.x = msg.linear_x
            twist.linear.y = msg.linear_y
            twist.angular.z = msg.angular_z
        
        # Publish velocity command
        self.vel_pub.publish(twist)
        
        rospy.loginfo(f"{self.robot_name}: Publishing velocity - linear: ({twist.linear.x:.3f}, {twist.linear.y:.3f}), angular: {twist.angular.z:.3f}")

def main():
    rospy.init_node('se2_to_vel_converter')
    
    # Get robot name from parameter or use default
    robot_name = rospy.get_param('~robot_name', 'bed_trolley')
    
    # Create converter
    converter = SE2ToVelConverter(robot_name)
    
    rospy.spin()

if __name__ == '__main__':
    main()
