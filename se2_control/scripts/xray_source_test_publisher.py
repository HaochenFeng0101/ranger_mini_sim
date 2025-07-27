#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Pose2D

class XraySourceTestPublisher:
    def __init__(self):
        rospy.init_node('xray_source_test_publisher')
        
        # Publisher for pose commands
        self.pose_pub = rospy.Publisher('/xray_source/position_cmd', Pose2D, queue_size=1)
        
        # Test poses
        self.test_poses = [
            (2.0, 0.0, 0.0),      # Forward 2m
            (2.0, 2.0, math.pi/4), # Forward and right
            (0.0, 2.0, math.pi/2), # Right 2m
            (-2.0, 2.0, 3*math.pi/4), # Back and right
            (-2.0, 0.0, math.pi),  # Back 2m
            (-2.0, -2.0, -3*math.pi/4), # Back and left
            (0.0, -2.0, -math.pi/2), # Left 2m
            (2.0, -2.0, -math.pi/4), # Forward and left
            (0.0, 0.0, 0.0),       # Return to origin
        ]
        
        self.current_pose_index = 0
        self.pose_delay = 10.0  # seconds between poses
        
        rospy.loginfo("X-ray Source Test Publisher initialized")
        rospy.loginfo("Will send poses every %d seconds", self.pose_delay)
        
    def run(self):
        rate = rospy.Rate(1.0)  # 1 Hz
        
        while not rospy.is_shutdown():
            if self.current_pose_index < len(self.test_poses):
                x, y, theta = self.test_poses[self.current_pose_index]
                
                pose = Pose2D()
                pose.x = x
                pose.y = y
                pose.theta = theta
                
                self.pose_pub.publish(pose)
                rospy.loginfo("Sent pose %d: x=%.1f, y=%.1f, theta=%.2f", 
                             self.current_pose_index + 1, x, y, theta)
                
                self.current_pose_index += 1
                
                # Wait before sending next pose
                rospy.sleep(self.pose_delay)
            else:
                rospy.loginfo("All test poses sent. Restarting...")
                self.current_pose_index = 0
                rospy.sleep(5.0)  # Wait 5 seconds before restarting
                
            rate.sleep()

if __name__ == '__main__':
    try:
        publisher = XraySourceTestPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass 