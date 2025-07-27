#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose2D

def publish_single_pose():
    rospy.init_node('drone_test_pose_publisher')
    pub = rospy.Publisher('/se2_position_cmd', Pose2D, queue_size=1)
    
    # Wait for publisher to connect
    rospy.sleep(1.0)
    
    # Send one pose forward
    pose = Pose2D()
    pose.x = 2.0
    pose.y = 0.0
    pose.theta = 0.0
    
    rospy.loginfo("Publishing drone pose: x=%.1f, y=%.1f, theta=%.1f", pose.x, pose.y, pose.theta)
    pub.publish(pose)
    
    # Wait 5 seconds
    rospy.sleep(5.0)
    
    # Send pose back to origin
    pose.x = 0.0
    pose.y = 0.0
    pose.theta = 0.0
    
    rospy.loginfo("Publishing drone pose back: x=%.1f, y=%.1f, theta=%.1f", pose.x, pose.y, pose.theta)
    pub.publish(pose)
    
    rospy.loginfo("Done publishing poses")

if __name__ == '__main__':
    try:
        publish_single_pose()
    except rospy.ROSInterruptException:
        pass 