#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose2D

def publish_test_pose():
    rospy.init_node('test_pose2d_publisher')
    pub = rospy.Publisher('/se2_position_cmd', Pose2D, queue_size=1)
    
    # Wait for publisher to connect
    rospy.sleep(2.0)
    
    # Publish a test pose
    pose = Pose2D()
    pose.x = 2.0
    pose.y = 1.0
    pose.theta = 0.0
    
    pub.publish(pose)
    rospy.loginfo("Published Pose2D: x=%.1f, y=%.1f, theta=%.1f", pose.x, pose.y, pose.theta)
    
    # Keep publishing for a few seconds
    rate = rospy.Rate(1)  # 1 Hz
    for i in range(5):
        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_test_pose()
    except rospy.ROSInterruptException:
        pass 