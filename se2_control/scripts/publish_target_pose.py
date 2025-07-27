#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose2D

if __name__ == "__main__":
    rospy.init_node("target_pose_publisher", anonymous=True)
    pub = rospy.Publisher("/xray_source/position_cmd", Pose2D, queue_size=1)
    rospy.sleep(2.0)  # Wait for publisher to connect
    msg = Pose2D()
    msg.x = 2.0
    msg.y = 1.0
    msg.theta = 0.0
    pub.publish(msg)
    rospy.loginfo("Published target pose: (2.0, 1.0, 0.0)")
    rospy.sleep(1.0)