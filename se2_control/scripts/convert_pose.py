#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Pose2D
from tf import transformations

def pose_stamped_to_pose2d(msg):
    # Convert PoseStamped to Pose2D
    pose2d = Pose2D()
    pose2d.x = msg.pose.position.x
    pose2d.y = msg.pose.position.y
    
    # Convert quaternion to yaw
    quaternion = (msg.pose.orientation.x, msg.pose.orientation.y, 
                  msg.pose.orientation.z, msg.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    pose2d.theta = euler[2]  # yaw
    
    pub.publish(pose2d)

rospy.init_node('pose_bridge')
pub = rospy.Publisher('/xray_source/position_cmd', Pose2D, queue_size=1)
sub = rospy.Subscriber('/xray_source/pose_stamped', PoseStamped, pose_stamped_to_pose2d)
rospy.spin()

#bridge file for rviz to set goal pose