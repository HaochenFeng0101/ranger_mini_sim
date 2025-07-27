#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Pose2D

class ManualPosePublisher:
    def __init__(self):
        rospy.init_node('manual_pose_publisher')
        
        # Publisher for pose commands
        self.pose_pub = rospy.Publisher('/xray_source/position_cmd', Pose2D, queue_size=1)
        
        rospy.loginfo("Manual Pose Publisher initialized")
        rospy.loginfo("Commands:")
        rospy.loginfo("  'f' - Forward 1m")
        rospy.loginfo("  'b' - Backward 1m")
        rospy.loginfo("  'l' - Left 1m")
        rospy.loginfo("  'r' - Right 1m")
        rospy.loginfo("  't' - Turn 90 degrees")
        rospy.loginfo("  'h' - Turn -90 degrees")
        rospy.loginfo("  's' - Stop (return to origin)")
        rospy.loginfo("  'q' - Quit")
        
    def send_pose(self, x, y, theta):
        pose = Pose2D()
        pose.x = x
        pose.y = y
        pose.theta = theta
        self.pose_pub.publish(pose)
        rospy.loginfo("Sent pose: x=%.1f, y=%.1f, theta=%.2f", x, y, theta)
        
    def run(self):
        while not rospy.is_shutdown():
            try:
                cmd = input("Enter command (f/b/l/r/t/h/s/q): ").strip().lower()
                
                if cmd == 'f':
                    self.send_pose(1.0, 0.0, 0.0)
                elif cmd == 'b':
                    self.send_pose(-1.0, 0.0, 0.0)
                elif cmd == 'l':
                    self.send_pose(0.0, 1.0, 0.0)
                elif cmd == 'r':
                    self.send_pose(0.0, -1.0, 0.0)
                elif cmd == 't':
                    self.send_pose(0.0, 0.0, math.pi/2)
                elif cmd == 'h':
                    self.send_pose(0.0, 0.0, -math.pi/2)
                elif cmd == 's':
                    self.send_pose(0.0, 0.0, 0.0)
                elif cmd == 'q':
                    rospy.loginfo("Quitting...")
                    break
                else:
                    rospy.loginfo("Unknown command: %s", cmd)
                    
            except EOFError:
                break
            except KeyboardInterrupt:
                break

if __name__ == '__main__':
    try:
        publisher = ManualPosePublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass 