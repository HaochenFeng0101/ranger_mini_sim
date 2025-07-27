#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class OdomToPath:
    def __init__(self):
        rospy.init_node('odom_to_path')
        
        # Subscribe to odometry
        self.odom_sub = rospy.Subscriber('/xray_source/odom', Odometry, self.odom_callback)
        
        # Publish path
        self.path_pub = rospy.Publisher('/xray_source/path', Path, queue_size=1)
        
        # Store path points
        self.path_points = []
        self.last_publish_time = rospy.Time.now()
        self.publish_interval = rospy.Duration(0.1)  # Publish every 100ms
        
        rospy.loginfo("Odom to Path converter initialized")
    
    def odom_callback(self, msg):
        """Callback for odometry data"""
        current_time = rospy.Time.now()
        
        # Add current position to path
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.path_points.append(pose_stamped)
        
        # Limit path length to last 1000 points
        if len(self.path_points) > 1000:
            self.path_points = self.path_points[-1000:]
        
        # Publish path periodically
        if current_time - self.last_publish_time > self.publish_interval:
            self.publish_path()
            self.last_publish_time = current_time
    
    def publish_path(self):
        """Publish the path as a Path message"""
        if len(self.path_points) < 2:
            return
        
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        path_msg.poses = self.path_points
        
        self.path_pub.publish(path_msg)

def main():
    try:
        converter = OdomToPath()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 