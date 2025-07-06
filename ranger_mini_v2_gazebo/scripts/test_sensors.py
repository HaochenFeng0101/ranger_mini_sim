#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge

class SensorTester:
    def __init__(self):
        rospy.init_node('sensor_tester', anonymous=True)
        
        # Initialize CV bridge for image processing
        self.bridge = CvBridge()
        
        # Subscribers
        self.pointcloud_sub = rospy.Subscriber('/mid/points', PointCloud2, self.pointcloud_callback)
        self.image_sub = rospy.Subscriber('/front/image_raw', Image, self.image_callback)
        
        # Flags to track if we've received data
        self.received_pointcloud = False
        self.received_image = False
        
        # Counters
        self.pointcloud_count = 0
        self.image_count = 0
        
        rospy.loginfo("Sensor tester initialized. Waiting for sensor data...")
        
    def pointcloud_callback(self, msg):
        if not self.received_pointcloud:
            rospy.loginfo("✓ PointCloud2 data received from /mid/points")
            self.received_pointcloud = True
        
        self.pointcloud_count += 1
        if self.pointcloud_count % 100 == 0:  # Log every 100 messages
            rospy.loginfo(f"PointCloud2 messages received: {self.pointcloud_count}")
    
    def image_callback(self, msg):
        if not self.received_image:
            rospy.loginfo("✓ Image data received from /front/image_raw")
            rospy.loginfo(f"Image size: {msg.width}x{msg.height}")
            self.received_image = True
        
        self.image_count += 1
        if self.image_count % 100 == 0:  # Log every 100 messages
            rospy.loginfo(f"Image messages received: {self.image_count}")
    
    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        
        while not rospy.is_shutdown():
            if self.received_pointcloud and self.received_image:
                rospy.loginfo("✓ All sensors are working correctly!")
                rospy.loginfo(f"PointCloud2 messages: {self.pointcloud_count}")
                rospy.loginfo(f"Image messages: {self.image_count}")
            elif self.received_pointcloud:
                rospy.loginfo("✓ PointCloud2 working, waiting for camera...")
            elif self.received_image:
                rospy.loginfo("✓ Camera working, waiting for PointCloud2...")
            else:
                rospy.loginfo("Waiting for sensor data...")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        tester = SensorTester()
        tester.run()
    except rospy.ROSInterruptException:
        pass 