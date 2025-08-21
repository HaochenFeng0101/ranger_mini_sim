#!/usr/bin/env python3
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import tf2_ros
import geometry_msgs.msg

class RangeSensorSimulator:
    """
    Simulates range sensors (LiDAR) for multi-robot coordination.
    Provides range measurements between robots without requiring absolute poses.
    """
    def __init__(self):
        rospy.init_node('range_sensor_simulator')
        
        # Get namespace for this robot
        self.namespace = rospy.get_namespace().strip('/')
        if not self.namespace:
            self.namespace = "drone"
        
        # Sensor parameters
        self.rate = 10.0  # Hz - typical LiDAR update rate
        self.dt = 1.0 / self.rate
        
        # LiDAR parameters
        self.max_range = 50.0  # Maximum detection range (meters) - increased from 20m
        self.min_range = 0.1   # Minimum detection range (meters)
        self.angle_increment = 0.05  # Angular resolution (radians) - improved from 0.1
        self.num_beams = int(2 * math.pi / self.angle_increment)  # 360 degree coverage
        
        # Robot positions (will be updated from odometry)
        self.robot_positions = {}
        self.robot_yaws = {}
        
        # Publishers and subscribers
        self.laser_pub = rospy.Publisher('laser_scan', LaserScan, queue_size=1)
        self.relative_poses_pub = rospy.Publisher('relative_poses', PoseArray, queue_size=1)
        
        # Subscribe to all robot odometry topics
        self.odom_subs = {}
        self.setup_odometry_subscribers()
        
        # TF listener for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Control timer
        self.sensor_timer = rospy.Timer(rospy.Duration(self.dt), self.sensor_callback)
        
        rospy.loginfo(f"Range Sensor Simulator ({self.namespace}) initialized")
        rospy.loginfo(f"LiDAR: {self.num_beams} beams, {math.degrees(self.angle_increment):.1f}° resolution")
        rospy.loginfo(f"Range: {self.min_range:.1f}m to {self.max_range:.1f}m")
        
    def setup_odometry_subscribers(self):
        """Setup subscribers for all robot odometry topics"""
        # Subscribe to all drone odometry topics
        for i in range(3):  # drone0, drone1, drone2
            drone_name = f"drone{i}"
            topic = f"/{drone_name}/sim/odom"
            self.odom_subs[drone_name] = rospy.Subscriber(
                topic, 
                Odometry,  # Use proper Odometry message type
                self.odom_callback,
                drone_name
            )
            rospy.loginfo(f"Subscribed to {topic}")
    
    def odom_callback(self, msg, drone_name):
        """Handle odometry messages from any drone"""
        try:
            # Extract position and orientation
            pos = msg.pose.pose.position
            orient = msg.pose.pose.orientation
            
            # Store position
            self.robot_positions[drone_name] = np.array([pos.x, pos.y])
            
            # Convert quaternion to yaw
            yaw = math.atan2(2.0 * (orient.w * orient.z + orient.x * orient.y),
                            1.0 - 2.0 * (orient.y * orient.y + orient.z * orient.z))
            self.robot_yaws[drone_name] = yaw
                
        except Exception as e:
            rospy.logwarn(f"Error processing odometry from {drone_name}: {e}")
    
    def calculate_relative_measurements(self):
        """Calculate relative measurements between robots with improved detection"""
        if len(self.robot_positions) < 2:
            return None, None
        
        current_robot = self.namespace
        if current_robot not in self.robot_positions:
            return None, None
        
        current_pos = self.robot_positions[current_robot]
        current_yaw = self.robot_yaws.get(current_robot, 0.0)
        
        # Initialize laser scan with maximum range
        laser_ranges = [self.max_range] * self.num_beams
        relative_poses = []
        
        # Process each other robot
        for robot_name, robot_pos in self.robot_positions.items():
            if robot_name == current_robot:
                continue
            
            # Calculate relative position
            relative_pos = robot_pos - current_pos
            distance = np.linalg.norm(relative_pos)
            
            # Skip if out of range
            if distance > self.max_range or distance < self.min_range:
                continue
            
            # Calculate relative angle (in robot frame)
            angle_world = math.atan2(relative_pos[1], relative_pos[0])
            relative_angle = angle_world - current_yaw
            
            # Normalize angle to [-pi, pi]
            while relative_angle > math.pi:
                relative_angle -= 2 * math.pi
            while relative_angle < -math.pi:
                relative_angle += 2 * math.pi
            
            # Convert to beam index
            beam_index = int((relative_angle + math.pi) / self.angle_increment)
            beam_index = max(0, min(self.num_beams - 1, beam_index))
            
            # Update laser scan (closest object at this angle)
            if distance < laser_ranges[beam_index]:
                laser_ranges[beam_index] = distance
            
            # Add to relative poses with improved data
            relative_pose = Pose()
            relative_pose.position.x = distance
            relative_pose.position.y = relative_angle
            relative_pose.position.z = 0.0
            
            # Store robot name in orientation (hack to pass string data)
            relative_pose.orientation.x = float(ord(robot_name[-1]))  # Last character of drone name
            
            # Add confidence and timestamp information
            relative_pose.orientation.y = 1.0  # Confidence level
            relative_pose.orientation.z = rospy.Time.now().to_sec()  # Timestamp
            
            relative_poses.append(relative_pose)
            
            # Debug logging for target detection
            if robot_name == 'drone2':  # Target drone
                rospy.logdebug(f"{current_robot}: Target detected at {distance:.2f}m, angle {math.degrees(relative_angle):.1f}°")
            else:
                rospy.logdebug(f"{current_robot}: Agent {robot_name} detected at {distance:.2f}m, angle {math.degrees(relative_angle):.1f}°")
        
        # REMOVED: Median filter that was interfering with measurements
        # laser_ranges = self.filter_laser_ranges(laser_ranges)
        
        return laser_ranges, relative_poses
    
    def filter_laser_ranges(self, ranges):
        """Apply noise filtering and smoothing to laser ranges"""
        filtered_ranges = ranges.copy()
        
        # Simple median filter to remove noise
        window_size = 3
        for i in range(len(ranges)):
            start_idx = max(0, i - window_size // 2)
            end_idx = min(len(ranges), i + window_size // 2 + 1)
            window = ranges[start_idx:end_idx]
            
            # Calculate median of window
            sorted_window = sorted(window)
            median_idx = len(sorted_window) // 2
            if len(sorted_window) % 2 == 0:
                median = (sorted_window[median_idx - 1] + sorted_window[median_idx]) / 2
            else:
                median = sorted_window[median_idx]
            
            filtered_ranges[i] = median
        
        return filtered_ranges
    
    def publish_laser_scan(self, ranges):
        """Publish simulated laser scan data"""
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = f"{self.namespace}/base_link"
        
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = self.angle_increment
        scan.time_increment = self.dt
        scan.scan_time = self.dt
        
        scan.range_min = self.min_range
        scan.range_max = self.max_range
        
        scan.ranges = ranges
        
        self.laser_pub.publish(scan)
    
    def publish_relative_poses(self, relative_poses):
        """Publish relative pose information"""
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = f"{self.namespace}/base_link"
        pose_array.poses = relative_poses
        
        self.relative_poses_pub.publish(pose_array)
    
    def sensor_callback(self, event):
        """Main sensor simulation loop"""
        # Calculate relative measurements
        laser_ranges, relative_poses = self.calculate_relative_measurements()
        
        if laser_ranges is not None:
            # Publish laser scan
            self.publish_laser_scan(laser_ranges)
            
            # Publish relative poses
            if relative_poses:
                self.publish_relative_poses(relative_poses)
            
            # Debug info
            if hasattr(self, 'debug_counter'):
                self.debug_counter += 1
            else:
                self.debug_counter = 0
                
            if self.debug_counter % 10 == 0:  # Every second
                rospy.loginfo(f"{self.namespace}: Detected {len(relative_poses)} robots")
                for i, pose in enumerate(relative_poses):
                    robot_id = int(pose.orientation.x)  # Extract robot ID
                    distance = pose.position.x
                    angle = math.degrees(pose.position.y)
                    rospy.loginfo(f"  Robot {robot_id}: {distance:.2f}m at {angle:.1f}°")

def main():
    try:
        simulator = RangeSensorSimulator()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
