#!/usr/bin/env python3
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray

def test_range_sensor():
    """Test script to verify range sensor measurements"""
    rospy.init_node('test_range_sensor')
    
    # Subscribers to monitor range sensor data
    drone0_laser = None
    drone1_laser = None
    drone2_laser = None
    drone0_poses = None
    drone1_poses = None
    drone2_poses = None
    
    def drone0_laser_callback(msg):
        nonlocal drone0_laser
        drone0_laser = msg
    
    def drone1_laser_callback(msg):
        nonlocal drone1_laser
        drone1_laser = msg
    
    def drone2_laser_callback(msg):
        nonlocal drone2_laser
        drone2_laser = msg
    
    def drone0_poses_callback(msg):
        nonlocal drone0_poses
        drone0_poses = msg
    
    def drone1_poses_callback(msg):
        nonlocal drone1_poses
        drone1_poses = msg
    
    def drone2_poses_callback(msg):
        nonlocal drone2_poses
        drone2_poses = msg
    
    # Setup subscribers
    drone0_laser_sub = rospy.Subscriber('/drone0/laser_scan', LaserScan, drone0_laser_callback)
    drone1_laser_sub = rospy.Subscriber('/drone1/laser_scan', LaserScan, drone1_laser_callback)
    drone2_laser_sub = rospy.Subscriber('/drone2/laser_scan', LaserScan, drone2_laser_callback)
    drone0_poses_sub = rospy.Subscriber('/drone0/relative_poses', PoseArray, drone0_poses_callback)
    drone1_poses_sub = rospy.Subscriber('/drone1/relative_poses', PoseArray, drone1_poses_callback)
    drone2_poses_sub = rospy.Subscriber('/drone2/relative_poses', PoseArray, drone2_poses_callback)
    
    # Wait for system to initialize
    rospy.sleep(3.0)
    
    print("🔍 Testing Range Sensor Measurements")
    print("=" * 40)
    
    # Test 1: Check if laser data is being received
    print("\n📡 Test 1: Laser Scan Data")
    print("-" * 40)
    
    if drone0_laser is None:
        print("❌ Drone0 laser scan not working")
    else:
        print(f"✅ Drone0 laser: {len(drone0_laser.ranges)} beams, max range: {drone0_laser.range_max:.1f}m")
        # Find minimum range (closest object)
        min_range = min(drone0_laser.ranges)
        if min_range < drone0_laser.range_max:
            print(f"   Closest object detected at: {min_range:.2f}m")
        else:
            print("   No objects detected in range")
    
    if drone1_laser is None:
        print("❌ Drone1 laser scan not working")
    else:
        print(f"✅ Drone1 laser: {len(drone1_laser.ranges)} beams, max range: {drone1_laser.range_max:.1f}m")
        min_range = min(drone1_laser.ranges)
        if min_range < drone1_laser.range_max:
            print(f"   Closest object detected at: {min_range:.2f}m")
        else:
            print("   No objects detected in range")
    
    if drone2_laser is None:
        print("❌ Drone2 laser scan not working")
    else:
        print(f"✅ Drone2 laser: {len(drone2_laser.ranges)} beams, max range: {drone2_laser.range_max:.1f}m")
        min_range = min(drone2_laser.ranges)
        if min_range < drone2_laser.range_max:
            print(f"   Closest object detected at: {min_range:.2f}m")
        else:
            print("   No objects detected in range")
    
    # Test 2: Check relative poses
    print("\n📍 Test 2: Relative Poses")
    print("-" * 40)
    
    if drone0_poses is None:
        print("❌ Drone0 relative poses not working")
    else:
        print(f"✅ Drone0 relative poses: {len(drone0_poses.poses)} objects detected")
        for i, pose in enumerate(drone0_poses.poses):
            robot_id = int(pose.orientation.x)
            distance = pose.position.x
            angle = pose.position.y
            print(f"   Object {robot_id}: {distance:.2f}m at {angle:.2f} rad")
    
    if drone1_poses is None:
        print("❌ Drone1 relative poses not working")
    else:
        print(f"✅ Drone1 relative poses: {len(drone1_poses.poses)} objects detected")
        for i, pose in enumerate(drone1_poses.poses):
            robot_id = int(pose.orientation.x)
            distance = pose.position.x
            angle = pose.position.y
            print(f"   Object {robot_id}: {distance:.2f}m at {angle:.2f} rad")
    
    if drone2_poses is None:
        print("❌ Drone2 relative poses not working")
    else:
        print(f"✅ Drone2 relative poses: {len(drone2_poses.poses)} objects detected")
        for i, pose in enumerate(drone2_poses.poses):
            robot_id = int(pose.orientation.x)
            distance = pose.position.x
            angle = pose.position.y
            print(f"   Object {robot_id}: {distance:.2f}m at {angle:.2f} rad")
    
    # Test 3: Monitor for changes
    print("\n🔄 Test 3: Monitoring for Changes")
    print("-" * 40)
    print("Monitoring range sensor data for 10 seconds...")
    
    start_time = time.time()
    while time.time() - start_time < 10.0:
        if drone0_laser and drone1_laser:
            min_range0 = min(drone0_laser.ranges)
            min_range1 = min(drone1_laser.ranges)
            print(f"Drone0 min range: {min_range0:.2f}m, Drone1 min range: {min_range1:.2f}m")
        
        rospy.sleep(1.0)
    
    print("\n✅ Range sensor test completed!")
    print("Check the output above to see if measurements are working correctly")

def main():
    try:
        test_range_sensor()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
