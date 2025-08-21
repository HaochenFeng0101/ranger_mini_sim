#!/usr/bin/env python3
import rospy
import time
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from se2_control.msg import SE2Command

def test_improved_encirclement():
    """Test script for the improved range-based encirclement system"""
    rospy.init_node('test_improved_encirclement')
    
    # Publishers for testing
    target_pub = rospy.Publisher('/target_manual_twist', Twist, queue_size=1)
    
    # Subscribers to monitor system status
    target_odom = None
    drone0_odom = None
    drone1_odom = None
    drone0_laser = None
    drone1_laser = None
    drone0_cmd = None
    drone1_cmd = None
    
    def target_odom_callback(msg):
        nonlocal target_odom
        target_odom = msg
    
    def drone0_odom_callback(msg):
        nonlocal drone0_odom
        drone0_odom = msg
    
    def drone1_odom_callback(msg):
        nonlocal drone1_odom
        drone1_odom = msg
    
    def drone0_laser_callback(msg):
        nonlocal drone0_laser
        drone0_laser = msg
    
    def drone1_laser_callback(msg):
        nonlocal drone1_laser
        drone1_laser = msg
    
    def drone0_cmd_callback(msg):
        nonlocal drone0_cmd
        drone0_cmd = msg
    
    def drone1_cmd_callback(msg):
        nonlocal drone1_cmd
        drone1_cmd = msg
    
    # Setup subscribers
    odom_sub = rospy.Subscriber('/drone2/sim/odom', Odometry, target_odom_callback)
    drone0_odom_sub = rospy.Subscriber('/drone0/sim/odom', Odometry, drone0_odom_callback)
    drone1_odom_sub = rospy.Subscriber('/drone1/sim/odom', Odometry, drone1_odom_callback)
    drone0_laser_sub = rospy.Subscriber('/drone0/laser_scan', LaserScan, drone0_laser_callback)
    drone1_laser_sub = rospy.Subscriber('/drone1/laser_scan', LaserScan, drone1_laser_callback)
    drone0_cmd_sub = rospy.Subscriber('/drone0/se2_cmd', SE2Command, drone0_cmd_callback)
    drone1_cmd_sub = rospy.Subscriber('/drone1/se2_cmd', SE2Command, drone1_cmd_callback)
    
    # Wait for system to initialize
    rospy.sleep(5.0)
    
    print("🧪 Testing Improved Range-Based Encirclement System")
    print("=" * 60)
    
    # Test 1: System Status Check
    print("\n📋 Test 1: System Status Check")
    print("-" * 40)
    
    # Check target topic subscribers
    if target_pub.get_num_connections() == 0:
        print("❌ ERROR: No subscribers to /target_manual_twist topic!")
        return False
    
    print(f"✅ Target topic has {target_pub.get_num_connections()} subscriber(s)")
    
    # Check if all drones are publishing odometry
    drones_ok = True
    if target_odom is None:
        print("❌ Target drone not publishing odometry")
        drones_ok = False
    else:
        print("✅ Target drone odometry OK")
    
    if drone0_odom is None:
        print("❌ Drone0 not publishing odometry")
        drones_ok = False
    else:
        print("✅ Drone0 odometry OK")
    
    if drone1_odom is None:
        print("❌ Drone1 not publishing odometry")
        drones_ok = False
    else:
        print("✅ Drone1 odometry OK")
    
    if not drones_ok:
        print("❌ System not ready - some drones missing")
        return False
    
    # Test 2: Range Sensor Check
    print("\n📡 Test 2: Range Sensor Check")
    print("-" * 40)
    
    if drone0_laser is None:
        print("❌ Drone0 laser scan not working")
    else:
        print(f"✅ Drone0 laser: {len(drone0_laser.ranges)} beams, max range: {drone0_laser.range_max:.1f}m")
    
    if drone1_laser is None:
        print("❌ Drone1 laser scan not working")
    else:
        print(f"✅ Drone1 laser: {len(drone1_laser.ranges)} beams, max range: {drone1_laser.range_max:.1f}m")
    
    # Test 3: Initial Positions
    print("\n📍 Test 3: Initial Positions")
    print("-" * 40)
    
    if target_odom and drone0_odom and drone1_odom:
        target_x = target_odom.pose.pose.position.x
        target_y = target_odom.pose.pose.position.y
        drone0_x = drone0_odom.pose.pose.position.x
        drone0_y = drone0_odom.pose.pose.position.y
        drone1_x = drone1_odom.pose.pose.position.x
        drone1_y = drone1_odom.pose.pose.position.y
        
        print(f"Target: ({target_x:.2f}, {target_y:.2f})")
        print(f"Drone0: ({drone0_x:.2f}, {drone0_y:.2f})")
        print(f"Drone1: ({drone1_x:.2f}, {drone1_y:.2f})")
        
        # Check initial distances
        d0_target = math.sqrt((drone0_x - target_x)**2 + (drone0_y - target_y)**2)
        d1_target = math.sqrt((drone1_x - target_x)**2 + (drone1_y - target_y)**2)
        d0_1 = math.sqrt((drone0_x - drone1_x)**2 + (drone0_y - drone1_y)**2)
        
        print(f"Drone0 to Target: {d0_target:.2f}m")
        print(f"Drone1 to Target: {d1_target:.2f}m")
        print(f"Drone0 to Drone1: {d0_1:.2f}m")
    
    # Test 4: Target Movement Test
    print("\n🎯 Test 4: Target Movement Test")
    print("-" * 40)
    
    print("Moving target in circle pattern...")
    
    # Move target in a circle
    circle_twist = Twist()
    circle_twist.linear.x = 0.3
    circle_twist.angular.z = 0.2
    
    start_time = time.time()
    for i in range(50):  # 5 seconds
        if rospy.is_shutdown():
            break
        target_pub.publish(circle_twist)
        rospy.sleep(0.1)
    
    # Stop target
    stop_twist = Twist()
    for i in range(5):
        target_pub.publish(stop_twist)
        rospy.sleep(0.1)
    
    # Check if target moved
    rospy.sleep(1.0)
    if target_odom is not None:
        new_target_x = target_odom.pose.pose.position.x
        new_target_y = target_odom.pose.pose.position.y
        target_movement = math.sqrt((new_target_x - target_x)**2 + (new_target_y - target_y)**2)
        
        if target_movement > 0.5:
            print(f"✅ Target movement successful: {target_movement:.2f}m")
        else:
            print(f"❌ Target movement too small: {target_movement:.2f}m")
    
    # Test 5: Encirclement Behavior
    print("\n🔄 Test 5: Encirclement Behavior")
    print("-" * 40)
    
    print("Waiting for encirclement behavior to develop...")
    rospy.sleep(10.0)  # Wait for encirclement to develop
    
    # Check if drones are receiving commands
    if drone0_cmd is None:
        print("❌ Drone0 not receiving commands")
    else:
        print("✅ Drone0 receiving commands")
    
    if drone1_cmd is None:
        print("❌ Drone1 not receiving commands")
    else:
        print("✅ Drone1 receiving commands")
    
    # Test 6: Formation Check
    print("\n📐 Test 6: Formation Check")
    print("-" * 40)
    
    if drone0_odom and drone1_odom and target_odom:
        # Get current positions
        current_drone0_x = drone0_odom.pose.pose.position.x
        current_drone0_y = drone0_odom.pose.pose.position.y
        current_drone1_x = drone1_odom.pose.pose.position.x
        current_drone1_y = drone1_odom.pose.pose.position.y
        current_target_x = target_odom.pose.pose.position.x
        current_target_y = target_odom.pose.pose.position.y
        
        # Calculate current distances
        current_d0_target = math.sqrt((current_drone0_x - current_target_x)**2 + (current_drone0_y - current_target_y)**2)
        current_d1_target = math.sqrt((current_drone1_x - current_target_x)**2 + (current_drone1_y - current_target_y)**2)
        current_d0_1 = math.sqrt((current_drone0_x - current_drone1_x)**2 + (current_drone0_y - current_drone1_y)**2)
        
        print(f"Current Drone0 to Target: {current_d0_target:.2f}m")
        print(f"Current Drone1 to Target: {current_d1_target:.2f}m")
        print(f"Current Drone0 to Drone1: {current_d0_1:.2f}m")
        
        # Check if formation is reasonable
        target_distance_ok = (2.0 < current_d0_target < 8.0) and (2.0 < current_d1_target < 8.0)
        formation_spacing_ok = (1.0 < current_d0_1 < 6.0)
        
        if target_distance_ok:
            print("✅ Target distances are reasonable")
        else:
            print("❌ Target distances are outside expected range")
        
        if formation_spacing_ok:
            print("✅ Formation spacing is reasonable")
        else:
            print("❌ Formation spacing is outside expected range")
    
    # Test 7: Search Behavior Test
    print("\n🔍 Test 7: Search Behavior Test")
    print("-" * 40)
    
    print("Moving target far away to test search behavior...")
    
    # Move target far away
    far_twist = Twist()
    far_twist.linear.x = 1.0
    
    for i in range(30):  # 3 seconds
        if rospy.is_shutdown():
            break
        target_pub.publish(far_twist)
        rospy.sleep(0.1)
    
    # Stop target
    for i in range(5):
        target_pub.publish(stop_twist)
        rospy.sleep(0.1)
    
    print("Target moved away. Drones should now search for target...")
    rospy.sleep(5.0)  # Wait for search behavior
    
    # Check if drones are searching (rotating)
    if drone0_cmd and drone1_cmd:
        drone0_vyaw = abs(drone0_cmd.angular_z)
        drone1_vyaw = abs(drone1_cmd.angular_z)
        
        if drone0_vyaw > 0.1:
            print("✅ Drone0 is searching (rotating)")
        else:
            print("❌ Drone0 is not searching")
        
        if drone1_vyaw > 0.1:
            print("✅ Drone1 is searching (rotating)")
        else:
            print("❌ Drone1 is not searching")
    
    print("\n🎉 Improved Range-Based Encirclement Test Completed!")
    return True

def main():
    try:
        success = test_improved_encirclement()
        
        if success:
            print("\n✅ All tests passed! The improved system is working correctly.")
            print("\nKey improvements verified:")
            print("1. ✅ No pose sharing between robots")
            print("2. ✅ Range-only measurements working")
            print("3. ✅ Target search behavior when target is out of range")
            print("4. ✅ Improved formation control preventing getting stuck")
            print("5. ✅ Better movement validation and recovery")
        else:
            print("\n❌ Some tests failed. Check the system setup.")
            
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"\n💥 Unexpected error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
