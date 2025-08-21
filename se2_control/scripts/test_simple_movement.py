#!/usr/bin/env python3
import rospy
import time
from se2_control.msg import SE2Command

def test_simple_movement():
    """Simple test to verify drones can move"""
    rospy.init_node('test_simple_movement')
    
    # Publishers for drone commands
    drone0_cmd_pub = rospy.Publisher('/drone0/se2_cmd', SE2Command, queue_size=1)
    drone1_cmd_pub = rospy.Publisher('/drone1/se2_cmd', SE2Command, queue_size=1)
    
    # Wait for connection
    rospy.sleep(2.0)
    
    print("🧪 Simple Movement Test")
    print("=" * 30)
    
    # Check connections
    if drone0_cmd_pub.get_num_connections() == 0:
        print("❌ No subscribers to /drone0/se2_cmd")
        return False
    else:
        print(f"✅ /drone0/se2_cmd has {drone0_cmd_pub.get_num_connections()} subscriber(s)")
    
    if drone1_cmd_pub.get_num_connections() == 0:
        print("❌ No subscribers to /drone1/se2_cmd")
        return False
    else:
        print(f"✅ /drone1/se2_cmd has {drone1_cmd_pub.get_num_connections()} subscriber(s)")
    
    # Test 1: Forward movement
    print("\n🧪 Test 1: Forward Movement")
    print("-" * 40)
    
    cmd = SE2Command()
    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = "base_link"
    cmd.linear_x = 0.3
    cmd.linear_y = 0.0
    cmd.angular_z = 0.0
    cmd.kx = [1.0, 1.0]
    cmd.kyaw = 1.0
    cmd.enable_motors = True
    cmd.use_external_yaw = False
    cmd.current_yaw = 0.0
    
    print("Sending forward command to both drones...")
    for i in range(30):  # 3 seconds
        drone0_cmd_pub.publish(cmd)
        drone1_cmd_pub.publish(cmd)
        rospy.sleep(0.1)
    
    # Test 2: Stop
    print("\n🧪 Test 2: Stop Command")
    print("-" * 40)
    
    cmd.linear_x = 0.0
    cmd.linear_y = 0.0
    cmd.angular_z = 0.0
    
    print("Sending stop command to both drones...")
    for i in range(10):  # 1 second
        drone0_cmd_pub.publish(cmd)
        drone1_cmd_pub.publish(cmd)
        rospy.sleep(0.1)
    
    print("\n✅ Simple movement test completed!")
    print("Check RViz to see if drones moved forward and then stopped")
    return True

def main():
    try:
        success = test_simple_movement()
        if success:
            print("\n🎉 Basic movement test passed!")
            print("If drones moved in RViz, the system is working correctly")
            print("If drones didn't move, there's a deeper issue to investigate")
        else:
            print("\n❌ Basic movement test failed!")
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
