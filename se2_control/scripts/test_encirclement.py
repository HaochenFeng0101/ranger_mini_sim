#!/usr/bin/env python3
import rospy
import time
from geometry_msgs.msg import Pose2D

def test_encirclement():
    """Test script for three-drone encirclement"""
    rospy.init_node('test_encirclement')
    
    # Publisher for encirclement commands
    cmd_pub = rospy.Publisher('/encirclement_cmd', Pose2D, queue_size=1)
    
    # Wait for publisher to be ready
    rospy.sleep(2.0)
    
    rospy.loginfo("Starting three-drone encirclement test sequence...")
    
    # Test 1: Start encirclement
    rospy.loginfo("Test 1: Starting encirclement")
    start_cmd = Pose2D()
    start_cmd.x = 0.0
    start_cmd.y = 0.0
    start_cmd.theta = 1.0  # Start encirclement
    cmd_pub.publish(start_cmd)
    
    rospy.loginfo("Encirclement started - drones should move to encircle the target")
    rospy.loginfo("Wait 20 seconds to observe encirclement behavior...")
    rospy.sleep(20.0)
    
    # Test 2: Move target while encircling
    rospy.loginfo("Test 2: Moving target to new position")
    move_target_cmd = Pose2D()
    move_target_cmd.x = 8.0
    move_target_cmd.y = 3.0
    move_target_cmd.theta = 2.0  # Move target
    cmd_pub.publish(move_target_cmd)
    
    rospy.loginfo("Target moved - drones should adapt and maintain encirclement")
    rospy.loginfo("Wait 15 seconds to observe adaptation...")
    rospy.sleep(15.0)
    
    # Test 3: Move target again
    rospy.loginfo("Test 3: Moving target to another position")
    move_target_cmd2 = Pose2D()
    move_target_cmd2.x = 2.0
    move_target_cmd2.y = 5.0
    move_target_cmd2.theta = 2.0  # Move target
    cmd_pub.publish(move_target_cmd2)
    
    rospy.loginfo("Target moved again - observing encirclement adaptation")
    rospy.loginfo("Wait 15 seconds...")
    rospy.sleep(15.0)
    
    # Test 4: Stop encirclement
    rospy.loginfo("Test 4: Stopping encirclement")
    stop_cmd = Pose2D()
    stop_cmd.x = 0.0
    stop_cmd.y = 0.0
    stop_cmd.theta = 0.0  # Stop encirclement
    cmd_pub.publish(stop_cmd)
    
    rospy.loginfo("Encirclement stopped - drones should stop moving")
    rospy.loginfo("Test sequence completed!")

def manual_control():
    """Manual control interface for encirclement"""
    rospy.init_node('manual_encirclement_control')
    
    cmd_pub = rospy.Publisher('/encirclement_cmd', Pose2D, queue_size=1)
    rospy.sleep(1.0)
    
    print("\n=== Three-Drone Encirclement Manual Control ===")
    print("Commands:")
    print("  's' - Start encirclement")
    print("  'q' - Stop encirclement")
    print("  'x y' - Move target to position (x, y)")
    print("  'exit' - Exit program")
    print("=" * 50)
    
    while not rospy.is_shutdown():
        try:
            user_input = input("Enter command: ").strip().lower()
            
            if user_input == 'exit':
                break
            elif user_input == 's':
                cmd = Pose2D()
                cmd.theta = 1.0  # Start
                cmd_pub.publish(cmd)
                print("Encirclement started")
            elif user_input == 'q':
                cmd = Pose2D()
                cmd.theta = 0.0  # Stop
                cmd_pub.publish(cmd)
                print("Encirclement stopped")
            elif len(user_input.split()) == 2:
                try:
                    x, y = map(float, user_input.split())
                    cmd = Pose2D()
                    cmd.x = x
                    cmd.y = y
                    cmd.theta = 2.0  # Move target
                    cmd_pub.publish(cmd)
                    print(f"Moving target to ({x}, {y})")
                except ValueError:
                    print("Invalid coordinates. Use format: x y")
            else:
                print("Unknown command. Use 's', 'q', 'x y', or 'exit'")
                
        except KeyboardInterrupt:
            break
        except EOFError:
            break
    
    print("Manual control ended")

if __name__ == '__main__':
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == 'manual':
        try:
            manual_control()
        except rospy.ROSInterruptException:
            pass
    else:
        try:
            test_encirclement()
        except rospy.ROSInterruptException:
            pass 