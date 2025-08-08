#!/usr/bin/env python3
import rospy
import time
from geometry_msgs.msg import Twist

def test_collision_scenario():
    """Test script to create collision scenarios"""
    rospy.init_node('test_collision')
    
    # Publisher for target velocity commands
    target_pub = rospy.Publisher('/target_manual_twist', Twist, queue_size=1)
    
    rospy.sleep(2.0)  # Wait for system to initialize
    
    print("\n=== COLLISION AVOIDANCE TEST ===")
    print("This script will move the target to create collision scenarios")
    print("Watch the terminal output for collision detection messages")
    print("Press Ctrl+C to stop\n")
    
    try:
        # Test sequence to create collisions
        test_scenarios = [
            {
                "name": "Normal Operation",
                "duration": 5,
                "twist": {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0},
                "description": "Let drones establish formation"
            },
            {
                "name": "Slow Forward Movement", 
                "duration": 8,
                "twist": {"linear_x": 0.2, "linear_y": 0.0, "angular_z": 0.0},
                "description": "Gentle movement, should not cause collision"
            },
            {
                "name": "Sharp Turn Left",
                "duration": 6,
                "twist": {"linear_x": 0.0, "linear_y": 0.5, "angular_z": 0.3},
                "description": "Sharp movement might bring drones close"
            },
            {
                "name": "Quick Direction Change",
                "duration": 4,
                "twist": {"linear_x": 0.5, "linear_y": -0.3, "angular_z": -0.4},
                "description": "Rapid direction change to test collision"
            },
            {
                "name": "Recovery Period",
                "duration": 8,
                "twist": {"linear_x": 0.1, "linear_y": 0.0, "angular_z": 0.0},
                "description": "Slow movement to let system recover"
            },
            {
                "name": "Circular Motion",
                "duration": 10,
                "twist": {"linear_x": 0.3, "linear_y": 0.0, "angular_z": 0.2},
                "description": "Circular motion to test continuous tracking"
            }
        ]
        
        for i, scenario in enumerate(test_scenarios):
            print(f"\n--- Test {i+1}: {scenario['name']} ---")
            print(f"Description: {scenario['description']}")
            print(f"Duration: {scenario['duration']} seconds")
            
            # Create twist message
            twist = Twist()
            twist.linear.x = scenario['twist']['linear_x']
            twist.linear.y = scenario['twist']['linear_y']
            twist.angular.z = scenario['twist']['angular_z']
            
            # Send commands
            rate = rospy.Rate(10)  # 10Hz
            for _ in range(scenario['duration'] * 10):
                if rospy.is_shutdown():
                    return
                target_pub.publish(twist)
                rate.sleep()
            
            print(f"Completed: {scenario['name']}")
        
        # Final stop
        print("\n--- Final Stop ---")
        stop_twist = Twist()
        for _ in range(20):
            target_pub.publish(stop_twist)
            rate.sleep()
            
        print("\n=== COLLISION TEST COMPLETED ===")
        print("Check the terminal output for collision detection results")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        # Send stop command
        stop_twist = Twist()
        for _ in range(10):
            target_pub.publish(stop_twist)
            time.sleep(0.1)

if __name__ == '__main__':
    try:
        test_collision_scenario()
    except rospy.ROSInterruptException:
        pass
