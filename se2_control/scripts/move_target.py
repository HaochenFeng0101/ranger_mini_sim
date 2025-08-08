#!/usr/bin/env python3
import rospy
import sys
from geometry_msgs.msg import Twist

def move_target():
    """Simple script to move the target drone with keyboard-like commands"""
    rospy.init_node('move_target')
    
    # Publisher for target velocity commands
    target_pub = rospy.Publisher('/target_manual_twist', Twist, queue_size=1)
    
    rospy.sleep(1.0)  # Wait for connection
    
    if len(sys.argv) < 2:
        print("\nUsage:")
        print("  rosrun se2_control move_target.py <command>")
        print("\nCommands:")
        print("  forward    - Move forward")
        print("  backward   - Move backward") 
        print("  left       - Move left")
        print("  right      - Move right")
        print("  stop       - Stop movement")
        print("  circle     - Move in a circle")
        print("  custom vx vy vyaw - Custom velocities")
        print("\nExample:")
        print("  rosrun se2_control move_target.py forward")
        print("  rosrun se2_control move_target.py custom 0.5 0.2 0.1")
        return
    
    command = sys.argv[1].lower()
    
    twist = Twist()
    
    if command == "forward":
        twist.linear.x = 0.5
        print("Moving target forward")
    elif command == "backward":
        twist.linear.x = -0.5
        print("Moving target backward")
    elif command == "left":
        twist.linear.y = 0.5
        print("Moving target left")
    elif command == "right":
        twist.linear.y = -0.5
        print("Moving target right")
    elif command == "stop":
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        print("Stopping target")
    elif command == "circle":
        twist.linear.x = 0.3
        twist.angular.z = 0.2
        print("Moving target in circle")
    elif command == "custom":
        if len(sys.argv) >= 5:
            twist.linear.x = float(sys.argv[2])
            twist.linear.y = float(sys.argv[3])
            twist.angular.z = float(sys.argv[4])
            print(f"Custom movement: vx={twist.linear.x}, vy={twist.linear.y}, vyaw={twist.angular.z}")
        else:
            print("Custom command needs 3 values: vx vy vyaw")
            return
    else:
        print(f"Unknown command: {command}")
        return
    
    # Send command for 5 seconds
    rate = rospy.Rate(10)  # 10Hz
    for i in range(50):  # 5 seconds
        if rospy.is_shutdown():
            break
        target_pub.publish(twist)
        rate.sleep()
    
    # Stop the target
    stop_twist = Twist()
    for i in range(10):
        target_pub.publish(stop_twist)
        rate.sleep()
    
    print("Command completed!")

if __name__ == '__main__':
    try:
        move_target()
    except rospy.ROSInterruptException:
        pass
