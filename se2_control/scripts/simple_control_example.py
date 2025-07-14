#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Pose2D, Twist

def main():
    rospy.init_node('simple_control_example', anonymous=True)
    
    # Create publishers
    position_pub = rospy.Publisher('/xray_source/position_cmd', Pose2D, queue_size=1)
    twist_pub = rospy.Publisher('/xray_source/twist_cmd', Twist, queue_size=1)
    
    # Wait for publishers to be ready
    rospy.sleep(1.0)
    
    print("SE2 Controller Simple Example")
    print("============================")
    print("1. Send position command")
    print("2. Send velocity command")
    print("3. Exit")
    
    while not rospy.is_shutdown():
        try:
            choice = input("\nEnter your choice (1-3): ")
            
            if choice == '1':
                # Position control example
                x = float(input("Enter target x position: "))
                y = float(input("Enter target y position: "))
                theta = float(input("Enter target yaw angle (radians): "))
                
                cmd = Pose2D()
                cmd.x = x
                cmd.y = y
                cmd.theta = theta
                position_pub.publish(cmd)
                
                print(f"Position command sent: x={x}, y={y}, theta={theta}")
                
            elif choice == '2':
                # Velocity control example
                vx = float(input("Enter forward velocity (m/s): "))
                vy = float(input("Enter lateral velocity (m/s): "))
                omega = float(input("Enter yaw rate (rad/s): "))
                
                cmd = Twist()
                cmd.linear.x = vx
                cmd.linear.y = vy
                cmd.angular.z = omega
                twist_pub.publish(cmd)
                
                print(f"Velocity command sent: vx={vx}, vy={vy}, omega={omega}")
                
            elif choice == '3':
                print("Exiting...")
                break
                
            else:
                print("Invalid choice. Please enter 1, 2, or 3.")
                
        except ValueError:
            print("Invalid input. Please enter numeric values.")
        except KeyboardInterrupt:
            print("\nExiting...")
            break

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 