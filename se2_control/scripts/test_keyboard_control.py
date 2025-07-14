#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

def test_keyboard_control():
    """Test the keyboard control by sending some commands"""
    rospy.init_node('test_keyboard_control')
    
    # Publishers
    twist_pub = rospy.Publisher('/xray_source/twist_cmd', Twist, queue_size=1)
    enable_pub = rospy.Publisher('/xray_source/enable_motors', Bool, queue_size=1)
    
    # Wait for publishers
    rospy.sleep(1.0)
    
    # Enable motors
    enable_msg = Bool()
    enable_msg.data = True
    enable_pub.publish(enable_msg)
    print("Motors enabled")
    
    # Test commands
    commands = [
        ("Forward", Twist(linear=Twist.Linear(x=0.5))),
        ("Stop", Twist()),
        ("Turn Left", Twist(angular=Twist.Angular(z=1.0))),
        ("Stop", Twist()),
        ("Turn Right", Twist(angular=Twist.Angular(z=-1.0))),
        ("Stop", Twist()),
        ("Backward", Twist(linear=Twist.Linear(x=-0.5))),
        ("Stop", Twist())
    ]
    
    print("Testing keyboard control...")
    for name, cmd in commands:
        print(f"Command: {name}")
        twist_pub.publish(cmd)
        time.sleep(2.0)
    
    # Disable motors
    enable_msg.data = False
    enable_pub.publish(enable_msg)
    print("Test completed")

if __name__ == '__main__':
    try:
        test_keyboard_control()
    except rospy.ROSInterruptException:
        pass 