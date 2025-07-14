#!/usr/bin/env python3

import rospy
import sys, select, termios, tty
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

msg = """
X-ray Source Keyboard Control
============================
H: Forward
J: Turn Left
K: Backward  
L: Turn Right

Q: Quit
CTRL-C to quit
"""

class XraySourceTeleop:
    def __init__(self):
        rospy.init_node('xray_source_keyboard_teleop')
        
        # Publishers
        self.twist_pub = rospy.Publisher('/xray_source/twist_cmd', Twist, queue_size=1)
        self.enable_pub = rospy.Publisher('/xray_source/enable_motors', Bool, queue_size=1)
        
        # Control parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 1.0  # rad/s
        
        # Get terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Enable motors
        self.enable_motors()
        
    def enable_motors(self):
        """Enable the motors"""
        enable_msg = Bool()
        enable_msg.data = True
        self.enable_pub.publish(enable_msg)
        rospy.loginfo("Motors enabled")
        
    def disable_motors(self):
        """Disable the motors"""
        enable_msg = Bool()
        enable_msg.data = False
        self.enable_pub.publish(enable_msg)
        rospy.loginfo("Motors disabled")
        
    def get_key(self):
        """Get a single keypress from the terminal"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
        
    def send_twist(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        """Send a twist command"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self.twist_pub.publish(twist)
        
    def run(self):
        """Main control loop"""
        try:
            print(msg)
            print("Current speeds: linear=%.1f m/s, angular=%.1f rad/s" % (self.linear_speed, self.angular_speed))
            print("Press H/J/K/L to move, Q to quit")
            
            while not rospy.is_shutdown():
                key = self.get_key()
                
                if key == 'h' or key == 'H':
                    # Forward
                    print("Moving FORWARD")
                    self.send_twist(linear_x=self.linear_speed)
                    
                elif key == 'k' or key == 'K':
                    # Backward
                    print("Moving BACKWARD")
                    self.send_twist(linear_x=-self.linear_speed)
                    
                elif key == 'j' or key == 'J':
                    # Turn left
                    print("Turning LEFT")
                    self.send_twist(angular_z=self.angular_speed)
                    
                elif key == 'l' or key == 'L':
                    # Turn right
                    print("Turning RIGHT")
                    self.send_twist(angular_z=-self.angular_speed)
                    
                elif key == 'q' or key == 'Q':
                    print("Quitting...")
                    break
                    
                elif key == '\x03':  # Ctrl-C
                    break
                    
                else:
                    # Stop
                    self.send_twist()
                    
        except Exception as e:
            print(e)
        finally:
            # Stop and disable motors
            self.send_twist()
            self.disable_motors()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == '__main__':
    try:
        teleop = XraySourceTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass 