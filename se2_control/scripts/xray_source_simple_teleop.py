#!/usr/bin/env python3

import rospy
import sys, select, termios, tty
import threading
import time
import math
from geometry_msgs.msg import Twist
from four_wheel_steering_msgs.msg import FourWheelSteeringStamped
from std_msgs.msg import Bool

msg = """
X-ray Source Keyboard Control
============================
H: Forward (hold to increase speed)
J: Turn Left (hold to increase speed)
K: Backward (hold to increase speed)
L: Turn Right (hold to increase speed)

Q: Quit
CTRL-C to quit

Speed increases while holding keys, stops when released
"""

class XraySourceSimpleTeleop:
    def __init__(self):
        rospy.init_node('xray_source_simple_teleop')
        
        # Publishers - directly to the four wheel steering controller
        self.twist_pub = rospy.Publisher('/xray_source/four_wheel_steering_controller/cmd_vel', Twist, queue_size=1)
        self.steering_pub = rospy.Publisher('/xray_source/four_wheel_steering_command', FourWheelSteeringStamped, queue_size=1)
        
        # Control parameters
        self.base_linear_speed = 0.3  # m/s
        self.base_angular_speed = 0.8  # rad/s
        self.max_linear_speed = 1.5    # m/s
        self.max_angular_speed = 2.0   # rad/s
        self.speed_increment = 0.1     # speed increase per second while holding
        self.speed_threshold = 0.1     # minimum speed to start moving
        
        # Current speeds
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        
        # Key state tracking
        self.key_pressed = None
        self.key_press_time = 0.0
        self.last_update_time = time.time()
        
        # Get terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Start speed control thread
        self.running = True
        self.speed_thread = threading.Thread(target=self.speed_control_loop)
        self.speed_thread.daemon = True
        self.speed_thread.start()
        
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
        
    def send_steering_command(self, linear_x=0.0, angular_z=0.0):
        """Send a four wheel steering command with proper steering angle calculations"""
        steering_cmd = FourWheelSteeringStamped()
        steering_cmd.header.stamp = rospy.Time.now()
        steering_cmd.header.frame_id = "base_link"
        steering_cmd.data.speed = abs(linear_x)
        steering_cmd.data.acceleration = 0.0
        steering_cmd.data.jerk = 0.0
        
        # Vehicle parameters (should match the controller configuration)
        wheel_base = 0.46  # Distance between front and rear axles
        track = 0.412      # Distance between left and right wheels
        steering_track = track - 2 * 0.1  # Adjust for steering offset
        
        # Calculate proper steering angles based on four-wheel steering geometry
        if abs(linear_x) > 0.01 and abs(angular_z) > 0.01:
            # When both linear and angular velocity are present
            front_steering = math.atan(angular_z * wheel_base / (2.0 * linear_x - angular_z * steering_track))
            rear_steering = -front_steering  # Counter-steering for stability
        elif abs(linear_x) > 0.01:
            # Straight line motion
            front_steering = 0.0
            rear_steering = 0.0
        elif abs(angular_z) > 0.01:
            # Pure rotation (spot turn)
            front_steering = math.copysign(math.pi/2, angular_z)  # 90 degrees
            rear_steering = -front_steering  # Counter-steering
        else:
            # No motion
            front_steering = 0.0
            rear_steering = 0.0
        
        # Limit steering angles to reasonable values
        max_steering_angle = math.pi/3  # 60 degrees
        front_steering = max(-max_steering_angle, min(max_steering_angle, front_steering))
        rear_steering = max(-max_steering_angle, min(max_steering_angle, rear_steering))
        
        steering_cmd.data.front_steering_angle = front_steering
        steering_cmd.data.rear_steering_angle = rear_steering
        steering_cmd.data.front_steering_angle_velocity = 0.0
        steering_cmd.data.rear_steering_angle_velocity = 0.0
        
        self.steering_pub.publish(steering_cmd)
        
    def speed_control_loop(self):
        """Thread to handle speed control and continuous movement"""
        rate = rospy.Rate(20)  # 20 Hz
        
        while self.running and not rospy.is_shutdown():
            current_time = time.time()
            dt = current_time - self.last_update_time
            self.last_update_time = current_time
            
            # Handle speed changes based on key press duration
            if self.key_pressed:
                key_hold_time = current_time - self.key_press_time
                
                if self.key_pressed in ['h', 'H', 'k', 'K']:
                    # Linear movement
                    if self.current_linear_speed < self.speed_threshold:
                        self.current_linear_speed = self.base_linear_speed
                    else:
                        self.current_linear_speed += self.speed_increment * dt
                        self.current_linear_speed = min(self.current_linear_speed, self.max_linear_speed)
                        
                elif self.key_pressed in ['j', 'J', 'l', 'L']:
                    # Angular movement
                    if self.current_angular_speed < self.speed_threshold:
                        self.current_angular_speed = self.base_angular_speed
                    else:
                        self.current_angular_speed += self.speed_increment * dt
                        self.current_angular_speed = min(self.current_angular_speed, self.max_angular_speed)
            
            # Send movement command
            linear_x = 0.0
            angular_z = 0.0
            
            if self.key_pressed == 'h' or self.key_pressed == 'H':
                linear_x = self.current_linear_speed
            elif self.key_pressed == 'k' or self.key_pressed == 'K':
                linear_x = -self.current_linear_speed
            elif self.key_pressed == 'j' or self.key_pressed == 'J':
                angular_z = self.current_angular_speed
            elif self.key_pressed == 'l' or self.key_pressed == 'L':
                angular_z = -self.current_angular_speed
            
            # Send both twist and steering commands
            self.send_twist(linear_x=linear_x, angular_z=angular_z)
            self.send_steering_command(linear_x=linear_x, angular_z=angular_z)
            
            rate.sleep()
        
    def run(self):
        """Main control loop"""
        try:
            print(msg)
            print("Base speeds: linear=%.1f m/s, angular=%.1f rad/s" % (self.base_linear_speed, self.base_angular_speed))
            print("Max speeds: linear=%.1f m/s, angular=%.1f rad/s" % (self.max_linear_speed, self.max_angular_speed))
            print("Press H/J/K/L to move, Q to quit")
            
            while not rospy.is_shutdown():
                key = self.get_key()
                
                if key in ['h', 'H', 'j', 'J', 'k', 'K', 'l', 'L']:
                    if self.key_pressed != key:
                        # New key pressed
                        self.key_pressed = key
                        self.key_press_time = time.time()
                        self.current_linear_speed = 0.0
                        self.current_angular_speed = 0.0
                        
                        if key in ['h', 'H']:
                            print("FORWARD - hold to increase speed")
                        elif key in ['k', 'K']:
                            print("BACKWARD - hold to increase speed")
                        elif key in ['j', 'J']:
                            print("LEFT - hold to increase speed")
                        elif key in ['l', 'L']:
                            print("RIGHT - hold to increase speed")
                            
                elif key == 'q' or key == 'Q':
                    print("Quitting...")
                    break
                    
                elif key == '\x03':  # Ctrl-C
                    break
                    
                else:
                    # No key pressed, stop movement
                    if self.key_pressed:
                        print("STOPPED")
                        self.key_pressed = None
                        self.current_linear_speed = 0.0
                        self.current_angular_speed = 0.0
                        self.send_twist()
                        self.send_steering_command()
                    
        except Exception as e:
            print(e)
        finally:
            # Stop and disable motors
            self.running = False
            self.send_twist()
            self.send_steering_command()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == '__main__':
    try:
        teleop = XraySourceSimpleTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass 