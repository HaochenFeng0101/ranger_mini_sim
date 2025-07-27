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
Bed Trolley Simple Keyboard Control (WASD)
=========================================
W: Forward (hold to increase speed)
S: Backward (hold to increase speed)
A: Turn Left (hold to increase speed)
D: Turn Right (hold to increase speed)

Q: Quit
CTRL-C to quit

Combine W/S with A/D for smooth arc turns.
Speed increases while holding keys, stops when released.
"""

class BedTrolleySimpleTeleop:
    def __init__(self):
        rospy.init_node('bed_trolley_simple_teleop')
        
        # Publishers - directly to the four wheel steering controller
        self.twist_pub = rospy.Publisher('/bed_trolley/four_wheel_steering_controller/cmd_vel', Twist, queue_size=1)
        self.steering_pub = rospy.Publisher('/bed_trolley/four_wheel_steering_command', FourWheelSteeringStamped, queue_size=1)
        
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
        self.keys_pressed = set()
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
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
        
    def send_twist(self, linear_x=0.0, angular_z=0.0):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.twist_pub.publish(twist)
        
    def send_steering_command(self, linear_x=0.0, angular_z=0.0):
        steering_cmd = FourWheelSteeringStamped()
        steering_cmd.header.stamp = rospy.Time.now()
        steering_cmd.header.frame_id = "base_link"
        steering_cmd.data.speed = abs(linear_x)
        steering_cmd.data.acceleration = 0.0
        steering_cmd.data.jerk = 0.0
        # Vehicle parameters
        wheel_base = 0.46
        track = 0.412
        steering_track = track - 2 * 0.1
        # Ackermann-like calculation
        if abs(linear_x) > 0.01 and abs(angular_z) > 0.01:
            R = linear_x / angular_z
            front_steering = math.atan(wheel_base / (R - track/2.0))
            if angular_z < 0:
                front_steering = -front_steering
            rear_steering = -front_steering * 0.3
        elif abs(linear_x) > 0.01:
            front_steering = 0.0
            rear_steering = 0.0
        elif abs(angular_z) > 0.01:
            front_steering = math.copysign(math.pi/2, angular_z)
            rear_steering = -front_steering
        else:
            front_steering = 0.0
            rear_steering = 0.0
        max_steering_angle = math.pi/3
        front_steering = max(-max_steering_angle, min(max_steering_angle, front_steering))
        rear_steering = max(-max_steering_angle, min(max_steering_angle, rear_steering))
        steering_cmd.data.front_steering_angle = front_steering
        steering_cmd.data.rear_steering_angle = rear_steering
        steering_cmd.data.front_steering_angle_velocity = 0.0
        steering_cmd.data.rear_steering_angle_velocity = 0.0
        self.steering_pub.publish(steering_cmd)
        
    def compute_movement(self):
        linear_x = 0.0
        angular_z = 0.0
        if 'w' in self.keys_pressed or 'W' in self.keys_pressed:
            linear_x = self.current_linear_speed
        elif 's' in self.keys_pressed or 'S' in self.keys_pressed:
            linear_x = -self.current_linear_speed
        if 'a' in self.keys_pressed or 'A' in self.keys_pressed:
            angular_z = self.current_angular_speed
        elif 'd' in self.keys_pressed or 'D' in self.keys_pressed:
            angular_z = -self.current_angular_speed
        return linear_x, angular_z
        
    def speed_control_loop(self):
        rate = rospy.Rate(20)
        while self.running and not rospy.is_shutdown():
            current_time = time.time()
            dt = current_time - self.last_update_time
            self.last_update_time = current_time
            if self.keys_pressed:
                if self.current_linear_speed < self.speed_threshold:
                    self.current_linear_speed = self.base_linear_speed
                else:
                    self.current_linear_speed += self.speed_increment * dt
                    self.current_linear_speed = min(self.current_linear_speed, self.max_linear_speed)
                if self.current_angular_speed < self.speed_threshold:
                    self.current_angular_speed = self.base_angular_speed
                else:
                    self.current_angular_speed += self.speed_increment * dt
                    self.current_angular_speed = min(self.current_angular_speed, self.max_angular_speed)
            linear_x, angular_z = self.compute_movement()
            self.send_twist(linear_x=linear_x, angular_z=angular_z)
            self.send_steering_command(linear_x=linear_x, angular_z=angular_z)
            rate.sleep()
        
    def run(self):
        try:
            print(msg)
            print("Base speeds: linear=%.1f m/s, angular=%.1f rad/s" % (self.base_linear_speed, self.base_angular_speed))
            print("Max speeds: linear=%.1f m/s, angular=%.1f rad/s" % (self.max_linear_speed, self.max_angular_speed))
            print("Press W/A/S/D to move, Q to quit")
            while not rospy.is_shutdown():
                key = self.get_key()
                if key in ['w', 'W', 'a', 'A', 's', 'S', 'd', 'D']:
                    self.keys_pressed.add(key)
                    self.key_press_time = time.time()
                    self.current_linear_speed = 0.0
                    self.current_angular_speed = 0.0
                    desc = []
                    if 'w' in self.keys_pressed or 'W' in self.keys_pressed:
                        desc.append("FORWARD")
                    if 's' in self.keys_pressed or 'S' in self.keys_pressed:
                        desc.append("BACKWARD")
                    if 'a' in self.keys_pressed or 'A' in self.keys_pressed:
                        desc.append("LEFT")
                    if 'd' in self.keys_pressed or 'D' in self.keys_pressed:
                        desc.append("RIGHT")
                    print("MOVING: " + " + ".join(desc) + " - hold to increase speed")
                elif key == 'q' or key == 'Q':
                    print("Quitting...")
                    break
                elif key == '\x03':
                    break
                else:
                    if self.keys_pressed:
                        print("STOPPED")
                        self.keys_pressed.clear()
                        self.current_linear_speed = 0.0
                        self.current_angular_speed = 0.0
                        self.send_twist()
                        self.send_steering_command()
        except Exception as e:
            print(e)
        finally:
            self.running = False
            self.send_twist()
            self.send_steering_command()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == '__main__':
    try:
        teleop = BedTrolleySimpleTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass 