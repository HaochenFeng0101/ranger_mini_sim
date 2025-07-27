#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from four_wheel_steering_msgs.msg import FourWheelSteering, FourWheelSteeringStamped

class XraySourceSE2Controller:
    def __init__(self):
        rospy.init_node('xray_source_se2_controller')
        
        # Control gains - similar to SO3 control
        self.kp_linear = 1.0
        self.kp_angular = 2.0
        self.max_linear_velocity = 1.5
        self.max_angular_velocity = 2.0
        
        # Two-phase control parameters
        self.position_tolerance = 0.1  # meters
        self.orientation_tolerance = 0.1  # radians
        self.control_phase = "position"  # "position" or "orientation"
        
        # Current pose
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Target pose
        self.target_x = None
        self.target_y = None
        self.target_yaw = None
        self.has_target = False
        
        # Vehicle parameters
        self.wheel_base = 0.46
        self.track = 0.412
        self.steering_track = self.track - 2 * 0.1
        
        # Publishers
        self.twist_pub = rospy.Publisher('/xray_source/four_wheel_steering_controller/cmd_vel', Twist, queue_size=1)
        self.steering_pub = rospy.Publisher('/xray_source/four_wheel_steering_command', FourWheelSteeringStamped, queue_size=1)
        
        # Subscribers
        self.odom_sub = rospy.Subscriber('/xray_source/four_wheel_steering_controller/odom', Odometry, self.odom_callback)
        self.target_sub = rospy.Subscriber('/xray_source/position_cmd', Pose2D, self.target_callback)
        
        rospy.loginfo("X-ray Source SE2 Controller initialized - Simple closed-loop control")
    
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        if self.has_target:
            self.calculate_control()
    
    def target_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.target_yaw = msg.theta
        self.has_target = True
        self.control_phase = "position"  # Reset to position phase for new target
        rospy.loginfo("Received target: x=%.2f, y=%.2f, theta=%.2f", self.target_x, self.target_y, self.target_yaw)
    
    def send_twist(self, linear_x=0.0, angular_z=0.0):
        try:
            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            self.twist_pub.publish(twist)
            rospy.loginfo("SENT TWIST: linear_x=%.2f, angular_z=%.2f", linear_x, angular_z)
        except Exception as e:
            rospy.logerr("Error in send_twist: %s", str(e))
        
    def send_steering_command(self, linear_x=0.0, angular_z=0.0):
        steering_cmd = FourWheelSteeringStamped()
        steering_cmd.header.stamp = rospy.Time.now()
        steering_cmd.header.frame_id = "base_link"
        steering_cmd.data.speed = abs(linear_x)
        steering_cmd.data.acceleration = 0.0
        steering_cmd.data.jerk = 0.0
        
        # Use the same Ackermann-like calculation as simple teleop
        if abs(linear_x) > 0.01 and abs(angular_z) > 0.01:
            R = linear_x / angular_z
            front_steering = math.atan(self.wheel_base / (R - self.track/2.0))
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
        
        try:
            self.steering_pub.publish(steering_cmd)
        except Exception as e:
            rospy.logerr("Error in send_steering_command: %s", str(e))
    
    def calculate_control(self):
        if self.target_x is None or self.target_y is None or self.target_yaw is None:
            rospy.logwarn("Invalid target received, stopping")
            self.has_target = False
            self.send_twist()
            self.send_steering_command()
            return

        # Calculate errors
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        position_error = math.sqrt(dx*dx + dy*dy)
        
        # Calculate heading error (direction to target)
        target_heading = math.atan2(dy, dx)
        heading_error = target_heading - self.current_yaw
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
            
        # Calculate yaw error (final orientation)
        yaw_error = self.target_yaw - self.current_yaw
        rospy.loginfo("Raw yaw error: %.3f (target: %.3f - current: %.3f)", yaw_error, self.target_yaw, self.current_yaw)
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi
        rospy.loginfo("Normalized yaw error: %.3f", yaw_error)

        # Log current and target pose, and errors
        rospy.loginfo("Current: x=%.2f y=%.2f yaw=%.2f | Target: x=%.2f y=%.2f yaw=%.2f | pos_err=%.3f, heading_err=%.3f, yaw_err=%.3f | Phase: %s",
                      self.current_x, self.current_y, self.current_yaw,
                      self.target_x, self.target_y, self.target_yaw,
                      position_error, heading_error, yaw_error, self.control_phase)

        # Two-phase control logic
        if self.control_phase == "position":
            # Phase 1: Move to target position
            if position_error < self.position_tolerance:
                rospy.loginfo("Position reached! Switching to orientation phase.")
                self.control_phase = "orientation"
                return
            
            # Position control: move toward target
            linear_x = self.kp_linear * position_error
            linear_x = max(-self.max_linear_velocity, min(self.max_linear_velocity, linear_x))
            
            # Angular control: turn toward target direction
            angular_z = self.kp_angular * heading_error
            angular_z = max(-self.max_angular_velocity, min(self.max_angular_velocity, angular_z))
            
        elif self.control_phase == "orientation":
            # Phase 2: Rotate to final orientation
            rospy.loginfo("Orientation phase: yaw_error=%.3f, tolerance=%.3f", abs(yaw_error), self.orientation_tolerance)
            if abs(yaw_error) < self.orientation_tolerance:
                rospy.loginfo("Orientation reached! Target achieved.")
                self.has_target = False
                self.send_twist()
                self.send_steering_command()
                return
            
            # Only angular control for orientation
            linear_x = 0.0
            angular_z = self.kp_angular * yaw_error
            angular_z = max(-self.max_angular_velocity, min(self.max_angular_velocity, angular_z))
            rospy.loginfo("Orientation control: angular_z=%.3f (kp=%.1f * error=%.3f)", angular_z, self.kp_angular, yaw_error)

        # Reduce speed as steering increases for stability (but don't go to zero)
        speed_reduction = abs(angular_z) / self.max_angular_velocity
        max_safe_speed = self.max_linear_velocity * max(0.3, 1 - speed_reduction)  # Minimum 30% speed
        linear_x = max(-max_safe_speed, min(max_safe_speed, linear_x))

        # Send commands
        self.send_twist(linear_x=linear_x, angular_z=angular_z)
        self.send_steering_command(linear_x=linear_x, angular_z=angular_z)
        
        rospy.loginfo("Control: linear=%.2f, angular=%.2f", linear_x, angular_z)

def main():
    try:
        controller = XraySourceSE2Controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 