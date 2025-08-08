#!/usr/bin/env python3
import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from se2_control.msg import SE2Command
from tf import transformations

class AutoEncirclement:
    def __init__(self):
        rospy.init_node('auto_encirclement')
        
        # Control parameters
        self.kx = 0.4
        self.ky = 0.4
        self.kyaw = 0.6
        self.max_linear_vel = 0.8
        self.max_angular_vel = 0.4
        
        # Encirclement parameters
        self.desired_radius = 3.0  # Desired encirclement radius
        self.angular_frequency = 0.15  # How fast to move around target (rad/s)
        self.distance_tolerance = 0.3  # Tolerance for distance errors
        
        # Current states
        self.drone1_x = 0.0
        self.drone1_y = 0.0
        self.drone1_yaw = 0.0
        
        self.drone2_x = 3.0
        self.drone2_y = 0.0
        self.drone2_yaw = 0.0
        
        self.target_x = 5.0
        self.target_y = 2.0
        self.target_yaw = 0.0
        
        # Phase angles for anti-synchronization (opposite positions)
        self.drone1_phase = 0.0
        self.drone2_phase = math.pi  # 180 degrees apart
        
        # Publishers for drone commands
        self.drone1_cmd_pub = rospy.Publisher('/drone1/se2_cmd', SE2Command, queue_size=1)
        self.drone2_cmd_pub = rospy.Publisher('/drone2/se2_cmd', SE2Command, queue_size=1)
        
        # Subscribers for drone odometry
        self.drone1_odom_sub = rospy.Subscriber('/drone1/sim/odom', Odometry, self.drone1_odom_callback)
        self.drone2_odom_sub = rospy.Subscriber('/drone2/sim/odom', Odometry, self.drone2_odom_callback)
        self.target_odom_sub = rospy.Subscriber('/target/sim/odom', Odometry, self.target_odom_callback)
        
        # Control timer - automatically start encirclement
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_callback)
        
        rospy.loginfo("Auto Encirclement Controller initialized")
        rospy.loginfo("Drones will automatically encircle the target")
        rospy.loginfo("Control the target using manual commands")
    
    def drone1_odom_callback(self, msg):
        """Update drone 1 position"""
        self.drone1_x = msg.pose.pose.position.x
        self.drone1_y = msg.pose.pose.position.y
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                     msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.drone1_yaw = euler[2]
    
    def drone2_odom_callback(self, msg):
        """Update drone 2 position"""
        self.drone2_x = msg.pose.pose.position.x
        self.drone2_y = msg.pose.pose.position.y
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                     msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.drone2_yaw = euler[2]
    
    def target_odom_callback(self, msg):
        """Update target position"""
        self.target_x = msg.pose.pose.position.x
        self.target_y = msg.pose.pose.position.y
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                     msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.target_yaw = euler[2]
    
    def calculate_desired_position(self, phase_angle):
        """Calculate desired position on circle around target"""
        desired_x = self.target_x + self.desired_radius * math.cos(phase_angle)
        desired_y = self.target_y + self.desired_radius * math.sin(phase_angle)
        return desired_x, desired_y
    
    def control_callback(self, event):
        """Main control loop - automatically active"""
        
        # Update phase angles for continuous movement
        self.drone1_phase += self.angular_frequency * 0.1  # dt = 0.1s
        self.drone2_phase += self.angular_frequency * 0.1
        
        # Normalize phases
        self.drone1_phase = self.drone1_phase % (2 * math.pi)
        self.drone2_phase = self.drone2_phase % (2 * math.pi)
        
        # Calculate desired positions for anti-synchronization
        drone1_desired_x, drone1_desired_y = self.calculate_desired_position(self.drone1_phase)
        drone2_desired_x, drone2_desired_y = self.calculate_desired_position(self.drone2_phase)
        
        # Calculate current distances to target
        drone1_dist = math.sqrt((self.drone1_x - self.target_x)**2 + (self.drone1_y - self.target_y)**2)
        drone2_dist = math.sqrt((self.drone2_x - self.target_x)**2 + (self.drone2_y - self.target_y)**2)
        
        # Calculate distance between drones
        inter_drone_dist = math.sqrt((self.drone1_x - self.drone2_x)**2 + (self.drone1_y - self.drone2_y)**2)
        
        # Control drone 1
        drone1_cmd = self.calculate_drone_control(
            self.drone1_x, self.drone1_y, self.drone1_yaw,
            drone1_desired_x, drone1_desired_y,
            drone1_dist, "Drone1"
        )
        self.drone1_cmd_pub.publish(drone1_cmd)
        
        # Control drone 2
        drone2_cmd = self.calculate_drone_control(
            self.drone2_x, self.drone2_y, self.drone2_yaw,
            drone2_desired_x, drone2_desired_y,
            drone2_dist, "Drone2"
        )
        self.drone2_cmd_pub.publish(drone2_cmd)
        
        # Log status every 2 seconds
        if int(rospy.Time.now().to_sec()) % 2 == 0:
            rospy.loginfo(f"Target: ({self.target_x:.1f}, {self.target_y:.1f}) | "
                         f"D1: ({self.drone1_x:.1f}, {self.drone1_y:.1f}) dist: {drone1_dist:.1f} | "
                         f"D2: ({self.drone2_x:.1f}, {self.drone2_y:.1f}) dist: {drone2_dist:.1f}")
    
    def calculate_drone_control(self, current_x, current_y, current_yaw, 
                               desired_x, desired_y, current_dist, drone_name):
        """Calculate control commands for a drone"""
        # Calculate position errors
        dx = desired_x - current_x
        dy = desired_y - current_y
        position_error = math.sqrt(dx*dx + dy*dy)
        
        # Calculate distance error from desired radius
        radius_error = current_dist - self.desired_radius
        
        # Control strategy: 
        # 1. If far from desired radius, move radially
        # 2. If close to desired radius, move tangentially for encirclement
        
        if abs(radius_error) > self.distance_tolerance:
            # Move radially to correct distance
            if position_error > 0.1:
                # Move toward desired position
                vx_world = self.kx * dx
                vy_world = self.ky * dy
            else:
                vx_world = 0.0
                vy_world = 0.0
        else:
            # Move tangentially for encirclement
            # Calculate tangent direction (perpendicular to radius)
            target_to_drone_x = current_x - self.target_x
            target_to_drone_y = current_y - self.target_y
            
            # Tangent vector (90 degrees to radius)
            tangent_x = -target_to_drone_y
            tangent_y = target_to_drone_x
            
            # Normalize tangent
            tangent_mag = math.sqrt(tangent_x**2 + tangent_y**2)
            if tangent_mag > 0.1:
                tangent_x /= tangent_mag
                tangent_y /= tangent_mag
            
            # Move in tangent direction
            speed = self.max_linear_vel * 0.6
            vx_world = speed * tangent_x
            vy_world = speed * tangent_y
            
            # Small radial correction
            radial_correction = -0.2 * radius_error
            if tangent_mag > 0.1:
                radial_x = target_to_drone_x / tangent_mag
                radial_y = target_to_drone_y / tangent_mag
                
                vx_world += radial_correction * radial_x
                vy_world += radial_correction * radial_y
        
        # Transform to robot frame
        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
        vx = vx_world * cos_yaw + vy_world * sin_yaw
        vy = -vx_world * sin_yaw + vy_world * cos_yaw
        
        # Simple yaw control - point towards movement direction
        if abs(vx_world) > 0.1 or abs(vy_world) > 0.1:
            desired_yaw = math.atan2(vy_world, vx_world)
            yaw_error = desired_yaw - current_yaw
            while yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            while yaw_error < -math.pi:
                yaw_error += 2 * math.pi
            vyaw = self.kyaw * yaw_error * 0.3
        else:
            vyaw = 0.0
        
        # Limit velocities
        vx = max(-self.max_linear_vel, min(self.max_linear_vel, vx))
        vy = max(-self.max_linear_vel, min(self.max_linear_vel, vy))
        vyaw = max(-self.max_angular_vel, min(self.max_angular_vel, vyaw))
        
        # Create command
        cmd = SE2Command()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "base_link"
        cmd.linear_x = vx
        cmd.linear_y = vy
        cmd.angular_z = vyaw
        cmd.kx = [self.kx, self.ky]
        cmd.kyaw = self.kyaw
        cmd.enable_motors = True
        cmd.use_external_yaw = False
        cmd.current_yaw = current_yaw
        
        return cmd

def main():
    try:
        controller = AutoEncirclement()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 