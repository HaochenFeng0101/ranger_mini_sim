#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from se2_control.msg import SE2Command
from tf import transformations

class ThreeDroneEncirclement:
    def __init__(self):
        rospy.init_node('three_drone_encirclement')
        
        # Control parameters
        self.kx = 0.5
        self.ky = 0.5
        self.kyaw = 0.8
        self.max_linear_vel = 0.6
        self.max_angular_vel = 0.4
        
        # Encirclement parameters
        self.desired_radius = 2.0  # Desired encirclement radius
        self.angular_frequency = 0.1  # How fast to move around target (rad/s)
        self.distance_tolerance = 0.2  # Tolerance for distance errors
        
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
        
        # Control flags
        self.encirclement_active = False
        self.target_moving = False
        
        # Publishers for drone commands
        self.drone1_cmd_pub = rospy.Publisher('/drone1/se2_cmd', SE2Command, queue_size=1)
        self.drone2_cmd_pub = rospy.Publisher('/drone2/se2_cmd', SE2Command, queue_size=1)
        self.target_cmd_pub = rospy.Publisher('/target/se2_cmd', SE2Command, queue_size=1)
        
        # Subscribers for drone odometry
        self.drone1_odom_sub = rospy.Subscriber('/drone1/sim/odom', Odometry, self.drone1_odom_callback)
        self.drone2_odom_sub = rospy.Subscriber('/drone2/sim/odom', Odometry, self.drone2_odom_callback)
        self.target_odom_sub = rospy.Subscriber('/target/sim/odom', Odometry, self.target_odom_callback)
        
        # Subscriber for encirclement commands
        self.cmd_sub = rospy.Subscriber('/encirclement_cmd', Pose2D, self.encirclement_cmd_callback)
        
        # Control timer
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_callback)
        
        rospy.loginfo("Three Drone Encirclement Controller initialized")
        rospy.loginfo("Send target position to /encirclement_cmd to start encirclement")
        rospy.loginfo("Example: rostopic pub /encirclement_cmd geometry_msgs/Pose2D \"x: 5.0\\ny: 2.0\\ntheta: 0.0\"")
    
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
    
    def encirclement_cmd_callback(self, msg):
        """Handle encirclement commands"""
        if msg.theta == 1.0:  # Start encirclement
            self.encirclement_active = True
            rospy.loginfo("Starting encirclement")
        elif msg.theta == 0.0:  # Stop encirclement
            self.encirclement_active = False
            rospy.loginfo("Stopping encirclement")
        elif msg.theta == 2.0:  # Move target
            self.target_moving = True
            # Send target to new position
            target_cmd = SE2Command()
            target_cmd.header.stamp = rospy.Time.now()
            target_cmd.linear_x = 0.1 * (msg.x - self.target_x)
            target_cmd.linear_y = 0.1 * (msg.y - self.target_y)
            target_cmd.angular_z = 0.0
            target_cmd.enable_motors = True
            self.target_cmd_pub.publish(target_cmd)
            rospy.loginfo(f"Moving target to ({msg.x}, {msg.y})")
    
    def calculate_desired_position(self, phase_angle):
        """Calculate desired position on circle around target"""
        desired_x = self.target_x + self.desired_radius * math.cos(phase_angle)
        desired_y = self.target_y + self.desired_radius * math.sin(phase_angle)
        return desired_x, desired_y
    
    def control_callback(self, event):
        """Main control loop"""
        if not self.encirclement_active:
            return
        
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
        
        # Log status
        rospy.loginfo(f"Target: ({self.target_x:.2f}, {self.target_y:.2f})")
        rospy.loginfo(f"Drone1: ({self.drone1_x:.2f}, {self.drone1_y:.2f}) dist: {drone1_dist:.2f}")
        rospy.loginfo(f"Drone2: ({self.drone2_x:.2f}, {self.drone2_y:.2f}) dist: {drone2_dist:.2f}")
        rospy.loginfo(f"Inter-drone distance: {inter_drone_dist:.2f}")
        rospy.loginfo(f"Phases: D1={self.drone1_phase:.2f}, D2={self.drone2_phase:.2f}")
        rospy.loginfo("=" * 50)
    
    def calculate_drone_control(self, current_x, current_y, current_yaw, 
                               desired_x, desired_y, current_dist, drone_name):
        """Calculate control commands for a drone"""
        # Calculate position errors
        dx = desired_x - current_x
        dy = desired_y - current_y
        position_error = math.sqrt(dx*dx + dy*dy)
        
        # Calculate distance error from desired radius
        radius_error = current_dist - self.desired_radius
        
        # Calculate heading to desired position
        desired_heading = math.atan2(dy, dx)
        heading_error = desired_heading - current_yaw
        
        # Normalize heading error
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
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
            vyaw = self.kyaw * heading_error * 0.5
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
            speed = self.max_linear_vel * 0.5
            vx_world = speed * tangent_x
            vy_world = speed * tangent_y
            
            # Small radial correction
            radial_correction = -0.1 * radius_error
            radial_x = target_to_drone_x / max(tangent_mag, 0.1)
            radial_y = target_to_drone_y / max(tangent_mag, 0.1)
            
            vx_world += radial_correction * radial_x
            vy_world += radial_correction * radial_y
            
            vyaw = self.kyaw * heading_error * 0.2
        
        # Transform to robot frame
        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
        vx = vx_world * cos_yaw + vy_world * sin_yaw
        vy = -vx_world * sin_yaw + vy_world * cos_yaw
        
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
        
        rospy.logdebug(f"{drone_name}: pos_err={position_error:.3f}, rad_err={radius_error:.3f}, cmd=({vx:.2f},{vy:.2f},{vyaw:.2f})")
        
        return cmd

def main():
    try:
        controller = ThreeDroneEncirclement()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 