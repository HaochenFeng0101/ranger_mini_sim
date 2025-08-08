#!/usr/bin/env python3
import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from se2_control.msg import SE2Command
from tf import transformations

class SimpleEncirclement:
    """
    Simple encirclement controller - two drones maintain opposite positions around target.
    No complex algorithms, just basic geometric control.
    """
    def __init__(self):
        rospy.init_node('simple_encirclement')
        
        # Multi-radius parameters
        self.agent1_radius = 1.0   # Red drone: inner circle (1m)
        self.agent2_radius = 5.0   # Green drone: outer circle (5m)
        self.kp_radial = 0.5      # Gain for radial position control
        self.kp_tangential = 0.3   # Gain for tangential movement
        self.max_vel = 0.8        # Maximum velocity
        self.angular_speed = 0.15  # Angular speed for circular motion (rad/s)
        
        # Collision detection parameters
        self.collision_distance = 1.5  # Minimum safe distance between drones (meters)
        self.agent1_frozen = False     # Red drone freeze status
        self.agent2_frozen = False     # Green drone freeze status
        self.freeze_timeout = 3.0      # Time to stay frozen (seconds)
        self.agent1_freeze_time = 0.0  # When agent1 was frozen
        self.agent2_freeze_time = 0.0  # When agent2 was frozen
        
        # Agent states (drone0 and drone1 are the encircling agents)
        self.agent1_pos = np.array([0.0, 0.0])
        self.agent1_yaw = 0.0
        
        self.agent2_pos = np.array([3.0, 0.0]) 
        self.agent2_yaw = 0.0
        
        # Target state (drone2 is the target)
        self.target_pos = np.array([5.0, 2.0])
        self.target_yaw = 0.0
        
        # Publishers for drone commands
        self.agent1_cmd_pub = rospy.Publisher('/drone0/se2_cmd', SE2Command, queue_size=1)
        self.agent2_cmd_pub = rospy.Publisher('/drone1/se2_cmd', SE2Command, queue_size=1)
        
        # Subscribers for drone odometry
        self.agent1_odom_sub = rospy.Subscriber('/drone0/sim/odom', Odometry, self.agent1_odom_callback)
        self.agent2_odom_sub = rospy.Subscriber('/drone1/sim/odom', Odometry, self.agent2_odom_callback)
        self.target_odom_sub = rospy.Subscriber('/drone2/sim/odom', Odometry, self.target_odom_callback)
        
        # Control timer
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_callback)  # 10Hz
        
        rospy.loginfo("Multi-Radius Encirclement Controller with Collision Detection initialized")
        rospy.loginfo(f"Red drone: {self.agent1_radius}m radius (inner circle)")
        rospy.loginfo(f"Green drone: {self.agent2_radius}m radius (outer circle)")
        rospy.loginfo(f"Collision detection: {self.collision_distance}m minimum distance")
        rospy.loginfo("Both agents will move in synchronized circular motion with collision avoidance")
        
    def agent1_odom_callback(self, msg):
        """Update agent 1 (drone0) state"""
        self.agent1_pos[0] = msg.pose.pose.position.x
        self.agent1_pos[1] = msg.pose.pose.position.y
        
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                     msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.agent1_yaw = euler[2]
        
    def agent2_odom_callback(self, msg):
        """Update agent 2 (drone1) state"""
        self.agent2_pos[0] = msg.pose.pose.position.x
        self.agent2_pos[1] = msg.pose.pose.position.y
        
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                     msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.agent2_yaw = euler[2]
        
    def target_odom_callback(self, msg):
        """Update target (drone2) state"""
        self.target_pos[0] = msg.pose.pose.position.x
        self.target_pos[1] = msg.pose.pose.position.y
        
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                     msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.target_yaw = euler[2]
        
    def calculate_desired_positions(self):
        """Calculate desired positions for agents with different radii and circular motion"""
        
        # Get current time for circular motion
        current_time = rospy.Time.now().to_sec()
        
        # Calculate desired angle for both agents (same angular position but different radii)
        base_angle = self.angular_speed * current_time
        
        # Agent1 (Red): Inner circle at 1m radius
        angle1 = base_angle
        desired_agent1 = self.target_pos + self.agent1_radius * np.array([
            math.cos(angle1),
            math.sin(angle1)
        ])
        
        # Agent2 (Green): Outer circle at 5m radius, same angle
        # Note: They move in sync but at different distances
        angle2 = angle1  # Same angle for synchronized movement
        desired_agent2 = self.target_pos + self.agent2_radius * np.array([
            math.cos(angle2), 
            math.sin(angle2)
        ])
        
        return desired_agent1, desired_agent2
        
    def check_collision_and_update_freeze_status(self):
        """Check for potential collision and update freeze status"""
        current_time = rospy.Time.now().to_sec()
        
        # Calculate distance between agents
        distance_between_agents = np.linalg.norm(self.agent1_pos - self.agent2_pos)
        
        # Check if collision is imminent
        collision_detected = distance_between_agents < self.collision_distance
        
        if collision_detected and not (self.agent1_frozen or self.agent2_frozen):
            # Collision detected and no one is frozen yet
            # Decide which drone to freeze (freeze the outer one - green drone)
            self.agent2_frozen = True
            self.agent2_freeze_time = current_time
            rospy.logwarn(f"COLLISION RISK! Distance: {distance_between_agents:.2f}m - Freezing GREEN drone")
            
        # Check if frozen agents should be unfrozen
        if self.agent1_frozen and (current_time - self.agent1_freeze_time) > self.freeze_timeout:
            self.agent1_frozen = False
            rospy.loginfo("RED drone unfrozen - resuming motion")
            
        if self.agent2_frozen and (current_time - self.agent2_freeze_time) > self.freeze_timeout:
            self.agent2_frozen = False
            rospy.loginfo("GREEN drone unfrozen - resuming motion")
            
        return distance_between_agents, collision_detected
        
    def calculate_control_command(self, current_pos, desired_pos, current_yaw, agent_id):
        """Calculate control command for an agent with circular motion"""
        
        # Get the appropriate radius for this agent
        agent_radius = self.agent1_radius if agent_id == 1 else self.agent2_radius
        
        # Position error (radial correction)
        pos_error = desired_pos - current_pos
        radial_vel = self.kp_radial * pos_error
        
        # Add tangential velocity for circular motion
        # Vector from target to current position
        target_to_agent = current_pos - self.target_pos
        distance_to_target = np.linalg.norm(target_to_agent)
        
        if distance_to_target > 0.1:
            # Tangent direction (perpendicular to radius, for circular motion)
            tangent_direction = np.array([-target_to_agent[1], target_to_agent[0]])
            tangent_direction = tangent_direction / np.linalg.norm(tangent_direction)
            
            # Tangential speed for circular motion (based on agent's radius)
            tangential_speed = self.angular_speed * agent_radius
            tangential_vel = tangential_speed * tangent_direction
        else:
            tangential_vel = np.array([0.0, 0.0])
        
        # Combine radial correction and tangential motion
        desired_vel_world = radial_vel + tangential_vel
        
        # Limit velocity
        vel_magnitude = np.linalg.norm(desired_vel_world)
        if vel_magnitude > self.max_vel:
            desired_vel_world = desired_vel_world * (self.max_vel / vel_magnitude)
        
        # Transform to robot frame
        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
        
        vel_robot_x = desired_vel_world[0] * cos_yaw + desired_vel_world[1] * sin_yaw
        vel_robot_y = -desired_vel_world[0] * sin_yaw + desired_vel_world[1] * cos_yaw
        
        # Yaw control - point toward movement direction
        if vel_magnitude > 0.1:
            desired_yaw = math.atan2(desired_vel_world[1], desired_vel_world[0])
            yaw_error = desired_yaw - current_yaw
            
            # Normalize yaw error
            while yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            while yaw_error < -math.pi:
                yaw_error += 2 * math.pi
                
            vel_yaw = 0.5 * yaw_error  # Yaw control gain
        else:
            vel_yaw = 0.0
        
        return vel_robot_x, vel_robot_y, vel_yaw
        
    def create_se2_command(self, vx, vy, vyaw):
        """Create SE2 command message"""
        cmd = SE2Command()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "base_link"
        cmd.linear_x = vx
        cmd.linear_y = vy
        cmd.angular_z = vyaw
        cmd.kx = [0.8, 0.8]
        cmd.kyaw = 0.5
        cmd.enable_motors = True
        cmd.use_external_yaw = False
        cmd.current_yaw = 0.0
        
        return cmd
        
    def control_callback(self, event):
        """Main control loop with collision detection"""
        
        # Check for collisions and update freeze status
        distance_between_agents, collision_detected = self.check_collision_and_update_freeze_status()
        
        # Calculate desired positions (same angle, different radii)
        desired_pos1, desired_pos2 = self.calculate_desired_positions()
        
        # Calculate control commands (but may be overridden if frozen)
        vx1, vy1, vyaw1 = self.calculate_control_command(self.agent1_pos, desired_pos1, self.agent1_yaw, 1)
        vx2, vy2, vyaw2 = self.calculate_control_command(self.agent2_pos, desired_pos2, self.agent2_yaw, 2)
        
        # Apply freeze logic - override commands if agent is frozen
        if self.agent1_frozen:
            vx1, vy1, vyaw1 = 0.0, 0.0, 0.0  # Freeze red drone
            
        if self.agent2_frozen:
            vx2, vy2, vyaw2 = 0.0, 0.0, 0.0  # Freeze green drone
        
        # Create and publish commands
        cmd1 = self.create_se2_command(vx1, vy1, vyaw1)
        cmd2 = self.create_se2_command(vx2, vy2, vyaw2)
        
        self.agent1_cmd_pub.publish(cmd1)
        self.agent2_cmd_pub.publish(cmd2)
        
        # Debug info (every 2 seconds)
        if hasattr(self, 'debug_counter'):
            self.debug_counter += 1
        else:
            self.debug_counter = 0
            
        if self.debug_counter % 20 == 0:
            dist1 = np.linalg.norm(self.agent1_pos - self.target_pos)
            dist2 = np.linalg.norm(self.agent2_pos - self.target_pos)
            current_time = rospy.Time.now().to_sec()
            angle = (self.angular_speed * current_time) % (2 * math.pi)
            
            # Collision status
            freeze_status = ""
            if self.agent1_frozen:
                freeze_status += "RED_FROZEN "
            if self.agent2_frozen:
                freeze_status += "GREEN_FROZEN "
            if collision_detected:
                freeze_status += "COLLISION_RISK "
            if not freeze_status:
                freeze_status = "NORMAL"
            
            rospy.loginfo(f"Target at ({self.target_pos[0]:.1f}, {self.target_pos[1]:.1f})")
            rospy.loginfo(f"Agent distances: red={dist1:.2f}m (target:{self.agent1_radius}m), green={dist2:.2f}m (target:{self.agent2_radius}m)")
            rospy.loginfo(f"Inter-agent distance: {distance_between_agents:.2f}m (safe:{self.collision_distance}m)")
            rospy.loginfo(f"Status: {freeze_status}")
            rospy.loginfo(f"Circular motion: angle={angle:.2f}rad ({math.degrees(angle):.1f}°)")
            rospy.loginfo(f"Commands: red vx={vx1:.2f} vy={vy1:.2f}, green vx={vx2:.2f} vy={vy2:.2f}")

def main():
    try:
        controller = SimpleEncirclement()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
