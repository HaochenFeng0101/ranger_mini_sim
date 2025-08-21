#!/usr/bin/env python3
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray
from se2_control.msg import SE2Command
from std_msgs.msg import Float32MultiArray

class RangeBasedEncirclement:
    """
    Improved range-based encirclement controller using only relative measurements.
    Achieves multi-robot coordination without requiring absolute poses.
    Features:
    - No pose sharing between robots
    - Target search behavior when target is out of range
    - Improved formation control to prevent getting stuck
    - Adaptive control gains based on measurement quality
    """
    def __init__(self):
        rospy.init_node('range_based_encirclement')
        
        # Control parameters - tuned for stability
        self.kp_distance = 0.8  # Reduced from 1.2 for stability
        self.kp_angle = 1.0     # Reduced from 1.5 for stability
        self.kp_formation = 0.6 # Reduced from 0.8 for stability
        self.max_vel = 1.0      # Reduced from 1.2 for stability
        
        # Formation parameters
        self.target_distance = 3.0
        self.formation_spacing = 2.5
        self.formation_angle = math.pi / 2  # 90 degrees
        
        # Formation angles for each robot (relative to target)
        self.formation_angles = {
            'drone0': -self.formation_angle / 2,  # -45 degrees
            'drone1': self.formation_angle / 2    # +45 degrees
        }
        
        # Formation directions (clockwise/counterclockwise)
        self.formation_directions = {
            'drone0': 1.0,   # Clockwise
            'drone1': -1.0   # Counterclockwise
        }
        
        # Search parameters
        self.search_angular_vel = 1.2  # Increased from 0.8 for more aggressive search
        self.last_target_seen = {
            'drone0': rospy.Time.now(),
            'drone1': rospy.Time.now()
        }
        
        # VELOCITY SMOOTHING: Store previous velocities to prevent oscillation
        self.previous_velocities = {
            'drone0': {'vx': 0.0, 'vy': 0.0, 'vyaw': 0.0},
            'drone1': {'vx': 0.0, 'vy': 0.0, 'vyaw': 0.0}
        }
        
        # Smoothing factor (0.0 = no smoothing, 1.0 = full smoothing)
        self.velocity_smoothing = 0.7
        
        # OSCILLATION DETECTION: Track velocity changes to detect oscillation
        self.velocity_history = {
            'drone0': {'vx': [], 'vy': [], 'vyaw': []},
            'drone1': {'vx': [], 'vy': [], 'vyaw': []}
        }
        self.max_history_length = 10  # Track last 10 velocity values
        
        # Robot states (relative measurements only)
        self.robot_measurements = {}    # Store range measurements for each robot
        self.relative_poses = {}        # Store relative pose information
        
        # Setup publishers and subscribers
        self.setup_communication()
        
        # Control loop
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_callback)
        
        rospy.loginfo("Range-based encirclement controller initialized")
        rospy.loginfo("CONTROLLING ONLY drone0 and drone1 for encirclement")
        rospy.loginfo("Target drone2 is controlled separately by manual commands")
        
    def setup_communication(self):
        """Initialize publishers and subscribers"""
        # Publishers for drone commands (ONLY for encirclement agents, NOT target)
        self.agent1_cmd_pub = rospy.Publisher('/drone0/se2_cmd', SE2Command, queue_size=1)
        self.agent2_cmd_pub = rospy.Publisher('/drone1/se2_cmd', SE2Command, queue_size=1)
        
        # Subscribers for range measurements (from all drones including target)
        self.agent1_laser_sub = rospy.Subscriber('/drone0/laser_scan', LaserScan, self.agent1_laser_callback)
        self.agent2_laser_sub = rospy.Subscriber('/drone1/laser_scan', LaserScan, self.agent2_laser_callback)
        self.target_laser_sub = rospy.Subscriber('/drone2/laser_scan', LaserScan, self.target_laser_callback)
        
        # Subscribers for relative poses (from encirclement agents only)
        self.agent1_relative_sub = rospy.Subscriber('/drone0/relative_poses', PoseArray, self.agent1_relative_callback)
        self.agent2_relative_sub = rospy.Subscriber('/drone1/relative_poses', PoseArray, self.agent2_relative_callback)
        
    def agent1_laser_callback(self, msg):
        """Handle laser scan from agent 1 (drone0)"""
        self.robot_measurements['drone0'] = {
            'laser': msg,
            'timestamp': rospy.Time.now()
        }
    
    def agent2_laser_callback(self, msg):
        """Handle laser scan from agent 2 (drone1)"""
        self.robot_measurements['drone1'] = {
            'laser': msg,
            'timestamp': msg.header.stamp
        }
    
    def target_laser_callback(self, msg):
        """Handle laser scan from target (drone2) - for reference only, NOT for control"""
        # Store target laser data for reference, but don't control the target
        self.robot_measurements['drone2'] = {
            'laser': msg,
            'timestamp': msg.header.stamp
        }
        rospy.logdebug("Target laser data received (for reference only - target controlled separately)")
    
    def agent1_relative_callback(self, msg):
        """Handle relative poses from agent 1"""
        self.process_relative_poses('drone0', msg)
    
    def agent2_relative_callback(self, msg):
        """Handle relative poses from agent 2"""
        self.process_relative_poses('drone1', msg)
    
    def process_relative_poses(self, robot_name, pose_array):
        """Process relative pose information"""
        if not hasattr(self, 'relative_poses'):
            self.relative_poses = {}
        
        self.relative_poses[robot_name] = pose_array.poses
        
        # Debug info
        rospy.logdebug(f"{robot_name}: Received {len(pose_array.poses)} relative poses")
    
    def find_target_in_laser_scan(self, laser_data):
        """Find target (drone2) in laser scan data with improved detection"""
        if not laser_data or 'laser' not in laser_data:
            return None, None
        
        laser = laser_data['laser']
        ranges = laser.ranges
        
        # Find the closest object within reasonable range (assumed to be the target)
        min_range = float('inf')
        min_angle = 0.0
        
        for i, range_val in enumerate(ranges):
            if range_val < laser.range_min or range_val > laser.range_max:
                continue
            
            # Look for objects within reasonable target distance (not too close, not too far)
            # Very flexible detection range to catch all possible drones
            if 0.2 < range_val < 30.0 and range_val < min_range:
                min_range = range_val
                min_angle = laser.angle_min + i * laser.angle_increment
        
        if min_range == float('inf'):
            return None, None
        
        # Normalize angle to [-pi, pi]
        while min_angle > math.pi:
            min_angle -= 2 * math.pi
        while min_angle < -math.pi:
            min_angle += 2 * math.pi
        
        return min_range, min_angle
    
    def find_other_agent_in_laser_scan(self, laser_data, agent_name):
        """Find the other agent in laser scan data with improved detection"""
        if not laser_data or 'laser' not in laser_data:
            return None, None
        
        laser = laser_data['laser']
        ranges = laser.ranges
        
        # Look for objects at expected formation distance
        formation_candidates = []
        
        for i, range_val in enumerate(ranges):
            if range_val < laser.range_min or range_val > laser.range_max:
                continue
            
            # Check if this could be the other agent (at formation distance)
            # More flexible detection range to prevent getting stuck
            if abs(range_val - self.formation_spacing) < 4.0:  # Increased from 3.0 to 4.0
                angle = laser.angle_min + i * laser.angle_increment
                formation_candidates.append((range_val, angle))
        
        if not formation_candidates:
            return None, None
        
        # Return the closest candidate
        return min(formation_candidates, key=lambda x: abs(x[0] - self.formation_spacing))
    
    def should_search_for_target(self, robot_name):
        """Determine if robot should search for target - IMMEDIATE search when no drones visible"""
        # IMMEDIATE SEARCH: If we haven't seen the target recently, start searching immediately
        if robot_name not in self.last_target_seen:
            rospy.loginfo(f"{robot_name}: No last seen time recorded - starting search immediately")
            return True
        
        time_since_last_seen = (rospy.Time.now() - self.last_target_seen[robot_name]).to_sec()
        
        # IMMEDIATE SEARCH: Start searching after only 0.5 seconds (was 1.0)
        should_search = time_since_last_seen > 0.5
        
        rospy.loginfo(f"{robot_name}: Time since last target seen: {time_since_last_seen:.1f}s, should_search: {should_search}")
        
        return should_search
    
    def calculate_search_behavior(self, robot_name):
        """Calculate MAXIMUM AGGRESSIVE search behavior when target is not visible"""
        # MAXIMUM AGGRESSIVE SEARCH: Strong rotation with NO smoothing for immediate response
        
        # DIRECT rotation without smoothing for immediate search response
        base_rotation = self.search_angular_vel * self.formation_directions.get(robot_name, 1.0)
        
        # MAXIMUM rotation strength - no smoothing during search
        vyaw = base_rotation * 3.0  # 3x stronger rotation for fast scanning
        
        # Forward movement to explore area while searching
        vx = 0.15 * self.formation_directions.get(robot_name, 1.0)  # Increased movement
        
        # No lateral movement during search
        vy = 0.0
        
        # Store for next iteration
        self.previous_velocities[robot_name] = {'vx': vx, 'vy': vy, 'vyaw': vyaw}
        
        rospy.loginfo(f"{robot_name}: MAXIMUM AGGRESSIVE SEARCH MODE - vx={vx:.2f}, vy={vy:.2f}, vyaw={vyaw:.2f}")
        
        return vx, vy, vyaw
    
    def calculate_encirclement_control(self, robot_name, target_range, target_angle, other_agent_range, other_agent_angle):
        """Calculate stable control commands for encirclement - prevents oscillation"""
        
        # SIMPLIFIED CONTROL LOGIC: Focus on one objective at a time to prevent conflicts
        
        # 1. PRIMARY: Distance control to target
        distance_error = target_range - self.target_distance
        if abs(distance_error) < 0.3:  # Increased deadzone for stability
            distance_error = 0.0
        
        # 2. SECONDARY: Angular positioning (encirclement)
        # Use formation direction to determine desired angle
        desired_angle = self.formation_angles[robot_name]
        angle_error = target_angle - desired_angle
        
        # Normalize angle error to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # 3. TERTIARY: Formation spacing (only if significant)
        formation_vel = 0.0
        if other_agent_range is not None:
            formation_error = other_agent_range - self.formation_spacing
            if abs(formation_error) > 0.5:  # Larger deadzone for formation
                formation_vel = 0.3 * formation_error  # Reduced gain
        
        # CALCULATE VELOCITIES with priority system
        # Forward velocity: primarily distance control
        vx = self.kp_distance * distance_error
        
        # Lateral velocity: primarily angular control, secondary formation
        vy = self.kp_angle * angle_error + 0.2 * formation_vel
        
        # Yaw rate: point toward target (smooth)
        vyaw = 0.2 * target_angle  # Reduced gain for stability
        
        # VELOCITY SMOOTHING: Apply smoothing to prevent oscillation
        prev_vel = self.previous_velocities[robot_name]
        vx = self.velocity_smoothing * prev_vel['vx'] + (1 - self.velocity_smoothing) * vx
        vy = self.velocity_smoothing * prev_vel['vy'] + (1 - self.velocity_smoothing) * vy
        vyaw = self.velocity_smoothing * prev_vel['vyaw'] + (1 - self.velocity_smoothing) * vyaw
        
        # Store current velocities for next iteration
        self.previous_velocities[robot_name] = {'vx': vx, 'vy': vy, 'vyaw': vyaw}
        
        # VELOCITY LIMITING: Prevent extreme values
        max_individual_vel = 0.8  # Individual velocity limit
        vx = max(-max_individual_vel, min(max_individual_vel, vx))
        vy = max(-max_individual_vel, min(max_individual_vel, vy))
        vyaw = max(-0.5, min(0.5, vyaw))  # Limit yaw rate
        
        # MINIMUM MOVEMENT: Ensure some movement to prevent getting stuck
        min_vel = 0.1
        if abs(vx) < min_vel and abs(vy) < min_vel:
            # Gentle forward movement in formation direction
            vx = min_vel * self.formation_directions.get(robot_name, 1.0)
            rospy.logdebug(f"{robot_name}: Applying minimum movement vx={vx:.2f}")
        
        rospy.logdebug(f"{robot_name}: Control - distance_err={distance_error:.2f}, angle_err={angle_error:.2f}, vx={vx:.2f}, vy={vy:.2f}, vyaw={vyaw:.2f}")
        
        return vx, vy, vyaw
    
    def detect_oscillation(self, robot_name, vx, vy, vyaw):
        """Detect oscillation patterns in velocity commands"""
        history = self.velocity_history[robot_name]
        
        # Add current velocities to history
        history['vx'].append(vx)
        history['vy'].append(vy)
        history['vyaw'].append(vyaw)
        
        # Keep only recent history
        if len(history['vx']) > self.max_history_length:
            history['vx'] = history['vx'][-self.max_history_length:]
            history['vy'] = history['vy'][-self.max_history_length:]
            history['vyaw'] = history['vyaw'][-self.max_history_length:]
        
        # Check for oscillation patterns (alternating signs)
        if len(history['vx']) >= 4:
            vx_signs = [1 if v > 0 else (-1 if v < 0 else 0) for v in history['vx'][-4:]]
            vy_signs = [1 if v > 0 else (-1 if v < 0 else 0) for v in history['vy'][-4:]]
            
            # Detect alternating pattern (oscillation)
            vx_oscillating = vx_signs.count(1) >= 2 and vx_signs.count(-1) >= 2
            vy_oscillating = vy_signs.count(1) >= 2 and vy_signs.count(-1) >= 2
            
            if vx_oscillating or vy_oscillating:
                rospy.logwarn(f"{robot_name}: OSCILLATION DETECTED! Applying damping...")
                return True
        
        return False
    
    def apply_oscillation_damping(self, robot_name, vx, vy, vyaw):
        """Apply damping to reduce oscillation"""
        # Reduce velocities when oscillation is detected
        damping_factor = 0.3  # Reduce to 30% of original
        
        vx_damped = vx * damping_factor
        vy_damped = vy * damping_factor
        vyaw_damped = vyaw * damping_factor
        
        rospy.loginfo(f"{robot_name}: Applied oscillation damping - vx: {vx:.2f}->{vx_damped:.2f}, vy: {vy:.2f}->{vy_damped:.2f}")
        
        return vx_damped, vy_damped, vyaw_damped
    
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
        """Main control loop - ONLY controls drone0 and drone1 for encirclement"""
        
        rospy.logdebug("Control callback triggered")
        
        # Formation angle is static for stable encirclement
        # No need to update dynamically - use fixed formation positions
        
        # Calculate control for agent 1 (drone0) - ENCIRCLEMENT AGENT
        agent1_target_range, agent1_target_angle = self.find_target_in_laser_scan(
            self.robot_measurements.get('drone0')
        )
        agent1_other_range, agent1_other_angle = self.find_other_agent_in_laser_scan(
            self.robot_measurements.get('drone0'), 'drone0'
        )
        
        # Calculate control for agent 2 (drone1) - ENCIRCLEMENT AGENT
        agent2_target_range, agent2_target_angle = self.find_target_in_laser_scan(
            self.robot_measurements.get('drone1')
        )
        agent2_other_range, agent2_other_angle = self.find_other_agent_in_laser_scan(
            self.robot_measurements.get('drone1'), 'drone1'
        )
        
        # DEBUG: Log target detection status
        rospy.loginfo(f"=== TARGET DETECTION STATUS ===")
        rospy.loginfo(f"Drone0: Target range={agent1_target_range}, angle={agent1_target_angle}")
        rospy.loginfo(f"Drone1: Target range={agent2_target_range}, angle={agent2_target_angle}")
        rospy.loginfo(f"Drone0: Other agent range={agent1_other_range}, angle={agent1_other_angle}")
        rospy.loginfo(f"Drone1: Other agent range={agent2_other_range}, angle={agent2_other_angle}")
        
        # Update last target seen times (ONLY for encirclement agents)
        if agent1_target_range is not None:
            self.last_target_seen['drone0'] = rospy.Time.now()
            rospy.loginfo("Drone0: Target detected - updating last seen time")
        if agent2_target_range is not None:
            self.last_target_seen['drone1'] = rospy.Time.now()
            rospy.loginfo("Drone1: Target detected - updating last seen time")
        
        # Calculate control commands for ENCIRCLEMENT AGENTS ONLY
        if agent1_target_range is not None:
            # Target visible - use encirclement control
            vx1, vy1, vyaw1 = self.calculate_encirclement_control(
                'drone0', agent1_target_range, agent1_target_angle, 
                agent1_other_range, agent1_other_angle
            )
            rospy.loginfo(f"Drone0: Target visible at {agent1_target_range:.2f}m - SWITCHING TO ENCIRCLEMENT MODE")
        else:
            # Target not visible - use search behavior
            if self.should_search_for_target('drone0'):
                vx1, vy1, vyaw1 = self.calculate_search_behavior('drone0')
                rospy.loginfo("Drone0: Target not found - ACTIVATING AGGRESSIVE SEARCH MODE")
            else:
                # SAFETY: Force search if no target and no other agent visible
                if agent1_other_range is None:
                    rospy.logwarn("Drone0: No drones visible at all - FORCING SEARCH MODE")
                    vx1, vy1, vyaw1 = self.calculate_search_behavior('drone0')
                else:
                    vx1, vy1, vyaw1 = 0.0, 0.0, 0.0
                    search_time = (rospy.Time.now() - self.last_target_seen['drone0']).to_sec()
                    rospy.loginfo(f"Drone0: Target not found but search timeout not reached ({search_time:.1f}s < 0.5s)")
        
        if agent2_target_range is not None:
            # Target visible - use encirclement control
            vx2, vy2, vyaw2 = self.calculate_encirclement_control(
                'drone1', agent2_target_range, agent2_target_angle,
                agent2_other_range, agent2_other_angle
            )
            rospy.loginfo(f"Drone1: Target visible at {agent2_target_range:.2f}m - SWITCHING TO ENCIRCLEMENT MODE")
        else:
            # Target not visible - use search behavior
            if self.should_search_for_target('drone1'):
                vx2, vy2, vyaw2 = self.calculate_search_behavior('drone1')
                rospy.loginfo("Drone1: Target not found - ACTIVATING AGGRESSIVE SEARCH MODE")
            else:
                # SAFETY: Force search if no target and no other agent visible
                if agent2_other_range is None:
                    rospy.logwarn("Drone1: No drones visible at all - FORCING SEARCH MODE")
                    vx2, vy2, vyaw2 = self.calculate_search_behavior('drone1')
                else:
                    vx2, vy2, vyaw2 = 0.0, 0.0, 0.0
                    search_time = (rospy.Time.now() - self.last_target_seen['drone1']).to_sec()
                    rospy.loginfo(f"Drone1: Target not found but search timeout not reached ({search_time:.1f}s < 0.5s)")
        
        # Create and publish commands for ENCIRCLEMENT AGENTS ONLY
        cmd1 = self.create_se2_command(vx1, vy1, vyaw1)
        cmd2 = self.create_se2_command(vx2, vy2, vyaw2)
        
        # OSCILLATION DETECTION AND DAMPING
        if self.detect_oscillation('drone0', vx1, vy1, vyaw1):
            vx1_damped, vy1_damped, vyaw1_damped = self.apply_oscillation_damping('drone0', vx1, vy1, vyaw1)
            cmd1 = self.create_se2_command(vx1_damped, vy1_damped, vyaw1_damped)
        
        if self.detect_oscillation('drone1', vx2, vy2, vyaw2):
            vx2_damped, vy2_damped, vyaw2_damped = self.apply_oscillation_damping('drone1', vx2, vy2, vyaw2)
            cmd2 = self.create_se2_command(vx2_damped, vy2_damped, vyaw2_damped)
        
        # SAFETY: Force movement if both drones have zero velocity (stuck situation)
        if abs(vx1) < 0.01 and abs(vy1) < 0.01 and abs(vyaw1) < 0.01:
            rospy.logwarn("Drone0 appears stuck - forcing emergency movement")
            cmd1.linear_x = 0.2 * self.formation_directions.get('drone0', 1.0)
            cmd1.angular_z = 0.3 * self.formation_directions.get('drone0', 1.0)
        
        if abs(vx2) < 0.01 and abs(vy2) < 0.01 and abs(vyaw2) < 0.01:
            rospy.logwarn("Drone1 appears stuck - forcing emergency movement")
            cmd2.linear_x = 0.2 * self.formation_directions.get('drone1', 1.0)
            cmd2.angular_z = 0.3 * self.formation_directions.get('drone1', 1.0)
        
        rospy.logdebug(f"Publishing commands - Drone0: vx={vx1:.3f}, vy={vy1:.3f}, vyaw={vyaw1:.3f}")
        rospy.logdebug(f"Publishing commands - Drone1: vx={vx2:.3f}, vy={vy2:.3f}, vyaw={vyaw2:.3f}")
        
        self.agent1_cmd_pub.publish(cmd1)
        self.agent2_cmd_pub.publish(cmd2)
        
        # Debug info (every 2 seconds)
        if hasattr(self, 'debug_counter'):
            self.debug_counter += 1
        else:
            self.debug_counter = 0
            
        if self.debug_counter % 20 == 0:
            rospy.loginfo("=== ENCIRCLEMENT AGENTS STATUS (drone0 & drone1 only) ===")
            rospy.loginfo(f"Formation angle: {math.degrees(self.formation_angle):.1f}°")
            
            # Agent 1 status (drone0)
            if agent1_target_range is not None:
                rospy.loginfo(f"Drone0: Target at {agent1_target_range:.2f}m, {math.degrees(agent1_target_angle):.1f}°")
            else:
                search_time = (rospy.Time.now() - self.last_target_seen['drone0']).to_sec()
                rospy.loginfo(f"Drone0: No target detected (searching for {search_time:.1f}s)")
            
            if agent1_other_range is not None:
                rospy.loginfo(f"Drone0: Other agent at {agent1_other_range:.2f}m, {math.degrees(agent1_other_angle):.1f}°")
            
            # Agent 2 status (drone1)
            if agent2_target_range is not None:
                rospy.loginfo(f"Drone1: Target at {agent2_target_range:.2f}m, {math.degrees(agent2_target_angle):.1f}°")
            else:
                search_time = (rospy.Time.now() - self.last_target_seen['drone1']).to_sec()
                rospy.loginfo(f"Drone1: No target detected (searching for {search_time:.1f}s)")
            
            if agent2_other_range is not None:
                rospy.loginfo(f"Drone1: Other agent at {agent2_other_range:.2f}m, {math.degrees(agent2_other_angle):.1f}°")
            
            rospy.loginfo(f"Commands: Drone0 vx={vx1:.2f} vy={vy1:.2f} vyaw={vyaw1:.2f}")
            rospy.loginfo(f"         Drone1 vx={vx2:.2f} vy={vy2:.2f} vyaw={vyaw2:.2f}")
            rospy.loginfo("NOTE: Target drone2 is controlled separately by manual commands")
            rospy.loginfo("=========================================================")

def main():
    try:
        controller = RangeBasedEncirclement()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
