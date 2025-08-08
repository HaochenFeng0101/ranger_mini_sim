#!/usr/bin/env python3
import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from se2_control.msg import SE2Command
from geometry_msgs.msg import PoseStamped
from tf import transformations

class EnhancedEncirclement:
    """
    Enhanced Encirclement Controller implementing TPE (Target Position Estimator) 
    and DASC (Distributed Anti-Synchronization Control) algorithms.
    
    Based on the paper algorithm with centralized implementation
    (agents know each other's positions).
    """
    def __init__(self):
        rospy.init_node('enhanced_encirclement')
        
        # Algorithm parameters
        self.r = 3.0  # Desired encirclement radius
        self.alpha = -0.3  # Controller gain (0 < |1 + alpha| < 1)
        self.gamma1 = 0.95  # Exponential forgetting factor
        self.gamma2 = 0.1   # New information utilization factor
        self.nu = 0.1       # Circumnavigation frequency (0 < nu < 1)
        
        # Control gains
        self.kx = [0.8, 0.8]  # Position control gains
        self.kyaw = 0.6       # Yaw control gain
        self.max_linear_vel = 1.2
        self.max_angular_vel = 0.8
        
        # Agent states
        self.x1 = np.array([0.0, 0.0])  # Agent 1 position
        self.x2 = np.array([3.0, 0.0])  # Agent 2 position  
        self.s_true = np.array([5.0, 2.0])  # True target position (known for centralized)
        self.s_hat = np.array([5.0, 2.0])   # Estimated target position
        
        # Agent orientations
        self.yaw1 = 0.0
        self.yaw2 = 0.0
        self.target_yaw = 0.0
        
        # TPE (Target Position Estimator) variables
        self.eta = np.eye(2) * 1.0  # Covariance matrix
        self.eta_inv = np.linalg.inv(self.eta)
        
        # Trajectory variables
        self.k = 0  # Time step counter
        self.zeta_r = np.array([self.r, 0.0])  # Initial circumnavigation vector
        
        # Distance measurements (simulated from positions)
        self.d1s = 0.0  # Distance from agent 1 to target
        self.d2s = 0.0  # Distance from agent 2 to target
        self.d12 = 0.0  # Distance between agents
        
        # Publishers for drone commands
        self.drone1_cmd_pub = rospy.Publisher('/drone0/se2_cmd', SE2Command, queue_size=1)
        self.drone2_cmd_pub = rospy.Publisher('/drone1/se2_cmd', SE2Command, queue_size=1)
        
        # Publisher for target position (for SO3 controller)
        self.target_cmd_pub = rospy.Publisher('/target_position', PoseStamped, queue_size=1)
        
        # Subscribers for drone odometry  
        self.drone1_odom_sub = rospy.Subscriber('/drone0/sim/odom', Odometry, self.drone1_odom_callback)
        self.drone2_odom_sub = rospy.Subscriber('/drone1/sim/odom', Odometry, self.drone2_odom_callback)
        self.target_odom_sub = rospy.Subscriber('/drone2/sim/odom', Odometry, self.target_odom_callback)
        
        # Control timer
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_callback)  # 10Hz
        
        rospy.loginfo("Enhanced Encirclement Controller initialized")
        rospy.loginfo(f"Parameters: r={self.r}, alpha={self.alpha}, nu={self.nu}")
        
    def drone1_odom_callback(self, msg):
        """Update agent 1 state"""
        self.x1[0] = msg.pose.pose.position.x
        self.x1[1] = msg.pose.pose.position.y
        
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                     msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.yaw1 = euler[2]
        
    def drone2_odom_callback(self, msg):
        """Update agent 2 state"""
        self.x2[0] = msg.pose.pose.position.x
        self.x2[1] = msg.pose.pose.position.y
        
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                     msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.yaw2 = euler[2]
        
    def target_odom_callback(self, msg):
        """Update true target position (for centralized control)"""
        self.s_true[0] = msg.pose.pose.position.x
        self.s_true[1] = msg.pose.pose.position.y
        
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                     msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.target_yaw = euler[2]
        
    def update_distance_measurements(self):
        """Update distance measurements from current positions"""
        self.d1s = np.linalg.norm(self.x1 - self.s_true)
        self.d2s = np.linalg.norm(self.x2 - self.s_true)
        self.d12 = np.linalg.norm(self.x1 - self.x2)
        
    def update_target_position_estimator(self):
        """
        Implement TPE (Target Position Estimator) algorithm
        Equation (6)-(8) from the paper
        """
        # Calculate p12(k) = x1(k) - x2(k)
        p12 = self.x1 - self.x2
        
        # Calculate π(k) using equation (5)
        pi_k = -0.5 * (self.d1s**2 - self.d2s**2 - np.dot(self.x1, self.x1) + np.dot(self.x2, self.x2))
        
        # Update covariance matrix η^(-1)(k) using equation (8)
        self.eta_inv = self.gamma1 * self.eta_inv + (1.0 / self.gamma2) * np.outer(p12, p12)
        
        # Calculate estimator gain K(k) using equation (7)
        self.eta = np.linalg.inv(self.eta_inv)
        
        denominator = self.gamma1 * self.gamma2 + np.dot(p12.T, np.dot(self.eta, p12))
        if abs(denominator) > 1e-6:
            K = np.dot(self.eta, p12) / denominator
        else:
            K = np.zeros(2)
            
        # Update target position estimate using equation (6)
        prediction_error = pi_k - np.dot(p12.T, self.s_hat)
        self.s_hat = self.s_hat + K * prediction_error
        
    def generate_circumnavigation_trajectory(self):
        """
        Generate circumnavigation trajectory ζ(r, k)
        """
        # Update time step
        self.k += 1
        
        # Generate circular trajectory
        angle = 2 * math.pi * self.nu * self.k / 100.0  # Normalize by control frequency
        self.zeta_r = self.r * np.array([math.cos(angle), math.sin(angle)])
        
    def calculate_dasc_control(self):
        """
        Implement DASC (Distributed Anti-Synchronization Control)
        Equation (10) from the paper
        """
        # Calculate relative positions to estimated target
        p_hat_10 = self.x1 - self.s_hat
        p_hat_20 = self.x2 - self.s_hat
        
        # DASC control law (equation 10)
        u1 = self.alpha * (p_hat_10 + self.zeta_r)
        u2 = self.alpha * (p_hat_20 - self.zeta_r)
        
        return u1, u2
        
    def control_callback(self, event):
        """Main control loop implementing the complete algorithm"""
        
        # Update distance measurements
        self.update_distance_measurements()
        
        # Update target position estimator (TPE)
        self.update_target_position_estimator()
        
        # Generate circumnavigation trajectory
        self.generate_circumnavigation_trajectory()
        
        # Calculate DASC control inputs
        u1, u2 = self.calculate_dasc_control()
        
        # Convert control inputs to drone commands
        cmd1 = self.create_se2_command(u1, self.x1, self.yaw1, 1)
        cmd2 = self.create_se2_command(u2, self.x2, self.yaw2, 2)
        
        # Publish commands
        self.drone1_cmd_pub.publish(cmd1)
        self.drone2_cmd_pub.publish(cmd2)
        
        # Debug logging (reduced frequency)
        if self.k % 20 == 0:  # Every 2 seconds
            rospy.loginfo(f"Published commands: drone1 vx={cmd1.linear_x:.2f}, vy={cmd1.linear_y:.2f}, motors={cmd1.enable_motors}")
            rospy.loginfo(f"Published commands: drone2 vx={cmd2.linear_x:.2f}, vy={cmd2.linear_y:.2f}, motors={cmd2.enable_motors}")
        
        # Send target movement command (example: slowly moving target)
        self.send_target_movement_command()
        
        # Debug information
        if self.k % 50 == 0:  # Every 5 seconds
            est_error = np.linalg.norm(self.s_hat - self.s_true)
            rospy.loginfo(f"Target estimation error: {est_error:.3f}m")
            rospy.loginfo(f"Agent distances to target: d1s={self.d1s:.2f}, d2s={self.d2s:.2f}")
            
    def create_se2_command(self, u_world, position, yaw, agent_id):
        """Convert world-frame control input to SE2 command"""
        
        # Transform control input to robot frame
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        # Transform from world frame to robot frame
        u_robot_x = u_world[0] * cos_yaw + u_world[1] * sin_yaw
        u_robot_y = -u_world[0] * sin_yaw + u_world[1] * cos_yaw
        
        # Limit velocities
        u_robot_x = max(-self.max_linear_vel, min(self.max_linear_vel, u_robot_x))
        u_robot_y = max(-self.max_linear_vel, min(self.max_linear_vel, u_robot_y))
        
        # Calculate desired yaw (point towards movement direction)
        if np.linalg.norm(u_world) > 0.1:
            desired_yaw = math.atan2(u_world[1], u_world[0])
            yaw_error = desired_yaw - yaw
            
            # Normalize yaw error
            while yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            while yaw_error < -math.pi:
                yaw_error += 2 * math.pi
                
            u_yaw = self.kyaw * yaw_error
        else:
            u_yaw = 0.0
            
        u_yaw = max(-self.max_angular_vel, min(self.max_angular_vel, u_yaw))
        
        # Create SE2 command
        cmd = SE2Command()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "base_link"
        cmd.linear_x = u_robot_x
        cmd.linear_y = u_robot_y
        cmd.angular_z = u_yaw
        cmd.kx = self.kx
        cmd.kyaw = self.kyaw
        cmd.enable_motors = True
        cmd.use_external_yaw = False
        cmd.current_yaw = yaw
        
        return cmd
        
    def send_target_movement_command(self):
        """Send movement commands to target drone"""
        # Example: Make target move in a slow pattern
        t = rospy.Time.now().to_sec()
        
        # Slow circular motion
        radius = 1.0
        frequency = 0.05  # Very slow
        
        target_x = 5.0 + radius * math.cos(2 * math.pi * frequency * t)
        target_y = 2.0 + radius * math.sin(2 * math.pi * frequency * t)
        target_z = 1.5
        
        # Create target position command
        target_cmd = PoseStamped()
        target_cmd.header.stamp = rospy.Time.now()
        target_cmd.header.frame_id = "world"
        target_cmd.pose.position.x = target_x
        target_cmd.pose.position.y = target_y
        target_cmd.pose.position.z = target_z
        
        # Set orientation
        q = transformations.quaternion_from_euler(0, 0, 0)
        target_cmd.pose.orientation.x = q[0]
        target_cmd.pose.orientation.y = q[1]
        target_cmd.pose.orientation.z = q[2]
        target_cmd.pose.orientation.w = q[3]
        
        self.target_cmd_pub.publish(target_cmd)
        
def main():
    try:
        controller = EnhancedEncirclement()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
