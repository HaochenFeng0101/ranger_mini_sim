#!/usr/bin/env python3
import rospy
import numpy as np
import math
from scipy.optimize import minimize
from nav_msgs.msg import Odometry
from se2_control.msg import SE2Command
from geometry_msgs.msg import Pose2D
from tf import transformations

class MPCController:
    """
    Model Predictive Controller for SE2 ground vehicles (like bed trolley).
    Implements a simple bicycle model with MPC optimization for trajectory tracking.
    """
    def __init__(self, vehicle_namespace="", max_speed=1.5, max_angular_vel=2.0):
        self.vehicle_namespace = vehicle_namespace
        self.node_name = f'{vehicle_namespace}_mpc_controller' if vehicle_namespace else 'mpc_controller'
        rospy.init_node(self.node_name)
        
        # Vehicle parameters (bicycle model)
        self.wheelbase = 0.46  # Distance between front and rear axles
        self.max_speed = max_speed
        self.max_angular_vel = max_angular_vel
        self.max_steering_angle = math.pi/3  # 60 degrees
        
        # MPC parameters
        self.prediction_horizon = 10  # Number of prediction steps
        self.control_horizon = 5      # Number of control steps
        self.dt = 0.1                 # Time step (100ms)
        
        # Cost function weights
        self.Q = np.diag([10.0, 10.0, 1.0])  # State weights [x, y, theta]
        self.R = np.diag([1.0, 1.0])         # Input weights [v, delta]
        self.Qf = np.diag([20.0, 20.0, 2.0]) # Terminal state weights
        
        # Current state
        self.current_state = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
        self.target_trajectory = None  # Target trajectory to follow
        self.current_target_index = 0
        
        # Control inputs (previous)
        self.prev_control = np.array([0.0, 0.0])  # [v, delta]
        
        # ROS interface
        if vehicle_namespace:
            odom_topic = f'/{vehicle_namespace}/four_wheel_steering_controller/odom'
            cmd_topic = f'/{vehicle_namespace}/se2_cmd'
            target_topic = f'/{vehicle_namespace}/mpc_target'
        else:
            odom_topic = '/odom'
            cmd_topic = '/se2_cmd'
            target_topic = '/mpc_target'
            
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        self.target_sub = rospy.Subscriber(target_topic, Pose2D, self.target_callback)
        self.cmd_pub = rospy.Publisher(cmd_topic, SE2Command, queue_size=1)
        
        # Control timer
        self.control_timer = rospy.Timer(rospy.Duration(self.dt), self.control_callback)
        
        rospy.loginfo(f"MPC Controller initialized for {vehicle_namespace or 'default'}")
        rospy.loginfo(f"Prediction horizon: {self.prediction_horizon}, Control horizon: {self.control_horizon}")
        rospy.loginfo(f"Listening to: {odom_topic}")
        rospy.loginfo(f"Publishing to: {cmd_topic}")
        rospy.loginfo(f"Target topic: {target_topic}")
    
    def odom_callback(self, msg):
        """Update current state from odometry"""
        self.current_state[0] = msg.pose.pose.position.x
        self.current_state[1] = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler = transformations.euler_from_quaternion(quaternion)
        self.current_state[2] = euler[2]  # yaw
    
    def target_callback(self, msg):
        """Set a single target point"""
        target_state = np.array([msg.x, msg.y, msg.theta])
        
        # Create a simple trajectory: current -> target
        self.target_trajectory = np.array([self.current_state, target_state])
        self.current_target_index = 0
        
        rospy.loginfo(f"MPC target set: ({msg.x:.2f}, {msg.y:.2f}, {msg.theta:.2f})")
    
    def set_trajectory(self, trajectory):
        """Set a trajectory for the controller to follow"""
        self.target_trajectory = trajectory
        self.current_target_index = 0
    
    def bicycle_model(self, state, control):
        """
        Bicycle model dynamics
        state: [x, y, theta]
        control: [v, delta] (velocity and steering angle)
        """
        x, y, theta = state
        v, delta = control
        
        # Limit steering angle
        delta = np.clip(delta, -self.max_steering_angle, self.max_steering_angle)
        v = np.clip(v, -self.max_speed, self.max_speed)
        
        # Bicycle model equations
        x_dot = v * np.cos(theta)
        y_dot = v * np.sin(theta)
        theta_dot = (v / self.wheelbase) * np.tan(delta)
        
        return np.array([x_dot, y_dot, theta_dot])
    
    def predict_trajectory(self, initial_state, control_sequence):
        """Predict trajectory given control sequence"""
        states = np.zeros((self.prediction_horizon + 1, 3))
        states[0] = initial_state
        
        for i in range(self.prediction_horizon):
            if i < len(control_sequence):
                control = control_sequence[i]
            else:
                # Use last control if sequence is shorter
                control = control_sequence[-1] if len(control_sequence) > 0 else np.array([0.0, 0.0])
            
            # Forward Euler integration
            state_dot = self.bicycle_model(states[i], control)
            states[i + 1] = states[i] + state_dot * self.dt
            
        return states
    
    def get_reference_trajectory(self):
        """Get reference trajectory for MPC prediction horizon"""
        if self.target_trajectory is None:
            # No target, stay at current position
            ref_traj = np.tile(self.current_state, (self.prediction_horizon + 1, 1))
            return ref_traj
        
        if len(self.target_trajectory) == 2:
            # Simple point-to-point: interpolate between current and target
            start = self.current_state
            end = self.target_trajectory[1]
            
            ref_traj = np.zeros((self.prediction_horizon + 1, 3))
            for i in range(self.prediction_horizon + 1):
                alpha = min(1.0, i / self.prediction_horizon)
                ref_traj[i] = (1 - alpha) * start + alpha * end
        else:
            # Multi-point trajectory: sample points
            ref_traj = np.zeros((self.prediction_horizon + 1, 3))
            for i in range(self.prediction_horizon + 1):
                idx = min(self.current_target_index + i, len(self.target_trajectory) - 1)
                ref_traj[i] = self.target_trajectory[idx]
        
        return ref_traj
    
    def mpc_cost(self, control_flat):
        """Cost function for MPC optimization"""
        # Reshape control sequence
        control_sequence = control_flat.reshape(self.control_horizon, 2)
        
        # Predict trajectory
        predicted_states = self.predict_trajectory(self.current_state, control_sequence)
        
        # Get reference trajectory
        ref_trajectory = self.get_reference_trajectory()
        
        # Calculate cost
        cost = 0.0
        
        # State tracking cost
        for i in range(self.prediction_horizon):
            state_error = predicted_states[i] - ref_trajectory[i]
            # Normalize angle error
            state_error[2] = self.normalize_angle(state_error[2])
            if i == self.prediction_horizon - 1:
                # Terminal cost
                cost += state_error.T @ self.Qf @ state_error
            else:
                cost += state_error.T @ self.Q @ state_error
        
        # Control cost
        for i in range(self.control_horizon):
            if i < len(control_sequence):
                control = control_sequence[i]
                cost += control.T @ self.R @ control
                
                # Smooth control changes
                if i > 0:
                    control_diff = control - control_sequence[i-1]
                    cost += 0.1 * (control_diff.T @ control_diff)
                else:
                    control_diff = control - self.prev_control
                    cost += 0.1 * (control_diff.T @ control_diff)
        
        return cost
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def solve_mpc(self):
        """Solve MPC optimization problem"""
        if self.target_trajectory is None:
            return np.array([0.0, 0.0])  # No target, stop
        
        # Initial guess for control sequence
        initial_guess = np.tile(self.prev_control, self.control_horizon)
        
        # Bounds for control inputs
        bounds = []
        for _ in range(self.control_horizon):
            bounds.append((-self.max_speed, self.max_speed))      # velocity
            bounds.append((-self.max_steering_angle, self.max_steering_angle))  # steering
        
        # Solve optimization
        try:
            result = minimize(
                self.mpc_cost,
                initial_guess,
                method='SLSQP',
                bounds=bounds,
                options={'maxiter': 100, 'disp': False}
            )
            
            if result.success:
                optimal_control = result.x.reshape(self.control_horizon, 2)
                return optimal_control[0]  # Return first control action
            else:
                rospy.logwarn("MPC optimization failed, using previous control")
                return self.prev_control
                
        except Exception as e:
            rospy.logerr(f"MPC optimization error: {e}")
            return np.array([0.0, 0.0])
    
    def control_to_se2_command(self, control):
        """Convert MPC control output to SE2Command"""
        v, delta = control
        
        # Convert bicycle model to SE2 velocities
        # For four-wheel steering, we approximate the bicycle model
        linear_x = v  # Forward velocity
        linear_y = 0.0  # No lateral velocity in bicycle model
        angular_z = (v / self.wheelbase) * np.tan(delta)  # Yaw rate
        
        # Limit outputs
        angular_z = np.clip(angular_z, -self.max_angular_vel, self.max_angular_vel)
        
        # Create SE2Command
        cmd = SE2Command()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "base_link"
        cmd.linear_x = linear_x
        cmd.linear_y = linear_y
        cmd.angular_z = angular_z
        cmd.kx = [0.8, 0.8]  # Default position gains
        cmd.kyaw = 0.5       # Default yaw gain
        cmd.enable_motors = True
        cmd.use_external_yaw = False
        cmd.current_yaw = self.current_state[2]
        
        return cmd
    
    def control_callback(self, event):
        """Main control loop"""
        if self.target_trajectory is None:
            # No target, send stop command
            cmd = self.control_to_se2_command(np.array([0.0, 0.0]))
            self.cmd_pub.publish(cmd)
            return
        
        # Check if we've reached the target
        if len(self.target_trajectory) == 2:
            target = self.target_trajectory[1]
            pos_error = np.linalg.norm(self.current_state[:2] - target[:2])
            angle_error = abs(self.normalize_angle(self.current_state[2] - target[2]))
            
            if pos_error < 0.2 and angle_error < 0.2:
                # Target reached, stop
                cmd = self.control_to_se2_command(np.array([0.0, 0.0]))
                self.cmd_pub.publish(cmd)
                rospy.loginfo("MPC: Target reached!")
                self.target_trajectory = None
                return
        
        # Solve MPC
        optimal_control = self.solve_mpc()
        
        # Update previous control
        self.prev_control = optimal_control
        
        # Convert to SE2Command and publish
        cmd = self.control_to_se2_command(optimal_control)
        self.cmd_pub.publish(cmd)
        
        # Debug info
        if hasattr(self, 'debug_counter'):
            self.debug_counter += 1
        else:
            self.debug_counter = 0
            
        if self.debug_counter % 10 == 0:  # Every 1 second
            rospy.loginfo(f"MPC: pos=({self.current_state[0]:.2f}, {self.current_state[1]:.2f}), "
                         f"yaw={self.current_state[2]:.2f}, v={optimal_control[0]:.2f}, "
                         f"delta={optimal_control[1]:.2f}")

def main():
    try:
        # Can be used standalone or with namespace
        import sys
        namespace = ""
        if len(sys.argv) > 1:
            namespace = sys.argv[1]
        
        controller = MPCController(namespace)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
