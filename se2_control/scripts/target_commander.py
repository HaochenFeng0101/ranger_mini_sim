#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped
from tf import transformations

class TargetCommander:
    """
    Simple commander to send position commands to the target drone.
    Can be controlled via command line or programmatically.
    """
    def __init__(self):
        rospy.init_node('target_commander')
        
        # Publisher for target position commands
        self.target_pub = rospy.Publisher('/target_position', PoseStamped, queue_size=1)
        
        rospy.loginfo("Target Commander initialized")
        rospy.loginfo("Use send_target_position(x, y, z, yaw) to command target")
        
    def send_target_position(self, x, y, z, yaw=0.0):
        """Send a target position command"""
        cmd = PoseStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "world"
        
        cmd.pose.position.x = x
        cmd.pose.position.y = y
        cmd.pose.position.z = z
        
        # Convert yaw to quaternion
        q = transformations.quaternion_from_euler(0, 0, yaw)
        cmd.pose.orientation.x = q[0]
        cmd.pose.orientation.y = q[1]
        cmd.pose.orientation.z = q[2]
        cmd.pose.orientation.w = q[3]
        
        self.target_pub.publish(cmd)
        rospy.loginfo(f"Target commanded to: ({x:.2f}, {y:.2f}, {z:.2f}, yaw={yaw:.2f})")
        
    def execute_mission(self):
        """Execute a predefined mission"""
        rospy.sleep(3.0)  # Wait for system to initialize
        
        positions = [
            (5.0, 2.0, 1.5, 0.0),      # Initial position
            (8.0, 4.0, 2.0, math.pi/4),   # Move to corner
            (10.0, 6.0, 1.8, math.pi/2),  # Higher position
            (7.0, 8.0, 1.2, math.pi),     # Different direction
            (4.0, 6.0, 1.8, -math.pi/2),  # Return direction
            (5.0, 2.0, 1.5, 0.0),      # Back to start
        ]
        
        for i, (x, y, z, yaw) in enumerate(positions):
            if rospy.is_shutdown():
                break
                
            rospy.loginfo(f"Mission step {i+1}/{len(positions)}")
            self.send_target_position(x, y, z, yaw)
            rospy.sleep(10.0)  # Wait 10 seconds between waypoints
            
        rospy.loginfo("Mission completed!")
        
    def circular_motion(self, center_x=5.0, center_y=2.0, radius=2.0, height=1.5):
        """Make target move in a circular pattern"""
        rate = rospy.Rate(10)  # 10 Hz
        t = 0.0
        
        while not rospy.is_shutdown():
            # Circular motion parameters
            angular_velocity = 0.1  # rad/s
            
            x = center_x + radius * math.cos(angular_velocity * t)
            y = center_y + radius * math.sin(angular_velocity * t)
            z = height + 0.3 * math.sin(2 * angular_velocity * t)  # Slight vertical motion
            yaw = angular_velocity * t  # Rotate as it moves
            
            self.send_target_position(x, y, z, yaw)
            
            t += 0.1
            rate.sleep()

def main():
    try:
        commander = TargetCommander()
        
        # Example usage:
        rospy.loginfo("Starting target mission in 5 seconds...")
        rospy.sleep(5.0)
        
        # Choose one of these missions:
        # 1. Execute predefined waypoint mission
        commander.execute_mission()
        
        # 2. Or continuous circular motion (uncomment below)
        # commander.circular_motion()
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
