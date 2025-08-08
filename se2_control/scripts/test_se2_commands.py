#!/usr/bin/env python3
import rospy
from se2_control.msg import SE2Command

def test_se2_commands():
    """Test script to verify SE2 command topics are working"""
    rospy.init_node('test_se2_commands')
    
    # Publishers for testing
    drone0_pub = rospy.Publisher('/drone0/se2_cmd', SE2Command, queue_size=1)
    drone1_pub = rospy.Publisher('/drone1/se2_cmd', SE2Command, queue_size=1)
    
    rospy.sleep(2.0)  # Wait for connections
    
    # Create test command
    cmd = SE2Command()
    cmd.header.stamp = rospy.Time.now()
    cmd.linear_x = 0.5  # Move forward
    cmd.linear_y = 0.0
    cmd.angular_z = 0.0
    cmd.kx = [0.8, 0.8]
    cmd.kyaw = 0.6
    cmd.enable_motors = True
    cmd.use_external_yaw = False
    cmd.current_yaw = 0.0
    
    rospy.loginfo("Sending test commands to drones...")
    
    # Send commands periodically
    rate = rospy.Rate(2)  # 2 Hz
    for i in range(10):
        if rospy.is_shutdown():
            break
            
        rospy.loginfo(f"Sending command {i+1}/10")
        drone0_pub.publish(cmd)
        drone1_pub.publish(cmd)
        rate.sleep()
        
    # Stop drones
    cmd.linear_x = 0.0
    cmd.linear_y = 0.0
    cmd.angular_z = 0.0
    
    rospy.loginfo("Stopping drones...")
    for i in range(5):
        drone0_pub.publish(cmd)
        drone1_pub.publish(cmd)
        rate.sleep()
        
    rospy.loginfo("Test completed!")

if __name__ == '__main__':
    try:
        test_se2_commands()
    except rospy.ROSInterruptException:
        pass
