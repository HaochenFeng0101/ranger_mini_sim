#!/usr/bin/env python3
"""
Test script for the range-based encirclement system.
This script helps verify that all components are working correctly.
"""

import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray
from se2_control.msg import SE2Command

class RangeSystemTester:
    def __init__(self):
        rospy.init_node('range_system_tester')
        
        # Test results
        self.test_results = {
            'drone0_laser': False,
            'drone1_laser': False,
            'drone2_laser': False,
            'drone0_relative': False,
            'drone1_relative': False,
            'drone0_cmd': False,
            'drone1_cmd': False
        }
        
        # Subscribers to test topics
        self.setup_test_subscribers()
        
        # Test timer
        self.test_timer = rospy.Timer(rospy.Duration(2.0), self.run_tests)
        
        rospy.loginfo("Range System Tester initialized")
        rospy.loginfo("Running tests every 2 seconds...")
        
    def setup_test_subscribers(self):
        """Setup subscribers to test various topics"""
        # Test laser scan topics
        rospy.Subscriber('/drone0/laser_scan', LaserScan, self.drone0_laser_callback)
        rospy.Subscriber('/drone1/laser_scan', LaserScan, self.drone1_laser_callback)
        rospy.Subscriber('/drone2/laser_scan', LaserScan, self.drone2_laser_callback)
        
        # Test relative pose topics
        rospy.Subscriber('/drone0/relative_poses', PoseArray, self.drone0_relative_callback)
        rospy.Subscriber('/drone1/relative_poses', PoseArray, self.drone1_relative_callback)
        
        # Test command topics
        rospy.Subscriber('/drone0/se2_cmd', SE2Command, self.drone0_cmd_callback)
        rospy.Subscriber('/drone1/se2_cmd', SE2Command, self.drone1_cmd_callback)
        
    def drone0_laser_callback(self, msg):
        """Test drone0 laser scan"""
        self.test_results['drone0_laser'] = True
        rospy.loginfo("✓ Drone0 laser scan working")
    
    def drone1_laser_callback(self, msg):
        """Test drone1 laser scan"""
        self.test_results['drone1_laser'] = True
        rospy.loginfo("✓ Drone1 laser scan working")
    
    def drone2_laser_callback(self, msg):
        """Test drone2 laser scan"""
        self.test_results['drone2_laser'] = True
        rospy.loginfo("✓ Drone2 laser scan working")
    
    def drone0_relative_callback(self, msg):
        """Test drone0 relative poses"""
        self.test_results['drone0_relative'] = True
        rospy.loginfo("✓ Drone0 relative poses working")
    
    def drone1_relative_callback(self, msg):
        """Test drone1 relative poses"""
        self.test_results['drone1_relative'] = True
        rospy.loginfo("✓ Drone1 relative poses working")
    
    def drone0_cmd_callback(self, msg):
        """Test drone0 commands"""
        self.test_results['drone0_cmd'] = True
        rospy.loginfo("✓ Drone0 commands working")
    
    def drone1_cmd_callback(self, msg):
        """Test drone1 commands"""
        self.test_results['drone1_cmd'] = True
        rospy.loginfo("✓ Drone1 commands working")
    
    def run_tests(self, event):
        """Run all tests and report results"""
        rospy.loginfo("\n" + "="*50)
        rospy.loginfo("RANGE SYSTEM TEST RESULTS")
        rospy.loginfo("="*50)
        
        # Check each component
        components = [
            ('Drone0 Laser Scan', 'drone0_laser'),
            ('Drone1 Laser Scan', 'drone1_laser'),
            ('Drone2 Laser Scan', 'drone2_laser'),
            ('Drone0 Relative Poses', 'drone0_relative'),
            ('Drone1 Relative Poses', 'drone1_relative'),
            ('Drone0 Commands', 'drone0_cmd'),
            ('Drone1 Commands', 'drone1_cmd')
        ]
        
        all_passed = True
        for name, key in components:
            status = "✓ PASS" if self.test_results[key] else "✗ FAIL"
            rospy.loginfo(f"{name:25} : {status}")
            if not self.test_results[key]:
                all_passed = False
        
        rospy.loginfo("="*50)
        if all_passed:
            rospy.loginfo("🎉 ALL TESTS PASSED! Range system is working correctly.")
        else:
            rospy.loginfo("❌ SOME TESTS FAILED. Check the system setup.")
        
        rospy.loginfo("="*50 + "\n")
        
        # Stop testing after first run
        self.test_timer.stop()
        rospy.loginfo("Tests completed. Shutting down tester...")
        rospy.signal_shutdown("Tests completed")

def main():
    try:
        tester = RangeSystemTester()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
