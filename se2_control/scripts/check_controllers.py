#!/usr/bin/env python3

import rospy
import subprocess
import sys

def check_controller_status(robot_name):
    """Check if controllers are loaded and running for a robot"""
    try:
        # Check controller list
        result = subprocess.run([
            'rosservice', 'call', f'/{robot_name}/controller_manager/list_controllers'
        ], capture_output=True, text=True, timeout=5)
        
        if result.returncode == 0:
            output = result.stdout
            if 'four_wheel_steering_controller' in output and 'running' in output:
                print(f"✓ {robot_name}: Controllers loaded and running")
                return True
            else:
                print(f"✗ {robot_name}: Controllers not running")
                return False
        else:
            print(f"✗ {robot_name}: Could not check controller status")
            return False
    except Exception as e:
        print(f"✗ {robot_name}: Error checking controllers: {e}")
        return False

def check_odometry(robot_name):
    """Check if odometry is being published"""
    try:
        result = subprocess.run([
            'rostopic', 'echo', f'/{robot_name}/four_wheel_steering_controller/odom', '-n', '1'
        ], capture_output=True, text=True, timeout=5)
        
        if result.returncode == 0 and 'header:' in result.stdout:
            print(f"✓ {robot_name}: Odometry publishing")
            return True
        else:
            print(f"✗ {robot_name}: No odometry data")
            return False
    except Exception as e:
        print(f"✗ {robot_name}: Error checking odometry: {e}")
        return False

def main():
    rospy.init_node('check_controllers')
    
    robots = ['xray_source', 'bed_trolley', 'xray_detector']
    
    print("Checking controller status for all robots...")
    print("=" * 50)
    
    all_good = True
    
    for robot in robots:
        print(f"\nChecking {robot}:")
        controller_ok = check_controller_status(robot)
        odom_ok = check_odometry(robot)
        
        if not controller_ok or not odom_ok:
            all_good = False
    
    print("\n" + "=" * 50)
    if all_good:
        print("✓ All robots have controllers loaded and odometry publishing!")
        print("You can now send pose commands to the robots.")
    else:
        print("✗ Some robots have issues. Check the launch file and try again.")
        print("You may need to manually load controllers using:")
        print("  rosservice call /robot_name/controller_manager/load_controller 'name: \"four_wheel_steering_controller\"'")
        print("  rosservice call /robot_name/controller_manager/switch_controller 'start_controllers: [\"joint_state_controller\", \"four_wheel_steering_controller\"]'")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 