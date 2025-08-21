#!/usr/bin/env python3
import rospy
import sys
import tty
import termios
import threading
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class TargetManualController:
    """Interactive keyboard controller for the target drone"""
    
    def __init__(self):
        rospy.init_node('target_manual_controller')
        
        # Publisher for target velocity commands
        self.target_pub = rospy.Publisher('/target_manual_twist', Twist, queue_size=1)
        self.status_pub = rospy.Publisher('/target_controller_status', String, queue_size=1)
        
        # Control parameters
        self.linear_vel = 0.5      # Linear velocity (m/s)
        self.angular_vel = 0.3     # Angular velocity (rad/s)
        self.vel_step = 0.1        # Velocity increment step
        
        # Current velocities
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vyaw = 0.0
        
        # Control flags
        self.running = True
        self.control_thread = None
        self.manual_control_active = False # Flag to indicate if manual control is currently active
        
        # Setup keyboard input
        self.setup_keyboard()
        
        rospy.loginfo("Target Manual Controller initialized")
        rospy.loginfo("Use WASD keys to move, QE to rotate, Space to stop")
        rospy.loginfo("Press 'h' for help, 'q' to quit")
        
        # Start control thread
        self.control_thread = threading.Thread(target=self.keyboard_control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        # Main loop
        self.main_loop()
    
    def setup_keyboard(self):
        """Setup keyboard for non-blocking input"""
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setraw(sys.stdin.fileno())
    
    def restore_keyboard(self):
        """Restore keyboard settings"""
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
    
    def get_key(self):
        """Get a single key press"""
        try:
            return sys.stdin.read(1)
        except:
            return None
    
    def process_key(self, key):
        """Process keyboard input and update velocities"""
        if key == 'w' or key == 'W':
            self.current_vx = self.linear_vel
            self.manual_control_active = True
            rospy.loginfo("Moving target FORWARD")
        elif key == 's' or key == 'S':
            self.current_vx = -self.linear_vel
            self.manual_control_active = True
            rospy.loginfo("Moving target BACKWARD")
        elif key == 'a' or key == 'A':
            self.current_vy = self.linear_vel
            self.manual_control_active = True
            rospy.loginfo("Moving target LEFT")
        elif key == 'd' or key == 'D':
            self.current_vy = -self.linear_vel
            self.manual_control_active = True
            rospy.loginfo("Moving target RIGHT")
        elif key == 'q' or key == 'Q':
            self.current_vyaw = self.angular_vel
            self.manual_control_active = True
            rospy.loginfo("Rotating target LEFT")
        elif key == 'e' or key == 'E':
            self.current_vyaw = -self.angular_vel
            self.manual_control_active = True
            rospy.loginfo("Rotating target RIGHT")
        elif key == ' ' or key == 'x' or key == 'X':
            self.stop_target()
            self.manual_control_active = False
            rospy.loginfo("Target STOPPED")
        elif key == 'h' or key == 'H':
            self.show_help()
        elif key == 'r' or key == 'R':
            self.reset_target()
            rospy.loginfo("Target position RESET")
        elif key == '+':
            self.linear_vel = min(2.0, self.linear_vel + self.vel_step)
            rospy.loginfo(f"Linear velocity increased to {self.linear_vel:.1f} m/s")
        elif key == '-':
            self.linear_vel = max(0.1, self.linear_vel - self.vel_step)
            rospy.loginfo(f"Linear velocity decreased to {self.linear_vel:.1f} m/s")
        elif key == '1':
            self.linear_vel = 0.2
            rospy.loginfo("Linear velocity set to 0.2 m/s (SLOW)")
        elif key == '2':
            self.linear_vel = 0.5
            rospy.loginfo("Linear velocity set to 0.5 m/s (NORMAL)")
        elif key == '3':
            self.linear_vel = 1.0
            rospy.loginfo("Linear velocity set to 1.0 m/s (FAST)")
    
    def stop_target(self):
        """Stop all target movement"""
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vyaw = 0.0
    
    def reset_target(self):
        """Reset target to initial position (this would need to be implemented)"""
        # For now, just stop movement
        self.stop_target()
        rospy.loginfo("Note: Position reset not implemented - just stopped movement")
    
    def show_help(self):
        """Show control help"""
        help_text = """
=== TARGET MANUAL CONTROLLER HELP ===
Movement Controls:
  W - Move FORWARD
  S - Move BACKWARD  
  A - Move LEFT
  D - Move RIGHT
  Q - Rotate LEFT
  E - Rotate RIGHT
  SPACE/X - STOP all movement

Speed Controls:
  +/- - Increase/decrease linear velocity
  1/2/3 - Set velocity to SLOW/NORMAL/FAST
  R - Reset target position
  H - Show this help
  Q - Quit controller

Current Settings:
  Linear velocity: {:.1f} m/s
  Angular velocity: {:.1f} rad/s
=====================================
""".format(self.linear_vel, self.angular_vel)
        
        rospy.loginfo(help_text)
    
    def keyboard_control_loop(self):
        """Main keyboard control loop"""
        rospy.loginfo("Keyboard control active - press keys to control target")
        
        while self.running and not rospy.is_shutdown():
            key = self.get_key()
            if key:
                if key == '\x1b':  # ESC key
                    break
                elif key == '\x03':  # Ctrl+C
                    break
                else:
                    self.process_key(key)
            
            rospy.sleep(0.01)  # Small delay to prevent high CPU usage
    
    def publish_velocities(self):
        """Publish current velocities to target"""
        # Only publish if there's actual movement or if we need to stop
        if (abs(self.current_vx) > 0.001 or abs(self.current_vy) > 0.001 or 
            abs(self.current_vyaw) > 0.001 or self.manual_control_active):
            
            twist = Twist()
            twist.linear.x = self.current_vx
            twist.linear.y = self.current_vy
            twist.angular.z = self.current_vyaw
            
            self.target_pub.publish(twist)
            
            # Publish status
            status_msg = String()
            status_msg.data = f"vx:{self.current_vx:.2f}, vy:{self.current_vy:.2f}, vyaw:{self.current_vyaw:.2f}"
            self.status_pub.publish(status_msg)
            
            # If we're stopping and no movement, deactivate manual control
            if (abs(self.current_vx) < 0.001 and abs(self.current_vy) < 0.001 and 
                abs(self.current_vyaw) < 0.001):
                self.manual_control_active = False
    
    def main_loop(self):
        """Main control loop"""
        rate = rospy.Rate(20)  # 20Hz control loop
        
        try:
            while not rospy.is_shutdown() and self.running:
                self.publish_velocities()
                rate.sleep()
                
        except KeyboardInterrupt:
            rospy.loginfo("Interrupted by user")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources"""
        rospy.loginfo("Cleaning up...")
        self.running = False
        
        # Stop target
        self.stop_target()
        self.publish_velocities()
        
        # Restore keyboard
        self.restore_keyboard()
        
        rospy.loginfo("Target Manual Controller shutdown complete")

def main():
    try:
        controller = TargetManualController()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error in Target Manual Controller: {e}")
        # Restore keyboard settings
        try:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, termios.tcgetattr(sys.stdin.fileno()))
        except:
            pass

if __name__ == '__main__':
    main()
