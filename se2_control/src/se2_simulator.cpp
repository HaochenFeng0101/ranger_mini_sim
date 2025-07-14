#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

class SE2Simulator
{
public:
  SE2Simulator()
    : x_(0.0), y_(0.0), yaw_(0.0)
    , vx_(0.0), vy_(0.0), vyaw_(0.0)
    , dt_(0.01)
  {
    ros::NodeHandle nh;
    
    // Publishers
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("/xray_source/odom", 1);
    
    // Subscribers
    cmd_vel_sub_ = nh.subscribe("/xray_source/cmd_vel", 1, &SE2Simulator::cmdVelCallback, this);
    
    // Timer for simulation
    timer_ = nh.createTimer(ros::Duration(dt_), &SE2Simulator::update, this);
    
    // TF broadcaster
    tf_broadcaster_ = new tf::TransformBroadcaster();
    
    ROS_INFO("SE2 Simulator initialized");
  }
  
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    // Store commanded velocities
    vx_cmd_ = msg->linear.x;
    vy_cmd_ = msg->linear.y;
    vyaw_cmd_ = msg->angular.z;
  }
  
  void update(const ros::TimerEvent& event)
  {
    // Simple vehicle dynamics simulation
    // Apply commanded velocities with some dynamics
    double alpha = 0.1; // Smoothing factor
    
    vx_ = vx_ * (1 - alpha) + vx_cmd_ * alpha;
    vy_ = vy_ * (1 - alpha) + vy_cmd_ * alpha;
    vyaw_ = vyaw_ * (1 - alpha) + vyaw_cmd_ * alpha;
    
    // Update position and orientation
    x_ += vx_ * dt_ * cos(yaw_) - vy_ * dt_ * sin(yaw_);
    y_ += vx_ * dt_ * sin(yaw_) + vy_ * dt_ * cos(yaw_);
    yaw_ += vyaw_ * dt_;
    
    // Normalize yaw to [-pi, pi]
    while (yaw_ > M_PI) yaw_ -= 2 * M_PI;
    while (yaw_ < -M_PI) yaw_ += 2 * M_PI;
    
    // Publish odometry
    publishOdometry();
    
    // Publish TF
    publishTF();
  }
  
  void publishOdometry()
  {
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";
    
    // Position
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    
    // Orientation
    tf::Quaternion q;
    q.setRPY(0, 0, yaw_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    
    // Velocity
    odom.twist.twist.linear.x = vx_;
    odom.twist.twist.linear.y = vy_;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = vyaw_;
    
    odom_pub_.publish(odom);
  }
  
  void publishTF()
  {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x_, y_, 0.0));
    
    tf::Quaternion q;
    q.setRPY(0, 0, yaw_);
    transform.setRotation(q);
    
    tf_broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
  }
  
private:
  // State variables
  double x_, y_, yaw_;
  double vx_, vy_, vyaw_;
  
  // Commanded velocities
  double vx_cmd_, vy_cmd_, vyaw_cmd_;
  
  // Simulation parameters
  double dt_;
  
  // ROS objects
  ros::Publisher odom_pub_;
  ros::Subscriber cmd_vel_sub_;
  ros::Timer timer_;
  tf::TransformBroadcaster* tf_broadcaster_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "se2_simulator");
  
  SE2Simulator simulator;
  
  ros::spin();
  
  return 0;
} 