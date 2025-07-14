#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <four_wheel_steering_msgs/FourWheelSteering.h>
#include <four_wheel_steering_msgs/FourWheelSteeringStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <se2_control/SE2Control.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>

namespace se2_control {
class SE2ControlNodelet : public nodelet::Nodelet
{
public:
  SE2ControlNodelet()
    : position_cmd_updated_(false)
    , position_cmd_init_(false)
    , des_yaw_(0)
    , des_yaw_rate_(0)
    , current_yaw_(0)
    , enable_motors_(true)
    , use_external_yaw_(false)
  {
  }

  void onInit(void);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  void publishFourWheelSteeringCommand(void);
  void position_cmd_callback(const geometry_msgs::Pose2D::ConstPtr& cmd);
  void twist_cmd_callback(const geometry_msgs::Twist::ConstPtr& cmd);
  void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);
  void enable_motors_callback(const std_msgs::Bool::ConstPtr& msg);
  void imu_callback(const sensor_msgs::Imu& imu);

  SE2Control controller_;
  ros::Publisher four_wheel_steering_command_pub_;
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber position_cmd_sub_;
  ros::Subscriber twist_cmd_sub_;
  ros::Subscriber enable_motors_sub_;
  ros::Subscriber imu_sub_;

  bool position_cmd_updated_, position_cmd_init_;
  std::string frame_id_;

  Eigen::Vector2d des_pos_, des_vel_, kx_;
  double des_yaw_, des_yaw_rate_;
  double current_yaw_;
  bool enable_motors_;
  bool use_external_yaw_;
  double kyaw_;
  double init_x_, init_y_, init_z_;
};

void SE2ControlNodelet::publishFourWheelSteeringCommand(void)
{
  controller_.calculateControl(des_pos_, des_vel_, des_yaw_, des_yaw_rate_, kx_, kyaw_);

  const Eigen::Vector2d& velocity = controller_.getComputedVelocity();
  const double& yaw_rate = controller_.getComputedYawRate();
  const double& front_steering_angle = controller_.getComputedFrontSteeringAngle();
  const double& rear_steering_angle = controller_.getComputedRearSteeringAngle();

  // Publish FourWheelSteering command
  four_wheel_steering_msgs::FourWheelSteeringStamped::Ptr steering_cmd(
    new four_wheel_steering_msgs::FourWheelSteeringStamped);
  steering_cmd->header.stamp = ros::Time::now();
  steering_cmd->header.frame_id = frame_id_;
  steering_cmd->data.front_steering_angle = front_steering_angle;
  steering_cmd->data.rear_steering_angle = rear_steering_angle;
  steering_cmd->data.front_steering_angle_velocity = 0.0;  // Could be computed from previous values
  steering_cmd->data.rear_steering_angle_velocity = 0.0;
  steering_cmd->data.speed = velocity.norm();
  steering_cmd->data.acceleration = 0.0;  // Could be computed from velocity difference
  steering_cmd->data.jerk = 0.0;
  four_wheel_steering_command_pub_.publish(steering_cmd);

  // Also publish as Twist for compatibility
  geometry_msgs::Twist::Ptr twist_cmd(new geometry_msgs::Twist);
  twist_cmd->linear.x = velocity(0);
  twist_cmd->linear.y = velocity(1);
  twist_cmd->angular.z = yaw_rate;
  cmd_vel_pub_.publish(twist_cmd);
}

void SE2ControlNodelet::position_cmd_callback(const geometry_msgs::Pose2D::ConstPtr& cmd)
{
  des_pos_ = Eigen::Vector2d(cmd->x, cmd->y);
  des_yaw_ = cmd->theta;
  des_vel_ = Eigen::Vector2d::Zero();  // No velocity command in Pose2D
  des_yaw_rate_ = 0.0;
  
  position_cmd_updated_ = true;
  position_cmd_init_ = true;

  publishFourWheelSteeringCommand();
}

void SE2ControlNodelet::twist_cmd_callback(const geometry_msgs::Twist::ConstPtr& cmd)
{
  des_vel_ = Eigen::Vector2d(cmd->linear.x, cmd->linear.y);
  des_yaw_rate_ = cmd->angular.z;
  
  // Keep current position and yaw targets
  position_cmd_updated_ = true;
  position_cmd_init_ = true;

  publishFourWheelSteeringCommand();
}

void SE2ControlNodelet::odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
  const Eigen::Vector2d position(odom->pose.pose.position.x,
                                 odom->pose.pose.position.y);
  const Eigen::Vector2d velocity(odom->twist.twist.linear.x,
                                 odom->twist.twist.linear.y);

  current_yaw_ = tf::getYaw(odom->pose.pose.orientation);
  double current_yaw_rate = odom->twist.twist.angular.z;

  controller_.setPosition(position);
  controller_.setVelocity(velocity);
  controller_.setYaw(current_yaw_);
  controller_.setYawRate(current_yaw_rate);

  if (position_cmd_init_)
  {
    if (!position_cmd_updated_)
      publishFourWheelSteeringCommand();
    position_cmd_updated_ = false;
  }
  else if (init_z_ > -9999.0)
  {
    des_pos_ = Eigen::Vector2d(init_x_, init_y_);
    des_vel_ = Eigen::Vector2d(0, 0);
    des_yaw_ = 0.0;
    des_yaw_rate_ = 0.0;
    publishFourWheelSteeringCommand();
  }
}

void SE2ControlNodelet::enable_motors_callback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data)
    ROS_INFO("Enabling motors");
  else
    ROS_INFO("Disabling motors");

  enable_motors_ = msg->data;
}

void SE2ControlNodelet::imu_callback(const sensor_msgs::Imu& imu)
{
  // Could use IMU data for better state estimation
  // For now, we'll rely on odometry
}

void SE2ControlNodelet::onInit(void)
{
  ros::NodeHandle n(getPrivateNodeHandle());

  std::string vehicle_name;
  n.param("vehicle_name", vehicle_name, std::string("xray_source"));
  frame_id_ = "/" + vehicle_name;

  double mass;
  n.param("mass", mass, 15.0);
  controller_.setMass(mass);

  double wheel_base;
  n.param("wheel_base", wheel_base, 0.46);
  controller_.setWheelBase(wheel_base);

  double track;
  n.param("track", track, 0.412);
  controller_.setTrack(track);

  double wheel_radius;
  n.param("wheel_radius", wheel_radius, 0.08);
  controller_.setWheelRadius(wheel_radius);

  n.param("use_external_yaw", use_external_yaw_, true);

  // Control gains
  double kx_x, kx_y, kyaw;
  n.param("kx_x", kx_x, 1.0);
  n.param("kx_y", kx_y, 1.0);
  n.param("kyaw", kyaw, 1.0);
  kx_ = Eigen::Vector2d(kx_x, kx_y);
  kyaw_ = kyaw;

  // Initial position
  n.param("init_x", init_x_, 0.0);
  n.param("init_y", init_y_, 0.0);
  n.param("init_z", init_z_, -9999.0);

  // Publishers
  four_wheel_steering_command_pub_ = n.advertise<four_wheel_steering_msgs::FourWheelSteeringStamped>(
    "four_wheel_steering_command", 1);
  cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // Subscribers
  odom_sub_ = n.subscribe("odom", 1, &SE2ControlNodelet::odom_callback, this);
  position_cmd_sub_ = n.subscribe("position_cmd", 1, &SE2ControlNodelet::position_cmd_callback, this);
  twist_cmd_sub_ = n.subscribe("twist_cmd", 1, &SE2ControlNodelet::twist_cmd_callback, this);
  enable_motors_sub_ = n.subscribe("enable_motors", 1, &SE2ControlNodelet::enable_motors_callback, this);
  imu_sub_ = n.subscribe("imu", 1, &SE2ControlNodelet::imu_callback, this);

  ROS_INFO("SE2 Control initialized for %s", vehicle_name.c_str());
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(se2_control::SE2ControlNodelet, nodelet::Nodelet) 