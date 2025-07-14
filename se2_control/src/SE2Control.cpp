#include <iostream>
#include <cmath>
#include <se2_control/SE2Control.h>

#include <ros/ros.h>

SE2Control::SE2Control()
  : mass_(15.0)
  , wheel_base_(0.46)  // Based on your X-ray source model
  , track_(0.412)      // Based on your X-ray source model
  , wheel_radius_(0.08) // Based on your X-ray source model
  , yaw_(0.0)
  , yaw_rate_(0.0)
{
  pos_.setZero();
  vel_.setZero();
  velocity_cmd_.setZero();
  yaw_rate_cmd_ = 0.0;
  front_steering_angle_ = 0.0;
  rear_steering_angle_ = 0.0;
}

void SE2Control::setMass(const double mass)
{
  mass_ = mass;
}

void SE2Control::setWheelBase(const double wheel_base)
{
  wheel_base_ = wheel_base;
}

void SE2Control::setTrack(const double track)
{
  track_ = track;
}

void SE2Control::setWheelRadius(const double wheel_radius)
{
  wheel_radius_ = wheel_radius;
}

void SE2Control::setPosition(const Eigen::Vector2d& position)
{
  pos_ = position;
}

void SE2Control::setVelocity(const Eigen::Vector2d& velocity)
{
  vel_ = velocity;
}

void SE2Control::setYaw(const double yaw)
{
  yaw_ = yaw;
}

void SE2Control::setYawRate(const double yaw_rate)
{
  yaw_rate_ = yaw_rate;
}

void SE2Control::calculateControl(const Eigen::Vector2d& des_pos,
                                  const Eigen::Vector2d& des_vel,
                                  const double des_yaw,
                                  const double des_yaw_rate,
                                  const Eigen::Vector2d& kx,
                                  const double kyaw)
{
  // Check for NaN values
  bool flag_use_pos = !(std::isnan(des_pos(0)) || std::isnan(des_pos(1)));
  bool flag_use_vel = !(std::isnan(des_vel(0)) || std::isnan(des_vel(1)));
  bool flag_use_yaw = !std::isnan(des_yaw);
  bool flag_use_yaw_rate = !std::isnan(des_yaw_rate);

  // Position error in global frame
  Eigen::Vector2d pos_error = Eigen::Vector2d::Zero();
  if (flag_use_pos) {
    pos_error = des_pos - pos_;
  }

  // Transform position error to body frame
  Eigen::Matrix2d R_body;
  R_body << cos(yaw_), -sin(yaw_),
             sin(yaw_), cos(yaw_);
  Eigen::Vector2d pos_error_body = R_body.transpose() * pos_error;

  // Velocity error in body frame
  Eigen::Vector2d vel_error_body = Eigen::Vector2d::Zero();
  if (flag_use_vel) {
    Eigen::Vector2d des_vel_body = R_body.transpose() * des_vel;
    vel_error_body = des_vel_body - vel_;
  }

  // Yaw error (normalized to [-pi, pi])
  double yaw_error = 0.0;
  if (flag_use_yaw) {
    yaw_error = computeYawError(des_yaw);
  }

  // Yaw rate error
  double yaw_rate_error = 0.0;
  if (flag_use_yaw_rate) {
    yaw_rate_error = des_yaw_rate - yaw_rate_;
  }

  // Compute control outputs
  // Longitudinal velocity control
  double vx_cmd = 0.0;
  if (flag_use_pos) {
    vx_cmd += kx(0) * pos_error_body(0);
  }
  if (flag_use_vel) {
    vx_cmd += vel_error_body(0);
  }

  // Lateral velocity control (for lateral position tracking)
  double vy_cmd = 0.0;
  if (flag_use_pos) {
    vy_cmd += kx(1) * pos_error_body(1);
  }
  if (flag_use_vel) {
    vy_cmd += vel_error_body(1);
  }

  // Yaw rate control
  double yaw_rate_cmd = 0.0;
  if (flag_use_yaw) {
    yaw_rate_cmd += kyaw * yaw_error;
  }
  if (flag_use_yaw_rate) {
    yaw_rate_cmd += yaw_rate_error;
  }

  // Set velocity command in body frame
  velocity_cmd_ << vx_cmd, vy_cmd;
  yaw_rate_cmd_ = yaw_rate_cmd;

  // Compute steering angles based on desired velocity and yaw rate
  computeSteeringAngles(velocity_cmd_, yaw_rate_cmd_);
}

Eigen::Vector2d SE2Control::computeVelocityError(const Eigen::Vector2d& des_vel)
{
  return des_vel - vel_;
}

double SE2Control::computeYawError(const double des_yaw)
{
  double error = des_yaw - yaw_;
  
  // Normalize to [-pi, pi]
  while (error > M_PI) error -= 2 * M_PI;
  while (error < -M_PI) error += 2 * M_PI;
  
  return error;
}

void SE2Control::computeSteeringAngles(const Eigen::Vector2d& velocity, const double yaw_rate)
{
  double vx = velocity(0);
  double vy = velocity(1);
  double v = sqrt(vx * vx + vy * vy);
  
  if (v < 0.01) {
    // If velocity is very small, set steering angles to zero
    front_steering_angle_ = 0.0;
    rear_steering_angle_ = 0.0;
    return;
  }

  // For a four-wheel steering vehicle, we can use different strategies
  // Here we use a simple approach where both front and rear wheels steer
  // to achieve the desired yaw rate
  
  // Calculate the required steering angles based on bicycle model
  // For four-wheel steering, we can distribute the steering between front and rear
  
  double beta = atan2(vy, vx);  // sideslip angle
  double desired_curvature = yaw_rate / v;
  
  // Front steering angle (positive is left turn)
  front_steering_angle_ = atan2(wheel_base_ * desired_curvature, 1.0) + beta;
  
  // Rear steering angle (negative for counter-steering effect)
  rear_steering_angle_ = -atan2(wheel_base_ * desired_curvature * 0.5, 1.0) + beta * 0.5;
  
  // Limit steering angles to reasonable values
  double max_steering_angle = 0.5;  // ~28.6 degrees
  front_steering_angle_ = std::max(-max_steering_angle, std::min(max_steering_angle, front_steering_angle_));
  rear_steering_angle_ = std::max(-max_steering_angle, std::min(max_steering_angle, rear_steering_angle_));
}

const Eigen::Vector2d& SE2Control::getComputedVelocity(void)
{
  return velocity_cmd_;
}

const double& SE2Control::getComputedYawRate(void)
{
  return yaw_rate_cmd_;
}

const double& SE2Control::getComputedFrontSteeringAngle(void)
{
  return front_steering_angle_;
}

const double& SE2Control::getComputedRearSteeringAngle(void)
{
  return rear_steering_angle_;
} 