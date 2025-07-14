#ifndef __SE2_CONTROL_H__
#define __SE2_CONTROL_H__

#include <Eigen/Geometry>
#include <Eigen/Dense>

class SE2Control
{
public:
  SE2Control();

  void setMass(const double mass);
  void setWheelBase(const double wheel_base);
  void setTrack(const double track);
  void setWheelRadius(const double wheel_radius);
  void setPosition(const Eigen::Vector2d& position);
  void setVelocity(const Eigen::Vector2d& velocity);
  void setYaw(const double yaw);
  void setYawRate(const double yaw_rate);

  void calculateControl(const Eigen::Vector2d& des_pos,
                        const Eigen::Vector2d& des_vel,
                        const double des_yaw,
                        const double des_yaw_rate,
                        const Eigen::Vector2d& kx,
                        const double kyaw);

  const Eigen::Vector2d& getComputedVelocity(void);
  const double& getComputedYawRate(void);
  const double& getComputedFrontSteeringAngle(void);
  const double& getComputedRearSteeringAngle(void);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  // Vehicle parameters
  double mass_;
  double wheel_base_;
  double track_;
  double wheel_radius_;

  // Current state
  Eigen::Vector2d pos_;
  Eigen::Vector2d vel_;
  double yaw_;
  double yaw_rate_;

  // Control outputs
  Eigen::Vector2d velocity_cmd_;
  double yaw_rate_cmd_;
  double front_steering_angle_;
  double rear_steering_angle_;

  // Helper functions
  Eigen::Vector2d computeVelocityError(const Eigen::Vector2d& des_vel);
  double computeYawError(const double des_yaw);
  void computeSteeringAngles(const Eigen::Vector2d& velocity, const double yaw_rate);
};

#endif 