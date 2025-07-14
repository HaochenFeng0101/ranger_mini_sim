#include <iostream>
#include <se2_control/SE2Control.h>

int main(int argc, char** argv)
{
  SE2Control controller;

  // Set vehicle parameters
  controller.setMass(15.0);
  controller.setWheelBase(0.46);
  controller.setTrack(0.412);
  controller.setWheelRadius(0.08);

  // Set current state
  controller.setPosition(Eigen::Vector2d(0.0, 0.0));
  controller.setVelocity(Eigen::Vector2d(0.0, 0.0));
  controller.setYaw(0.0);
  controller.setYawRate(0.0);

  // Set desired state
  Eigen::Vector2d des_pos(5.0, 2.0);
  Eigen::Vector2d des_vel(1.0, 0.0);
  double des_yaw = 0.5;
  double des_yaw_rate = 0.0;

  // Control gains
  Eigen::Vector2d kx(1.0, 1.0);
  double kyaw = 1.0;

  // Calculate control
  controller.calculateControl(des_pos, des_vel, des_yaw, des_yaw_rate, kx, kyaw);

  // Get control outputs
  const Eigen::Vector2d& velocity = controller.getComputedVelocity();
  const double& yaw_rate = controller.getComputedYawRate();
  const double& front_steering = controller.getComputedFrontSteeringAngle();
  const double& rear_steering = controller.getComputedRearSteeringAngle();

  std::cout << "Control outputs:" << std::endl;
  std::cout << "Velocity: [" << velocity(0) << ", " << velocity(1) << "]" << std::endl;
  std::cout << "Yaw rate: " << yaw_rate << std::endl;
  std::cout << "Front steering angle: " << front_steering << " rad (" << front_steering * 180.0 / M_PI << " deg)" << std::endl;
  std::cout << "Rear steering angle: " << rear_steering << " rad (" << rear_steering * 180.0 / M_PI << " deg)" << std::endl;

  return 0;
} 