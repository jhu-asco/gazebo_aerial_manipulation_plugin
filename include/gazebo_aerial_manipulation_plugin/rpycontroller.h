#pragma once
#include <gazebo/common/common.hh>

/**
 * @brief achieves roll, pitch, yawrate, thrust commanded using PID controllers
 */
class RpyController
{
private:
  using GazeboTime = gazebo::common::Time;
  GazeboTime previous_sim_time_;///< Last simulation time
  double max_dt_;///< Maximum time diff allowed
  math::Vector3 max_torque_;///< Maximum allowed torque
  math::Vector3 min_torque_;///< Maximum allowed torque
  math::Vector3 p_gains_;
  math::Vector3 i_gains_;
  math::Vector3 d_gains_;
  math::Vector3 rpy_command_prev_;
  math::Vector3 i_error_;
  math::Vector3 prev_body_torque_;
  bool invalid_prev_command_;
public:
  RpyController(math::Vector3 p_gains, math::Vector3 i_gains, math::Vector3 d_gains, double max_torque);
  gazebo::math::Vector3 update(gazebo::math::Vector3 rpy_command, math::Quaternion orientation, math::Vector3 omega, GazeboTime sim_time);
  gazebo::math::Vector3 rpydotToOmega(math::Vector3 rpy, math::Vector3 rpydot);
  gazebo::math::Vector3 omegaToRpydot(math::Vector3 rpy, math::Vector3 omega);
  void reset() {
    invalid_prev_command_ = true;
    i_error_ = gazebo::math::Vector3::Zero;
  }
};
