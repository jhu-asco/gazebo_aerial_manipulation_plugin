#pragma once
#include <gazebo/common/common.hh>

/**
 * @brief achieves roll, pitch, yawrate, thrust commanded using PID controllers
 */
class RpytController
{
private:
  using GazeboTime = gazebo::common::Time;
  GazeboTime previous_sim_time_;///< Last simulation time
  double max_thrust_;///< Maximum thrust applied
  double min_thrust_;///< Minimum thrust applied
  double max_dt_;///< Maximum time diff allowed
  // Body XYZ format
  gazebo::common::PID roll_pid_controller_;///< PID for roll
  gazebo::common::PID pitch_pid_controller_;///< PID for pitch
  gazebo::common::PID yaw_pid_controller_;///< PID for yaw
public:
  struct BodyWrench {
    gazebo::math::Vector3 force;
    gazebo::math::Vector3 torque;
  };
public:
  RpytController(gazebo::math::Vector3 roll_pid_gains, gazebo::math::Vector3 pitch_pid_gains, gazebo::math::Vector3 yaw_pid_gains,
                 double max_torque, double max_thrust, double min_thrust=0.0);
  BodyWrench update(gazebo::math::Vector3 roll_pitch_yawrate, double thrust, GazeboTime sim_time);
};
