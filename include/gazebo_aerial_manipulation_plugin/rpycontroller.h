#pragma once
#include <gazebo/common/common.hh>
#include <gazebo_aerial_manipulation_plugin/atomic.h>

/**
 * @brief achieves roll, pitch, yawrate, thrust commanded using PID controllers
 */
class RpyController
{
private:
  using GazeboTime = gazebo::common::Time;
  using GazeboVector = gazebo::math::Vector3;
  GazeboTime previous_sim_time_;///< Last simulation time
  double max_dt_;///< Maximum time diff allowed
  double max_torque_;///< Maximum allowed torque
  GazeboVector p_gains_;
  GazeboVector i_gains_;
  GazeboVector d_gains_;
  GazeboVector rpy_command_prev_;
  Atomic<GazeboVector> i_error_;
  GazeboVector prev_body_torque_;
  bool invalid_prev_command_;
  GazeboVector clampVector(GazeboVector vector, double ub, double lb) {
    GazeboVector out;
    out.x = vector.x > ub? ub: vector.x < lb?lb:vector.x;
    out.y = vector.y > ub? ub: vector.y < lb?lb:vector.y;
    out.z = vector.z > ub? ub: vector.z < lb?lb:vector.z;
    return out;
  }

public:
  RpyController(GazeboVector p_gains, GazeboVector i_gains, GazeboVector d_gains, double max_torque);
  GazeboVector update(GazeboVector rpy_command, gazebo::math::Quaternion orientation, GazeboVector omega, GazeboTime sim_time);
  GazeboVector rpydotToOmega(GazeboVector rpy, GazeboVector rpydot);
  GazeboVector omegaToRpydot(GazeboVector rpy, GazeboVector omega);
  void reset() {
    invalid_prev_command_ = true;
    i_error_ = GazeboVector::Zero;
  }
};
