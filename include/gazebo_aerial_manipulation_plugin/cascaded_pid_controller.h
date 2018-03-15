#pragma once
#include <gazebo/common/common.hh>
#include <gazebo_aerial_manipulation_plugin/atomic.h>

/**
 * @brief achieves commanded joint angle using a cascaded P-PI controller
 * where position is used to compute desired velocity and the desired
 * velocity is used to achieve a commanded Force using PI controller
 */
class CascadedPIDController
{
private:
  using GazeboTime = gazebo::common::Time;
  GazeboTime previous_sim_time_;///< Last simulation time
  double max_dt_;///< Maximum time diff allowed
  double max_command_;///< Maximum allowed command
  double max_velocity_;///< Maximum allowed desired velocity
  double max_i_error_;///< Maximum allowed integrator contribution
  double p_gain_;///< Gain for computing desired velocity
  double i_gain_;///< Gain for computing integral error on velocity
  double d_gain_;///< Gain on velocity error to compute force
  Atomic<double> i_error_;///< Integral error

public:
  /**
  * @brief Initialize the controller with the specified gains
  *
  * @param p_gain The gain used to compute the desired velocity
  * @param i_gain  The gain used to integrate velocity error
  * @param d_gain  The derivative gain used to compute force based on desired velocity
  * @param max_command Maximum command
  * @param max_vel Maximum desired velocity
  * @param max_i_error Maximum allowed force due to integrator
  */
  CascadedPIDController(double p_gain, double i_gain, double d_gain, double max_command, double max_vel, double max_i_error);
  /**
  * @brief Update the P-PI controller based on input goal, position, velocity
  * and sim time
  *
  * @param goal Goal position
  * @param x Current position
  * @param xdot Current velocity
  * @param sim_time Current time (Used by integrator)
  *
  * @return commanded force
  */
  double update(double goal, double x, double xdot, GazeboTime sim_time);
  /**
  * @brief Resets the integral error for the controller
  */
  void reset() {
    i_error_ = 0.0;
  }
  /**
  * @brief Clamp the input between max and min
  *
  * @param in input value
  * @param max Max value
  * @param min Min value
  *
  * @return clamped value
  */
  double clamp(double in, double max, double min) {
    return (in > max)?max:(in < min)?min:in;
  }
};
