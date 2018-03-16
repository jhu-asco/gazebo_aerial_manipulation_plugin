#include <gazebo_aerial_manipulation_plugin/cascaded_pid_controller.h>

using namespace gazebo;

CascadedPIDController::CascadedPIDController(double p_gain, double i_gain, double d_gain, double max_command, double max_vel, double max_i_error): previous_sim_time_(0.0)
                                                                                                                      , max_dt_(1.0)
                                                                                                                      , max_command_(max_command)
                                                                                                                      , max_velocity_(max_vel)
                                                                                                                      , max_i_error_(max_i_error)
                                                                                                                      , p_gain_(p_gain)
                                                                                                                      , i_gain_(i_gain)
                                                                                                                      , d_gain_(d_gain)
                                                                                                                      , i_error_(0.0)
{
}

double CascadedPIDController::update(double pd, double p, double v, common::Time sim_time) {
  double out_command = 0.0;
  common::Time dt = sim_time - previous_sim_time_;
  if(dt > max_dt_) {
    gzwarn<<"Time difference between previous and current sim time is larger than 1 second: "<<previous_sim_time_.Double()<<", "<<sim_time.Double()<<std::endl;
  } else {
    // Compute vd
    double vd = -p_gain_*(p - pd);
    // Clamp vd
    vd = clamp(vd, max_velocity_, -max_velocity_);
    // Compute v_error
    double v_error = v - vd;
    // Integrate v_error
    double i_error = i_error_.get() - i_gain_*v_error*dt.Double();
    i_error = clamp(i_error, max_i_error_, -max_i_error_);
    i_error_.set(i_error);
    // Compute and return force
    out_command = -d_gain_*v_error + i_error;
    out_command = clamp(out_command, max_command_, -max_command_);
  }
  // Update time
  previous_sim_time_ = sim_time;
  return out_command;
}
