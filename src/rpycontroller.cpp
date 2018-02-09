#include <gazebo_aerial_manipulation_plugin/rpycontroller.h>

using namespace gazebo;

RpyController::RpyController(math::Vector3 p_gains, math::Vector3 i_gains, math::Vector3 d_gains, double max_torque): previous_sim_time_(0.0)
                                                                                                                      , max_dt_(1.0)
                                                                                                                      , max_torque_(max_torque, max_torque, max_torque)
                                                                                                                      , min_torque_(-1*max_torque_)
                                                                                                                      , p_gains_(p_gains)
                                                                                                                      , i_gains_(i_gains)
                                                                                                                      , d_gains_(d_gains)
                                                                                                                      , rpy_command_prev_(math::Vector3::Zero)
                                                                                                                      , i_error_(math::Vector3::Zero)
                                                                                                                      , prev_body_torque_(math::Vector3::Zero)
                                                                                                                      , invalid_prev_command_(true)
{
}

math::Vector3 RpytController::rpydotToOmega(math::Vector3 rpy, math::Vector3 rpydot) {
  double s_roll = sin(rpy.x), c_roll = cos(rpy.x);
  double s_pitch = sin(rpy.y), c_pitch = cos(rpy.y);
  math::Matrix M(1, 0,          -s_pitch,
                 0, c_roll,  c_pitch*s_roll,
                 0, -s_roll, c_pitch*c_roll);
  return M*rpydot;
}

math::Vector3 RpytController::omegaToRpydot(math::Vector3 rpy, math::Vector3 omega) {
  double s_roll = sin(rpy.x), c_roll = cos(rpy.x);
  double s_pitch = sin(rpy.y), c_pitch = cos(rpy.y);
  double t_pitch = s_pitch/c_pitch;
  math::Matrix M(1, s_roll*t_pitch,  c_roll*t_pitch,
                 0, c_roll,  -s_roll,
                 0, -s_roll/c_pitch, c_roll/c_pitch);
  return M*omega;
}

math::Vector3 RpyController::update(math::Vector3 rpy_command, math::Quaternion orientation, math::Vector3 omega, common::Time sim_time) {
  math::Vector3 out_torque;
  common::Time dt = sim_time - previous_sim_time_;
  if(dt > max_dt_) {
    gzwarn<<"Time difference between previous and current sim time is larger than 1 second: "<<previous_sim_time_.Double()<<", "<<sim_time.Double()<<std::endl;
    out_torque = math::Vector3::Zero;
  } else if (dt < 1e-4) {
    out_torque = prev_body_torque_;
  }else {
    // Compute angle error
    // quat = (R.T Rd)
    desired_orientation = math::Quaternion::EulerToQuaternion(rpy_command);
    // quat -> angle*theta*p_gains gives p error
    error_orientation = orientation.GetInverse()*desired_orientation;
    math::Vector3 error_axis;
    double error_angle;
    error_orientation.GetAsAxis(error_axis, error_angle);
    math::Vector3 p_error = (error_axis*p_gains_)*error_angle;
    // omega_d = M(rpy)*rpydot
    math::Vector3 d_error = math::Vector3::Zero;
    if(!invalid_prev_command_) {
      math::Vector3 rpyd_dot = (rpy_command - rpy_command_prev_)*(1.0/dt);
      math::Vector3 omega_d = rpydotToOmega(rpy_command_prev_, rpyd_dot);
      d_error = d_gains_*(omega_d - omega);
    }
    // integral in terms of angle error
    i_error_ += p_error*dt*i_gains_;
    // Clamp integral error
    i_error_ = math::clamp(i_error_, 0.1*min_torque_, 0.1*max_torque_);
    // evaluate pid output
    out_torque = p_error + i_error_ + d_error;
    // clamp between min and max
    out_torque = math::clamp(out_torque, min_torque_, max_torque_);
  }
  // Update previous commands and time
  rpy_command_prev_ = rpy_command;
  previous_sim_time_ = sim_time;
  prev_body_torque_ = out_torque;
  invalid_prev_command_ = false;
  return out_torque;
}
