#include <gazebo_aerial_manipulation_plugin/rpytcontroller.h>

using namespace gazebo;

RpytController::RpytController(math::Vector3 roll_pid_gains, math::Vector3 pitch_pid_gains, math::Vector3 yaw_pid_gains, double max_torque, double max_thrust, double min_thrust): previous_sim_time_(0.0)
  , roll_pid_controller_(roll_pid_gains.x, roll_pid_gains.y, roll_pid_gains.z, max_torque/2.0, -max_torque/2.0, max_torque, -max_torque)
  , pitch_pid_controller_(pitch_pid_gains.x, pitch_pid_gains.y, pitch_pid_gains.z, max_torque/2.0, -max_torque/2.0, max_torque, -max_torque)
  , yaw_pid_controller_(yaw_pid_gains.x, yaw_pid_gains.y, yaw_pid_gains.z, max_torque/2.0, -max_torque/2.0, max_torque, -max_torque)
  , max_thrust_(max_thrust)
  , min_thrust_(min_thrust)
  , max_dt_(1.0)
{
}

RpytController::BodyWrench RpytController::update(math::Vector3 error_roll_pitch_yawrate, double thrust, common::Time sim_time) {
  RpytController::BodyWrench body_wrench;
  common::Time dt = sim_time - previous_sim_time_;
  previous_sim_time_ = sim_time;
  if(dt > max_dt_) {
    gzwarn<<"Time difference between previous and current sim time is larger than 1 second: "<<previous_sim_time_.Double()<<", "<<sim_time.Double()<<std::endl;
    body_wrench.torque = math::Vector3::Zero;
    roll_pid_controller_.Reset();
    pitch_pid_controller_.Reset();
    yaw_pid_controller_.Reset();
  } else if(dt < 0) {
    gzwarn<<"Time difference between previous and current sim time is negative: "<<previous_sim_time_.Double()<<", "<<sim_time.Double()<<std::endl;
    body_wrench.torque = math::Vector3::Zero;
  } else {
    body_wrench.torque.x = roll_pid_controller_.Update(error_roll_pitch_yawrate.x, dt);
    body_wrench.torque.y = pitch_pid_controller_.Update(error_roll_pitch_yawrate.y, dt);
    body_wrench.torque.z = yaw_pid_controller_.Update(error_roll_pitch_yawrate.z, dt);
  }
  body_wrench.force.x = body_wrench.force.y = 0;
  body_wrench.force.z = thrust>max_thrust_?max_thrust_:(thrust<min_thrust_)?min_thrust_:thrust;
  return body_wrench;
}
