/**
   GazeboAerialManipulation plugin for manipulating objects in Gazebo
   Author: Modified by Gowtham.
   Date: 7th Feb 2018
 */
#include <ros/ros.h>
#include <algorithm>
#include <assert.h>

#include <gazebo_aerial_manipulation_plugin/gazebo_aerial_manipulation_plugin.h>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GazeboAerialManipulation)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboAerialManipulation::GazeboAerialManipulation() : p_gains_(5,5,5)
, i_gains_(0,0,0)
, d_gains_(5,5,5)
, kt_(0.16)
, max_torque_(10)
, pose_subscriber_count_(0)
, rpy_controller_(p_gains_, i_gains_, d_gains_, max_torque_)
{
  gazebo_aerial_manipulation_plugin::RollPitchYawThrust rpyt_msg;
  rpyt_msg.roll = 0;
  rpyt_msg.pitch = 0;
  rpyt_msg.yaw = 0;
  rpyt_msg.thrust = 0;
  this->rpyt_msg_ = rpyt_msg;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboAerialManipulation::~GazeboAerialManipulation()
{
  this->update_connection_.reset();

  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();
  this->publish_pose_thread_.join();

  delete this->rosnode_;
}

void GazeboAerialManipulation::LoadRPYController(sdf::ElementPtr _sdf) {
  if(_sdf->HasElement("roll_pid_gains")) {
    auto r_pid_gains = _sdf->GetElement("roll_pid_gains");
    p_gains_.x = r_pid_gains->Get<double>("p");
    i_gains_.x = r_pid_gains->Get<double>("i");
    d_gains_.x = r_pid_gains->Get<double>("d");
  }
  if(_sdf->HasElement("pitch_pid_gains")) {
    auto p_pid_gains = _sdf->GetElement("pitch_pid_gains");
    p_gains_.y = p_pid_gains->Get<double>("p");
    i_gains_.y = p_pid_gains->Get<double>("i");
    d_gains_.y = p_pid_gains->Get<double>("d");
  }
  if(_sdf->HasElement("yaw_pid_gains")) {
    auto y_pid_gains = _sdf->GetElement("yaw_pid_gains");
    p_gains_.z = y_pid_gains->Get<double>("p");
    i_gains_.z = y_pid_gains->Get<double>("i");
    d_gains_.z = y_pid_gains->Get<double>("d");
  }
  if(_sdf->HasElement("max_torque")) {
    max_torque_ = _sdf->GetElement("max_torque")->Get<double>();
  }
  rpy_controller_ = RpyController(p_gains_, i_gains_, d_gains_, max_torque_);
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboAerialManipulation::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->world_ = _model->GetWorld();

  // Store the model pointer
  this->model_ = _model;

  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("bodyName"))
  {
    ROS_FATAL_NAMED("force", "force plugin missing <bodyName>, cannot proceed");
    return;
  }
  else
    this->link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();

  this->link_ = _model->GetLink(this->link_name_);
  if (!this->link_)
  {
    ROS_FATAL_NAMED("force", "aerial_manipulation_plugin plugin error: link named: %s does not exist\n",this->link_name_.c_str());
    return;
  }
  // Get rpy controller
  LoadRPYController(_sdf);

  // Get Thrust gain
  if(_sdf->HasElement("thrust_gain")) {
    kt_ = _sdf->GetElement("thrust_gain")->Get<double>();
  }

  /**
  * If ros has not been initialized start ros. Is not compatible with ros api plugin
  * TODO Add a stupid world plugin that only initializes ros and nothing else
  */
  if(!ros::isInitialized()) {
    int argc = 0;
    char **argv;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler);
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);
  // Can use model name here - _model->GetName()
  // Custom Callback Queue
  ros::SubscribeOptions so = ros::SubscribeOptions::create<gazebo_aerial_manipulation_plugin::RollPitchYawThrust>(
    "rpyt_command",1,
    boost::bind( &GazeboAerialManipulation::UpdateObjectForce,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->rpyt_command_sub_ = this->rosnode_->subscribe(so);

  ros::SubscribeOptions reset_so = ros::SubscribeOptions::create<std_msgs::Empty>(
    "model_reset",1,
    boost::bind( &GazeboAerialManipulation::reset,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->reset_sub_ = this->rosnode_->subscribe(reset_so);

  ros::SubscribeOptions model_pose_so = ros::SubscribeOptions::create<geometry_msgs::Pose>(
    "set_model_pose",1,
    boost::bind( &GazeboAerialManipulation::setModelPose,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->model_pose_sub_ = this->rosnode_->subscribe(model_pose_so);

  ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<gazebo_aerial_manipulation_plugin::RPYPose>(
      "base_pose",1,
      boost::bind( &GazeboAerialManipulation::poseConnect,this),
      boost::bind( &GazeboAerialManipulation::poseDisconnect,this), ros::VoidPtr(), &this->queue_);
  this->pose_pub_ = this->rosnode_->advertise(ao);


  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboAerialManipulation::QueueThread,this ) );

  // Publish pose thread
  this->publish_pose_thread_ = boost::thread( boost::bind( &GazeboAerialManipulation::publishPoseThread,this ) );

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboAerialManipulation::UpdateChild, this));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboAerialManipulation::UpdateObjectForce(const gazebo_aerial_manipulation_plugin::RollPitchYawThrust::ConstPtr& _msg)
{
  rpyt_msg_ = *_msg; 
}

////////////////////////////////////////////////////////////////////////////////
// reset the model
void GazeboAerialManipulation::reset(const std_msgs::Empty::ConstPtr&) {
  if(this->model_) {
    this->model_->Reset();
    rpy_controller_.reset();
    gazebo_aerial_manipulation_plugin::RollPitchYawThrust rpyt_msg;
    rpyt_msg.roll = 0;
    rpyt_msg.pitch = 0;
    rpyt_msg.yaw = 0;
    rpyt_msg.thrust = 0;
    this->rpyt_msg_ = rpyt_msg;
  }
}

////////////////////////////////////////////////////////////////////////////////
// set the model pose
void GazeboAerialManipulation::setModelPose(const geometry_msgs::Pose::ConstPtr& _pose) {
  if(this->world_) {
    this->world_->SetPaused(true);
    gazebo::math::Vector3 target_pos(_pose->position.x,_pose->position.y,_pose->position.z);
    gazebo::math::Quaternion target_rot(_pose->orientation.w, _pose->orientation.x, _pose->orientation.y, _pose->orientation.z);
    target_rot.Normalize(); // eliminates invalid rotation (0, 0, 0, 0)
    if(this->model_) {
      this->model_->SetWorldPose(gazebo::math::Pose(target_pos, target_rot));
    } else {
      ROS_WARN("GazeboAerialManipulation: Model not available to set model pose");
    }
    this->world_->SetPaused(false);
  } else {
    ROS_WARN("GazeboAerialManipulation: World not available to set model pose");
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboAerialManipulation::UpdateChild()
{
  math::Pose current_pose = this->link_->GetWorldPose();
  math::Vector3 omega_i = this->link_->GetWorldAngularVel();
  auto rpyt_msg = rpyt_msg_.get();
  math::Vector3 rpy_command(rpyt_msg.roll, rpyt_msg.pitch, rpyt_msg.yaw);
  math::Vector3 body_force(0, 0, kt_*rpyt_msg.thrust);
  if(pose_subscriber_count_.get() > 0) {
    current_pose_.header.stamp = ros::Time::now();
    current_pose_.position.x = current_pose.pos.x;
    current_pose_.position.y = current_pose.pos.y;
    current_pose_.position.z = current_pose.pos.z;
    current_pose_.rpy.x = current_pose.rot.GetRoll();
    current_pose_.rpy.y = current_pose.rot.GetPitch();
    current_pose_.rpy.z = current_pose.rot.GetYaw();
  }
  math::Vector3 omega_b = current_pose.rot.RotateVectorReverse(omega_i);
  math::Vector3 body_torque = rpy_controller_.update(rpy_command, current_pose.rot, omega_b, this->world_->GetSimTime());
  math::Vector3 force = current_pose.rot.RotateVector(body_force);
  math::Vector3 torque = current_pose.rot.RotateVector(body_torque);
  this->link_->AddForce(force);
  this->link_->AddTorque(torque);
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboAerialManipulation::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void GazeboAerialManipulation::publishPoseThread()
{
  while (this->rosnode_->ok()) {
    if(this->pose_subscriber_count_.get() > 0) {
      this->pose_pub_.publish(this->current_pose_);
    }
    boost::this_thread::sleep(boost::posix_time::milliseconds(20));
  }
}

}
