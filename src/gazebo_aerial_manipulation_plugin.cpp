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
, rpy_controller_(p_gains_, i_gains_, d_gains_, max_torque_)
{
  this->rpyt_msg_.roll = 0;
  this->rpyt_msg_.pitch = 0;
  this->rpyt_msg_.yaw = 0;
  this->rpyt_msg_.thrust = 0;
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

  delete this->rosnode_;
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

  /**
  * If ros has not been initialized start ros. Is not compatible with ros api plugin
  * TODO Add a stupid world plugin that only initializes ros and nothing else
  */
  if(!ros::isInitialized()) {
    int argc = 0;
    char **argv;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler);
  }
  // Make sure the ROS node for Gazebo has already been initialized
  /*if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("force", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
  */

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

  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboAerialManipulation::QueueThread,this ) );

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
    this->rpyt_msg_.roll = this->rpyt_msg_.pitch = this->rpyt_msg_.yaw = this->rpyt_msg_.thrust = 0;
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
  this->lock_.lock();
  math::Vector3 rpy_command(rpyt_msg_.roll, rpyt_msg_.pitch, rpyt_msg_.yaw);
  math::Pose current_pose = this->link_->GetWorldPose();
  math::Vector3 omega_i = this->link_->GetWorldAngularVel();
  math::Vector3 omega_b = current_pose.rot.RotateVectorReverse(omega_i);
  math::Vector3 body_torque = rpy_controller_.update(rpy_command, current_pose.rot, omega_b, this->world_->GetSimTime());
  math::Vector3 body_force(0, 0, kt_*rpyt_msg_.thrust);
  math::Vector3 force = current_pose.rot.RotateVector(body_force);
  math::Vector3 torque = current_pose.rot.RotateVector(body_torque);
  this->link_->AddForce(force);
  this->link_->AddTorque(torque);
  this->lock_.unlock();
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

}
