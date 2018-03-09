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
, pose_pub_milliseconds_(20)
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

void GazeboAerialManipulation::LoadJointInfo(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  gzdbg<<"Loading Joints..."<<endl;
  if(_sdf->HasElement("joints"))
  {
    string joint_string = _sdf->GetElement("joints")->Get<string>();
    ///\todo Remove later debug message
    gzdbg<<joint_string<<std::endl;

    std::stringstream joint_stream(joint_string);//Create a stream from the link strings
    while(joint_stream.good())
    {
      string substr;
      getline(joint_stream, substr, ';');//String delimited by semicolon
      physics::JointPtr joint = boost::dynamic_pointer_cast<gazebo::physics::Joint>(_model->GetByName(substr));
      if(!joint)
      {
        gzdbg<<"Failed to load: "<<substr<<std::endl;
        continue;
      } else if(joint->HasType(physics::Base::HINGE_JOINT)) {
        gzdbg<< "Joint type is not revolute"<<std::endl;
      }
      joint_info_.emplace_back(joint);
  }
  else
  {
    gzdbg<<"Cannot Find joints to load"<<endl;
  }
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
  
  // Get joint joint_info
  LoadJointInfo(_model, _sdf);

  // Get Thrust gain
  if(_sdf->HasElement("thrust_gain")) {
    kt_ = _sdf->GetElement("thrust_gain")->Get<double>();
  }

  if(_sdf->HasElement("pose_pub_freq")) {
    double freq = _sdf->GetElement("pose_pub_freq")->Get<double>();
    if(freq > 1e-2 && freq < 1e3) {
      pose_pub_milliseconds_ = std::ceil(1000.0/freq);
    }
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
  updateJointEfforts();
}

void GazeboAerialManipulation::updateJointEfforts()
{
  //Control all the servos irrespective of whether they are commanded or not:
  for (JointInfoVec::iterator it=joint_info_.begin(); it!=joint_info_.end(); ++it)
  {
    physics::JointPtr &joint = it->joint_;
    double error = 0.0;
    if(it->control_type == 0)
    {
      //Position Control:
      error = std::remainder(joint->GetAngle(0).Radian(),2*M_PI) - it->desired_target;
      error = (error > M_PI)?(error - 2*M_PI):(error < -M_PI)?(error + 2*M_PI):error;//Map error into (pi to pi) region
    }
    else if(it->control_type == 1)
    {
      //Velocity Control
      error = joint->GetVelocity(0) - it->desired_target;
      // cout<<"Error: "<<joint->GetVelocity(0)<<"\t"<<it->desired_target<<endl;
    }
    double effort = it->pidcontroller.Update(error, common::Time(physicsEngine->GetMaxStepSize()));
    /*{
      double debugerrors[3];
      it->pidcontroller.GetErrors(debugerrors[0],debugerrors[1],debugerrors[2]);
      gzdbg<<"Errors: "<<debugerrors[0]<<"\t"<<debugerrors[1]<<"\t"<<debugerrors[2]<<"\t"<<endl;
      gzdbg<<"Effort: "<<effort<<endl;
    }
    */
    joint->SetForce(0, effort);
  }
  // Add joint message if joint subscribers are not empty
  // Fill joint position and joint velocity in JointState (sensor_msgs)
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
// Add another joint publishing thread!!
void GazeboAerialManipulation::publishPoseThread()
{
  while (this->rosnode_->ok()) {
    if(this->pose_subscriber_count_.get() > 0) {
      this->pose_pub_.publish(this->current_pose_);
    }
    boost::this_thread::sleep(boost::posix_time::milliseconds(pose_pub_milliseconds_));
  }
}

}
