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
, joint_pub_milliseconds_(50)
, previous_sim_time_(0, 0)
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
  std::cout<<"Loading Joints..."<<std::endl;
  sdf::ElementPtr joint_elem = _sdf->GetFirstElement();
  gazebo_aerial_manipulation_plugin::JointCommand joint_command;

  while(joint_elem)
  {
    physics::JointPtr joint;
    std::string joint_name;
    double default_target = 0.0;
    double max_torque_joint = 10.0;
    double max_integral_joint = 1.0;
    double max_velocity_joint = 1.0;
    math::Vector3 pid_gains_joint;
    if(joint_elem->HasAttribute("name")) {
      joint_name = joint_elem->Get<std::string>("name");
      joint = boost::dynamic_pointer_cast<gazebo::physics::Joint>(_model->GetByName(joint_name));
    } else {
      std::cout<<"Cannot find name in joint element: "<<joint_elem->GetName()<<std::endl;
      joint_elem = joint_elem->GetNextElement();
      continue;
    }
    if(!joint)
    {
      std::cout<<"Failed to load: "<<joint_name<<std::endl;
      joint_elem = joint_elem->GetNextElement();
      continue;
    } else if(!joint->HasType(physics::Base::HINGE_JOINT)) {
      std::cout<< "Joint type is not revolute"<<std::endl;
      joint_elem = joint_elem->GetNextElement();
      continue;
    } else {
      std::cout<<"Found Hinge joint: "<<joint_name<<std::endl;
    }
    if(joint_elem->HasElement("pid_gains")) {
      auto pid_gains = joint_elem->GetElement("pid_gains");
      pid_gains_joint.x = pid_gains->Get<double>("p");
      pid_gains_joint.y = pid_gains->Get<double>("i");
      pid_gains_joint.z = pid_gains->Get<double>("d");
    } else {
      std::cout<< "Cannot find pid gains for the joint!"<<std::endl;
      pid_gains_joint.x = pid_gains_joint.y = pid_gains_joint.z = 0.1;
    }
    if(joint_elem->HasElement("integral_max")) {
      max_integral_joint = joint_elem->GetElement("integral_max")->Get<double>();
      std::cout<<"Setting max integral value: "<<max_integral_joint<<std::endl;
    }
    if(joint_elem->HasElement("max_velocity")) {
      max_velocity_joint = joint_elem->GetElement("max_velocity")->Get<double>();
      std::cout<<"Setting max velocity for joint: "<<max_velocity_joint<<std::endl;
    }
    if(joint_elem->HasElement("max_torque")) {
      max_torque_joint = joint_elem->GetElement("max_torque")->Get<double>();
      std::cout<<"Setting max torque: "<<max_torque_joint<<std::endl;
    }
    if(joint_elem->HasElement("default_target")) {
      default_target = joint_elem->GetElement("default_target")->Get<double>();
      std::cout<<"Setting default target: "<<default_target<<std::endl;
    }
    joint_info_.emplace_back(joint, joint_name, pid_gains_joint, max_torque_joint, max_velocity_joint, max_integral_joint);
    joint_state_.name.push_back(joint_name);
    joint_state_.position.push_back(0);
    joint_state_.velocity.push_back(0);
    joint_state_.effort.push_back(0);
    joint_elem = joint_elem->GetNextElement();
    joint_command.desired_joint_angles.push_back(default_target);
  }
  joint_command_ = joint_command;
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
    ROS_INFO("Cannot find body name, will not perform rpyt control");
  }
  else {
    std::string link_name = _sdf->GetElement("bodyName")->Get<std::string>();

    this->link_ = _model->GetLink(link_name);

    if (!this->link_)
    {
      ROS_FATAL_NAMED("force", "aerial_manipulation_plugin plugin error: link named: %s does not exist\n", link_name.c_str());
      return;
    }
    // Get rpy controller
    LoadRPYController(_sdf);
  }
  
  // Load joint_info if provided
  if(_sdf->HasElement("joints")) {
    LoadJointInfo(_model, _sdf->GetElement("joints"));
  } else {
    std::cout<<"Cannot Find joints to load"<<std::endl;
  }

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

  if(_sdf->HasElement("joint_pub_freq")) {
    double freq = _sdf->GetElement("joint_pub_freq")->Get<double>();
    if(freq > 1e-2 && freq < 1e3) {
      joint_pub_milliseconds_ = std::ceil(1000.0/freq);
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

  ros::SubscribeOptions joint_so = ros::SubscribeOptions::create<gazebo_aerial_manipulation_plugin::JointCommand>(
    "joint_command",1,
    boost::bind( &GazeboAerialManipulation::UpdateJointCommand,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->joint_command_sub_ = this->rosnode_->subscribe(joint_so);

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

  ros::AdvertiseOptions ao_joint = ros::AdvertiseOptions::create<sensor_msgs::JointState>(
      "joint_state", 1,
      ros::SubscriberStatusCallback(),
      ros::SubscriberStatusCallback(),
      ros::VoidPtr(), &this->queue_);
  this->joint_state_pub_ = this->rosnode_->advertise(ao_joint);

  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboAerialManipulation::QueueThread,this ) );

  // Publish pose thread
  this->publish_pose_thread_ = boost::thread( boost::bind( &GazeboAerialManipulation::publishPoseThread,this ) );

  // Publish joint thread
  this->publish_joint_thread_ = boost::thread( boost::bind( &GazeboAerialManipulation::publishJointStateThread,this ) );

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

void GazeboAerialManipulation::UpdateJointCommand(const gazebo_aerial_manipulation_plugin::JointCommand::ConstPtr& _msg)
{
  auto joint_command = joint_command_.get();
  int size1 = joint_command.desired_joint_angles.size();
  int size2 = _msg->desired_joint_angles.size();
  if(size1 == size2) {
    joint_command_ = *_msg;
  } else if(size1 > 0) {
    ROS_WARN("Provide all the joint angles to be commanded. Size error:%d, %d", size1, size2);
  }
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
  if (link_) {
    UpdateBaseLink();
  }
  UpdateJointEfforts();
  previous_sim_time_ = world_->GetSimTime();
}

void GazeboAerialManipulation::UpdateBaseLink() {
  math::Pose current_pose = link_->GetWorldPose();
  math::Vector3 omega_i = link_->GetWorldAngularVel();
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
  math::Vector3 body_torque = rpy_controller_.update(rpy_command, current_pose.rot, omega_b, world_->GetSimTime());
  math::Vector3 force = current_pose.rot.RotateVector(body_force);
  math::Vector3 torque = current_pose.rot.RotateVector(body_torque);
  //link_->AddForce(force);
  //link_->AddTorque(torque);
  link_->SetForce(force);
  link_->SetTorque(torque);
}

void GazeboAerialManipulation::UpdateJointEfforts()
{
  auto joint_command = joint_command_.get();
  auto current_time = world_->GetSimTime();
  //Control all the servos irrespective of whether they are commanded or not:
  for(int i = 0; i < joint_info_.size(); ++i)
  {
    JointInfo &joint_info = joint_info_.at(i);
    physics::JointPtr &joint = joint_info.joint_;
    double j_angle = std::remainder(joint->GetAngle(0).Radian(),2*M_PI);
    double j_desired = joint_command.desired_joint_angles.at(i);
    double j_velocity = joint->GetVelocity(0);
    //Position Control:
    double error = j_angle - j_desired;
    if(error > M_PI) {
      j_angle = j_angle - 2*M_PI;
    } else if(error < -M_PI) {
      j_angle = j_angle + 2*M_PI;
    }
    double j_effort = joint_info.pidcontroller.update(j_desired, j_angle, j_velocity, current_time);
    joint->SetForce(0, j_effort);
    // Fill joint state info
    joint_state_.header.stamp = ros::Time::now();
    joint_state_.position.at(i) = j_angle;
    joint_state_.velocity.at(i) = j_velocity;
    joint_state_.effort.at(i) = j_effort;
  }
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

void GazeboAerialManipulation::publishJointStateThread()
{
  while (this->rosnode_->ok()) {
    if(joint_state_.position.size() > 0) {
      this->joint_state_pub_.publish(joint_state_);
    }
    boost::this_thread::sleep(boost::posix_time::milliseconds(joint_pub_milliseconds_));
  }
}

}
