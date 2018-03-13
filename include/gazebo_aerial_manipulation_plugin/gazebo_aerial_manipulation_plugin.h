#pragma once
/**
 * Aerial manipulation controller that provides interface to apply
 * rpyt commands to quadrotor and pid controls to joints
 * Author: Gowtham Garimella
 * Date: 7th February 2018
 */

#include <string>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>

#include <ros/ros.h>
#include <boost/thread.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <gazebo_aerial_manipulation_plugin/RollPitchYawThrust.h>
#include <gazebo_aerial_manipulation_plugin/rpycontroller.h>
#include <gazebo_aerial_manipulation_plugin/atomic.h>
#include <gazebo_aerial_manipulation_plugin/RPYPose.h>


namespace gazebo
{
  /**
   * @brief Store information about each joint
   */
  struct JointInfo{
    common::PID pidcontroller;
    double desired_target;// Servo goal
    int control_type;//Position control 0; Velocity Control 1
    physics::JointPtr joint_;
    std::string joint_name_;
    JointInfo(physics::JointPtr joint, std::string joint_name, math::Vector3 pid_gains, double max_cmd):desired_target(0),
        control_type(0),
        joint_(joint),
        joint_name_(joint_name),
        pidcontroller(pid_gains.x, pid_gains.y, pid_gains.z, 0.1*max_cmd, -0.1*max_cmd, max_cmd, -max_cmd)
    {
    }
  };
  /**
   * @brief Map between joint indices and servos attached
   */
  typedef std::vector<JointInfo> JointInfoVec;
/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup GazeboAerialManipulation Plugin XML Reference and Example

  \brief Ros Force Plugin.

  This is a Plugin that collects data from a ROS topic and applies wrench to a body accordingly.

  Example Usage:
  \verbatim
      <gazebo>
        <plugin filename="libgazebo_ros_force.so" name="gazebo_ros_force">
          <bodyName>box_body</bodyName>
          <topicName>box_force</topicName>
        </plugin>
      </gazebo>
  \endverbatim

\{
*/

/**
           .

*/

class GazeboAerialManipulation : public ModelPlugin
{
  /// \brief Constructor
  public: GazeboAerialManipulation();

  /// \brief Destructor
  public: virtual ~GazeboAerialManipulation();

  // Load rpy controller using provided params
  protected: void LoadRPYController(sdf::ElementPtr _sdf);

  // Load joints and servos using provided params
protected: void LoadJointInfo(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  // Documentation inherited
  protected: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  // Documentation inherited
  ///\brief Run all things when model gets updated
  protected: virtual void UpdateChild();
  ///\brief Update base link pose and rpyt controller
  protected: virtual void UpdateBaseLink();
  ///\brief Control joints using servos
  protected: virtual void UpdateJointEfforts();

  /// \brief call back when a rpy command is published
  /// \param[in] _msg The Incoming ROS message representing the new force to exert.
  private: void UpdateObjectForce(const gazebo_aerial_manipulation_plugin::RollPitchYawThrust::ConstPtr& _msg);

  /// \brief reset the model when this message is published
  private: void reset(const std_msgs::Empty::ConstPtr&);

  /// \brief set the pose of the model wrt world
  /// \param[in] _pose The Incoming ROS message representing the target pose for model
  private: void setModelPose(const geometry_msgs::Pose::ConstPtr& _pose);

  /// \brief The custom callback queue thread function.
  private: void QueueThread();

  /// \brief The thread function to publish pose
  private: void publishPoseThread();

  /// \brief The thread function to publish joint state
  private: void publishJointStateThread();

  /// \brief A pointer to the gazebo world.
  private: physics::WorldPtr world_;

  /// \brief A pointer to the gazebo model.
  private: physics::ModelPtr model_;

  /// \brief A pointer to the Link, where force is applied
  private: physics::LinkPtr link_;

  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: ros::NodeHandle* rosnode_;
  private: ros::Subscriber rpyt_command_sub_;
  private: ros::Subscriber reset_sub_;
  private: ros::Subscriber model_pose_sub_;

  private: ros::Publisher pose_pub_;
  private: ros::Publisher joint_state_pub_;

  /// \brief for setting ROS name space
  private: std::string robot_namespace_;

  // Custom Callback Queue
  private: ros::CallbackQueue queue_;
  /// \brief Thead object for the running callback Thread.
  private: boost::thread callback_queue_thread_;
  /// \brief Thread object for publishing pose
  private: boost::thread publish_pose_thread_;
  /// \brief Thread object for publishing joint state
  private: boost::thread publish_joint_thread_;
  /// \brief Container for the wrench force that this plugin exerts on the body.
  private: Atomic<gazebo_aerial_manipulation_plugin::RollPitchYawThrust> rpyt_msg_;

  // Pointer to the update event connection
  private: event::ConnectionPtr update_connection_;

  private:
    math::Vector3 p_gains_;
    math::Vector3 i_gains_;
    math::Vector3 d_gains_;
    double kt_;
    double max_torque_;
    Atomic<int> pose_subscriber_count_;
    int pose_pub_milliseconds_;
    int joint_pub_milliseconds_;
    common::Time previous_sim_time_;

  /// \brief computes body torques based on rpy commands
  private: RpyController rpy_controller_;
  private: gazebo_aerial_manipulation_plugin::RPYPose current_pose_;
  private: sensor_msgs::JointState joint_state_;
  private: JointInfoVec joint_info_;
  private:
    void poseConnect() {
      pose_subscriber_count_.set(pose_subscriber_count_.get() + 1);
    }
    void poseDisconnect() {
      pose_subscriber_count_.set(pose_subscriber_count_.get() - 1);
    }
};
/** \} */
/// @}
}
