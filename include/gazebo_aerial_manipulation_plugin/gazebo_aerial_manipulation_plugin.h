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

  // Documentation inherited
  protected: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  // Documentation inherited
  protected: virtual void UpdateChild();

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

  /// \briedf The thread function to publish pose
  private: void publishPoseThread();

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

  /// \brief The Link this plugin is attached to, and will exert forces on.
  private: std::string link_name_;

  /// \brief for setting ROS name space
  private: std::string robot_namespace_;

  // Custom Callback Queue
  private: ros::CallbackQueue queue_;
  /// \brief Thead object for the running callback Thread.
  private: boost::thread callback_queue_thread_;
  /// \brief Thread object for publishing pose
  private: boost::thread publish_pose_thread_;
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

  /// \brief computes body torques based on rpy commands
  private: RpyController rpy_controller_;
  private: gazebo_aerial_manipulation_plugin::RPYPose current_pose_;
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
