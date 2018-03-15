#!/usr/bin/env python2

import numpy as np
import rospy
from sensor_msgs.msg import Joy
from gazebo_aerial_manipulation_plugin.msg import RollPitchYawThrust, JointCommand

class JoyTransport:
  def __init__(self):
    self.joint_pub = rospy.Publisher('joint_command', JointCommand, queue_size=1)
    self.rpyt_pub = rospy.Publisher('rpyt_command', RollPitchYawThrust, queue_size=1)
    self.joy_sub = rospy.Subscriber('joy', Joy, self.joyCallback, queue_size=1)
    self.rpyt_min = [-np.pi/6, -np.pi/6, -np.pi, 40]
    self.rpyt_max = [np.pi/6, np.pi/6, np.pi, 140]
    self.joy_limits = [0.55, 0.55, 0.55, 0.65]
    self.yaw_cmd = 0.0
    self.dt = 1.0/30.0 # Frequency of joy publisher
    #j_amp_scale = 0.5
    j_amp_scale = 1.0
    self.j_amp = j_amp_scale*np.array([0.75, 2.2])
    self.j_max_gain = 20
    self.j_gain = 1
    self.j_nu = np.array([0.1, 0.1])
    self.j_t0 = rospy.get_rostime()
    self.joint_command = JointCommand()
    self.j_prev_time = rospy.get_rostime()
    # Angles when at rest
    self.j_rest = [-0.1, 0.0]
    # Copy j_rest to commanded angles
    self.joint_command.desired_joint_angles = list(self.j_rest)
    self.joint_command.desired_joint_angles.append(0)
    # Offset for sine angles
    self.j_offset = np.array([0.52, 0.78])

  @staticmethod
  def map(in_value, in_min, in_max, out_min, out_max):
    if in_value < in_min:
      output = out_min
    elif  in_value > in_max:
      output = out_max
    else:
      gradient = (out_max - out_min)/(in_max - in_min)
      output = out_min + gradient*(in_value - in_min)
    return output

  def joyCallback(self, joy_msg):
    rpyt_msg = RollPitchYawThrust()
    rpyt_msg.roll = self.map(-1*joy_msg.axes[0], -self.joy_limits[0], self.joy_limits[0], self.rpyt_min[0], self.rpyt_max[0])
    rpyt_msg.pitch = self.map(-1*joy_msg.axes[1], -self.joy_limits[1], self.joy_limits[1], self.rpyt_min[1], self.rpyt_max[1])
    yaw_rate = self.map(joy_msg.axes[5], -self.joy_limits[2], self.joy_limits[2], self.rpyt_min[2], self.rpyt_max[2])
    rpyt_msg.thrust = self.map(joy_msg.axes[2], -self.joy_limits[3], self.joy_limits[3], self.rpyt_min[3], self.rpyt_max[3])
    self.yaw_cmd = self.yaw_cmd + yaw_rate*self.dt
    rpyt_msg.yaw = self.yaw_cmd
    rpyt_msg.header.stamp = rospy.get_rostime()
    self.rpyt_pub.publish(rpyt_msg)

    # Publish joint command
    if joy_msg.axes[3] > 0:
      self.joint_command.header.stamp = rospy.get_rostime()
      # Exp filter
      self.j_gain = (1-0.0001)*self.j_gain + 0.0001*self.map(joy_msg.axes[4],-0.9, 0.9, 0, self.j_max_gain)
      omega = 2*np.pi*self.j_nu*self.j_gain
      off = self.j_offset
      amp = self.j_amp
      dt = (rospy.get_rostime() - self.j_t0).to_sec()
      self.joint_command.desired_joint_angles = list(amp*np.sin(dt*omega)+off)
    else:
      self.j_t0 = rospy.get_rostime()
      self.joint_command.header.stamp = rospy.get_rostime()
      self.joint_command.desired_joint_angles = self.j_rest
    # Publish at 20Hz
    if (rospy.get_rostime() - self.j_prev_time).to_sec() > 0.05:
      self.j_prev_time = rospy.get_rostime()
      self.joint_pub.publish(self.joint_command )

if __name__ == '__main__':
  rospy.init_node('joy_transporter')
  joy_transport = JoyTransport()
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    print "Received Interrupt"
    pass
