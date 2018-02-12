#!/usr/bin/env python2

import numpy as np
import rospy
from sensor_msgs.msg import Joy
from gazebo_aerial_manipulation_plugin.msg import RollPitchYawThrust

class JoyTransport:
  def __init__(self):
    self.rpyt_pub = rospy.Publisher('rpyt_command', RollPitchYawThrust, queue_size=1)
    self.joy_sub = rospy.Subscriber('joy', Joy, self.joyCallback, queue_size=1)
    self.rpyt_min = [-np.pi/6, -np.pi/6, -np.pi, 30]
    self.rpyt_max = [np.pi/6, np.pi/6, np.pi, 120]
    self.joy_limits = [0.55, 0.55, 0.55, 0.65]
    self.yaw_cmd = 0.0
    self.dt = 1.0/30.0 # Frequency of joy publisher

  @staticmethod
  def map(in_value, in_min, in_max, out_min, out_max):
    if in_value < in_min:
      output = out_min
    elif  in_value > in_max:
      output = out_max
    else:
      gradient = (out_max - out_min)/(in_max - in_min)
      output = out_min + gradient*(in_value - in_min);
    return output

  def joyCallback(self, joy_msg):
    rpyt_msg = RollPitchYawThrust()
    rpyt_msg.roll = self.map(-1*joy_msg.axes[0], -self.joy_limits[0], self.joy_limits[0], self.rpyt_min[0], self.rpyt_max[0]);
    rpyt_msg.pitch = self.map(-1*joy_msg.axes[1], -self.joy_limits[1], self.joy_limits[1], self.rpyt_min[1], self.rpyt_max[1]);
    yaw_rate = self.map(joy_msg.axes[5], -self.joy_limits[2], self.joy_limits[2], self.rpyt_min[2], self.rpyt_max[2]);
    rpyt_msg.thrust = self.map(joy_msg.axes[2], -self.joy_limits[3], self.joy_limits[3], self.rpyt_min[3], self.rpyt_max[3]);
    self.yaw_cmd = self.yaw_cmd + yaw_rate*self.dt
    rpyt_msg.yaw = self.yaw_cmd
    self.rpyt_pub.publish(rpyt_msg)

if __name__ == '__main__':
  rospy.init_node('joy_transporter')
  joy_transport = JoyTransport()
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    print "Received Interrupt"
    pass
