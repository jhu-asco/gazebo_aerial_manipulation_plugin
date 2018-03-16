#!/usr/bin/env python
"""
Created on Mon Feb 12 13:04:22 2018

Convert the rpyt commands and pose messages into numpy matrices for
parsing using tensorflow.

@author: gowtham
"""

import os
import numpy as np
import argparse
import rosbag
import matplotlib.pyplot as plt
# %%
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert bag file')
    parser.add_argument('--bag_name', type=str)
    parser.add_argument('--max_t', type=float, default=100)
    parser.add_argument('--no_plot_data', dest='plot_data', action='store_false', default=True)
    parser.add_argument('--write_data', dest='write_data', action='store_true', default=False)
    args = parser.parse_args()
    out_file = os.path.splitext(args.bag_name)[0]
    bag = rosbag.Bag(args.bag_name)
    t0 = None
    tEnd = None
    z_base = 0.19
    pose_data_list = []
    message_streaming_started = False
    max_tdiff = args.max_t
    for _,msg,_ in bag.read_messages(['/base_pose']):
        if msg.position.z - z_base > 0:
            if t0 is None:
                t0 = msg.header.stamp
            tdiff = (msg.header.stamp - t0).to_sec()
            tEnd = msg.header.stamp
            if max_tdiff is not None and tdiff > max_tdiff:
                break
            pose_data_list.append(np.array(
                 [tdiff, msg.position.x, msg.position.y, msg.position.z, 
                  msg.rpy.x, msg.rpy.y, msg.rpy.z]))
        else:
            if message_streaming_started:
                tEnd = msg.header.stamp
                break
            elif t0 is not None:
                message_streaming_started = True
    pose_data = np.vstack(pose_data_list)
    rpyt_data_list = []
    for _,msg, t1 in bag.read_messages(['/rpyt_command']):
        tdiff = (msg.header.stamp - t0).to_sec()
        if tdiff >= 0:
            if (msg.header.stamp < tEnd):
                rpyt_data_list.append(np.array([tdiff, msg.roll, msg.pitch,
                                                msg.yaw, msg.thrust]))
            else:
                break
    rpyt_data = np.vstack(rpyt_data_list)
    joint_state_list = []
    for _,msg, _ in bag.read_messages(['/joint_state']):
        tdiff = (msg.header.stamp - t0).to_sec()
        if tdiff >= 0:
            if (msg.header.stamp < tEnd):
                joint_state_list.append(np.hstack([tdiff, msg.position, msg.velocity, msg.effort]))
            else:
                break
    if joint_state_list:
      joint_command_list = []
      for _, msg, _ in bag.read_messages(['/joint_command']):
          tdiff = (msg.header.stamp - t0).to_sec()
          if tdiff >= 0:
              if (msg.header.stamp < tEnd):
                  joint_command_list.append(np.hstack([tdiff, msg.desired_joint_angles]))
              else:
                  break
      joint_state_data = np.vstack(joint_state_list)
      joint_command_data = np.vstack(joint_command_list)
    else:
      joint_command_data = None
      joint_state_data = None
      joint_command_data = np.vstack(joint_command_list)
    if args.write_data:
        np.savez(out_file, control_data=[rpyt_data, joint_command_data], sensor_data=[pose_data, joint_state_data])
    if args.plot_data:
        # %% Plot
        plt.figure(1)
        axis_name = ['roll', 'pitch', 'yaw']
        for i in range(3):
            plt.subplot(3,1,i+1)
            plt.plot(pose_data[:,0], pose_data[:,i+4])
            plt.plot(rpyt_data[:,0], rpyt_data[:,i+1])
            plt.ylabel('Angle (rad)')
            plt.legend([axis_name[i], axis_name[i]+'_command'])
        plt.xlabel('Time(sec)')
        plt.figure(2)
        axis_name = ['x', 'y', 'z']
        for i in range(3):
            plt.subplot(3,1,i+1)
            plt.plot(pose_data[:,0], pose_data[:,i+1])
            plt.ylabel(axis_name[i] + ' (m)')
        plt.xlabel('Time(sec)')
        # Plot joint state and commands if available
        if joint_state_list:
          plt.figure(3)
          axis_name = ['angle_0', 'angle_1', 'omega_0', 'omega_1']
          for i in range(4):
            plt.subplot(2,2, i+1)
            plt.plot(joint_state_data[:, 0], joint_state_data[:, i+1])
            if i < 2:
              plt.plot(joint_command_data[:, 0], joint_command_data[:, i+1])
              plt.legend(('angle', 'desired_angle'))
              plt.ylabel(axis_name[i] + ' (rad)')
            plt.ylabel(axis_name[i])
            if i >= 2:
              plt.xlabel('Time (seconds)')
              plt.ylabel(axis_name[i] + ' (rad/s)')
        plt.show()
