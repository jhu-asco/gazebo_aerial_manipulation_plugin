#!/usr/bin/env python
"""
Created on Mon Feb 12 13:04:22 2018

Convert the rpyt commands and pose messages into numpy matrices for
parsing using tensorflow.

@author: gowtham
"""

import os
import sys
import numpy as np
import argparse
import rosbag
import matplotlib.pyplot as plt
# %%
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert bag file')
    parser.add_argument('--log_folder', type=str)
    parser.add_argument('--bag_name', type=str)
    parser.add_argument('--no-plot-data', dest='plot_data', action='store_false', default=True)
    parser.add_argument('--write-data', dest='write_data', action='store_true', default=False)
    args = parser.parse_args()
    if not os.path.isdir(args.log_folder):
        print "Log folder not found: ", args.log_folder
        sys.exit(-1)
    bag_file = os.path.join(args.log_folder, args.bag_name)
    out_file = os.path.splitext(bag_file)[0]
    bag = rosbag.Bag(bag_file)
    t0 = None
    tEnd = None
    z_base = 0.19
    pose_data_list = []
    message_streaming_started = False
    max_tdiff = 60
    for _,msg,t in bag.read_messages(['/base_pose']):
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
    for _,msg, t in bag.read_messages(['/rpyt_command']):
        tdiff = (t - t0).to_sec()
        if tdiff >= 0:
            if (t < tEnd):
                rpyt_data_list.append(np.array([tdiff, msg.roll, msg.pitch,
                                                msg.yaw, msg.thrust]))
            else:
                break
    rpyt_data = np.vstack(rpyt_data_list)
    if args.write_data:
        np.savez(out_file, rpyt_data=rpyt_data, pose_data=pose_data)
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
        plt.show()
