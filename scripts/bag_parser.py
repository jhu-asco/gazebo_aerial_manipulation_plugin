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
import matplotlib.pyplot as plt
from bag_topic_extraction import *

# %%
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert bag file')
    parser.add_argument('--bag_name', type=str)
    parser.add_argument('--max_dt', type=float, default=100)
    parser.add_argument('--no_plot_data', dest='plot_data',
                        action='store_false', default=True)
    parser.add_argument('--write_data', dest='write_data',
                        action='store_true', default=False)
    parser.add_argument('--aerial_manip', dest='aerial_manip',
                        action='store_true', default=False)
    args = parser.parse_args()
    out_file = os.path.splitext(args.bag_name)[0]
    bag = rosbag.Bag(args.bag_name)
    z_base = 0.19
    t0 = getTakeoffTimeStamp(bag, '/base_pose', z_base)
    poses = extractTopicWithHeader(bag, '/base_pose',
                                   transformXYZRPYToList,
                                   t0, args.max_dt)

    # RPYT Data
    quad_ctrls = extractTopicWithHeader(bag, '/rpyt_command',
                                        transformRPYTToList,
                                        t0, args.max_dt)
    # Create Data to write
    sensor_data = [poses]
    control_data = [quad_ctrls]
    if args.aerial_manip:
        # Joint stuff
        joint_states = extractTopicWithHeader(bag, '/joint_state',
                                              transformJointStateToList,
                                              t0, args.max_dt)

        joint_commands = extractTopicWithHeader(bag, '/joint_command',
                                                transformJointCommandToList,
                                                t0, args.max_dt)
        sensor_data.append(joint_states)
        control_data.append(joint_commands)

    if args.write_data:
        np.savez(out_file, control_data=control_data, sensor_data=sensor_data)
    if args.plot_data:
        # %% Plot
        plt.figure(1)
        axis_name = ['roll', 'pitch', 'yaw']
        for i in range(3):
            plt.subplot(3, 1, i + 1)
            plt.plot(poses[:, 0], poses[:, i + 4])
            plt.plot(quad_ctrls[:, 0], quad_ctrls[:, i + 1])
            plt.ylabel('Angle (rad)')
            plt.legend([axis_name[i], axis_name[i] + '_command'])
        plt.xlabel('Time(sec)')
        plt.figure(2)
        axis_name = ['x', 'y', 'z']
        for i in range(3):
            plt.subplot(3, 1, i + 1)
            plt.plot(poses[:, 0], poses[:, i + 1])
            plt.ylabel(axis_name[i] + ' (m)')
        plt.xlabel('Time(sec)')
        # Plot joint state and commands if available
        if args.aerial_manip:
            plt.figure(3)
            axis_name = ['angle_0', 'angle_1', 'omega_0', 'omega_1']
            for i in range(4):
                plt.subplot(2, 2, i + 1)
                plt.plot(joint_states[:, 0], joint_states[:, i + 1])
                if i < 2:
                    plt.plot(joint_commands[:, 0], joint_commands[:, i + 1])
                    plt.legend(('angle', 'desired_angle'))
                    plt.ylabel(axis_name[i] + ' (rad)')
                plt.ylabel(axis_name[i])
                if i >= 2:
                    plt.xlabel('Time (seconds)')
                    plt.ylabel(axis_name[i] + ' (rad/s)')
        plt.show()
