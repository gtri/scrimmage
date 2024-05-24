#!/usr/bin/python

#
# @file
#
# @section LICENSE
#
# Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
#
# This file is part of SCRIMMAGE.
#
#   SCRIMMAGE is free software: you can redistribute it and/or modify it under
#   the terms of the GNU Lesser General Public License as published by the
#   Free Software Foundation, either version 3 of the License, or (at your
                                                                   #   option) any later version.
#
#   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
#   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
#   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
#   License for more details.
#
#   You should have received a copy of the GNU Lesser General Public License
#   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
#
# @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
# @author Eric Squires <eric.squires@gtri.gatech.edu>
# @date 31 July 2017
# @version 0.1.0
# @brief Brief file description.
# @section DESCRIPTION
# A Long description goes here.
#

from __future__ import division, print_function
import argparse
import os
import sys
import glob
import time
import shutil
import struct
import scrimmage as sc
import scrimmage.proto_utils as utils
from scrimmage.proto.Vector3d_pb2 import Vector3d
from scrimmage.proto.Quaternion_pb2 import Quaternion

## imports for mplot3d
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
from itertools import cycle
from math import ceil, floor, log2



def find_frames(framesLoc=''):
    # Take argument of path to location of desired frames.bin
    # (either containing directory or frames.bin itself)
    # If no argument provided, most recently-generated results
    # directory is checked for frames.bin.
    # Return directory containing the frames.bin that will be
    # read and plotted.

    if not framesLoc:
        # if no path was provided (framesLoc = ''), look for the most recent run's results

        print('Location of results not specified.\n')

        # assuming default location of swarm-logs in user's home directory
        log_path = os.environ['HOME'] + '/swarm-log/*'
        # read back the results
        resultsDirsList = glob.glob(log_path)

        if not resultsDirsList:
            raise Exception('Error getting directories in ' + log_path + '\n')

        timeStampedDir = max(resultsDirsList, key=os.path.getctime)
        finalFramesDir = os.path.join(log_path, timeStampedDir)
        print('Looking for frames.bin in most recent results directory:\n' + finalFramesDir)

    else:
        framesLocation = framesLoc
        if framesLocation.startswith('~/'):
            framesLocation = framesLocation.replace('~', os.environ['HOME'])

        print('Looking for frames.bin in the following location:\n' + framesLocation)

        if os.path.exists(framesLocation):
            if not os.path.isfile(framesLocation):
                # if specified location exists and is not a file, it's a directory
                finalFramesDir = framesLocation
            else:
                # specified location is a file
                if framesLocation.endswith('frames.bin'):
                    finalFramesDir = framesLocation[:-11]
                else:
                    # specified file was not frames.bin
                    raise Exception('Specified location was not frames.bin or its containing directory!\n')
        else:
            raise Exception('Specified location does not exist!\n')

    # Ensure finalFramesDir has trailing /
    if not finalFramesDir.endswith('/'):
        finalFramesDir = finalFramesDir + '/'
    # Is there a frames.bin file in this directory?
    if not os.path.isfile(finalFramesDir + 'frames.bin'):
        raise Exception('No frames.bin in ' + finalFramesDir + '\n')

    return finalFramesDir

def parse_frames(frame_file):
    finalFramesDir = find_frames(frame_file)
    finalFramesLoc = finalFramesDir + '/frames.bin'

    ## Read in the frames from the protocol buffer
    #print('Reading frames...\n')
    frames = utils.read_frames(finalFramesLoc)
    return frames

def Vector3d_Diff(lhs, rhs):
    diff = Vector3d()
    diff.x = lhs.x - rhs.x
    diff.y = lhs.y - rhs.y
    diff.z = lhs.z - rhs.z
    return diff

def Vector3d_Norm(vec):
    return np.sqrt(vec.x**2 + vec.y**2 + vec.z**2)

def Quaternion_Scale(quat, scalar):
    scale_quat = quat
    scale_quat.w *= scalar
    scale_quat.x *= scalar
    scale_quat.y *= scalar
    scale_quat.z *= scalar
    return scale_quat

def Quaternion_Norm(quat):
    norm_quat = Quaternion()
    norm = np.sqrt(quat.w**2 + quat.x**2 + quat.y**2 + quat.z**2)
    norm_quat.w = quat.w / norm
    norm_quat.x = quat.x / norm
    norm_quat.y = quat.y / norm
    norm_quat.z = quat.z / norm
    if(norm_quat.w < 0):
        norm_quat = Quaternion_Scale(norm_quat, -1)
    return norm_quat

def Quaternion_Conj(quat):
    conj = Quaternion()
    conj.w = quat.w
    conj.x = -quat.x
    conj.y = -quat.y
    conj.z = -quat.z
    return conj

def Quaternion_Mul(lhs, rhs):
    prod = Quaternion()
    prod.w = lhs.w*rhs.w - lhs.x*rhs.x - lhs.y*rhs.y - lhs.z*rhs.z
    prod.x = lhs.w*rhs.x + lhs.x*rhs.w + lhs.y*rhs.z - lhs.z*rhs.y
    prod.y = lhs.w*rhs.y - lhs.x*rhs.z + lhs.y*rhs.w + lhs.z*rhs.x
    prod.z = lhs.w*rhs.z + lhs.x*rhs.y - lhs.y*rhs.x + lhs.z*rhs.w
    return prod

def Quaternion_Diff(lhs, rhs):
    # Gives the quat that defines the rotation between lhs to rhs
    return Quaternion_Norm(Quaternion_Mul(lhs, Quaternion_Conj(rhs)))

def Quaternion_Angle(quat):
    return 2*np.arccos(Quaternion_Norm(quat).w)

def main():

    ## Command-line argument parsing ##
    parser=argparse.ArgumentParser(description='Plots trajectories of all entities after SCRIMMAGE simulation')
    parser.add_argument(type=str,
                        dest='cpu_frames',
                        default='',
                        help='path of frames.bin or its containing directory;\nif not specified, most recent frames.bin will be processed')
    parser.add_argument(type=str,
                        dest='gpu_frames',
                        default='',
                        help='path of frames.bin or its containing directory;\nif not specified, most recent frames.bin will be processed')
    #parser.add_argument('--2d', dest='two_d', default=False, action='store_true')

    args = parser.parse_args()
#    print(args.cpu_frames)
#    print(args.gpu_frames)

    cpu_frames = parse_frames(args.cpu_frames)
    gpu_frames = parse_frames(args.gpu_frames)

    pos_abs_err = []
    vel_abs_err = []
    quat_abs_err = []
    pos_rel_err = []
    vel_rel_err = []
    quat_rel_err = []
    
    pos_eps_err = []
    vel_eps_err = []

    time = []
    dts = []

    eps = np.finfo(float).eps
    
    for (cpu_frame, gpu_frame) in zip(cpu_frames, gpu_frames):
        # Assume a single entity
        cpu_state = cpu_frame.contact[0].state
        gpu_state = gpu_frame.contact[0].state

        time.append(cpu_frame.time)
        if len(time) > 1:
            dts.append(time[-1] - time[-2])

        pos_diff = Vector3d_Diff(cpu_state.position, gpu_state.position)
        vel_diff = Vector3d_Diff(cpu_state.linear_velocity, gpu_state.linear_velocity)
        quat_diff = Quaternion_Diff(cpu_state.orientation, gpu_state.orientation)
        quat_angle_diff = Quaternion_Angle(quat_diff)
        
        pos_abs_err.append(Vector3d_Norm(pos_diff))
        vel_abs_err.append(Vector3d_Norm(vel_diff))
        quat_abs_err.append(np.abs(quat_angle_diff))

        true_pos = Vector3d_Norm(cpu_state.position)
        true_vel = Vector3d_Norm(cpu_state.linear_velocity)

        pos_rel_err.append(pos_abs_err[-1] / true_pos)
        vel_rel_err.append(vel_abs_err[-1] / true_vel)

        pos_base = ceil(log2(true_pos))
        vel_base = ceil(log2(true_vel))

        pos_eps = eps*pos_base
        vel_eps = eps*vel_base
        
        pos_eps_err.append(pos_abs_err[-1] / (pos_eps))
        vel_eps_err.append(vel_abs_err[-1] / (vel_eps))

    dt = np.round(np.mean(dts), 3)
    timestep = np.array(time) / dt
    
    pos_rmse = np.sqrt(np.mean(np.power(pos_abs_err, 2)))
    vel_rmse = np.sqrt(np.mean(np.power(vel_abs_err, 2)))
    quat_rmse = np.sqrt(np.mean(np.power(quat_abs_err, 2)))

    print("Position RMSE: {}".format(pos_rmse))
    print("Velocity RMSE: {}".format(vel_rmse))
    print("Quaternion RMSE: {}".format(quat_rmse))

    ratio = .3
    fig_abs, ax_abs = plt.subplots()
    ax_abs.plot(timestep, pos_abs_err, label="Position")
    ax_abs.plot(timestep, vel_abs_err, label="Velocity")
    #ax_abs.plot(timestep, quat_abs_err, label="Quaternion")
    ax_abs.legend()
    ax_abs.grid()
    ax_abs.set_xlabel("Number of Timesteps")
    ax_abs.set_ylabel("Absolute Error")
    x_left, x_right = ax_abs.get_xlim()
    y_low, y_high = ax_abs.get_ylim()
    ax_abs.set_aspect(abs((x_right-x_left)/(y_low-y_high))*ratio)
    fig_abs.savefig(os.environ['HOME'] + '/trajectory_abs_error_plot.png')

    fig_rel, ax_rel = plt.subplots()
    ax_rel.plot(timestep, pos_rel_err, label="Position")
    ax_rel.plot(timestep, vel_rel_err, label="Velocity")
    #ax_rel.plot(time, quat_rel_err, label="Quaternion")
    ax_rel.legend()
    ax_rel.grid()
    ax_rel.set_xlabel("Number of Timesteps")
    ax_rel.set_ylabel("Relative Error")
    x_left, x_right = ax_rel.get_xlim()
    y_low, y_high = ax_rel.get_ylim()
    ax_rel.set_aspect(abs((x_right-x_left)/(y_low-y_high))*ratio)
    fig_rel.savefig(os.environ['HOME'] + '/trajectory_rel_error_plot.png')

    fig_eps, ax_eps = plt.subplots()
    #ax_eps.plot(time, pos_eps_err, label="Position")
    ax_eps.plot(time, vel_eps_err, label="Velocity")
    #ax_eps.plot(time, quat_eps_err, label="Quaternion")
    ax_eps.legend()
    ax_eps.grid()
    ax_abs.set_xlabel("Number of Timesteps")
    ax_abs.set_ylabel("Epsilon Error")
    fig_eps.savefig(os.environ['HOME'] + '/trajectory_eps_error_plot.png')


if __name__ == '__main__':
    main()
