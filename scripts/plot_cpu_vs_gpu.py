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
#
# This script generates a 3D plot of the trajectories of all of the entities
# as recorded from a run of SCRIMMAGE.
#
# The trajectory data can be read from the frames.bin file that SCRIMMAGE
# stores in a timestamped directory located in your `$HOME/swarm-log/` directory.
#
# To plot the most-recently modified frames.bin, run the following from
# the `/path/to/scrimmage/scripts` directory:
#
#     $ python2.7 plot_3d_fr.py
#
# To plot a specific frames.bin, include that directory's path or the
# frames.bin's path as an argument with the `-f` flag, e.g.:
#
#     $ python2.7 plot_3d_fr.py -f '$HOME/swarm-log/2016-09-22_10-37-38/'
#
#     or
#
#     $ python2.7 plot_3d_fr.py -f '$HOME/swarm-log/2016-09-22_10-37-38/frames.bin'
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

## imports for mplot3d
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
from itertools import cycle
from math import ceil

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
    print(args.cpu_frames)
    print(args.gpu_frames)


    ## Preparing 3D trajectory plot
    trajFig = plt.figure(figsize=(18,12))
    if False:
        ax = trajFig.add_subplot()
    else:
        ax = trajFig.add_subplot(projection='3d')
        ax.set_zlabel('z position (m)', fontsize=20)

    #ax.axis('equal')
    ax.grid(linestyle='dotted', linewidth=1)

    ax.set_xlabel('x-position (m)', fontsize=20)
    ax.set_ylabel('y-position (m)', fontsize=20)
    ax.set_title('Agent Trajectories', fontsize=20)

    ### find and read in the frames.bin for the case we want (or most recent case) ##
    lsCycler = cycle(["solid", "dotted", "dashed"])
    colorCycler = cycle(["red", "blue"])
    labelCycler = cycle(["cpu", "gpu"])
    markers = mpl.markers.MarkerStyle.filled_markers[:]
    z_offset = 0
    for frame_file in [args.cpu_frames, args.gpu_frames]: 
        mkCycler = cycle(markers)
        finalFramesDir = find_frames(frame_file)
        finalFramesLoc = finalFramesDir + '/frames.bin'

        ## Read in the frames from the protocol buffer
        #print('Reading frames...\n')
        frames = utils.read_frames(finalFramesLoc)

        ### Organize data for plotting ##
        ## Make a list of lists for each coordinate (one list per entity)
        allentX = [[] for ent in frames[0].contact]
        allentY = [[] for ent in frames[0].contact]
        allentZ = [[] for ent in frames[0].contact]

        for fr in frames: # at each timestep
            for con in fr.contact: # for each contact/entity
                conidx = con.id.id - 1
                # append each entity's x, y, z position to the appropriate list
                allentX[conidx].append(con.state.position.x)
                allentY[conidx].append(con.state.position.y)
                allentZ[conidx].append(con.state.position.z + z_offset)

        ### Plot data ##


        ## Markers the plot can use
        ## How many markers on the longest-lived entity's trajectory line
        num_mk = 8
        if num_mk > len(allentX):
            num_mk = len(allentX)
        ## How many data points before a marker is placed
        mkInterval = ceil(len(frames)/num_mk)
        ## Marker fillstyles (set by team)
        fs = ['full','none']
        #fs = mpl.markers.MarkerStyle.fillstyles # more fillstyles

        try:
            ## Plot each entity's trajectory and label with team number and agent number ##
            linestyle = next(lsCycler)
            device = next(labelCycler)
            for con in frames[0].contact:
                conidx = con.id.id - 1
                labelstr = device + ': T' + str(con.id.team_id) + ', #' + str(conidx+1)

                if False:
                    ax.plot(allentX[conidx],allentY[conidx],
                            marker=next(mkCycler),markevery=mkInterval,
                            fillstyle=fs[con.id.team_id-1],label=labelstr,
                            linewidth=3)
                else:
                    ax.plot(allentX[conidx],allentY[conidx],allentZ[conidx],
                            marker=next(mkCycler),markevery=mkInterval,
                            fillstyle=fs[con.id.team_id-1],label=labelstr,
                            linestyle=linestyle, linewidth=1)
        except:
            raise Exception('Error during plotting procedure! Please verify that you are plotting a valid case.\n')


        max_x = max(max(allentX)) 
        max_y = max(max(allentY)) 
        max_z = max(max(allentZ)) 
        min_x = max(min(allentX)) 
        min_y = max(min(allentY)) 
        min_z = max(min(allentZ)) 
        (ax_min_x, ax_max_x) = ax.get_xlim()
        (ax_min_y, ax_max_y) = ax.get_ylim()
        (ax_min_z, ax_max_z) = ax.get_zlim()
    

        ax.set_xlim(min(ax_min_x, min_x), max(ax_max_x, max_x))
        ax.set_ylim(min(ax_min_y, min_y), max(ax_max_y, max_y))
        ax.set_zlim(min(ax_min_z, min_z), max(ax_max_z, max_x))

        # Create legend
        numEntPerCol = 30
        numLegCols = int(len(allentX)/numEntPerCol)
        if numLegCols < 1:
            numLegCols = 1
        leg = ax.legend(loc='center left',fontsize=9,markerscale=0.8,ncol=numLegCols,bbox_to_anchor=(1,0.5))
        leg.set_draggable(True)
        z_offset += 100

    # Shrink plot's axes to make some room for legend
    axLoc = ax.get_position()
    ax.set_position([axLoc.x0, axLoc.y0, axLoc.width*0.8, axLoc.height])
    try:
        ## Show and save figure ##
        trajFig.show()
        trajFig.savefig(os.environ['HOME']+'/trajectory_plot.png')

    except:
        raise Exception('Could not show plot. Please verify that you are plotting a valid case.')

    # Press Enter to close the plot and end the program
    if sys.version_info <= (3,0):
        raw_input('Press Enter to close plot and exit.')
    else:
        input('Press Enter to close plot and exit.')

if __name__ == '__main__':
    main()
