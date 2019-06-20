#!/usr/bin/env python

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

import sys
import time
import logging
import os
import re
import argparse

from watchdog.observers import Observer
from watchdog.events import LoggingEventHandler
from watchdog.events import FileSystemEventHandler

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

class CSVPlot (FileSystemEventHandler):
    def __init__(self):
        parser = argparse.ArgumentParser(description='SCRIMMAGE Plotter')
        add = parser.add_argument
        add('-c', '--csv_filename', help='CSV filename. (e.g., trajectory.csv)', required=True)
        add('-x', '--x_axis', default='t', help='CSV header for x-axis plot.')
        add('-y', '--y_axis', nargs='+', help='<Required> Space separated list of variables to plot on y-axis', required=True)
        add('-z', '--z_axis', help='CSV header for z-axis plot.')
        add('-l', '--log_dir', default=os.path.expanduser("~/.scrimmage/logs"), help='SCRIMMAGE log directory')
        add('-e', '--equal_axes', action="store_true", help='Ensure that x and y axes are equal scale')
        add('-t', '--title', default='Variables', help='Title for plot')
        add('-d', '--dir', default='latest', help='Directory containing CSV file. (e.g., "latest" ')
        add('-s', '--static', action="store_true", help='Stops plots from updating when new data is recieved')

        args = parser.parse_args()

        self.plot_title = args.title
        self.equal_axes = args.equal_axes
        self.static = args.static

        self.x_axis_vars = args.x_axis
        self.z_axis_vars = args.z_axis
        self.y_axis_vars = args.y_axis

        self.csv_filename = args.csv_filename

        self.watch(args.log_dir + '/' + args.dir)

        self.y_plot_dict = {0:[]}
        self.subplot_num = 1
        self.plot_names = {}
        for i in range(len(self.y_axis_vars)):
            col_name = self.y_axis_vars[i]
            colon_loc = col_name.find(':')
            if colon_loc == -1:
                subplot = 0
                colon_loc = len(col_name)
            else:
                subplot = int(col_name[colon_loc+1:colon_loc+2])
                if subplot >= self.subplot_num:
                    self.subplot_num = subplot+1
            
            equal_loc = col_name.find('=')
            if equal_loc == -1:
                if subplot not in self.plot_names:
                    self.plot_names[subplot] = ''
            else:
                self.plot_names[subplot]=col_name[0:equal_loc]
            col_name = col_name.split('=', 1)[-1]
            col_name = col_name.split(':', 1)[0]
            self.y_plot_dict.setdefault(subplot, []).append(col_name[0:colon_loc])

    def on_any_event(self, event):
        #if event.is_directory:
        #    return None

        if event.event_type == 'deleted':
            self.watch(self.path)

        #elif event.event_type == 'created':
        #    pass
        #elif event.event_type == 'modified':
        #    pass
        #
        #elif event.event_type == 'moved':
        #    pass

    def watch(self, path):
        self.path = path
        if os.path.islink(self.path):
            self.full_path = os.readlink(self.path)
        else:
            self.full_path = self.path

        self.event_handler = self
        self.observer = Observer()
        self.observer.schedule(self.event_handler, self.path, recursive=True)
        self.observer.start()

    def stop(self):
        self.observer.stop()

    def join(self):
        self.observer.join()

    def plot(self,i):
        df = pd.read_csv(self.full_path + '/' + self.csv_filename)

        locals().update(df)

        if df.shape[0] == 0:
            # If there are no rows in the dataframe, just return
            return

        for i in range(0,self.subplot_num):
            self.axarr[i].clear()
            if i in self.y_plot_dict:
                for var in self.y_plot_dict[i]:
                    if self.z_axis_vars is not None:
                        z = re.match(r'\{(.*)\}', var)
                        if z != None:
                            exec('new_var = ' + z.group(1))
                            self.axarr[i].plot(df[self.x_axis_vars].values,
                                       new_var)
                        else:
                            self.axarr[i].plot(df[self.x_axis_vars].values,
                                               df[var].values,
                                               df[self.z_axis_vars].values)
                    else:
                        z = re.match(r'\{(.*)\}', var)
                        if z != None:
                            exec('new_var = ' + z.group(1))
                            self.axarr[i].plot(df[self.x_axis_vars].values,
                                       new_var)
                        else:
                            self.axarr[i].plot(df[self.x_axis_vars].values,
                                               df[var].values)
                if self.plot_names[i] == '':
                    self.axarr[i].set_ylabel(self.y_plot_dict[i])
                else:
                    self.axarr[i].set_ylabel(self.plot_names[i])

        self.axarr[len(self.axarr)-1].set_xlabel(self.x_axis_vars)
        self.axarr[0].set_title(self.plot_title)

    def run(self):
        #self.fig, self.axarr = plt.subplots(len(self.y_axis_vars), sharex=True,
        #                                    squeeze=False)
        self.fig = plt.figure()
        #self.fig.suptitle(self.plot_title)

        plot_prj = None
        if self.z_axis_vars is not None:
            plot_prj = '3d'

        self.axarr = [self.fig.add_subplot(self.subplot_num, 1, i+1,
                                           projection=plot_prj) for i in range(self.subplot_num)]

        if self.static:
            self.plot(0)
        else:
            self.ani = animation.FuncAnimation(self.fig, self.plot)

        if self.equal_axes:
            plt.axis('equal')

        plt.show()

if __name__ == "__main__":
    c = CSVPlot()
    c.run()
