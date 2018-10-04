"""Utilities for interacting with data.

@file

@section LICENSE

Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)

This file is part of SCRIMMAGE.

  SCRIMMAGE is free software: you can redistribute it and/or modify it under
  the terms of the GNU Lesser General Public License as published by the
  Free Software Foundation, either version 3 of the License, or (at your
  option) any later version.

  SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
  License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.

@author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
@author Eric Squires <eric.squires@gtri.gatech.edu>
@date 31 July 2017
@version 0.1.0
@brief Brief file description.
@section DESCRIPTION
A Long description goes here.
"""
from __future__ import division, print_function

import os
import subprocess
import re
import time

import numpy as np
try:
    import matplotlib.pyplot as plt
except ImportError:
    pass


def find_mission(fname):
    """Return the filename with path of a given mission file.

    Raises a StopIteration exception when it cannot find the file.
    """
    files = (os.path.join(path, fname)
             for path in os.environ["SCRIMMAGE_MISSION_PATH"].split(':'))
    return next((f for f in files if os.path.exists(f)))


def tornado(ax, data, labels, base=None, bar_width=0.8, whitespace_buffer=0.1):
    """Create a tornado plot on the given axis.

    data: (n x 2) iterable with each row containing the low and high value

    """
    # sort so that the largest difference is at the top
    data = np.asarray(data)
    sorted_indexes = np.abs(data[:, 1] - data[:, 0]).argsort()
    labels = np.asarray(labels)[sorted_indexes]
    sorted_indexes = sorted_indexes[::-1]
    data = data[sorted_indexes, :]

    if base is None:
        base = np.mean(data)

    for i, (low, high) in enumerate(data):

        low_width = abs(base - low)
        high_width = abs(high - base)

        yrange = (len(labels) - 1 - i - bar_width / 2, bar_width)

        if low < high < base:
            ax.broken_barh(
                xranges=[(low, low_width), (high, high_width)],
                yrange=yrange)
        elif low < base < high:
            ax.broken_barh(
                xranges=[(low, low_width), (base, high_width)],
                yrange=yrange)
        elif base < low < high:
            ax.broken_barh(
                xranges=[(base, high_width), (base, low_width)],
                yrange=yrange)
        elif high < low < base:
            ax.broken_barh(
                xranges=[(high, high_width), (low, low_width)],
                yrange=yrange)
        elif high < base < low:
            ax.broken_barh(
                xranges=[(high, high_width), (base, low_width)],
                yrange=yrange)

    # Draw a vertical line down the middle
    ax.axvline(base, color='black')

    # Position the x-axis on the top, hide all the other spines (=axis lines)
    ax.spines['left'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['bottom'].set_visible(False)
    ax.xaxis.set_ticks_position('top')
    ax.xaxis.set_label_position('top')

    # Make the y-axis display the variables
    plt.yticks(range(len(labels)), labels)

    # Set the portion of the x- and y-axes to show
    span = np.max(np.abs(data - base)[:])
    ax.set_xlim(base - (span * (1 + whitespace_buffer)),
                base + (span * (1 + whitespace_buffer)))
    ax.set_ylim(-1, len(labels))


def qsub(num_runs, mission, nodes=None, stdout_dir=None, stderr_dir=None):
    """Submit a grid engine job.

    num_runs: how many runs this job should submit to grid engine.

    nodes: either an integer, range (beg-end), or a comma separated list (of
        integers or ranges). If None (default), then the job will be submitted
        to all available nodes.

    mission: either a mission file (which will be run repeatedly for num_runs)
        or a directory which should contain mission files named "1.xml" to
        "num_runs.xml".

    stdout_dir: where to place stderr for the runs from gridendine. defaults
        to '~/.scrimmage/logs/stdout'

    stderr_dir: where to place stderr for the runs from gridendine. defaults
        to '~/.scrimmage/logs/stdout'
    """
    log_dir = os.path.join(os.path.expanduser('~'), '.scrimmage', 'logs')
    stdout_dir = stdout_dir or os.path.join(log_dir, 'qsub_stdout')
    stderr_dir = stderr_dir or os.path.join(log_dir, 'qsub_stderr')

    # look for scrimmage.sh in path
    sc_script = next((os.path.join(p, 'scrimmage.sh')
                      for p in os.environ['PATH'].split(':')
                      if os.path.isfile(os.path.join(p, 'scrimmage.sh'))))
    for d in [stdout_dir, stderr_dir]:
        try:
            os.makedirs(d)
        except OSError:
            pass

    cmd = ['qsub',
           '-t', '1-' + str(num_runs),
           '-e', stderr_dir,
           '-o', stdout_dir]

    if nodes:
        s = ""
        for n in nodes.split(','):
            if '-' in n:
                beg, end = n.split("-")
                tmp = \
                    "|".join(["node" + str(num)
                              for num in range(int(beg), int(end) + 1)])
                s += tmp
            else:
                s += "node" + n
        cmd += ["-l", 'h=' + s]

    cmd += [sc_script, '-d', mission]
    # print(" ".join(cmd))
    output = subprocess.check_output(cmd)
    job_id = re.match(r"Your job-array (\d+)", output).group(1)
    return int(job_id)


def wait_for_job(job_id):
    """Block until a grid engine job is complete."""
    FNULL = open(os.devnull, 'w')
    while subprocess.call(['qstat', '-j', str(job_id)],
                          stdout=FNULL, stderr=FNULL) == 0:
        time.sleep(1)


def indent(elem, level=0):
    """Make sure the written xml file has indents.

    example
    -------
    import xml.etree.ElementTree as ET
    tree = ET.parse(mission_file)
    root = tree.getroot()
    # alter tree as desired

    indent(root)
    tree.write(out)

    source
    ------
    https://stackoverflow.com/a/33956544
    """
    i = "\n" + level*"  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i


def parallel(num_runs, mission, cores):
    """Runs multiple parallel simulations using GNU Parallel.

    num_runs: how many times to repeat a scrimmage run
    mission: mission file
    cores: how many parallel jobs to run

    This function will block until all the jobs are complete.
    """
    cmd = ['parallel', '--no-notice', '-n', '2', '-j', str(cores),
           'scrimmage', '-j', '{}', '{}', ':::']

    for i in range(num_runs):
        cmd.append(str(i))
        cmd.append(mission)

    FNULL = open(os.devnull, 'w')
    subprocess.call(cmd, stdout=FNULL, stderr=FNULL)
