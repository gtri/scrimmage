#!/usr/bin/env python

import os
import sys
import subprocess
import argparse

class cd:
    """Context manager for changing the current working directory"""
    def __init__(self, newPath):
        self.newPath = os.path.expanduser(newPath)

    def __enter__(self):
        self.savedPath = os.getcwd()
        os.chdir(self.newPath)

    def __exit__(self, etype, value, traceback):
        os.chdir(self.savedPath)

def is_process_running(pid):
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False

def main():
    parser=argparse.ArgumentParser(description='Run MOOS in parallel with SCRIMMAGE.')
    parser.add_argument('--init_script',
                        type=str,
                        help='Initial script to execute before sim starts.')
    parser.add_argument('--scrimmage_mission_file',
                        type=str, default='../missions/moos-ex1.xml',
                        help='SCRIMMAGE mission file')
    parser.add_argument('--moos_launch_script',
                        type=str, default='./launch.sh',
                        help='MOOS launch script')
    parser.add_argument('--moos_mission_dir',
                        type=str,
                        default='../data/moos/missions/s1_alpha',
                        help='MOOS mission dir')
    parser.add_argument('--time_warp',
                        type=str, default='10',
                        help='MOOS time warp'),
    parser.add_argument('--finalize_script',
                        type=str,
                        help='Script to call after simulation is complete.')
    parser.add_argument('--mission_default',
                        type=str,
                        help='Use mission default settings (m2_berta, s1_alpha)')
    args = parser.parse_args()

    if args.mission_default is not None:
        if args.mission_default == 'm2_berta':
            args.init_script = "./launch.sh --just_make " + args.time_warp
            args.scrimmage_mission_file = '../missions/moos-ex2.xml'
            args.moos_launch_script = './launch.sh'
            args.moos_mission_dir = '../data/moos/missions/m2_berta'
            args.time_warp = "8"
            args.finalize_script = "./clean.sh"
        elif args.mission_default == 's1_alpha':
            args.scrimmage_mission_file = '../missions/moos-ex1.xml'
            args.moos_launch_script = './launch.sh'
            args.moos_mission_dir = '../data/moos/missions/s1_alpha'
            args.time_warp = "8"
            args.finalize_script = "./clean.sh"
        else:
            print('Invalid mission default.')
            return -1

    if args.init_script is not None:
        with cd(args.moos_mission_dir):
            subprocess.call(args.init_script, shell=True)

    procs = [] # list to hold processes that are forked from main

    # Launch scrimmage
    procs.append(subprocess.Popen(["scrimmage",
                                   args.scrimmage_mission_file]))

    # Launch the user's custom MOOS script. This MOOS script should block using
    # uMAC and clean up its own child processes.
    with cd(args.moos_mission_dir):
        script_cmd = args.moos_launch_script + " " + str(args.time_warp)
        subprocess.call(script_cmd, shell=True)

    # Kill off any forked processes
    for p in procs:
        p.terminate()  # Sends SIGTERM
        p.wait()

    if args.finalize_script is not None:
        with cd(args.moos_mission_dir):
            subprocess.call(args.finalize_script)

    return 0


if __name__ == '__main__':
    sys.exit(main())
