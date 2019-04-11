#!/usr/bin/env python

import subprocess

def run(args):
    subprocess.call(["scrimmage-playback", args.logdir_path])
