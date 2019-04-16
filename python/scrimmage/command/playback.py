#!/usr/bin/env python

import subprocess

def main(args):
    subprocess.call(["scrimmage-playback", args.logdir_path])
