#!/usr/bin/env python

import subprocess

def run(args):
    subprocess.call(["scrimmage-plugin", args.plugin])
