#!/usr/bin/env python

import subprocess

def main(args):
    subprocess.call(["scrimmage-plugin", args.plugin])
