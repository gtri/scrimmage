#!/usr/bin/env python

import subprocess

def main(args):
    fwd_args = ["scrimmage-viz"]

    # FIXME find a more elegant solution here
    if args.local_ip:
        fwd_args += ["--local_ip"] + [args.local_ip]
    if args.local_port:
        fwd_args += ["--local_port"] + [args.local_port]
    if args.remote_ip:
        fwd_args += ["--remote_ip"] + [args.remote_ip]
    if args.remote_port:
        fwd_args += ["--remote_port"] + [args.remote_port]
    if args.pos:
        fwd_args += ["--pos"] + [args.pos]
    if args.focal_point:
        fwd_args += ["--focal_point"] + [args.focal_point]

    subprocess.call(fwd_args)
