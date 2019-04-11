#!/usr/bin/env python
# PYTHON_ARGCOMPLETE_OK

import sys
import argparse, argcomplete
from command import scrimmage_run, scrimmage_viz, scrimmage_playback, \
    scrimmage_plugin

CMD_DISPATCHER = {
    'run': scrimmage_run.run,
    'viz': scrimmage_viz.run,
    'playback': scrimmage_playback.run,
    'plugin': scrimmage_plugin.run
}

def parse_commands():
    # Create top-level parser
    parser = argparse.ArgumentParser()
    parser.add_argument('-v','--version', action='version', version='%(prog)s 0.3',
        help='Show scrimmage version.')
    subparsers = parser.add_subparsers(dest='command')

    # main scrimmage run program
    parser_run = subparsers.add_parser('run')
    parser_run.add_argument('mission', metavar='mission.xml', type=str,
        help='Scrimmage mission file to run.')

    # scrimmage viz
    parser_viz = subparsers.add_parser('viz')
    parser_viz.add_argument('-i','--local-ip', type=str,
        help='The local IP address where this viewer will run.')
    parser_viz.add_argument('-p', '--local-port', type=str,
        help='The local port where this viewer will listen.')
    parser_viz.add_argument('-r','--remote-ip', type=str,
        help='The remote IP address where SCRIMMAGE is running.')
    parser_viz.add_argument('-o','--remote-port', type=str,
        help='The remote port where SCRIMMAGE is running.')
    parser_viz.add_argument('--pos', type=float,
        help='Camera position.')
    parser_viz.add_argument('--focal-point', type=float,
        help='Camera focal point.')

    # playback tool
    parser_playback = subparsers.add_parser('playback')
    parser_playback.add_argument('logdir_path', metavar='path', type=str,
        help='Path to log directory.')

    # plugin finder
    parser_plugin = subparsers.add_parser('plugin')
    parser_plugin.add_argument('plugin', metavar='Plugin', type=str,
        help='Plugin name.')

    argcomplete.autocomplete(parser, exclude=['-h', '--help'])

    args = parser.parse_args()
    CMD_DISPATCHER[args.command](args)

def main(script_name='scrimpy'):
    parse_commands()

if __name__ == '__main__':
    sys.exit(main())
