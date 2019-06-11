#!/usr/bin/env python
# PYTHON_ARGCOMPLETE_OK

import sys
import argparse, argcomplete

def parse_commands():
    # Create top-level parser
    parser = argparse.ArgumentParser()
    parser.add_argument('-v','--version', action='version', version='%(prog)s 0.3',
        help='show scrimmage version')
    subparsers = parser.add_subparsers(dest='command')

    # list environment
    parser_playback = subparsers.add_parser('env',
        help='list scrimmage environment variables')

    # playback tool
    parser_playback = subparsers.add_parser('playback',
        help='replay a previous simulation from the log directory')
    parser_playback.add_argument('logdir_path', metavar='path', type=str,
        help='path to log directory')

    # plugin finder
    parser_plugin = subparsers.add_parser('plugin',
        help='find a plugin on the SCRIMMAGE_PLUGIN_PATH by name')
    parser_plugin.add_argument('plugin', metavar='Plugin', type=str,
        help='plugin name')
    parser_plugin.add_argument('--path', metavar='Plugin', type=str,
        help='plugin name')

    # main scrimmage run program
    parser_run = subparsers.add_parser('run',
        help='run a scrimmage simulation from a mission xml file')
    parser_run.add_argument('mission', metavar='mission.xml', type=str,
        help='scrimmage mission file to run')

    # scrimmage viz
    parser_viz = subparsers.add_parser('viz',
        help='start the scrimmage visualizer')
    parser_viz.add_argument('-i','--local-ip', type=str,
        help='the local IP address where this viewer will run')
    parser_viz.add_argument('-p', '--local-port', type=str,
        help='the local port where this viewer will listen')
    parser_viz.add_argument('-r','--remote-ip', type=str,
        help='the remote IP address where SCRIMMAGE is running')
    parser_viz.add_argument('-o','--remote-port', type=str,
        help='the remote port where SCRIMMAGE is running')
    parser_viz.add_argument('--pos', type=float,
        help='camera position')
    parser_viz.add_argument('--focal-point', type=float,
        help='camera focal point')

    # print help if no arguments given
    if len(sys.argv)==1:
        parser.print_help(sys.stderr)
        sys.exit(1)

    # for tab completion
    argcomplete.autocomplete(parser, exclude=['-h', '--help'])
    args = parser.parse_args()
    return args

def dispatch(args):
    from command import run, viz, playback, plugin, util
    CMD_DISPATCHER = {
        'run': run.main,
        'viz': viz.main,
        'playback': playback.main,
        'plugin': plugin.main,
        'env': util.env,
    }

    # run command
    CMD_DISPATCHER[args.command](args)

def main(script_name='scrimpy'):
    args = parse_commands()
    dispatch(args);

if __name__ == '__main__':
    sys.exit(main())
