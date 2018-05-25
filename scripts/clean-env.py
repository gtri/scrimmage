#!/usr/bin/env python

import os
import argparse
import re
import sys

def main():
    parser=argparse.ArgumentParser(description='Cleans scrimmage files found on the system and environment variables set by scrimmage.')
    parser.add_argument('-r','--remove-files',
                        action='store_true',
                        help='Remove scrimmage files from system')
    parser.add_argument('-p', '--prefix',
                        type=str, action='store', default='/usr/local',
                        help='Specific the system prefix to search (default: /usr/local)')
    parser.add_argument('-c', '--clear-env',
                        action='store_true',
                        help='Clear environment variables set by scrimmage')

    args = parser.parse_args()

    # walk the prefix path and find files
    prefix_full = os.path.expanduser(args.prefix)
    full_list = []
    r = re.compile(".*scrimmage.*", flags=re.I)
    for dname, dirs, files in os.walk(prefix_full):
        full_paths = [dname + '/' + f for f in files]
	newlist = filter(r.match, full_paths)
	full_list += newlist

    # print or remove matching files
    if args.remove_files:
        print("Removed the following files:")
    else:
        print("Scrimmage files found:")
    for f in full_list:
        if args.remove_files:
	    os.remove(f)
        print(f)

    # print or clean environment variables
    r = re.compile("SCRIMMAGE.*")
    scrimmage_env_vars = filter(r.match, os.environ.keys())
    if args.clear_env:
        #print("\nUnset the following environment variables:")
        pass
    else:
        print("\nScrimmage environment variables found:")
    for var in scrimmage_env_vars:
        if args.clear_env:
            # FIXME - this doesn't actually clear the parent process's env vars
            del os.environ[var]
        print(var)

    return 0

if __name__ == '__main__':
    sys.exit(main())
