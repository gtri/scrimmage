#!/usr/bin/env python

import os
import shutil
import sys
import argparse

def main():
    parser=argparse.ArgumentParser(description='Create a SCRIMMAGE plugins project.')
    parser.add_argument('project_name',
                        type=str,
                        help='SCRIMMAGE plugins project name.')
    parser.add_argument('out_dir',
                        type=str,
                        help='Output directory.')

    args = parser.parse_args()

    if not os.path.isdir(args.out_dir):
        print("Output directory doesn't exist: %s" % args.out_dir)
        return -1

    # Get this script's current directory
    this_dir = os.path.dirname(os.path.realpath(__file__))

    # Copy the cmake project template to the output directory
    dst_dir = args.out_dir + "/" + args.project_name
    try:
        shutil.copytree(this_dir + "/templates/cmake-project", dst_dir)
    except OSError:
        print('The destination directory already exists: %s' % dst_dir)
        print('Choose a new output directory or remove the existing directory.')
        return -1
    
    # Search and replace project name over all project files
    project_name_no_dashes = args.project_name.replace('-', '_')
    for dname, dirs, files in os.walk(dst_dir):
        for fname in files:
            fpath = os.path.join(dname, fname)
            with open(fpath) as f:
                s = f.read()
                s = s.replace("(>>>PROJECT_NAME<<<)", args.project_name)
                s = s.replace("(>>>PROJECT_NAME_NO_DASHES<<<)",
                              project_name_no_dashes)
            with open(fpath, "w") as f:
                f.write(s)

    # Rename the ./include/cmake-project to the appropriate project
    # name
    os.rename(dst_dir+"/include/cmake-project",
              dst_dir+"/include/" + args.project_name)

    return 0

if __name__ == '__main__':
    sys.exit(main())
