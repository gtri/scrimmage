"""Static style analysis tests."""
from __future__ import print_function
import subprocess
import os


def test_cpplint():
    """Code style analysis using cpplint."""
    scrimmage_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    dirs = [os.path.join(scrimmage_root, d)
            for d in ['include', 'share', 'src', 'tools', 'plugins']]

    cmd = ['cpplint', '--verbose=3', '--quiet', '--recursive',
           '--repository', scrimmage_root] + dirs
    print('running the following command:')
    print(' '.join(cmd))
    subprocess.check_call(cmd)

if __name__ == '__main__':
    test_cpplint()
