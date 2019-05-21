"""Static analysis tests."""
from __future__ import print_function
import subprocess
import os


def test_cppcheck():
    """Code static analysis using cppcheck."""
    root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    dirs = [os.path.join(root_dir, d)
            for d in ['include', 'share', 'src', 'tools', 'plugins']]
    enabled_checks = \
        'warning,style,information,performance,portability,missingInclude'
    cmd = \
        ['cppcheck', '--quiet', '--language=c++',
         '--inline-suppr',
         '--error-exitcode=1', '--enable=' + enabled_checks,
         '--suppress=copyCtorAndEqOperator',
         '-I', os.path.join(root_dir, 'python/scrimmage/bindings/include'),
         '-I', os.path.join(root_dir, 'include')] + dirs
    print('running the following command:')
    print(' '.join(cmd))
    subprocess.check_call(cmd)

if __name__ == '__main__':
    test_cppcheck()
