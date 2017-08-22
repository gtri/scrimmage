"""Static analysis tests."""
from __future__ import print_function
import subprocess
import os


def test_cpplint():
    """Code style analysis using cpplint."""
    cmd = ['cpplint', '--verbose=3', '--quiet', '--recursive'] + _get_dirs()[1]
    _run_cmd(cmd)


def test_cppcheck():
    """Code static analysis using cppcheck."""
    root_dir, dirs = _get_dirs()
    enabled_checks = \
        'warning,style,information,performance,portability,missingInclude'
    cmd = \
        ['cppcheck', '--quiet', '--language=c++',
         '--error-exitcode=1', '--enable=' + enabled_checks,
         '-I', os.path.join(root_dir, 'python/scrimmage/bindings/include'),
         '-I', os.path.join(root_dir, 'include')] + dirs
    _run_cmd(cmd)


def _get_dirs():
    scrimmage_root = os.path.join(os.path.dirname(__file__), '..', '..')
    dirs = [os.path.join(scrimmage_root, d)
            for d in ['include', 'share', 'src', 'tools', 'plugins']]
    return scrimmage_root, dirs


def _run_cmd(cmd):
    print('running the following command:')
    print(' '.join(cmd))
    subprocess.check_call(cmd)

if __name__ == '__main__':
    test_cpplint()
    test_cppcheck()
