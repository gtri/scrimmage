"""Tests for unneeded python debugging includes."""
import fnmatch
import os


def test_debug():
    """Open single entity scenario and make sure it banks."""
    python_path = os.path.join(os.path.dirname(__file__), '..')

    # see https://stackoverflow.com/a/2186565
    results = []
    my_fname = os.path.basename(__file__)
    for root, _, filenames in os.walk(python_path):
        for fname in fnmatch.filter(filenames, '*.py'):
            if 'proto' not in fname and my_fname != fname:
                file_w_path = os.path.join(root, fname)
                with open(file_w_path, 'r') as f:
                    lines = f.read().splitlines()
                for i, l in enumerate(lines):
                    if 'import lvdb' in l or \
                       'import ipdb' in l or \
                       'import pdb' in l:

                        results.append(file_w_path + ":" + str(i))

    if results:
        print("The following files have debugging imports:\n" +
              "\n".join(results))
        raise NameError("Files include debug imports")

if __name__ == '__main__':
    test_debug()
