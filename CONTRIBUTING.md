# How to contribute

## Bug reports or feature requests

[Open an issue](https://github.com/gtri/scrimmage/issues) with your bug report
or feature request.

## Contributing code

1. [Create a fork](https://github.com/gtri/scrimmage/fork)

2. Before creating a new commit, make sure your changes pass our cpplint and
   cppcheck tests by first installing pytest, cppcheck, and cpplint:

        $ sudo apt-get install cppcheck python-pytest
        $ sudo pip install cpplint

   Now run the static code tests, which have to pass:

        $ cd /path/to/scrimmage/test
        $ py.test test_cppcheck.py
        $ py.test test_cpplint.py


3. Push a commit with a meaningful message to your own fork.

4. If you have multiple commits related to the same feature, please consider
   squashing your commits into a single commit (e.g., git rebase -i HEAD~10)

5. [Submit a pull request](https://github.com/gtri/scrimmage/compare)

6. Your pull request will not be accepted unless the commit passes all of the
   tests on [Travis](https://travis-ci.org/gtri/scrimmage). If you want to run
   the complete build and tests locally before submitting a pull request, use
   Docker:

        $ cd /path/to/scrimmage
        $ docker build -t local/scrimmage:test -f ./ci/dockerfiles/ubuntu-16.04 .
