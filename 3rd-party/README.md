# SCRIMMAGE Dependencies, Packaging, and PPAs

SCRIMMAGE currently targets Ubuntu Xenial. This means that if a package exists
in Ubuntu Xenial that SCRIMMAGE depends on, apt-get will be used to install the
package.

However, in some cases, we need much newer versions of the software than are
provided by Canonical. This subproject is for building the few dependencies
that have to be built from source because we require custom or newer versions
of the software.

This project can build deb, rpm, source binaries (for PPA), and local builds
for the required dependencies.

## Build Local Versions of dependencies

This is the old process for installing SCRIMMAGE's dependencies, but it will be
maintained for system flexibility. After running the following commands, the
dependencies will be installed under `~/.local`:

    $ cd /path/to/3rd-party
    $ mkdir -p build && cd build
    $ cmake ..
    $ source ~/.scrimmage/setup.bash
    $ make

After building these libraries, you can make use of them by adding them to the
`LD_LIBRARY_PATH` and then following the standard build process:

    $ export LD_LIBRARY_PATH="$HOME/.local/lib/"
    $ cd build
    $ cmake .. -DCMAKE_PREFIX_PATH="$HOME/.local"
    $ make

## Build deb packages

deb packages for the dependencies can be built by running the following
commands:

    $ cd /path/to/3rd-party
    $ sudo apt-get install build-essential devscripts
    $ mkdir -p build && cd build
    $ cmake .. -DBUILD_BINARY_PACKAGES=ON -DPACKAGE_OUTPUT_TYPE=deb
    $ make

Similarly, rpm packages can be built:

    $ cd /path/to/3rd-party
    $ sudo apt-get install build-essential rpm
    $ cmake .. -DBUILD_BINARY_PACKAGES=ON -DPACKAGE_OUTPUT_TYPE=rpm
    $ make

## Build Debian source packages

Launchpad requires Debian source packages in order to provide compiled
packages. These source packages are built with the CMake build targets in this
directory and uploaded to Launchpad with
[dput](http://manpages.ubuntu.com/manpages/xenial/man1/dput.1.html).

It is recommended that you use a different build directory for building PPA
source packages since some of the CMake variables may conflict with your
locally built binary packages.

You have to manually specify PPA targets to build since they are not
built by default to discourage uploading to Launchpad accidentally. By default,
the packages are uploaded to Kevin DeMarco's PPA
(`ppa:kevin-demarco/scrimmage`). Thus, if you don't have his GPG keys (which you
don't), but you still want to upload to Launchpad, you will need to create a
Launchpad account, a PPA, and setup your GPG keys with the Ubuntu
keyserver. You can change the PPA and GPG key ID with the `PPA` and
`GPG_KEY_ID` CMake cache variables.

The CMakeDebSrc cmake project is used to build the debian source
packages. CMakeDebSrc is a collection of cmake scripts that wrap around
pbuilder commands to build packages for various Linux distributions and
architectures.

First, clone and install the [CMakeDebSrc project from
GitHub](https://github.com/SyllogismRXS/CMakeDebSrc). You will also need to use
pbuilder to initialize a base tarball and setup a `/.pbuilderrc` file (see
CMakeDebSrc's installation instructions).

    $ cd /path/to/3rd-party
    $ mkdir -p build-ppa && cd build-ppa
    $ cmake -DBUILD_SOURCE_PACKAGES=ON ..
    $ make scrimmage-pybind11-xenial-debuild # build the deb source package
    $ make scrimmage-pybind11-xenial-amd64-local-test  # build a deb binary package locally
    $ make scrimmage-pybind11-xenial-upload-ppa  # upload deb src to PPA

As changes are made to the dependencies source code and the debian packaging
configuration files, the package maintainer needs to bump the versions for the
`SOURCE_VERSION` and `PPA_NUMBER` in the root `CMakeLists.txt` file.

Note: Allow protobuf3 to build successfully on the Launchpad server before
building grpc. Then, allow grpc to build successfully before building the
dependencies-ppa.

## Build GRPC and Protobuf Python Packages from Source

### Install Protobuf Python package

Protobuf should be installed using the source files that are compiled during
build. Using Protobuf from PyPI (i.e. what you get with `pip install protobuf`
) is known to cause crashes.

    $ cd /path/to/scrimmage/3rd-party/build/src/protobuf/python
    $ python setup.py build
    $ sudo python setup.py install

### Build GRPC Python Bindings

    $ cd /path/to/scrimmage/3rd-party/build/src/grpc
    $ sudo pip install -rrequirements.txt
    $ GRPC_PYTHON_BUILD_WITH_CYTHON=1 sudo python setup.py install

If you are using python 3, make sure the futures package isn't installed.

    $ sudo pip3 uninstall futures

# Test Local Build of Source Package

Reference: https://wiki.ubuntu.com/akshmakov/sandbox/Packaging

## Setup pbuilder system

    $ sudo apt-get install pbuilder ubuntu-dev-tools
    $ pbuilder create
    $ pbuilder-dist xenial create

Allow pbuilder to have access to the network during build-time:

    $ echo 'USENETWORK=yes' | sudo tee -a /etc/pbuilderrc

## Test the Debian Source Package Build

## Using CMake

    $ make scrimmage-pybind11-local-test

## Manual

Change directories to the location of the *.dsc file that was created and run
pbuilder:

    $ cd /path/to/*.dsc
    $ pbuilder-dist xenial build scrimmage-vtk8_8.1.0.0-0ppa0.dsc

## Test Resulting Binary Debian Packages

The result of the package-name-local-test target is a Debian binary package,
which is usually output to `~/pbuilder/xenial_result`. To see the contents of
the output binary package, run the following command:

    $ dpkg -c ~/pbuilder/xenial_result/scrimmage-pybind11_0.1.1-0ppa1_amd64.deb

or install it:

    $ sudo dpkg -i ~/pbuilder/xenial_result/scrimmage-pybind11_0.1.1-0ppa1_amd64.deb
