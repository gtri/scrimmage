# SCRIMMAGE Dependencies, Packaging, and PPAs

SCRIMMAGE currently targets Ubuntu Xenial. This means that if a package exists
in Ubuntu Xenial that SCRIMMAGE depends on, apt-get will be used to install the
package. However, in some cases, we need much newer versions of the software
than are provided by Canonical. This subproject is for building the few
dependencies that have to be built from source because we require custom or
newer versions of the software. This project can build deb, rpm, source
binaries (for PPA), and local builds for the required dependencies.

## Build Local Versions of dependencies

This is the old process for installing SCRIMMAGE's dependencies, but it will be
maintained for system flexibility. After running the following commands, the
dependencies will be installed under ~/.local :

    $ cd /path/to/3rd-party
    $ mkdir -p build && cd build
    $ cmake ..
    $ source ~/.scrimmage/setup.bash
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

## Build debian source packages

In order for Launchpad to provide debian binary packages, debian source
packages have to be uploaded to Launchpad via dput. This project provides cmake
build targets for the building of these source packages. It is recommended that
you use a different build directory for building PPA source packages since some
of the CMake variables may conflict with your locally built binary
packages. You have to manually specify PPA targets to build since they are not
built by default to discourage uploading to Launchpad accidently. By default,
the packages are uploaded to Kevin DeMarco's PPA
(ppa:kevin-demarco/scrimmage). Thus, if you don't have his GPG keys (which you
don't), but you still want to upload to Launchpad, you will need to create a
Launchpad account, a PPA, and setup your GPG keys with the Ubuntu
keyserver. You can change the PPA and GPG key ID with the "PPA" and
"GPG\_KEY\_ID" CMake cache variables.

    $ cd /path/to/3rd-party
    $ mkdir -p build-ppa && cd build-ppa
    $ cmake -DBUILD_SOURCE_PACKAGES=ON ..
    $ make pybind11-ppa
    $ make jsbsim-ppa
    $ make protobuf3-ppa         # wait for successful build on server
    $ make grpc-ppa              # wait for successful build on server
    $ make dependencies-ppa
    
As changes are made to the dependencies source code and the debian packaging
configuration files, the package maintainer needs to bump the versions for the
SOURCE\_VERSION and PPA\_NUMBER in the root CMakeLists.txt file. Note: Allow
protobuf3 to build successfully on the Launchpad server before building
grpc. Then, allow grpc to build successfully before building the
dependencies-ppa.

## Build GRPC and Protobuf Python Packages from Source

### Install protobuf Python package

Protobuf should be installed using the source files that are compiled during
build.  Using protobuf from PyPI (i.e. what you get with `pip install protobuf`
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
