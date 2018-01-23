.. _install_scrimmage:

Install SCRIMMAGE
-----------------

SCRIMMAGE development is currently targeting the Ubuntu 16.04 operating
system. This means that Ubuntu's package manager (apt-get) is used to install
dependent projects when applicable and the SCRIMMAGE project only builds
dependent projects from source when required. If a SCRIMMAGE user wants to run
SCRIMMAGE on a different operating system, dependent packages will have to be
compiled from source.

The user should refer to the README.md file in SCRIMMAGE's core repository for
detailed installation instructions. However, the following installation steps
are provided for users new to Linux and assume that the user is building
SCRIMMAGE in Ubuntu 16.04. If the user doesn't have an Ubuntu 16.04
installation yet, they should install Ubuntu 16.04 in a VirtualBox
image. Download an `ISO image of Ubuntu 16.04
<https://www.ubuntu.com/download/desktop/>`_ and `VirtualBox
<https://www.virtualbox.org/wiki/VirtualBox/>`_. Make sure to "Install Guest
Additions" in the Ubuntu VirtualBox image to enable hardware acceleration.

Setup git
~~~~~~~~~

Ensure git is installed using your system's package manager. For example,
Ubuntu users should type:

::

    $ sudo apt-get update && sudo apt-get install git

Enter the following to configure git. Replace the example email address and
name below with your own email address and name:

::

    $ git config --global user.email "your_email@example.edu"
    $ git config --global user.name "Your Name"

Install SCRIMMAGE Core and Standard Plugins
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Create a working directory for SCRIMMAGE and change directories to the
   directory that you just created:

   ::

      $ mkdir -p ~/scrimmage
      $ cd ~/scrimmage

2. Use git to download SCRIMMAGE:

   ::

      $ git clone <scrimmage-git-url>
      $ cd scrimmage

3. Use the install-binaries.sh script to install packages that are provided by
   your system's package manager:

   ::

      $ sudo ./setup/install-binaries.sh

4. Install dependencies provided by SCRIMMAGE's PPA:

   ::

      $ sudo add-apt-repository ppa:kevin-demarco/scrimmage
      $ sudo apt-get update

   ::

      $ sudo apt-get install scrimmage-dependencies

   Run the SCRIMMAGE setup script, which adds the ~/.scrimmage directory to
   your local system and sets up some environment variables:

      ::

         $ source /opt/scrimmage/setup.sh

5. Build SCRIMMAGE core and its standard plugins

   ::

      $ mkdir build
      $ cd build
      $ cmake ..
      $ make

   Whenever, you want to use scrimmage, you need to source the
   ~/.scrimmage/setup.bash file or you can place a line in your ~/.bashrc file
   to source it automatically:

   ::

      $ echo "source ~/.scrimmage/setup.bash" >> ~/.bashrc

6. Test that SCRIMMAGE has been installed correctly:

   ::

      $ source ~/.scrimmage/setup.bash
      $ scrimmage ../missions/straight.xml

   You should see the visualization GUI open up and display the simulation. The
   user interface controls are described in :doc:`viewer`.
