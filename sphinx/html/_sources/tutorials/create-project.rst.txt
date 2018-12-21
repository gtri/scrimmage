.. _create_project:

Create a SCRIMMAGE Plugins Project
----------------------------------

The SCRIMMAGE core project provides a general purpose agent-based simulation
framework. The SCRIMMAGE core project also provides example behaviors in the
form of Autonomy plugins, motion model plugins, low-level controller plugin,
metrics plugins, sensor plugins, and entity interaction plugins. While
SCRIMMAGE core developers will make modifications to the SCRIMMAGE core
project, most third-party developers will not modify the SCRIMMAGE core or its
plugins. Instead, third-party developers will create their own SCRIMMAGE
plugins project and develop their plugins in their third-party project. To
facilitate setting up this third-party project, SCRIMMAGE provides a script
that generates a template third-party project.  The user just has to provide a
name for the project and the location to place the project.

To create a SCRIMMAGE plugins project with the name "my-scrimmage-plugins" in
the ~/scrimmage directory, run the following script: ::

  $ cd /path/to/scrimmage/scripts
  $ mkdir ~/scrimmage
  $ ./create-scrimmage-project.py my-scrimmage-plugins ~/scrimmage

Now build your project: ::

  $ cd ~/scrimmage/my-scrimmage-plugins
  $ mkdir build && cd build
  $ cmake ..
  $ make
  
Now to make sure scrimmage will find the new plugins: ::

  $ source ~/.scrimmage/setup.bash

Running the ``cmake`` command will be successful if the SCRIMMAGE core project has
already been built/installed on your system. The final output of the CMake
command will look like the following if it was successful: ::

  -- Configuring done
  -- Generating done
  -- Build files have been written to: /path/to/scrimmage/my-scrimmage-plugins/build

The ``make`` command should be succesful, but since we haven't created any
plugins yet, nothing will be built.
