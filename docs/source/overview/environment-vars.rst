.. _environmet_vars:

SCRIMMAGE Environment
=====================

Environment Variables
---------------------

SCRIMMAGE relies on several environment variables to find XML files, plugins,
3D mesh files, binary data files, and configuration files. The following is a
description of SCRIMMAGE's environment variables.

- **SCRIMMAGE_PLUGIN_PATH** : SCRIMMAGE searches directories on the
  SCRIMMAGE_PLUGIN_PATH for a plugin's XML file and its shared library binary
  file. For example, when a user specifies the name "Straight" for an entity's
  ``autonomy`` tag in a SCRIMMAGE XML mission file:

  .. code-block:: xml

     <autonomy>Straight</autonomy>

  SCRIMMAGE searches for the file ``Straight.xml`` by recursively iterating
  over the directories specified in SCRIMMAGE_PLUGIN_PATH. In a standard
  SCRIMMAGE project, the following directories are appended to the
  SCRIMMAGE_PLUGIN_PATH: ``/path/to/scrimmage-project/plugin_libs`` and
  ``/path/to/scrimmage-project/include/scrimmage/plugins``. The ``plugin_libs``
  directory contains the shared library files and the include directory
  contains the plugin XML files.

- **SCRIMMAGE_DATA_PATH** : SCRIMMAGE searches the data path for XML files that
  load terrain data, 3D meshes, and images. In a standard SCRIMMAGE project,
  the following directory is appended to the SCRIMMAGE_DATA_PATH:
  ``/path/to/scrimmage-project/scrimmage/data``.

Environment Management
----------------------  

The SCRIMMAGE environment is managed in the user's ``~/.scrimmage``
directory. When a user runs the ``cmake`` command in a SCRIMMAGE project, an
environment variable file for the project is written to
``~/.scrimmage/env/<PROJECT_NAME>-setenv``, where ``<PROJECT_NAME>`` is the
name of the SCRIMMAGE project. Also, when cmake is run, the file
``~/.scrimmage/setup.bash`` is modified to load the environment variables in
the ``<PROJECT_NAME>-setenv`` file. The user is discouraged from directly
editing files of the form ``<PROJECT_NAME>-setenv``, but the user can edit the
``~/.scrimmage/setup.bash`` by commenting out (using a #) or changing the order
in which setenv files are loaded. SCRIMMAGE will not uncomment a setenv file
that has been commented out, but it will append to the setup.bash file if the
project changes directories and the user runs cmake again.

SCRIMMAGE will never directly edit a user's ``~/.bashrc`` file. Instead, the
installation instructions suggest that the user manually sources
``~/.scrimmage/setup.bash`` in their ``~/.bashrc`` file. As per the README
instructions, the user can add the sourcing of the SCRIMMAGE environment to the
``~/.bashrc`` by running the following command: ::

  $ echo "source ~/.scrimmage/setup.bash" >> ~/.bashrc

Thus, SCRIMMAGE environment variable files are typically sourced in the
following order:

1. ``~/.bashrc`` points to ``~/.scrimmage/setup.bash``.
2. ``~/.scrimmage/setup.bash`` points to ``~/.scrimmage/env/scrimmage-deps-setenv``
3. ``~/.scrimmage/setup.bash`` points to ``~/.scrimmage/env/scrimmage-setenv``
4. ``~/.scrimmage/setup.bash`` points to ``~/.scrimmage/env/<PROJECT_NAME>-setenv``
   
SCRIMMAGE Log Files
-------------------

SCRIMMAGE log files are located under ~/.scrimmage/logs by default. However,
the user can change the log directory by modifying the ``log_dir`` XML tag in a
SCRIMMAGE mission XML file.
