===========================
SCRIMMAGE / ROS Integration
===========================

For more intricate simulation setups, you may want to be able generate
scrimmage mission files more dynamically. This is especially important when
using scrimmage with ROS. For example, we may want to launch several ROS
autonomies in their own ROS namespaces and start scrimmage with a mission file
that matches the ROS configuration. In this case, scrimmage will generate the
sensor data required by the ROS autonomy, the ROS autonomy will publish a
control signal, scrimmage will subscribe to the control signal, and scrimmage
will apply the control signal to the simulated motion model. The scrimmage
sensor plugins that provide the sensor data publish to ROS topics and the
scrimmage autonomy plugins subscribe to the control signals being published by
the ROS autonomy. In other words, the ROS nodes are performing the real
decision making for the agent and the scrimmage autonomy plugin is just a
translator from ROS messages to internal scrimmage data structures.

In this tutorial, we will be using the scrimmage python project and the
`scrimmage_ros <https://github.com/syllogismrxs/scrimmage_ros>`_ repository to
generate a scrimmage mission file at run-time and launch multiple ROS 2D
navigation stacks in their own ROS namespaces.

Build SCRIMMAGE with Python and ROS
-----------------------------------

To get started, let's build scrimmage with ROS plugins and python bindings in a
virtual environment.

.. code-block:: bash

   # Setup a python virtual environment
   $ cd /path/to/scrimmage
   $ mkdir -p build && cd build
   $ python3 -m venv env         # Create the python environment
   $ . ./env/bin/activate        # Activate the environment

   # Install scrimmage python package dependencies
   $ pip install -r ../python/requirements.txt

   # Enable scrimmage dependencies
   $ . /opt/scrimmage/*/setup.sh

   # Enable the ROS dependencies. Replace ${ROS_VERSION} with "kinetic",
   # "melodic", etc.
   $ . /opt/ros/${ROS_VERSION}/setup.sh

   # Configure scrimmage to build ROS plugins and python 3.5+
   $ cmake .. -DBUILD_ROS_PLUGINS=ON -DPYTHON_MIN_VERSION=3.5

   $ make -j7  # Build scrimmage

   # Install the scrimmage python package
   $ pip install -e ../python

After successfully building scrimmage, let's run a simple test of the scrimmage
Mission Generator python class

.. code-block:: bash

   # Activate the locally built scrimmage environment, which puts the scrimmage
   # executable on the $PATH and updates the $SCRIMMAGE_PLUGIN_PATH.
   $ . ~/.scrimmage/setup.sh

   # Run the Mission Generator test, which generates a mission and runs it
   # with scrimmage.
   $ python ../python/tests/test_MissionGenerator.py

The ``MissionGenerator`` class uses template files, the ``jinja2`` python
package, and a ``mission.yaml`` file to generate scrimmage mission files. For
example, the `test_MissionGenerator.py
<https://github.com/gtri/scrimmage/blob/master/python/tests/test_MissionGenerator.py>`_
test program, uses the template files in `scrimmage/python/tests/templates
<https://github.com/gtri/scrimmage/tree/master/python/tests/templates>`_ and
`mission.yaml
<https://github.com/gtri/scrimmage/blob/master/python/tests/templates/mission.yaml>`_.
The ``MissionGenerator`` class constructor takes the path to the
``mission.yaml`` file as input:

.. code-block:: python

   mg = MG.MissionGenerator('/path/to/mission.yaml')

You can access the contents of the generated scrimmage mission XML through the
``MissionGenerator`` class' ``mission`` member variable.

.. code-block:: python

   mission_xml = mg.mission

Typically, the string that contains the contents of the scrimmage mission file
is written to a temporary file:

.. code-block:: python

   with tempfile.NamedTemporaryFile('w') as fp:
       fp.write(mg.mission)
       fp.flush()

and then the python ``subprocess`` package can be used to execute
``scrimmage``, where the path to the temporary file is passed as an argument to
``scrimmage``.

.. code-block:: python

       cmd = "scrimmage %s" % fp.name
       subprocess.check_call(cmd.split(), env=os.environ.copy())

Mission YAML File Format
------------------------

Let's take a look at the ``mission.yaml`` file format to understand how the
``MissionGenerator`` class generates a scrimmage mission file from template
files.

.. code-block:: yaml

   version: "1.0"

   # Relative to current file
   template_directories:
       - .

   # Configuration for overall mission file
   mission_file:
       template: mission.template.xml
       config:
           time_warp: 0
           latitude_origin: 35.721025
           longitude_origin: -120.767925
           altitude_origin: 0.0

   # Configuration for each entity that will be injected into the mission file
   entities:
       - template: entity.template.xml
         config:
             id: 1
             x: -10
             y: 10
             z: -5
             heading: 270

       - template: entity.template.xml
         config:
             id: 2
             x: 10
             y: 10
             z: -5
             heading: 0

The ``version`` specifies the YAML format version number for the
``MissionGenerator`` class. The ``template_directories`` variable specifies a
list of absolute or relative (to the ``mission.yaml`` file) paths to be used
when searching for template files. In this case, the template files are located
in the same directory as the ``mission.yaml`` file since the ``.`` path is
included. The ``mission_file`` specifies the top-level scrimmage mission file
template (``mission.template.xml``) and the ``config`` values that should be
used during variable substitution by ``jinja2``. For example, the
``mission.template.xml`` file contains the line:

.. code-block:: xml

   time_warp="{{ config.time_warp }}"

and the ``config`` block contains the value of ``0`` for the ``time_warp``:

.. code-block:: yaml

   config:
       time_warp: 0

After mission file generation, the scrimmage mission file will contain the
text:

.. code-block:: xml

   time_warp="0"

The ``mission.yaml`` file also contains a list of ``entities``, where each
entity is specified by a ``template`` file and a variable ``config`` block that
is used during variable substitution.

.. code-block:: yaml

   entities:
       - template: entity.template.xml
         config:
             id: 1
             x: -10
             y: 10
             z: -5
             heading: 270

       - template: entity.template.xml
         config:
             id: 2
             x: 10
             y: 10
             z: -5
             heading: 0

The top-level scrimmage mission template file (``mission.template.xml``) uses a
``jinja2`` for-loop to generate each entity:

.. code-block:: python

   {% for entity in config.entities %}
   {{ entity }}
   {% endfor %}

An advantage to this template process is that entity templates can be reused
across multiple scrimmage mission files without having to copy and paste XML
blocks.

Launching SCRIMMAGE and ROS in Parallel
---------------------------------------

We will now use the ``scrimmage_ros`` package to launch two ROS agents running
the 2D navigation stack, generate a scrimmage mission file, and pass the
generated mission file to the ``scrimmage`` executable.

Build scrimmage_ros
~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   $ mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
   $ git clone git@github.com:SyllogismRXS/scrimmage_ros.git

   # Source the ROS, scrimmage, and scrimmage python environments
   $ . /opt/ros/${ROS_VERSION}/setup.sh              # ROS
   $ . ~/.scrimmage/setup.bash                       # scrimmage
   $ . ~/scrimmage/scrimmage/build/env/bin/activate  # scrimmage python

   # Build scrimmage_ros
   $ cd ~/catkin_ws
   $ catkin_make

Now that the ``scrimmage_ros`` package has been built, source the catkin_ws
environment and run the python script that will launch the ROS agents and
scrimmage:

.. code-block:: bash

   $ . devel/setup.sh
   $ python ./src/scrimmage_ros/src/scrimmage_ros/test/test_entity_launch.py

Enter CTRL+c to end the simulation. Let's take a look at the
``test_entity_launch.py`` script and the associated template files to see how
the simulation was configured with the ``EntityLaunch`` class:

.. code-block:: python

   entity_launch = EntityLaunch(args.sim_mission_yaml_file, run_dir,
                                args.ros_package_name, args.launch_file,
                                launch_args, None, args.terminal)

The ``EntityLaunch`` takes a ``mission.yaml`` file as input, generates the
scrimmage mission file, executes scrimmage with the generated mission file, and
executes roslaunch for each entity specified by the ``args.ros_package_name``
and ``args.launch_file``. After the initial ``EntityLaunch`` instance is
created, we can also append to the ``processes`` that will be launched:

.. code-block:: python

   # Append a custom ros launch file to the processes list to launch a common
   # map server for both agents. The map server exists outside of a ROS
   # namespace
   entity_launch.processes.append(
       { 'command': sru.ros_launch('scrimmage_ros', 'map_server.launch', ''),
         'env': os.environ.copy(),
         'console': False,
         'terminal': Terminal.gnome
       }
   )

Finally, the processes are actually executed in parallel with the ``run()``
method:

.. code-block:: python

   # Run the processes. Blocking.
   entity_launch.run()
