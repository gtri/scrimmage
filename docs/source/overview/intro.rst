SCRIMMAGE Description
---------------------

Simulating Collaborative Robots in Massive Multi-Agent Game Execution, or
SCRIMMAGE, provides a flexible simulation environment for the experimentation
and testing of novel mobile robotics algorithms.  The development of SCRIMMAGE
was inspired by the Stage robotics simulator
(http://playerstage.sourceforge.net/doc/stage-svn/). Specifically, Stage's
ability to efficiently simulate multiple robots and its plugin interface for
robotic control, motion models, sensors, and autonomy communication are
desirable. However, Stage is inherently a two-dimensional simulator, which
limits its usefulness for aerial robotic platforms. SCRIMMAGE provides a
three-dimensional robotics environment that can simulate varying levels of
sensor and motion model fidelity due to its flexible plugin interface. This
allows a robotics researcher to simulate hundreds of aircraft with low-fidelity
motion models or tens of aircraft with high-fidelity motion models on a
standard consumer laptop. If greater performance is desired, SCRIMMAGE was
designed with parallelization in mind, such that it can execute on a cluster.

An overview of the types of SCRIMMAGE plugins and how data flows through the
SCRIMMAGE simulation is shown in :numref:`plugin_order`.

.. figure:: ../images/plugin-order.png
   :name: plugin_order
   :alt: SCRIMMAGE's plugin interface

   SCRIMMAGE's plugin interface.

A SCRIMMAGE developer creates and uses existing plugins to assemble agents,
also known as entities, in a SCRIMMAGE mission or scenario. An entity consists
of an assemblage of sensor, communication, autonomy, controller, and motion
model plugins, as shown in :numref:`entity_plugins`.

.. figure:: ../images/entity_plugins.png
   :name: entity_plugins
   :alt: A SCRIMMAGE entity is composed of multiple plugins.

   A SCRIMMAGE entity is composed of multiple plugins.


A robotics researcher uses SCRIMMAGE to develop, visualize, and test novel
autonomy and control algorithms for both single-vehicle and swarm-based
robotics. The researcher accomplishes this by first deciding on the robotic
platform's motion model and sensor suite. The researcher can use SCRIMMAGE's
XML configuration files to “build” a simulated robotic model based on the
built-in motion models and sensors or the researcher can use the C++ plugin
interface to design their own motion models and sensors.

In addition to the motion model and sensor plugin interface, SCRIMMAGE also has
an Autonomy plugin interface. Through this interface, the robotics researcher
can write C++ code that queries SCRIMMAGE's World Model and sensor models and
then provide a desired future state for the robot being controlled. It is in
this C++ code that the robotics researcher develops the higher-level autonomy
algorithms. The Autonomy C++ code has an associated XML parameter file that
allows the user to modify arbitrary values in the Autonomy C++ code. SCRIMMAGE
provides scripts to enable the user to run simulations iteratively while
varying the input parameters based on a desired range of values and the outcome
of the simulation. This allows the researcher to implement reinforcement
learning algorithms with SCRIMMAGE. Thus far, researchers have implemented
greedy-based algorithms, classical swarming algorithms, and biologically
inspired algorithms through this Autonomy plugin interface. The Autonomy
interface can also be used with autonomous systems that make use of common
communication architectures, such as the Robot Operating System (ROS), the
Mission Oriented Operating Suite (MOOS), and Data Distribution Server (DDS). It
is through the Autonomy plugin interface that the real autonomy research takes
place.

After building the models and autonomy control logic for the robotic platforms,
the researcher uses SCRIMMAGE's XML configuration files to design the World
Model and initial conditions for all robotic platforms. SCRIMMAGE is especially
designed to handle the initialization of swarms of homogeneous and
heterogeneous robotic platforms, which can be tedious to define in XML if not
handled correctly by the simulator. Since an XML file is used to define the
mission, the SCRIMMAGE code does not have to be recompiled; thus, autonomy
development time is decreased.

While executing, SCRIMMAGE is designed to run on either a standard consumer
laptop or a cluster via parallelization. When running on a cluster, SCRIMMAGE
uses the Google Protocol Buffer serialization standard to communicate with
remote processes. The remote processes are typically the high-level autonomy
logic and high-fidelity motion models. The user can visualize the simulation
while it's running or after the simulation is complete via SCRIMMAGE's
visualization GUI. The GUI was written with the open source Visualization
Toolkit (VTK) (http://www.vtk.org/), but it is not required during simulation.
