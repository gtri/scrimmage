=========================
Capture the Flag Scenario
=========================

The purpose of this document is to discuss the ideas behind the design of the
"Capture the Flag" scrimmage mission. The design concepts used in this scenario
should serve as a guide for how to develop reusable plugins for other specific
scenarios. scrimmage provides a framework for performing agent-based
simulations, but it is typically up to the researcher/developer to implement
scenario specific plugins.

The game of Capture the Flag involves competing teams attempting to "capture"
the flag of the opposing team and return the captured flag back to their own
area of control. When an agent enters the opposing team's area of control,
agents on the opposing team can capture the intruding agent. When an agent is
captured, it is removed from the game.

There are a number of ways to "score" the game of Capture the Flag. For
example, the score could be a weighted sum of flag captures and opposing agent
captures. There are also a number of initial game configurations. For example,
there could be multiple flags per team and there could even be multiple
teams. The Capture the Flag scrimmage plugins were designed with this kind of
flexibility in mind.

Entity interaction, metrics, network, autonomy, controller, and motion model
plugins are used in the scrimmage Capture the Flag scenario. The controller and
motion model plugins are agnostic to the Capture the Flag game and can easily
be replaced with different controller and motion model plugins. The core of the
Capture the Flag rules are implemented as entity interaction plugins. The
metrics plugins use the output of the entity interaction plugins to create an
overall game score. The network plugins provide basic communication between
agents and game-specific messages between rules and metrics plugins.

The reader should follow along with the `capture-the-flag.xml
<https://github.com/gtri/scrimmage/blob/master/missions/capture-the-flag.xml>`_
scrimmage mission file in the ``missions`` directory while reading this
document.

Entity Interaction Plugins
--------------------------

Boundary
~~~~~~~~

The Boundary plugin is an entity interaction plugin that is used to define
abstract boundaries, areas, or zones. Each boundary consists of a geometry
(e.g., a scrimmage shape) and an ID (which consists of a team ID and a boundary
ID). At the beginning of the simulation, the Boundary plugin publishes its
information on the "Boundary" topic over the "GlobalNetwork" network. If the
Boundary plugin is initialized with the "show_boundary" parameter set to
"true", it will also draw its shape in the scrimmage viewer. There are two
types of boundaries used in the Capture the Flag game: a team boundary and a
team flag. The team boundary is a cuboid that defines the 3D space "controlled"
by the team. For example, the following boundary defines the blue team's
boundary:

.. code-block:: xml

   <entity_interaction name="blue_boundary"
                       type="cuboid"
                       lengths="500, 500, 500"
                       center="250, 0, 250"
                       color="0 0 255"
                       id="1"
                       team_id="1"
                       >Boundary</entity_interaction>

This code block defines a cuboid-shaped boundary with the name
``blue_boundary,`` a team ID of 1, and a boundary ID 1. The ``center`` and
``lengths`` define the position and size of the cuboid.

The blue team's flag is defined with a sphere-shaped boundary:

.. code-block:: xml

   <entity_interaction name="blue_flag"
                       id="2"
                       team_id="1"
                       type="sphere"
                       radius="5"
                       center="350, 0, 200"
                       color="0 0 255"
                       opacity="0.5"
                       >Boundary</entity_interaction>

The ``blue_flag`` boundary represents the area that the other team is
attempting to enter in order to "capture" the flag. It is important to note
that the ``id`` for each boundary is unique. Similar boundaries are created for
the red team, where the ``id`` fields are unique and the ``team_id`` is
different.

EnforceBoundaryInteraction
~~~~~~~~~~~~~~~~~~~~~~~~~~

The purose of the ``EnforceBoundaryInteraction`` plugin is to remove agents
from the game that leave the designated game play area. This plugin subscribes
to the ``Boundary`` topic (thus, it has all information about the previously
defined boundaries) and removes agents that are not in the space defined by the
union of the ``active_boundary_ids`` plugin parameter. In this case, the
``game_boundary`` is specified by the boundary with ID 5, which encompasses the
red and blue boundaries. Thus, the ``active_boundary_ids`` is set to ``5``:

.. code-block:: xml

   <entity_interaction active_boundary_ids="5">EnforceBoundaryInteraction</entity_interaction>

At every simulation time-step, this plugin compares the position of each agent
and removes it from the simulation if it doesn't fall within the boundaries
specified by the ``active_boundary_ids`` parameter.

Note: We could pass a list of boundaries to ``active_boundary_ids``, such as
``1, 3``, which already encompasses the red and blue boundary areas. However,
the ``Boundary`` class' ``contains()`` function only returns ``true`` if the
agent is within the boundary. If the agent is on the edge of the boundary, it
will return ``false``. In this mission, when the agent is directly on the plane
between the red and blue boundaries it is technically not within either
boundary. Thus, we use a ``game_boundary`` to enclose both red and blue
boundaries.

CaptureInBoundaryInteraction
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The purpose of the ``CaptureInBoundaryInteraction`` plugin is to enforce the
rule that agents can be "captured" (i.e., removed from the game) by agents on
the opposing team when in the opposing team's boundary. The following XML
configures the plugin for the blue team's boundary:

.. code-block:: xml

   <entity_interaction name="BlueCaptureBoundary"
                       boundary_id="1"
                       capture_range="5">CaptureInBoundaryInteraction</entity_interaction>

The ``boundary_id`` is set to 1, which means that this plugin will enforce the
agent capture rule only in this boundary. Another instantiation of the
``CaptureInBoundaryInteraction`` plugin is used to enforce the rule for the red
team's boundary. If the blue agent moves within the ``capture_range`` of a red
agent in boundary 1, the red agent will be removed from the simulation. This
plugin also has a ``cool_down_period`` parameter that can be used to limit the
rate at which agents can be captured.

This plugin publishes a message on the "NonTeamCapture" topic when an agent is
captured. The message contains the ID of the agent being captured and the ID of
the agent performing the capture.


FlagCaptureInteraction
~~~~~~~~~~~~~~~~~~~~~~

The purpose of this plugin is to determine when a flag has been captured and
carried to the opposing team's boundary. The ``BlueFlagCapture`` boundary is
configured with the following XML:

.. code-block:: xml

   <entity_interaction name="BlueFlagCapture"
                       flag_boundary_id="2"
                       capture_boundary_id="3">FlagCaptureInteraction</entity_interaction>

The previous plugin is configured to detect when an agent from the red team
enters the blue flag's boundary (specified by boundary ID 2) and brings the
flag to boundary ID 3 (which is the red team's boundary). This plugin publishes
separate messages to the "FlagTaken" and "FlagCaptured" events when the flag is
taken by an agent and when the flag is carried to their team's boundary,
respectively.

SimpleCollision
~~~~~~~~~~~~~~~

The ``SimpleCollision`` plugin is used to remove agents from the simulation
that are within a specified distance of each other, simulating a collision
between two agents. The range at which a collision is detected is specified by
the ``collision_range`` parameter. In the Capture the Flag scenario, this range
should be less than the ``capture_range`` specified in the
``CaptureInBoundaryInteraction`` plugin to ensure that the agent capture
occured before both agents collide. Thus, with this configuration, the
``SimpleCollision`` plugin will only detect collisions between agents on the
same team.

Metrics Plugins
---------------

Typically, the purpose of metrics plugins is to subscribe to topics, count
events occuring during the simulation, and provide a structured report and/or
score to the simulation engine.

FlagCaptureMetrics
~~~~~~~~~~~~~~~~~~

This plugin subscribes to the ``FlagTaken`` and ``FlagCaptured`` topics and
counts the occurances of the received events/messages. At the end of the
simulation, the events are recorded to the ``summary.csv`` file in the
scrimmage log directory (e.g., ``~/.scrimmage/logs/latest``) and printed to the
screen.

SimpleCaptureMetrics
~~~~~~~~~~~~~~~~~~~~

This plugin subscribes to the ``TeamCapture`` and ``NonTeamCapture`` topics and
counts the occurances of the received events/messages. In the typical Capture
the Flag scenario, there will not be any ``TeamCapture`` events, but the
``CaptureInBoundaryInteraction`` plugin will publish ``NonTeamCapture``
messages. At the end of the simulation, the events are recorded to the
``summary.csv`` file in the scrimmage log directory (e.g.,
``~/.scrimmage/logs/latest``) and printed to the screen.

SimpleCollisionMetrics
~~~~~~~~~~~~~~~~~~~~~~

This plugin subscribes to the following topics and counts the occurance of each
event/message:

1. ``TeamCollision`` : When two agents on the same team are within the range
   specified by ``collision_range``.

2. ``NonTeamCollision`` : When two agents that are not on the same team are
   within the range specified by ``collision_range``.

3. ``GroundCollision`` : When an agent is at or below the z-position specified
   by the ``ground_collision_z`` in the ``GroundCollision`` interaction
   plugin. In this case, the ``GroundCollision`` plugin is not being used, so
   there will not be any ``GroundCollision`` messages.

4. ``EntityGenerated`` : When an entity is instantiated in the simulation.

5. ``EntityRemoved`` : When an entity is removed from the simulation. For
   example, when it collides with another agent or it is captured by another
   agent.

6. ``EntityPresentAtEnd`` : When an entity is still active in the simulation
   when the simulation ends.

Network Plugins
---------------

Network plugins allow other plugins to publish messages to each other. Every
topic must be created on a specific network. Network plugins are used to
simulate lossy communications networks, more reliable local ethernet networks,
and simulation-specific networks that are completely reliable during.

GlobalNetwork
~~~~~~~~~~~~~

Messages that are published on the ``GlobalNetwork`` are never dropped or
delayed. Any agent that subscribes to a topic on a ``GlobalNetwork`` will
receive messages that are published to that topic. This network is often used
by the entity interaction plugins to publish event messages to the
corresponding metrics plugin. The ``Boundary`` plugins publishes its
information on the ``GlobalNetwork`` to make sure all other plugins have access
to the simulation-specific boundary information.

LocalNetwork
~~~~~~~~~~~~

The purpose of the ``LocalNetwork`` is simulate a highly reliable local network
connection on a single vehicle. This network is useful to add a local namespace
around a topic that might have the same name across multiple
agents/entities. For example, an agent might subscribe to the ``imu`` topic,
but you don't want to subscribe to the ``imu`` topic on a different platform,
you only want your own vehicle's ``imu``. If you have multiple agents that use
the ``imu`` topic then you will want to use the ``LocalNetwork`` to ensure that
messages on the ``imu`` topic are kept local to the entity.

SphereNetwork
~~~~~~~~~~~~~

The ``SphereNetwork`` is used to simulate a lossy communications channel
between agents.

Entity Configuration
--------------------

The following autonomy plugins are specific to entities in the Capture the Flag
game.

BoundaryDefense
~~~~~~~~~~~~~~~

This autonomy plugin implements a simple boundary defense behavior. With this
behavior, the agent finds the nearest entity on the opposing team within the
boundary specified by the ``boundary_id`` parameter and moves towards the
opponent. If there are no opponents in the boundary, the agent moves toward the
center of the boundary.

TakeFlag
~~~~~~~~

This autonomy plugin implements a simple capture flag behavior. If the agent
doesn't have the flag, it moves towards the flag boundary specified by the
``flag_boundary_id`` parameter. Once it has the flag, it moves towards the
boundary specified by the ``capture_boundary_id`` parameter. This behavior
doesn't try to avoid team or non-team agents.
