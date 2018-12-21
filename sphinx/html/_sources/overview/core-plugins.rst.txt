Core Plugins
============

The SCRIMMAGE core project provides several plugins that can be used for
multi-agent simulation. The following is a brief description of the available
plugins.

Autonomy Plugins
----------------

- **AuctionAssign** : An example of using Auction-based methods and SCRIMMAGE's
  messaging system to generate auctions and bids between multiple entities.

- **Boids** : An implementation of Boid's swarm control
  model. (http://www.red3d.com/cwr/boids/)

- **MOOSAutonomy** : Allows an entity to be controlled by MOOS-IvP. See the
  ``moos_scrimmage.py`` script in the scripts directory for help on running
  MOOS with SCRIMMAGE.

- **MotorSchemas** : An implementation of the Motor Schemas autonomous control
  framework. The entity can move towards a waypoint while avoiding other
  entities.

- **PyAutonomy** : Allows an entity to be controlled by a Python script.

- **ROSAutonomy** : Allows an entity to be controlled by a ROS
  node. (experimental).

- **Straight** : Moves the entity in the forward direction from its initial
  heading.

Motion Model Plugins
--------------------

- **DoubleIntegrator** : Implements a 3D double integrator motion model.

- **FixedWing6DOF** : A dynamics-based with quaternion integration fixed-wing
  model.

- **JSBSimControl** : An example of controlling JSBSim's actuators directly
  from an Autonomy plugin.

- **JSBSimModel** : An example of controlling JSBSim's autopilot / model from
  an Autonomy plugin.

- **RigidBody6DOF** : A general purpose rigid body 6DOF model.

- **SimpleAircraft** : Implements a simple 3D aircraft model.

- **SimpleCar** : Implements a simple car
  model. (http://planning.cs.uiuc.edu/node658.html)

- **SimpleQuadrotor** : Implements a simple 3D quadrotor model.

- **SingleIntegrator** : Implements a 3D single integrator motion model. The
  entity can instantaneously change velocity and heading.

- **Unicycle** : Implements a simple unicycle
  model. (http://planning.cs.uiuc.edu/node660.html)


Controller Plugins
------------------

- **DoubleIntegratorControllerWaypoint** : A waypoint controller for the
  DoubleIntegrator.

- **FixedWing6DOFControllerPID** : Controls speed, altitude, and heading.

- **FixedWing6DOFControllerROS** : Controls speed, altitude, and heading from
  ROS node (experimental).

- **JSBSimControlControllerHeadingPID** : Directly controls actuators of JSBSim
  model from SCRIMMAGE.

- **JSBSimModelControllerHeadingPID** : Controls speed, altitude, and
  heading. Uses an autopilot defined in JSBSim's XML syntax.

- **RigidBody6DOFControllerPID** : Controls speed, altitude, and heading.

- **RigidBody6DOFControllerROS**: Controls speed, altitude, and heading from
  ROS node (experimental).

- **SimpleAircraftControllerPID** : Controls speed, altitude, and heading.

- **SimpleCarControllerHeading**: Controls speed and heading.

- **SimpleQuadrotorControllerLQR** : Implements linear-quadratic regulator
  (LQR) controller for quadrotor model. Controls heading, speed, and altitude.

- **SingleIntegratorControllerSimple** : Directly controls 3D velocity
  components of single integrator.

- **SingleIntegratorControllerWaypoint** : Implements waypoint control of
  single integrator model.

- **UnicycleControllerDirect** : Directly controls the speed and angular
  velocity.

- **UnicycleControllerPoint** : Drives the unicycle to a desired X/Y position.

Sensor Plugins
--------------

- **ContactBlobCamera** : Converts contacts within a field-of-view and range
  into a circle projected onto a 2D camera image. A bounding box is generated
  around each contact. See the ContactBlobCamera.xml file for complete sensor
  configuration.

- **NoisyContacts** : Converts the states of contacts within a field-of-view
  and range into noisy states. Simulates noisy range-based sensors.

- **NoisyState** : Converts the entity's own state into a noisy
  version. Simulates noisy localization.


Entity Interaction Plugins
--------------------------

- **BulletCollision** : Perform collision detection between entities
  with the open source Bullet physics engine.

- **GroundCollision** : Determine if an entity falls below a z-position
  threshold. If so, remove the entity from the simulation.

- **SimpleCollision** : Determine if the distance between two entities is below
  a given threshold. If so, remove the two entities from the simulation.

Metrics Plugins
---------------

- **SimpleCollisionMetrics** : Keeps track of each entity's total simulation
  time, collisions with teammates, collisions with non teammates, and
  collisions with the ground. This metric computes a score for each team based
  on the counts and the weights, which are defined in
  SimpleCollisionMetrics.xml
