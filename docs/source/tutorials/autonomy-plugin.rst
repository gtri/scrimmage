.. _autonomy_plugin:

Create an Autonomy Plugin
=========================

SCRIMMAGE ships with a script that generates an Autonomy plugin template. The
developer needs to provide the name of the Autonomy plugin and the root
directory of your SCRIMMAGE plugins project. We will use the plugins project we
created in the previous tutorial ( :doc:`create-project` ) to build our new
Autonomy plugin.

To create an Autonomy plugin, enter the following at the terminal: ::

  $ cd /path/to/scrimmage/scripts
  $ ./generate-plugin.sh autonomy MyFollowBehavior ~/scrimmage/my-scrimmage-plugins

Now, let's build the plugin that was placed in the my-scrimmage-plugins
project: ::

  $ cd ~/scrimmage/my-scrimmage-plugins/build
  $ cmake ..
  $ make

If the plugin built successfuly, you will see the following output: ::

  [100%] Built target MyFollowBehavior_plugin

  
Examine Autonomy Plugin
-----------------------

Autonomy Plugin Initialization
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Let's take a look at the Autonomy plugin template code to better understand the
types of information that are available to the Autonomy developer. Use your
favorite text editor to open the file at
``~/scrimmage/my-scrimmage-plugins/src/plugins/autonomy/MyFollowBehavior/MyFollowBehavior.cpp``.

After the relevant header files are included, there is the plugin's
constructor: 

.. code-block:: c++
   :linenos:
                
   MyFollowBehavior::MyFollowBehavior()
   {
   }

and the plugin's ``init`` function

.. code-block:: c++
   :linenos:
                
   void MyFollowBehavior::init(std::map<std::string,std::string> &params)
   {
       double initial_speed = sc::get<double>("initial_speed", params, 21);
       desired_state_->vel() = Eigen::Vector3d::UnitX()*initial_speed;
       desired_state_->quat().set(0,0,state_->quat().yaw());
       desired_state_->pos() = Eigen::Vector3d::UnitZ()*state_->pos()(2);
   }
                     
All SCRIMMAGE plugins must implement an ``init`` function. The ``init``
function is called one time when the plugin is initialized and takes a
parameter ``params`` that contains a map of strings that point to strings. This
``params`` map is populated with the values specified in the plugin's XML
configuration file. The XML configuration file is located in the ``include``
path of your project. Thus, MyFollowBehavior's XML file exists at
``~/scrimmage/my-scrimmage-plugins/include/my-scrimmage-plugins/plugins/autonomy/MyFollowBehavior/MyFollowBehavior.xml``.

Since the ``initial_speed`` tag exists in the ``MyFollowBehavior.xml`` file:

.. code-block:: xml
   :linenos:

   <?xml version="1.0"?>
   <?xml-stylesheet type="text/xsl" href="http://gtri.gatech.edu"?>
   <params>
      <library>MyFollowBehavior_plugin</library>
      <initial_speed>24.3</initial_speed>
   </params>

the ``sc::get<double>()`` function can convert the string value of
``initial_speed`` to a double value. The Autonomy plugin developer can add
arbitrarly define tags in the Autonomy plugin's XML file. The ``sc::get``
supports all standard C++ data types, such as ``int``, ``double``,
``std::string``, ``bool``, ``unsigned int``, etc.

The ``desired_state_`` member variable is a shared pointer to a
``scrimmage::State`` type, which should be set by the Autonomy plugin. A
Controller plugin converts the ``desired_state_`` variable into actuator
commands for the motion model plugin. The plugin developer can determine what
the velocity, quaternion (orientation), and position member variables of the
``State`` type represent. However, we typically define the velocity portion
with respect to the body frame, the quaternion with the inertial frame, and the
position with the inertial frame. The position and velocity are of
``Eigen::Vector3d`` type and the quaternion is a ``scrimmage::Quaterion`` type,
which inherits from Eigen's Quaternion class. In this example, we want to set
the initial velocity to be 24.3 m/s in the vehicle's forward direction, the
velocity is set to

.. code-block:: c++

   // Set forward speed to initial_speed
   desired_state_->vel() = Eigen::Vector3d::UnitX()*initial_speed;
                
since the X-axis is defined in the forward direction in the body frame. Since
we want the entity to move in the forward direction on initialization, the
desired yaw is set to the current yaw (which is the initial yaw).

.. code-block:: c++

   // set: roll, pitch, yaw
   desired_state_->quat().set(0,0,state_->quat().yaw());
                
The Controller plugin that will be paired with this Autonomy plugin uses the Z
element in the ``State``'s position vector to control altitude. To remain at
the same altitude, the desired state's Z-position is set to the current
Z-position.

.. code-block:: c++

   // Remain at current altitude
   desired_state_->pos() = Eigen::Vector3d::UnitZ()*state_->pos()(2);

Step Autonomy
~~~~~~~~~~~~~

The real work of the Autonomy plugin is typically implemented in the
``step_autonomy`` method.

.. code-block:: c++

   bool MyFollowBehavior::step_autonomy(double t, double dt)
                
The ``step_autonomy`` method takes the current simulation time, ``t``, and the
simulation step size ``dt``, as inputs. If an error is detected during method
execution, the Autonomy developer can return ``false`` to inform SCRIMMAGE's
simulation controller, but typically, ``true`` should be returned when the
method finishes execution. The example Autonomy plugin that was generated
implements the following simple behavior:

1. Calculate the distance to each contact in the simulation.
2. Find the the closest contact that is not on my team.
3. Head in the direction of the closest contact.

The first and second bullet points are implemented by

.. code-block:: c++
   :linenos:
                
    // Find nearest entity on other team. Loop through each contact, calculate
    // distance to entity, save the ID of the entity that is closest.
    double min_dist = std::numeric_limits<double>::infinity();
    for (auto it = contacts_->begin(); it != contacts_->end(); it++) {

        // Skip if this contact is on the same team
        if (it->second.id().team_id() == id_.team_id()) {
            continue;
        }

        // Calculate distance to entity
        double dist = (it->second.state()->pos() - state_->pos()).norm();

        if (dist < min_dist) {
            // If this is the minimum distance, save distance and reference to
            // entity
            min_dist = dist;
            follow_id_ = it->first;
        }
    }
    
Line 4 is a common way to iterate over all contacts. In line 7, we ignore
contacts that are on the same team by checking team IDs. ``parent_`` is a
reference to the plugin's parent entity. The parent entity holds references to
the entity's ID and other plugins. The ``scrimmage::ID`` class has three member
variables that refer to the entity's ID, the sub_swarm_id, and the team_id,
respectively. In line 12, we use Eigen's ``norm()`` function to calculate the
distance between our entity and the contact. In lines 14 to 18, we determine if
this is the small distance encountered so far and save the distance and ID of
the contact if it is the closest distance.

Next, we set the Autonomy's ``desired_state_`` to move in the direction of the
closest non-team member, if it exists.

.. code-block:: c++
   :linenos:

    // Head toward entity on other team
    if (contacts_->count(follow_id_) > 0) {
        // Get a reference to the entity's state.
        sc::StatePtr ent_state = contacts_->at(follow_id_).state();

        // Calculate the required heading to follow the other entity
        double heading = atan2(ent_state->pos()(1) - state_->pos()(1),
                               ent_state->pos()(0) - state_->pos()(0));

        // Set the heading
        desired_state_->quat().set(0, 0, heading); // roll, pitch, heading

        // Match entity's altitude
        desired_state_->pos()(2) = ent_state->pos()(2);
    }

Line 2 ensures that the ID of the contact that we want to follow exists. Next,
we get a pointer to the contact's ``scrimmage::State`` in line 4. Using basic
trigonometry, we calculate the required heading to follow the contact in lines
7 and 8.  In line 11, we set the desired heading, similarl to how we set the
heading in the initialization method. Finally, we set the desired altitude to
the same altitude of the contact in line 14. If the closest contact search
didn't succeed in the first part of ``step_autonomy``, then the
``desired_state_`` won't be updated due to the guard in line 2 and our Autonomy
plugin will just drive our entity in the forward direction.

By just using the state's of other contacts, Autonomy plugins that implement
formation controllers, spatial search algorithms, biologically-inspired
algorithms, and many other algorithms associated with multi-robot problems. The
contacts in this example are ground truth contacts without any noise. When you
complete the Sensor plugin tutorial (:doc:`sensor-plugin`), you will learn how
to add noise and filter out contacts based on a sensor model.
