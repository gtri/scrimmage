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

Let's take a look at the header file
``~/scrimmage/my-scrimmage-plugins/include/my-scrimmage-plugins/plugins/autonomy/MyFollowBehavior/MyFollowBehavior.h``:

.. code-block:: c++
   :linenos:

    #ifndef INCLUDE_MY_SCRIMMAGE_PLUGINS_PLUGINS_AUTONOMY_MYFOLLOWBEHAVIOR_MYFOLLOWBEHAVIOR_H_
    #define INCLUDE_MY_SCRIMMAGE_PLUGINS_PLUGINS_AUTONOMY_MYFOLLOWBEHAVIOR_MYFOLLOWBEHAVIOR_H_
    #include <scrimmage/autonomy/Autonomy.h>

    #include <string>
    #include <map>

    namespace scrimmage {
    namespace autonomy {
    class MyFollowBehavior : public scrimmage::Autonomy {
     public:
        void init(std::map<std::string, std::string> &params) override;
        bool step_autonomy(double t, double dt) override;

     protected:
        int desired_alt_idx_ = 0;
        int desired_speed_idx_ = 0;
        int desired_heading_idx_ = 0;
    };
    } // namespace autonomy
    } // namespace scrimmage
    #endif // INCLUDE_MY_SCRIMMAGE_PLUGINS_PLUGINS_AUTONOMY_MYFOLLOWBEHAVIOR_MYFOLLOWBEHAVIOR_H_

Note that it inherits from the scrimmage base class ``Autonomy.h`` located at
``scrimmage/include/scrimmage/autonomy/Autonomy.h`` which provides a couple virtual methods,
``init`` and ``step``,
we will be overriding. The member variables ``desired_alt_idx_``, ``desired_speed_idx_``,
and ``desired_heading_idx_`` will be used to interact with the controller. For more details
on interacting with controllers, see :ref:`variableio`.

Now let's take a look at the Autonomy plugin template code to better understand the
types of information that are available to the Autonomy developer. Use your
favorite text editor to open the file at
``~/scrimmage/my-scrimmage-plugins/src/plugins/autonomy/MyFollowBehavior/MyFollowBehavior.cpp``:

.. code-block:: c++
   :linenos:

    #include <my-scrimmage-plugins/plugins/autonomy/MyFollowBehavior/MyFollowBehavior.h>

    #include <scrimmage/plugin_manager/RegisterPlugin.h>
    #include <scrimmage/entity/Entity.h>
    #include <scrimmage/math/State.h>
    #include <scrimmage/parse/ParseUtils.h>

    #include <limits>

    namespace sc = scrimmage;

    REGISTER_PLUGIN(scrimmage::Autonomy,
                    scrimmage::autonomy::MyFollowBehavior,
                    MyFollowBehavior_plugin)

After the relevant header files are included, there is a ``REGISTER_PLUGIN``
macro that adds information to your source file so scrimmage can dynamically load it
at runtime. The plugin's ``init`` function is pasted below:

.. code-block:: c++
   :linenos:

    void MyFollowBehavior::init(std::map<std::string, std::string> &params) {
        double initial_speed = sc::get<double>("initial_speed", params, 21);
        desired_alt_idx_ = vars_.declare(VariableIO::Type::desired_altitude, VariableIO::Direction::Out);
        desired_speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::Out);
        desired_heading_idx_ = vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::Out);

        vars_.output(desired_speed_idx_, initial_speed);
        vars_.output(desired_alt_idx_, state_->pos()(2));
        vars_.output(desired_heading_idx_, state_->quat().yaw());
    }

This is just setting up the VariableIO to interact with the controller. See
:ref:`variableio` for more details.
The ``initial_speed`` tag is in the ``MyFollowBehavior.xml`` file, which
we get from the function ``get`` found in the ``ParseUtils.h`` header available
from scrimmage:

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

Set Loop Rate
~~~~~~~~~~~~~

Normally, the loop rate for a plugin should be set with the ``loop_rate``
xml tag inside the plugin's block of the mission file. Using this API the
user doesn't have to write any code to set the loop rate. However, it's also
possible to change the loop rate for a plugin dynamically during a simulation.
This is done by calling the plugin's ``set_loop_rate()`` method, which
sets the plugin's loop rate in Hz:

.. code-block:: c++
   :linenos:

    void MyFollowBehavior::update_loop_rate(double new_loop_rate) {
        set_loop_rate(new_loop_rate);
    }

The current loop rate can also be accessed with the ``loop_rate()`` function.

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

.. code-block:: c++
   :linenos:

    bool MyFollowBehavior::step_autonomy(double t, double dt) {
        // Find nearest entity on other team. Loop through each contact, calculate
        // distance to entity, save the ID of the entity that is closest.
        int follow_id_ = -1;
        double min_dist = std::numeric_limits<double>::infinity();
        for (auto &kv : *contacts_) {

            int contact_id = kv.first;
            sc::Contact &contact = kv.second;

            // Skip if this contact is on the same team
            if (contact.id().team_id() == parent_->id().team_id()) {
                continue;
            }

            // Calculate distance to entity
            double dist = (contact.state()->pos() - state_->pos()).norm();

            if (dist < min_dist) {
                // If this is the minimum distance, save distance and reference to
                // entity
                min_dist = dist;
                follow_id_ = contact_id;
            }
        }

        // Head toward entity on other team
        if (contacts_->count(follow_id_) > 0) {
            // Get a reference to the entity's state.
            sc::StatePtr ent_state = contacts_->at(follow_id_).state();

            // Calculate the required heading to follow the other entity
            double heading = atan2(ent_state->pos()(1) - state_->pos()(1),
                                   ent_state->pos()(0) - state_->pos()(0));

            // Set the heading
            vars_.output(desired_heading_idx_, heading);

            // Match entity's altitude
            vars_.output(desired_alt_idx_, ent_state->pos()(2));
        }

        return true;
    }

Line 6 is a common way to iterate over all contacts
using range-based for loops available from c++-11. In line 12, we ignore
contacts that are on the same team by checking team IDs. ``parent_`` is a
reference to the plugin's parent entity. The parent entity holds references to
the entity's ID and other plugins. The ``scrimmage::ID`` class has three member
variables that refer to the entity's ID, the sub_swarm_id, and the team_id,
respectively. In line 12, we use Eigen's ``norm()`` function to calculate the
distance between our entity and the contact. In lines 14 to 18, we determine if
this is the small distance encountered so far and save the distance and ID of
the contact if it is the closest distance.

In the second ``if`` block,
we set the Autonomy's controller inputs using :ref:`variableio`.
Line 28 ensures that the ID of the contact that we want to follow exists. Next,
we get a pointer to the contact's ``scrimmage::State`` in line 30. Using basic
trigonometry, we calculate the required heading to follow the contact in line
33.  In line 37, we set the desired heading, similarly to how we set the
heading in the initialization method. Finally, we set the desired altitude to
the same altitude of the contact in line 40. If the closest contact search
didn't succeed in the first part of ``step_autonomy``, then the
controller inputs won't be updated due to the guard.

By just using the states of other contacts, Autonomy plugins can implement
formation controllers, spatial search algorithms, biologically-inspired
algorithms, and many other algorithms associated with multi-robot problems. The
contacts in this example are ground truth contacts without any noise. When you
complete the Sensor plugin tutorial (:doc:`sensor-plugin`), you will learn how
to add noise and filter out contacts based on a sensor model.

Running the Plugin
------------------

In our project, open ``missions/example.xml`` and go to the second entity block.
Change ``count`` to 1 and ``autonomy`` to ``MyFollowBehavior`` and save the file:

.. code-block:: xml
   :linenos:

    <entity>
      <team_id>2</team_id>
      <color>255 0 0</color>
      <count>1</count>
      <health>1</health>
      <radius>2</radius>

      <!--
      <generate_rate> 1 / 2 </generate_rate>
      <generate_count>2</generate_count>
      <generate_start_time>0</generate_start_time>
      <generate_time_variance>0.10</generate_time_variance>
      -->

      <variance_x>20</variance_x>
      <variance_y>20</variance_y>
      <variance_z>20</variance_z>

      <x>50</x>
      <y>0</y>
      <z>200</z>

      <heading>180</heading>
      <altitude>200</altitude>
      <controller>SimpleAircraftControllerPID</controller>
      <motion_model>SimpleAircraft</motion_model>
      <visual_model>zephyr-red</visual_model>
      <autonomy>MyFollowBehavior</autonomy>
      <base>
        <latitude>35.719961</latitude>
        <longitude>-120.767304</longitude>
        <altitude>300</altitude>
        <radius>25</radius>
      </base>
    </entity>

Now we can run the simulation:

.. code-block:: bash

  scrimmage missions/example.xml

You can change the viewer by pressing ``a`` once, scrolling out and moving the picture
so both vehicles are in view. You can additionally press ``+`` so that the vehicles
are larger. You should see two vehicles approach each other and circle.
