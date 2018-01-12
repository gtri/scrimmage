.. _openai_plugin:

Create an OpenAI Plugin
=========================

SCRIMMAGE can be used in conjunction with OpenAI to generate environments for
learning scenarios. OpenAI environments need to have a state space, action
space, and a reward function. In this tutorial, we shall go through a quick
example of how to set up a scrimmage agent to interact with OpenAI by providing
a state space, action space, and reward. We will use the plugins project we
created in a previous tutorial ( :doc:`create-project` ) to build our new OpenAI
plugins. To start with, we will need to create an Autonomy and a Sensor plugin.

We can use the script provided with SCRIMMAGE to create these plugins. To do so,
enter the following at the terminal: ::

  $ cd /path/to/scrimmage/scripts
  $ ./generate-plugin.sh autonomy SimpleLearner ~/scrimmage/my-scrimmage-plugins
  $ ./generate-plugin.sh sensor MyOpenAISensor ~/scrimmage/my-scrimmage-plugins

Now, let's build the plugins that was placed in the my-scrimmage-plugins
project: ::

  $ cd ~/scrimmage/my-scrimmage-plugins/build
  $ cmake ..
  $ make

If the plugin built successfuly, you will see the following output: ::

  [ 50%] Built target MyOpenAISensor_plugin
  [100%] Built target SimpleLearner

For this tutorial, we want to have an environment in which there is one player.
This player can either go forward, turn left, or turn right. The goal is for the
player to fly over the other team's base. We will set up the reward to be 1 when
the player is over the base and 0 otherwise. The state space of this environment
consists of 4 variables: x,y, cos(heading), and sin(heading). Together, these
describe a continous 2D plane and our player's heading. Now let's go and see how
these are implemented.

Rewrite the Autonomy Plugin
---------------------------

OpenAI Autonomy Plugin Header File
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In this tutorial, we are going to take the autonomy plugin and rewrite for use
with OpenAI. Let's start with the header file located at
``~/scrimmage/my-scrimmage-plugins/include/my-scrimmage-plugins/plugins/autonomy/SimpleLearner/SimpleLearner.h``.

Normal autonomy plugins extend the Autonomy class. For the OpenAI autonomy
plugin, we will instead extend the ExternalController autonomy plugin. This
class covers the normal componenets of the autonomy plugin and allows to only
worry about creating a few methods specfic for OpenAI. We will thus have to
change our include statements to the following:

.. code-block:: c++
   :linenos:

   #include <scrimmage/plugins/autonomy/ExternalControl/ExternalControl.h>

   #include <map>
   #include <string>
   #include <utility>


Below the include statements, we have the following class definition:

.. code-block:: c++
   :linenos:

   class TutorialOpenAIAutonomy : public scrimmage::autonomy::ExternalControl {
    public:
       virtual void init(std::map<std::string, std::string> &params);

    protected:
       double radius_;

       virtual std::pair<bool, double> calc_reward(double t);
       virtual bool handle_action(
           double t, double dt, const scrimmage_proto::Action &action);
       virtual scrimmage_proto::SpaceParams action_space_params();
   };


As can be seen, there only four methods we need to create in our source file.
They handle the initialization of our autonomy plugin, how the reward is
calculated, the type of action space that will be seen by the OpenAI
environment, and how those actions are interpreted in SCRIMMAGE.


OpenAI Autonomy Plugin Source File
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Now let's open our source file located at
``~/scrimmage/my-scrimmage-plugins/src/plugins/autonomy/SimpleLearner/SimpleLearner.cpp``.

We will first change the includes at the top of the file to be:

.. code-block:: c++
   :linenos:

   #include <scrimmage/plugin_manager/RegisterPlugin.h>
   #include <scrimmage/entity/Entity.h>
   #include <scrimmage/math/State.h>
   #include <scrimmage/parse/ParseUtils.h>
   #include <scrimmage/sensor/Sensor.h>
   #include <scrimmage/proto/ExternalControl.pb.h>
   #include <Tutorial-plugins/plugins/autonomy/SimpleLearner/SimpleLearner.h>

   namespace sc = scrimmage;
   namespace sp = scrimmage_proto;

Next, let us look at the ``init``:

.. code-block:: c++
   :linenos:

   void SimpleLearner::init(std::map<std::string,std::string> &params)
   {
       desired_state_->pos()(2) = state_->pos()(2);
       ExternalControl::init(params);
   }

In our case, the ``init`` just calls the inherited ``init`` from
ExternalControl. This would also be where parameters from the
``SimpleLearner.xml`` file would be initialized. We set the desired altitude
to be the same as our current altitude in order to preserve the 2D plane
environment. From here, we can then move on to look at the action space
representation for OpenAI:

.. code-block:: c++
   :linenos:

   scrimmage_proto::SpaceParams SimpleLearner::action_space_params() {
       sp::SpaceParams space_params;
       sp::SingleSpaceParams *single_space_params = space_params.add_params();
       single_space_params->set_discrete(true);
       single_space_params->set_num_dims(1);
       single_space_params->add_minimum(0);
       single_space_params->add_maximum(2);
       return space_params;
   }

In this method, we set up a discrete action space of one dimension, with 3
possible values: 0, 1, 2. Next, let's see what we do when we receive those
actions from OpenAI in our ``handle_action`` method:

.. code-block:: c++
   :linenos:

   bool SimpleLearner::handle_action(double t, double dt, const scrimmage_proto::Action &action) {
       if (!check_action(action, 1, 0)) {
           return false;
       }
       Eigen::Vector3d velocity_cmd;
       double turn_rate = (action.discrete(0) - 1)*.2;
       Eigen::Vector3d velocity_cmd;
       velocity_cmd << 1, turn_rate, 0;
       return true;
   }

Here, we can see that we take our ``action`` of 0, 1, or 2 and convert it
into a corresponding turning rate. We limit the turning rate to .2 and also
tell the player to constantly fly forward at a speed of 1 m/s. Notice that we
are not using the ``t`` or ``dt`` parameters in our specific example. However,
those are available for other environment setups if needed.

Finally, we need to set up the reward for this environment in ``calc_reward``. This
function returns a `std::pair<bool, double>` which corresponds to whether the
environment is done and the reward:

.. code-block:: c++
   :linenos:

   std::pair<bool, double> TutorialOpenAIAutonomy::calc_reward(double time) {
       const bool done = false;
       double reward = 0.0;

       for (auto &kv : parent_->mp()->team_info()) {
           // same team
           if (kv.first == parent_->id().team_id()) {
               continue;
           }

           // For each base
           int i = 0;
           for (Eigen::Vector3d &base_pos : kv.second.bases) {
               Eigen::Vector3d base_2d_pos(base_pos.x(), base_pos.y(), state_->pos().z());
               double radius = kv.second.radii.at(i);
               if ((state_->pos()-base_2d_pos).norm() < radius) {
                   reward += 1;
               }
               i++;
           }
       }
       return std::make_pair(done, reward);
   }

For this example, we do not use the time parameter but it is there for cases
where you want to include the time in your reward function. Our ``calc_reward``
goes through every team and looks at every base. For every base radius the
player is within from a top down view, the reward is incremented up by 1.

With this, we have created the action space and reward function for the OpenAI
enviornment. The only part left is creating the state space. Before working on
the sensor code, we do have two more files to edit for ``SimpleLearner``.

Rewrite CMakeLists.txt for OpenAI Autonomy
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``SimpleLearner`` C++ code is now finished. Before we can build it though,
we do need to make a small edit to the ``CMakeLists.txt``. Open up
``~/scrimmage/my-scrimmage-plugins/src/plugins/autonomy/SimpleLearner/CMakeLists.txt``
and change line 15 from

.. code-block:: cmake
   :lineno-start: 15

   TARGET_LINK_LIBRARIES(${LIBRARY_NAME}
     )

to

.. code-block:: cmake
   :lineno-start: 15
   :emphasize-lines: 2

   TARGET_LINK_LIBRARIES(${LIBRARY_NAME}
     ExternalControl_plugin
     )

in order for ``SimpleLearner`` to see the ``ExternalControl`` autonomy plugin.
From here, we can now build the project: ::

  $ cd ~/scrimmage/my-scrimmage-plugins/build
  $ cmake ..
  $ make

Xml file for OpenAI Autonomy
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The last part we need to edit is the ``SimpleLearner.xml`` located at:
``~/scrimmage/my-scrimmage-plugins/include/my-scrimmage-plugins/plugins/autonomy/SimpleLearner/SimpleLearner.xml``.
Here, we need to add a field for a server address to run SCRIMMAGE on. To do so,
simply add the following line to the ``<params>`` field.

.. code-block:: xml

  <server_address>localhost:50051</server_address>

With that, we are now done with ``SimpleLearner`` and can now focus on the
sensor plugin.

Rewrite the Sensor Plugin
-------------------------

OpenAI Sensor Plugin Header File
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Now let's move on to defining the state space. We shall do this with through a
sensor plugin to OpenAI. We shall start by rewriting the header file for the
sensor plugin we created above. You can find it at
``~/scrimmage/my-scrimmage-plugins/include/my-scrimmage-plugins/plugins/sensor/MyOpenAISensor/MyOpenAISensor.h``.

First up, we shall rewrite the includes in ``MyOpenAISensor.h`` to be the
following:

.. code-block:: c++
   :linenos:

   #include <scrimmage/sensor/Sensor.h>

   #include <map>
   #include <string>
   #include <vector>

   namespace boost {
   template <class T> class optional;
   }

   namespace scrimmage_proto {
   class SpaceParams;
   class SpaceSample;
   }

Below that, we create the ``MyOpenAISensor`` class defined as:

.. code-block:: c++
   :linenos:

   class MyOpenAISensor : public scrimmage::Sensor {
    public:
       virtual boost::optional<scrimmage_proto::SpaceParams> observation_space_params();
       virtual boost::optional<scrimmage::MessagePtr<scrimmage_proto::SpaceSample>>
           sensor_msg_flat(double t);
   };

``observation_space_params`` sets up the state space model for the OpenAI
environment. ``sensor_msg_flat`` then creates the state space message to send to
as the state in the OpenAI environment. It is important to note that
``sensor_msg_flat`` is different from the default sensor method ``sensor_msg``.

OpenAI Sensor Plugin Source File
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

From here, we can now look at the implementation of these methods in
``~/scrimmage/my-scrimmage-plugins/src/plugins/sensor/MyOpenAISensor/MyOpenAISensor.cpp``.

In this source file, we need to add the following includes:

.. code-block:: c++
   :linenos:

   #include <scrimmage/plugin_manager/RegisterPlugin.h>
   #include <scrimmage/entity/Entity.h>
   #include <scrimmage/math/State.h>
   #include <scrimmage/common/Keys.h>
   #include <scrimmage/proto/ExternalControl.pb.h>

   #include <boost/optional.hpp>

   #include <Tutorial-plugins/plugins/sensor/MyOpenAISensor/MyOpenAISensor.h>
   #include <boost/optional.hpp>

   namespace sc = scrimmage;
   namespace sp = scrimmage_proto;

From there, we then look at method implementation. ``observation_space_params``
creates a 4 dimensional state space for x,y, cos(heading), and sin(heading). It
sets the limits for each state as well as defines them as continous variables.
If we had more players in this space, it would add 4 states for each of them as
well.

.. code-block:: c++
   :linenos:

   boost::optional<scrimmage_proto::SpaceParams>
   MyOpenAISensor::observation_space_params() {
       sp::SpaceParams space_params;

       const double inf = std::numeric_limits<double>::infinity();
       for (size_t i = 0; i < parent_->contacts()->size(); i++) {
           sp::SingleSpaceParams *single_space_params = space_params.add_params();
           single_space_params->set_num_dims(4);

           const std::vector<double> lims {inf, inf, 1, 1}; // x, y, cos(yaw), sin(yaw)
           for (double lim : lims) {
             single_space_params->add_minimum(-lim);
             single_space_params->add_maximum(lim);
           }
           single_space_params->set_discrete(false);
       }

       return space_params;
   }

``sensor_msg_flat`` sets up a message type, fills in the message with the new
state information and then returns the message as its output. As the list of
players is unordered, we call ``sc::keys`` to create an ordered list of them.
This is to ensure that the state message has the same format every time. Again,
our message consists of 4 states: x,y, cos(heading), and sin(heading). If there
are more than one players on the field, it would also add their states to the
message as well.

.. code-block:: c++
   :linenos:

   boost::optional<scrimmage::MessagePtr<scrimmage_proto::SpaceSample>>
   MyOpenAISensor::sensor_msg_flat(double t) {
       auto msg = std::make_shared<sc::Message<sp::SpaceSample>>();

       // we need these sorted but contacts are an unordered map
       auto keys = sc::keys(*parent_->contacts());
       std::set<int> contact_ids(keys.begin(), keys.end());

       for (int contact_id : contact_ids) {
           sc::State &s = *parent_->contacts()->at(contact_id).state();
           const double yaw = s.quat().yaw();
           msg->data.add_value(s.pos()(0));
           msg->data.add_value(s.pos()(1));
           msg->data.add_value(cos(yaw));
           msg->data.add_value(sin(yaw));
       }

       return msg;
   }

OpenAI Mission XML File
-----------------------

Now that our code for SCRIMMGAE has been compiled, we can then create a simple
mission xml file for it. We will save this xml at:
``~/scrimmage/my-scrimmage-plugins/missions/openai.xml``.

To create the environment as we described above, the mission xml would need the
following blocks (More detail on creating mission files is located at
:ref:`scrimmage_xml` ):

.. code-block:: xml
   :linenos:

   <entity_common name="all">
       <count>1</count>
       <health>1</health>
       <radius>1</radius>
       <motion_model>SimpleAircraft</motion_model>
       <controller>SimpleAircraftControllerPID</controller>
   </entity_common>

   <entity entity_common="all">
     <x>0</x>
     <y>0</y>
     <z>200</z>
     <heading>0</heading>
     <team_id>1</team_id>
     <color>77 77 255</color>
     <autonomy>SimpleLearner</autonomy>
     <visual_model>zephyr-blue</visual_model>
     <sensor order="0">MyOpenAISensor</sensor>
     <motion_model>Unicycle</motion_model>
     <controller>UnicycleControllerDirect</controller>
   </entity>

   <entity entity_common="all">
     <team_id>2</team_id>
     <color>255 0 0</color>
     <count>0</count>

     <base>
       <x>30</x>
       <y>0</y>
       <z>200</z>
       <radius>25</radius>
     </base>
   </entity>

This example environment is intended to work with the ``Unicycle`` motion model
through the ``UnicycleControllerDirect controller``. If we wanted to use a
different motion model/controller, we would need to change our ``handle_action``
in ``SimpleLearner.cpp`` to output the correct control for the new setup. If
``openai.xml`` is opened without using OpenAI, SCRIMMAGE will

Now we have completed our work on the SCRIMMAGE side. Now all that is left is to
write the python code to run our OpenAI environment.

OpenAI Python File
------------------

The following python code will create a scrimmage environment, using the mission
file we create above. It will then do a simple environment test by stepping
through the environment and keeping track of the observations. It also sends
a straight ahead action for the first 100 timesteps and afterwards sends a turn
right action. At the end, it closes the environment and prints out the total
reward. We will save this python file at
``~/scrimmage/my-scrimmage-plugins/test/test_openai.py``.

.. code-block:: python
   :linenos:

   import numpy as np
   import gym
   import scrimmage


   def test_openai():
       """Open single entity scenario and make sure it banks."""
       try:
           env = gym.make('scrimmage-v0')
       except gym.error.Error:
           mission_file = scrimmage.find_mission('openai.xml')

           gym.envs.register(
               id='scrimmage-v0',
               entry_point='scrimmage.external_control:ScrimmageEnv',
               max_episode_steps=1e9,
               reward_threshold=1e9,
               kwargs={"enable_gui": True,
                       "mission_file": mission_file}
           )
           env = gym.make('scrimmage-v0')

       # the observation is the state of the aircraft
       obs = env.reset()
       total_reward = 0
       for i in range(200):

           action = 1 if i < 100 else 2
           obs, reward, done = env.step(action)[:3]
           total_reward += reward

           if done:
               break

       env.env.close()
       print("Total Reward: %2.2f" % total_reward)

   if __name__ == '__main__':
       test_openai()

Now that we have completed all of the code, we can simply type the following
into the terminal to see it run! ::

  $ python test_openai.py
