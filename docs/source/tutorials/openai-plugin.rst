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
The files described below are located in the following places::

    scrimmage/include/scrimmage/plugins/autonomy/RLSimple
    scrimmage/src/plugins/autonomy/RLSimple
    scrimmage/include/scrimmage/plugins/sensor/RLSimpleSensor
    scrimmage/src/plugins/sensor/RLSimpleSensor
    scrimmage/missions/rlsimple.xml
    scrimmage/test/test_openai.py

We can use the script provided with SCRIMMAGE to create these plugins (we will 
call them something different in this tutorial). To do so,
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
class covers the normal components of the autonomy plugin and allows to only
worry about creating a few methods specfic for OpenAI. We will thus have to
change our include statements to the following:

.. code-block:: c++
   :linenos:

   #include <scrimmage/plugins/autonomy/ExternalControl/ExternalControl.h>

   #include <map>
   #include <string>
   #include <utility>

   class SimpleLearner : public scrimmage::autonomy::ExternalControl {
    public:
       virtual void init(std::map<std::string, std::string> &params);
       virtual std::pair<bool, double> calc_reward(double t);

    protected:

       double radius_;
       virtual bool handle_action(
           double t, double dt, const scrimmage_proto::Action &action);
       virtual scrimmage_proto::SpaceParams action_space_params();

       uint8_t output_vel_x_idx_ = 0;
       uint8_t output_vel_y_idx_ = 0;
       uint8_t output_vel_z_idx_ = 0;
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

   #include <scrimmage/math/State.h>
   #include <scrimmage/plugin_manager/RegisterPlugin.h>
   #include <scrimmage/proto/ExternalControl.pb.h>

   #include <my-scrimmage-plugins/plugins/autonomy/SimpleLearner/SimpleLearner.h>

   namespace sp = scrimmage_proto;
   namespace sc = scrimmage;

   REGISTER_PLUGIN(scrimmage::Autonomy, SimpleLearner, SimpleLearner_plugin)


Next, let us look at the ``init``:

.. code-block:: c++
   :linenos:

   void SimpleLearner::init(std::map<std::string, std::string> &params) {
       output_vel_x_idx_ = vars_.declare(sc::VariableIO::Type::velocity_x, sc::VariableIO::Direction::Out);
       output_vel_y_idx_ = vars_.declare(sc::VariableIO::Type::velocity_y, sc::VariableIO::Direction::Out);
       output_vel_z_idx_ = vars_.declare(sc::VariableIO::Type::velocity_z, sc::VariableIO::Direction::Out);

       radius_ = std::stod(params.at("radius"));
       ExternalControl::init(params);
   }


In our case, the ``init`` just calls the inherited ``init`` from
ExternalControl and sets up the :ref:`variableio` class. This would also be
where parameters from the ``SimpleLearner.xml`` file would be initialized. From
here, we can then move on to look at
the action space representation for OpenAI:

.. code-block:: c++
   :linenos:

   scrimmage_proto::SpaceParams SimpleLearner::action_space_params() {
       sp::SpaceParams space_params;
       sp::SingleSpaceParams *single_space_params = space_params.add_params();
       single_space_params->set_discrete(true);
       single_space_params->set_num_dims(1);
       single_space_params->add_minimum(0);
       single_space_params->add_maximum(1);
       return space_params;
   }

In this method, we set up a discrete action space of one dimension.
This action will be interpreted in the ``handle_action`` method:

.. code-block:: c++
   :linenos:

   bool SimpleLearner::handle_action(double t, double dt, const scrimmage_proto::Action &action) {
       if (!check_action(action, 1, 0)) return false;

       double x_vel = action.discrete(0) == 1 ? 1 : -1;
       vars_.output(output_vel_x_idx_, x_vel);
       return true;
   }


Here, we can see that we take our ``action`` of 0 or 1, each being intepreted
to mean a negative or positive velocity, respectively.

Finally, we need to set up the reward for this environment in ``calc_reward``. This
function returns a `std::pair<bool, double>` which corresponds to whether the
environment is done and the reward:

.. code-block:: c++
   :linenos:

   std::pair<bool, double> SimpleLearner::calc_reward(double t) {
       return {false, state_->pos().head<2>().norm() < radius_};
   }


For this example, we do not use the time parameter but it is there for cases
where you want to include the time in your reward function. Our ``calc_reward``
gives 1 point if it is within some distance of the origin.

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
   :emphasize-lines: 2-3

   TARGET_LINK_LIBRARIES(${LIBRARY_NAME}
        ${SCRIMMAGE_LIBRARIES}
        ${SCRIMMAGE_PLUGINS}
     )

in order for ``SimpleLearner`` to see the ``ExternalControl`` autonomy plugin.
From here, we can now build the project: ::

  $ cd ~/scrimmage/my-scrimmage-plugins/build
  $ cmake ..
  $ make

xml file for OpenAI Autonomy
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The last part we need to edit is the ``SimpleLearner.xml`` located at:
``~/scrimmage/my-scrimmage-plugins/include/my-scrimmage-plugins/plugins/autonomy/SimpleLearner/SimpleLearner.xml``.
Here, we need to add a field for a server address to run SCRIMMAGE on. To do so,
simply add the following line to the ``<params>`` field.

.. code-block:: xml

  <library>SimpleLearner_plugin</library>
  <server_address>localhost:50051</server_address>
  <radius>5</radius>

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

   namespace scrimmage_proto {
   class SpaceParams;
   class SpaceSample;
   }

   class MyOpenAISensor : public scrimmage::Sensor {
    public:
       virtual scrimmage_proto::SpaceParams observation_space_params();
       virtual scrimmage::MessagePtr<scrimmage_proto::SpaceSample> sensor_msg_flat(double t);
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


   #include <my-scrimmage-plugins/plugins/sensor/MyOpenAISensor/MyOpenAISensor.h>

   #include <scrimmage/entity/Entity.h>
   #include <scrimmage/math/State.h>
   #include <scrimmage/plugin_manager/RegisterPlugin.h>
   #include <scrimmage/proto/ExternalControl.pb.h>

   namespace sc = scrimmage;
   namespace sp = scrimmage_proto;

   REGISTER_PLUGIN(scrimmage::Sensor, MyOpenAISensor, MyOpenAISensor_plugin)

From there, we then look at method implementation. ``observation_space_params``
creates a 4 dimensional state space for x,y, cos(heading), and sin(heading). It
sets the limits for each state as well as defines them as continous variables.
If we had more players in this space, it would add 4 states for each of them as
well.

.. code-block:: c++
   :linenos:

   scrimmage_proto::SpaceParams MyOpenAISensor::observation_space_params() {
       sp::SpaceParams space_params;

       const double inf = std::numeric_limits<double>::infinity();
       sp::SingleSpaceParams *single_space_params = space_params.add_params();
       single_space_params->set_num_dims(1);
       single_space_params->add_minimum(-inf);
       single_space_params->add_maximum(inf);
       single_space_params->set_discrete(false);

       return space_params;
   }


``sensor_msg_flat`` sets up a message type, fills in the message with the new
state information and then returns the message as its output. In
this case we are just outputting the x position:

.. code-block:: c++
   :linenos:

   scrimmage::MessagePtr<scrimmage_proto::SpaceSample>
   MyOpenAISensor::sensor_msg_flat(double t) {
       auto msg = std::make_shared<sc::Message<sp::SpaceSample>>();
       msg->data.add_value(parent_->state()->pos()(0));
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


          <motion_model>SingleIntegrator</motion_model>
          <controller>SingleIntegratorControllerSimple</controller>
      </entity_common>

      <entity entity_common="all">
        <x>0</x>
        <y>0</y>
        <z>0</z>
        <heading>0</heading>
        <team_id>1</team_id>
        <color>77 77 255</color>
        <autonomy server_address="localhost:50051">SimpleLearner</autonomy>
        <visual_model>Sphere</visual_model>
        <sensor order="0">MyOpenAISensor</sensor>
      </entity>

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
           mission_file = scrimmage.find_mission('rlsimple.xml')

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

           action = 1 if i < 100 else 0
           obs, reward, done = env.step(action)[:3]
           total_reward += reward

           if done:
               break

       env.close()
       print("Total Reward: %2.2f" % total_reward)

   if __name__ == '__main__':
      test_openai()

Now that we have completed all of the code, we can simply type the following
into the terminal to see it run! ::

  $ python test_openai.py
