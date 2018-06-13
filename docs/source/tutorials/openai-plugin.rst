.. _openai_plugin:

Create an OpenAI Plugin
=========================

SCRIMMAGE can be used in conjunction with OpenAI to generate environments for
learning scenarios. OpenAI environments need to have a state space, action
space, and a reward function. In this tutorial, we will go through a quick
example of how to set up a scrimmage agent to interact with OpenAI by providing
a state space, action space, and reward. We will use the plugins project we
created in a previous tutorial (:doc:`create-project`) to build our new OpenAI
plugins. To start with, we will need to create an Autonomy and a Sensor plugin.
Files similar to what is described below are located in the following places::

    scrimmage/include/scrimmage/plugins/autonomy/RLSimple
    scrimmage/src/plugins/autonomy/RLSimple
    scrimmage/include/scrimmage/plugins/sensor/RLSimpleSensor
    scrimmage/src/plugins/sensor/RLSimpleSensor
    scrimmage/missions/rlsimple.xml
    scrimmage/test/test_openai.py

In addition, the interface between python and scrimmage is defined in
``scrimmage/python/scrimmage/bindings/src/py_openai_env.h``.
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

In general, the scrimmage openai interface allows you to customize the environment
in the following ways:

    * n agents - The most common scenario is 1 agent but SCRIMMAGE also
      supports multi-agent reinforcement learning. In particular, it supports 
      centralized approaches where actions and observations for individual
      learning agents are combined into a single action. Alternatively,
      one can have multiple agents operate independently.

    * Action/Observation space - this can be discrete, continuous, or combined
      (the latter would a a ``TupleSpace`` in openai).

For a demonstration of these basic combinations, see ``test/test_openai.py``. 
For this tutorial, we will only be investigating how to develop a discrete
action space and a continuous observation space.

Rewrite the Autonomy Plugin
---------------------------

OpenAI Autonomy Plugin Header File
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In this tutorial, we are going to take the autonomy plugin and rewrite for use
with OpenAI. Let's start with the header file located at
``~/scrimmage/my-scrimmage-plugins/include/my-scrimmage-plugins/plugins/autonomy/SimpleLearner/SimpleLearner.h``.

Normal autonomy plugins extend the Autonomy class. For the OpenAI autonomy
plugin, we will instead extend the ``ScrimmageOpenAIAutonomy`` autonomy plugin.
In addition to the normal ``init`` and ``step_autonomy`` virtual methods,
``ScrimmageOpenAIAutonomy`` defines some extra components:

    * ``reward_range`` - this setting maps directly to an openai environment's
      reward_range.

    * ``action_space`` - this is a struct containing a vector of ``discrete_counts``
      and ``continuous_extrema``. For each discrete action you want your autonomy
      to have, add an element to ``action_space.discrete_counts`` with the number
      of discrete actions (this will result in a ``Discrete`` or ``MultiDiscrete``
      openai space, depending on whether there is one or multiple elements
      in ``action_space.discrete_counts``). Similarly, if you want continuous actions,
      set ``action_space.continuous_extrema`` with the low and high values
      (this will result in a ``Box`` space). If both ``discrete_counts``
      and ``continuous_extrema`` have elements then the resulting space will
      be an openai ``TupleSpace``.

    * ``set_environment`` - a virtual method that will be called after
      all agents have been generated. This is meant to set ``reward_range``
      and ``action_space``

    * ``calc_reward`` - called every timestep, this function returns
      a pair representing whether the environment is done and what 
      the reward for that step is.

In python, when you call ``env.step(action)``, the action will be copied
to your autonomy's ``action`` member (defined in ``ScrimmageOpenAIAutonomy``. 
Here is the include file for ``SimpleLearner``:

.. code-block:: c++
   :linenos:

    #include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/ScrimmageOpenAIAutonomy.h>

    #include <map>
    #include <string>
    #include <utility>

    class SimpleLearner : public scrimmage::autonomy::ScrimmageOpenAIAutonomy {
     public:
        void init(std::map<std::string, std::string> &params) override;
        bool step_autonomy(double t, double dt) override;

        void set_environment() override;
        std::pair<bool, double> calc_reward(double t, double dt) override;

     protected:
        double radius_;
        uint8_t output_vel_x_idx_ = 0;
    };


Note that we are overriding the normal two virtual functions of ``init``
and ``step_autonomy`` but in addition we will override ``set_environment``
and ``calc_reward``.

OpenAI Autonomy Plugin Source File
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Now let's open our source file located at
``~/scrimmage/my-scrimmage-plugins/src/plugins/autonomy/SimpleLearner/SimpleLearner.cpp``.

We will first change the includes at the top of the file to be:

.. code-block:: c++
   :linenos:

    #include <scrimmage/math/State.h>
    #include <scrimmage/parse/ParseUtils.h>
    #include <scrimmage/plugin_manager/RegisterPlugin.h>

    // override this for your namespace
    #include <scrimmage/plugins/autonomy/RLSimple/RLSimple.h>

    // override this for your namespace
    REGISTER_PLUGIN(scrimmage::Autonomy, SimpleLearner, SimpleLearner_plugin)

Next, let us look at the ``init``:

.. code-block:: c++
   :linenos:

    void SimpleLearner::init(std::map<std::string, std::string> &params) {
        using Type = scrimmage::VariableIO::Type;
        using Dir = scrimmage::VariableIO::Direction;

        output_vel_x_idx_ = vars_.declare(Type::velocity_x, Dir::Out);
        const uint8_t output_vel_y_idx = vars_.declare(Type::velocity_y, Dir::Out);
        const uint8_t output_vel_z_idx = vars_.declare(Type::velocity_z, Dir::Out);

        vars_.output(output_vel_x_idx_, 0);
        vars_.output(output_vel_y_idx, 0);
        vars_.output(output_vel_z_idx, 0);

        radius_ = std::stod(params.at("radius"));

        ScrimmageOpenAIAutonomy::init(params);
    }

We now define the environment:
.. code-block:: c++
   :linenos:
  
    void SimpleLearner::set_environment() {
        reward_range = std::make_pair(0, 1);
        action_space.discrete_count.push_back(2);
    }

This says that the reward range will be between 0 and 1
and we will have a single discrete action that can take values of 0 or 1.
We now define the ``calc_reward`` function:

.. code-block:: c++
   :linenos:

    std::pair<bool, double> SimpleLearner::calc_reward(double /*t*/, double /*dt*/) {
        const bool done = false;
        const double x = state_->pos()(0);
        const bool within_radius = std::round(std::abs(x)) < radius_;
        double reward = within_radius ? 1 : 0;
        return {done, reward};
    }

This says that the autonomy is never going to end the simulation and gives
a reward for being within ``1`` of the origin. We now define ``step_autonomy``
to handle actions given from python. It will have positive x-velocity
when the action is 1 and negative x-velocity when the action is 0:

.. code-block:: c++
   :linenos:

    bool SimpleLearner::step_autonomy(double /*t*/, double /*dt*/) {
        const double x_vel = action.discrete[0] ? 1 : -1;
        vars_.output(output_vel_x_idx_, x_vel);
        return true;
    }

Rewrite CMakeLists.txt for OpenAI Autonomy
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``SimpleLearner`` C++ code is now finished. Before we can build it though,
we do need to make a small edit to the ``CMakeLists.txt``. Open up
``~/scrimmage/my-scrimmage-plugins/src/plugins/autonomy/SimpleLearner/CMakeLists.txt``
and change line 15 from

.. code-block:: cmake
   :linenos:

   TARGET_LINK_LIBRARIES(${LIBRARY_NAME}
    scrimmage
    ScrimmageOpenAIAutonomy_plugin
     )

This makes sure the plugin links to the libraries it needs. 

Plugin Parameter File 
~~~~~~~~~~~~~~~~~~~~~

The following is the parameter file for ``SimpleLearner``:

.. code-block:: xml
    :linenos"

    <?xml version="1.0"?>
    <?xml-stylesheet type="text/xsl" href="http://gtri.gatech.edu"?>
    <params>
      <library>SimpleLearner_plugin</library>
      <radius>2</radius>
    </params>
        
From here, we can now build the project::

  $ cd ~/scrimmage/my-scrimmage-plugins/build
  $ cmake ..
  $ make

Rewrite the Sensor Plugin
-------------------------

OpenAI Sensor Plugin Header File
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The sensor plugin is very similar to the autonomy plugin. It inherits
from ``ScrimmageOpenAISensor`` which provides
the following:

    * ``observation_space`` - this has the same effect as ``action_space`` above
      but will determine the environment's observation space.

    * ``set_observation_space`` - this is similar to ``set_environment`` above
      but is designed to set the variable ``observation_space`` after all entities
      have been generates.

    * ``get_observation`` - there are two versions of this virtual function:
      one for discrete observations and another for continuous observations.
      Note that because observations can sometimes be high dimensional,
      these functions directly edit the underlying python buffers.
      This avoids a needless copy.

Now let's move on to defining the observation space. We shall do this with through a
sensor plugin to OpenAI. We shall start by rewriting the header file for the
sensor plugin we created above. You can find it at
``~/scrimmage/my-scrimmage-plugins/include/my-scrimmage-plugins/plugins/sensor/MyOpenAISensor/MyOpenAISensor.h``.

First up, we shall rewrite the includes in ``MyOpenAISensor.h`` to be the
following. The main thing to note is that it inherits from ``ScrimmageOpenAISensor``
and overrides two virtual methods:

.. code-block:: c++
   :linenos:

    #include <scrimmage/plugins/sensor/ScrimmageOpenAISensor/ScrimmageOpenAISensor.h>

    #include <map>
    #include <string>
    #include <vector>

    class MyOpenAISensor : public scrimmage::sensor::ScrimmageOpenAISensor {
     public:
        void set_observation_space() override;
        void get_observation(double* data, uint32_t beg_idx, uint32_t end_idx) override;
    };

OpenAI Sensor Plugin Source File
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

From here, we can now look at the implementation of these methods in
``~/scrimmage/my-scrimmage-plugins/src/plugins/sensor/MyOpenAISensor/MyOpenAISensor.cpp``.

In this source file, we need to add the following includes:

.. code-block:: c++
   :linenos:

    // override this for your namespace
    #include <scrimmage/plugins/sensor/MyOpenAISensor/MyOpenAISensor.h>

    #include <scrimmage/entity/Entity.h>
    #include <scrimmage/math/State.h>
    #include <scrimmage/plugin_manager/RegisterPlugin.h>

    // override this for your namespace
    REGISTER_PLUGIN(scrimmage::Sensor, MyOpenAISensor, MyOpenAISensor_plugin)

    void MyOpenAISensor::get_observation(double *data, uint32_t beg_idx, uint32_t /*end_idx*/) {
        data[beg_idx] = parent_->state()->pos()(0);
    }

    void MyOpenAISensor::set_observation_space() {
        const double inf = std::numeric_limits<double>::infinity();
        observation_space.continuous_extrema.push_back(std::make_pair(-inf, inf));
    }

Plugin Parameter File 
~~~~~~~~~~~~~~~~~~~~~

.. code-block: xml
    :linenos:

    <?xml version="1.0"?>
    <?xml-stylesheet type="text/xsl" href="http://gtri.gatech.edu"?>
    <params>
      <library>MyOpenAISensor_plugin</library>
    </params>

Rewrite CMakeLists.txt for OpenAI Sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``MyOpenAISensor`` C++ code is now finished. Before we can build it though,
we do need to make a small edit to the ``CMakeLists.txt``. Open up
``~/scrimmage/my-scrimmage-plugins/src/plugins/autonomy/SimpleLearner/CMakeLists.txt``
and change line 15 from

.. code-block:: cmake
   :linenos:

   TARGET_LINK_LIBRARIES(${LIBRARY_NAME}
    scrimmage
    ScrimmageOpenAISensor_plugin
     )


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

          <team_id>1</team_id>
          <visual_model>Sphere</visual_model>
          <motion_model>SingleIntegrator</motion_model>
          <controller>SingleIntegratorControllerSimple</controller>
          <sensor order="0">MyOpenAISensor</sensor>
          <autonomy
            discrete_x="true"
            discrete_y="true"
            ctrl_y="false">SimpleLearner</autonomy>
          <y>0</y>
          <z>0</z>
      </entity_common>

      <entity entity_common="all">
        <x>0</x>
        <color>77 77 255</color>
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

    import copy
    import gym
    import scrimmage


    def test_openai():
        try:
            env = gym.make('scrimmage-v0')
        except gym.error.Error:
            mission_file = scrimmage.find_mission('rlsimple.xml')

            gym.envs.register(
                id='scrimmage-v0',
                entry_point='scrimmage:ScrimmageOpenAIEnv',
                max_episode_steps=1e9,
                reward_threshold=1e9,
                kwargs={"enable_gui": True,
                        "mission_file": mission_file}
            )
            env = gym.make('scrimmage-v0')

        # the observation is the x position of the vehicle
        # note that a deepcopy is used when a history
        # of observations is desired. This is because
        # the sensor plugin edits the data in-place
        obs = []
        obs.append(copy.deepcopy(env.reset()))
        total_reward = 0
        for i in range(200):

            action = 1 if i < 100 else 0
            temp_obs, reward, done = env.step(action)[:3]
            obs.append(copy.deepcopy(temp_obs))
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
