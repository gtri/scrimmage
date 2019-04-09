.. _sensor_plugin:

Create a Sensor Plugin
======================

The purpose of a Sensor plugin is to transform ground truth information in the
simulation to altered or "noisy" sensor measurements. Engineers and developers
can model their noise sources and simulate how noisy data will affect their
autonomy, control, and tracking algorithms. To create a Sensor plugin, enter
the following at the terminal: ::

  $ cd /path/to/scrimmage/scripts
  $ ./generate-plugin.sh sensor MyNoisyState ~/scrimmage/my-scrimmage-plugins

Build the sensor plugin: ::

  $ cd ~/scrimmage/my-scrimmage-plugins/build
  $ cmake ..
  $ make

Examine Sensor Plugin
---------------------

Similar to an Autonomy plugin, a Sensor plugin has an ``init`` method that is
called during plugin initialization. In this plugin, we setup the Gaussian
noise sources that will be used to add noise to our own state's position.

.. code-block:: c++
   :linenos:

   void MyNoisyState::init(std::map<std::string, std::string> &params) {
       // Use the same generator as the parent so that the simulation is
       // completely deterministic with respect to the simulation seed.
       gener_ = parent_->random()->gener();

       // Create three independent gaussian noise generators. They will use the
       // same generator seed.
       for (int i = 0; i < 3; i++) {
           std::string tag_name = "pos_noise_" + std::to_string(i);
           std::vector<double> vec;
           bool status = sc::get_vec(tag_name, params, " ", vec, 2);
           if (status) {
               pos_noise_.push_back(parent_->random()->make_rng_normal(vec[0], vec[1]));
           } else {
               pos_noise_.push_back(parent_->random()->make_rng_normal(0, 1));
           }
       }

       pub_ = advertise("LocalNetwork", "NoisyState");
   }


A Sensor plugin must implement the ``step`` method, which is generally
expected to publish some messages.
It should return a boolean indicating if the function was successful.

.. code-block:: c++
   :linenos:

   bool MyNoisyState::step() {
       // Make a copy of the current state
       sc::State ns = *(parent_->state_truth());

       // Create a message to hold the modified state
       auto msg = std::make_shared<sc::Message<sc::State>>();

       // Add noise to the three scalars in the 3D position vector.
       for (int i = 0; i < 3; i++) {
           msg->data.pos()(i) = ns.pos()(i) + (*pos_noise_[i])(*gener_);
       }

       // Return the sensor message.
       pub_->publish(msg);
       return true;
   }

An Autonomy or Controller plugin can use this sensor by adding the sensor to
the entity block in the mission file:

.. code-block:: xml

   <entity>
   ...
   <sensor>MyNoiseState</sensor>
   ...
   </entity>

and then "sampling" from the sensor by creating a subscriber:

.. code-block:: c++
   :linenos:

   void Straight::init(std::map<std::string, std::string> &params) {
       auto state_cb = [&](auto &msg) {
           noisy_state_set_ = true;
           noisy_state_ = msg->data;
       };
   }
