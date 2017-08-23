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

   void MyNoisyState::init(std::map<std::string,std::string> &params)
   {
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
           
       return;
   }
 
A Sensor plugin must implement the ``sensor_msg`` method, which returns message
containing the sampled sensor data. The sensor message is wrapped by a
boost::optional instance, which allows sensor_msg to return an "invalid" sensor
message by returning ``boost::optional<sc::MessageBasePtr>{}`` if the sensor
plugins wants to simulate invalid readings or enforce a sampling rate.

.. code-block:: c++
   :linenos:

   boost::optional<scrimmage::MessageBasePtr> MyNoisyState::sensor_msg(double t)
   {
       // Make a copy of the current state
       sc::State ns = *(parent_->state());
   
       // Create a message to hold the modified state
       auto msg = std::make_shared<sc::Message<sc::State>>();
   
       // Add noise to the three scalars in the 3D position vector.
       for (int i = 0; i < 3; i++) {
           msg->data.pos()(i) = ns.pos()(i) + (*pos_noise_[i])(*gener_);    
       }    
   
       // Return the sensor message.
       return boost::optional<sc::MessageBasePtr>(msg);
   }
     
An Autonomy or Controller plugin can use this sensor by adding the sensor to
the entity block in the mission file:

.. code-block:: xml

   <entity>
   ...
   <sensor>MyNoiseState</sensor>
   ...
   </entity>

and then "sampling" from the sensor by calling its ``sense`` method in the
Autonomy's ``step_autonomy`` method:

.. code-block:: c++
   :linenos:  

   bool Straight::step_autonomy(double t, double dt)
   {
       sc::State own_state; // Will hold noisy own state measurement

       // Loop through all possible sensors defined for entity
       for (auto kv : parent_->sensors()) {
           bool valid;
           if (kv.first == "MyNoisyState") {
               // Take a measurement from the MyNoisyState sensor
               auto msg = kv.second->sense<sc::Message<sc::State>>(t, valid);
               if (valid) {
                   own_state = msg->data; // Save valid measurement
               } else {
                   // Should handle an invalid own_state here by keeping track
                   // of the last valid own_state
               }
           }
       }

       // Use the noisy state own_state below for decision making...

       ...

       
