.. _motion_model_plugin:

Create a Motion Model Plugin
============================

A Motion Model plugin applies the controller's inputs to the current vehicle's
state to generate a new state. To create a motion model plugin, enter
the following at the terminal: ::

  $ cd /path/to/scrimmage/scripts
  $ ./generate-plugin.sh motion MyCarModel ~/scrimmage/my-scrimmage-plugins

Build the motion model plugin: ::

  $ cd ~/scrimmage/my-scrimmage-plugins/build
  $ cmake ..
  $ make

Examine Motion Model Plugin
---------------------------

The Motion Model plugin implements the ``init`` method and is used to
initialize the motion model's state variables as well as the ``state_``
instance used by the main SCRIMMAGE simulation controller. It is the job of the
motion model to update the ``state_`` variable when the ``step`` method is
called and during initialization.

.. code-block:: c++
   :linenos:

   bool MyCarModel::init(std::map<std::string, std::string> &info,
                        std::map<std::string, std::string> &params)
   {
       // Get the initial internal state values
       x_[X] = std::stod(info["x"]); //x
       x_[Y] = std::stod(info["y"]); //y
       x_[Z] = std::stod(info["z"]); //y
       x_[THETA] = sc::Angles::deg2rad(std::stod(info["heading"]));    
           
       length_ = sc::get<double>("length", params, 100.0);
   
       // Set the state_ variable that is used by the entity and the main
       // simulation controller
       state_->vel() << 0, 0, 0;
       state_->pos() << x_[X], x_[Y], x_[Z];
       state_->quat().set(0, 0, x_[THETA]);
       
       return true;
   }

This motion model plugin uses a simple car model that has a minimum turning
radius. We will use boost's ODE solver to numerically integrate the car model's
differential state equations. To do this, the plugin has to implement the
``model`` method, which is called when ``ode_step`` is called in ``step``. The
``model`` method retrieves the controller's inputs, applies possible state
saturations, and implements the differential equations (dx/dt form).

.. code-block:: c++
   :linenos:

   void MyCarModel::model(const vector_t &x , vector_t &dxdt , double t)
   {
       // Get the controller's actuator inputs
       Eigen::Vector2d &u = std::static_pointer_cast<Controller>(parent_->controllers().back())->u();
       double u_vel = u(FORWARD_VELOCITY);
       double u_theta = u(TURN_RATE);
       
       // Saturate wheel angle:
       if (u_theta >= M_PI/4) {
           u_theta = M_PI/4 - 0.0001;
       } else if (u_theta <= -M_PI/4) {
           u_theta = -M_PI/4 + 0.0001;
       }
   
       // Define the state differential
       dxdt[X] = u_vel*cos(x[THETA]);
       dxdt[Y] = u_vel*sin(x[THETA]);
       dxdt[THETA] = u_vel/length_*tan(u_theta);
   
       dxdt[Z] = 0; // Remain at initial z-position    
   }

The ``model`` method is called when ``ode_step`` is called in ``step.``

.. code-block:: c++
   :linenos:

   bool MyCarModel::step(double time, double dt)
   {
       // Save the previous x, y, z to calculate velocities
       double prev_x = x_[X];
       double prev_y = x_[Y];
       double prev_z = x_[Z];
       
       ode_step(dt); // step the motion model ODE solver
   
       // Save state (position, velocity, orientation) used by simulation
       // controller
       state_->vel() << (x_[X] - prev_x) / dt,
           (x_[Y] - prev_y) / dt,
           (x_[Z] - prev_z) / dt;
       
       state_->pos() << x_[X], x_[Y], x_[Z];
       state_->quat().set(0, 0, x_[THETA]);
       return true;
   }

It is the responsibility of the ``step`` method to run the numerical
integration and update the ``state_`` variable, which is later used by the main
SCRIMMAGE simulation controller.

The motion model is assigned to an entity by setting the ``motion_model`` XML
tag in the entity block:

.. code-block:: xml

   <entity>
   ...
   <motion_model>MyCarModel</motion_model>
   ...
   </entity>
