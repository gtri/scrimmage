.. _controller_plugin:

Create a Controller Plugin
==========================

A Controller plugin converts the ``desired_state_`` variable and the current
``state_`` variable into actuator inputs that drive the motion model's
dynamics. Currently, there isn't an automated script to create a Controller
plugin, so if you need to create a controller plugin, you will need to manually
create files and name them appropriately. This will be fixed in the near
future. Currently, Controller plugins are directly associated with Motion Model
plugins, so by convention, Controller plugins are placed in the corresponding
Motion model's directory. (e.g., ``SimpleAircraftControllerPID`` is placed
under the ``motion/SimpleAircraft`` directory).

Examine Controller Plugin
-------------------------

Let's take a look at the SimpleAircraftControllerPID Controller plugin. The
SimpleAircraftControllerPID controls the thrust, roll, pitch, and yaw of the
SimpleAircraft in order to reach the desired speed, altitude, and heading set
by the Autonomy plugin. The SimpleAircraftControllerPID plugin makes use of
SCRIMMAGE's built-in PID controller. The ``init`` function sets the PID gains
with the help of a helper function, ``set_pid``:

.. code-block:: c++
   :linenos:

   void set_pid(sc::PID &pid, std::string str, bool is_angle) {
       std::vector<std::string> str_vals;
       boost::split(str_vals, str, boost::is_any_of(","));
   
       if (str_vals.size() != 4) {
           std::cout << "error parsing in SimpleAircraftControllerPID" << std::endl;
       } else {
           double p = std::stod(str_vals[0]);
           double i = std::stod(str_vals[1]);
           double d = std::stod(str_vals[2]);
           pid.set_parameters(p, i, d);
   
           if (is_angle) {
               double i_lim = sc::Angles::deg2rad(std::stod(str_vals[3]));
               pid.set_integral_band(i_lim);
               pid.set_is_angle(true);
           } else {
               double i_lim = std::stod(str_vals[3]);
               pid.set_integral_band(i_lim);
           }
       }
   }
   
   void SimpleAircraftControllerPID::init(std::map<std::string, std::string> &params) {
       set_pid(heading_pid_, params["heading_pid"], true);
       set_pid(alt_pid_, params["alt_pid"], false);
       set_pid(vel_pid_, params["vel_pid"], false);
       u_ = std::make_shared<Eigen::Vector3d>();
   }

The plugin's XML file contains the actual PID gain values:

.. code-block:: xml

   <params>

   ...
   
   <heading_pid>0.2, 0.01, 0.001, 9</heading_pid>
   <alt_pid>0.0025, 0.0001, 0.0002, 1</alt_pid>
   <vel_pid>1, 0.1, 0, 1</vel_pid>

   ...

   </params>

The Controller's ``step`` method merely assigns the new setpoints coming from
the Autonomy plugin to each PID controller, tells each PID the current value of
the state variable being tracked, and tells each PID to update itself. The PID
output values are assigned to the ``vars_`` variable available to all SCRIMMAGE 
plugins, which is retrieved later by the Motion Model plugin when it runs.

.. code-block:: c++
   :linenos:
   
   bool SimpleAircraftControllerPID::step(double t, double dt) {
       heading_pid_.set_setpoint(vars_.input(input_roll_or_heading_idx_));
       double u_roll_rate = use_roll_ ?
           -heading_pid_.step(dt, state_->quat().roll()) :
           heading_pid_.step(dt, state_->quat().yaw());

       alt_pid_.set_setpoint(vars_.input(input_altitude_idx_));
       double u_pitch_rate = -alt_pid_.step(dt, state_->pos()(2));

       vel_pid_.set_setpoint(vars_.input(input_velocity_idx_));
       double u_throttle = vel_pid_.step(dt, state_->vel().norm());

       vars_.output(output_roll_rate_idx_, u_roll_rate);
       vars_.output(output_pitch_rate_idx_, u_pitch_rate);
       vars_.output(output_throttle_idx_, u_throttle);
       return true;
   }