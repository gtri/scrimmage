=================
Coordinate Frames
=================

The origin of the SCRIMMAGE coordinate system is a right-handed East-North-Up
(ENU) system centered on the ``latitude_origin``, ``altitude_origin``, and
``altitude_origin`` specified in the SCRIMMAGE mission file. When accessing an
entity's ``State`` variable, the elements of the ``State`` are with respect to
the ENU system's origin. The ``State`` class is used to represent an entity's
own state. The ``State`` class is also used when refering to the states of
other entities in the ``contacts_`` map.

Position
--------

In an autonomy plugin, the vehicle's own position relative to the ENU system's
origin can be accessed through the ``state_->pos()`` function. The ``pos()``
function returns an ``Eigen::Vector3d`` that contains the x, y, and z positions
of the entity.

.. code-block:: c++

   Eigen::Vector3d my_pos = state_->pos();

   my_pos(0); // x-position in ENU frame
   my_pos(1); // y-position in ENU frame
   my_pos(2); // z-position in ENU frame

Orientation
-----------

In an autonomy plugin, the entity's orientation can be accessed through the
``state_->quat()`` function, which returns a ``scrimmage::Quaternion``
instance. The ``scrimmage::Quaternion`` class (found in
``scrimmage/math/Quaternion.h``) has helper functions for returning the roll,
pitch, and yaw of the orientation (In SCRIMMAGE, the yaw is typically
synonomous with the heading).

.. code-block:: c++

   scrimmage::Quaternion q = state_->quat();

   q.roll();  // roll (radians) in ENU frame
   q.pitch(); // pitch (radians) in ENU frame
   q.yaw();   // yaw / heading (radians) in ENU frame

   // Create a quaternion that is initialized with roll, pitch, and yaw values
   double roll_rad = 0.1;
   double pitch_rad = 0.4;
   double yaw_rad = -0.5;
   scrimmage::Quaternion my_quat_0(roll_rad, pitch_rad, yaw_rad);

   // Create a quaternion from the standard w, x, y, z components:
   double w = 0;
   double x = 1;
   double y = 0;
   double z = 0;
   scrimmage::Quaternion my_quat_1(w, x, y, z);

In the SCRIMMAGE ENU frame, "East" points in the direction of the origin's
X-axis, "North" points in the direction of the origin's Y-axis, and "Up" points
in the direction of the origin's Z-axis. This means, that in SCRIMMAGE's ENU
frame, the positive X-axis points in the direction of 0 degrees yaw/heading and
the positive Y-axis points in the direction of 90 degrees
yaw/heading. Likewise, the negative X-axis points in the direction of 180
degrees yaw/heading and the negative Y-axis points in the direction of 270
degrees yaw/heading. This is opposed to the GPS heading standard, where 0
degrees heading points North and 90 degrees heading points East. To facilitate
translating between GPS and ENU frame orienations, the ``scrimmage::Angles``
class can be leveraged. For example, to convert from SCRIMMAGE ENU to GPS
heading, the following code segment can be used:

.. code-block:: c++

   #include <scrimmage/math/Angles.h>

   double enu_heading_rad = scrimmage::Angles::deg2rad(45); // ENU heading in radians

   // Input orientation is "EUCLIDEAN" (i.e., ENU) and output orientation is "GPS".
   scrimmage::Angles angles_to_gps(0, Angles::Type::EUCLIDEAN, Angles::Type::GPS);

   // The set_angle() function takes the yaw/heading in degrees
   angles_to_gps.set_angle(scrimmage::Angles::rad2deg(enu_heading_rad));

   // The angle() function returns the heading in degrees.
   double gps_heading_deg = angles_to_gps.angle();

   // The deg2rad() function
   double gps_heading_rad = scrimmage::Angles::deg2rad(gps_heading_deg);

Velocity
--------

The entity's linear and angular velocities are also with respect to the
SCRIMMAGE ENU frame. In an autonomy plugin, the linear and angular velocities
can be accessed through the ``state_->vel()`` and ``state_->ang_vel()``
functions, respectively. Since the ``vel()`` and ``ang_vel()`` functions return
``Eigen::Vector3d`` types, the linear speed can be computed using the ``norm()`` function:

.. code-block:: c++

   double speed = state_->vel().norm();

   // If the speed is NaN, then the vel() vector is probably all zeros.
   if (std::isnan(speed)) {
       speed = 0;
   }

ENU to GPS Conversion
---------------------

In SCRIMMAGE, `GeographicLib <https://geographiclib.sourceforge.io/>`_ is used
to perform conversions between local ENU positions and latitude, longitude, and
altitude. For example, to convert from latitude, longitude, and altitude to ENU
x, y, and z, a SCRIMMAGE plugin can use the
``parent_->projection()->Forward()`` function.

.. code-block:: c++

   #include <GeographicLib/LocalCartesian.hpp>

   double lat = 35.721025;
   double lon = -120.767925;
   double alt = 300;

   double x, y, z;
   parent_->projection()->Forward(lat, lon, alt, x, y, z);

   // x, y, z are now the local ENU positions relative to the SCRIMMAGE origin
   // specified in the mission file.

Likewise, the GeographicLib's ``Reverse()`` function is used to convert from
ENU x, y, and z positions to latitude, longitude, and altitude.

.. code-block:: c++

   #include <GeographicLib/LocalCartesian.hpp>

   // The x, y, and z positions relative to the SCRIMMAGE ENU origin.
   double x = 100;
   double y = 150;
   double z = 10;

   double lat, lon, alt;
   parent_->projection()->Reverse(x, y, z, lat, lon, alt);

   // lat, lon, and alt now contain the latitude, longitude, and altitude values

Motion Model Plugins
--------------------

When developing a motion model, it can be useful to model the system with
respect to a North-East-Down (NED) body frame instead of the ENU
frame. However, the rest of the SCRIMMAGE system requires the entity's
``State`` to be in the ENU frame. Thus, if the developer wishes to model in the
NED body frame, during the ``step()`` function of the motion model, the
vehicle's orientation in the ENU frame should be converted to the NED local
body frame by rotating the ``state_->quat()`` function. For example, the
`FixedWing6DOF
<https://github.com/gtri/scrimmage/blob/master/src/plugins/motion/FixedWing6DOF/FixedWing6DOF.cpp>`_
motion model uses the following code segment to obtain the NED frame's
quaterion from the ENU frame's quaterion:

.. code-block:: c++

   scrimmage::Quaternion quat_body_;

   ...

   quat_body_ = rot_180_x_axis_ * state_->quat();
   quat_body_.set(sc::Angles::angle_pi(quat_body_.roll()+M_PI),
                  quat_body_.pitch(), quat_body_.yaw());
   quat_body_.normalize();

At the end of the motion model's ``step()`` function, the (possibly rotated)
NED frame has to be transformed back into the ENU frame, so that the ``State``
orientation is in the same frame as the rest of the simulation.

.. code-block:: c++

   // Rotate back to Z-axis pointing up
   state_->quat() = rot_180_x_axis_ * quat_body_;
   state_->quat().set(sc::Angles::angle_pi(state_->quat().roll()+M_PI),
                      state_->quat().pitch(), state_->quat().yaw());
   state_->quat().normalize();

The remaining elements of the ``State`` class must be in the ENU frame as well:

.. code-block:: c++

   // A motion model must ensure that the State classes elements are in the ENU frame:
   state_->pos();     // position
   state_->quat();    // orientation
   state_->vel();     // linear velocity
   state_->ang_vel(); // angular velocity
