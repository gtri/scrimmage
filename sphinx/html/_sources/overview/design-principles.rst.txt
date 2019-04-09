Design Principles
======================

Entity state() vs state_truth()
-------------------------------

The purpose of the separate ``state()`` and ``state_truth()`` shared pointers
is to allow the researcher to simulate the effects of introducing noise into
the entity's own position, velocity, orientation, etc. When the researcher is
not concerned with introducing state noise, the ``state()`` and
``state_truth()`` pointers point to the same memory location. However, if the
researcher wants to introduce state noise, a sensor plugin can be used to
create a new state pointer instance and assign the pointers appropriately. For
example, in the ``init()`` function of the ``SimpleINS`` plugin, the following
code creates a new State pointer to hold noisy state information:

.. code-block:: c++
   :linenos:

   // Create a new State instance
   parent_->state() = std::make_shared<State>();

   // Initialize the new state's values with the ground truth values
   *(parent_->state()) = *(parent_->state_truth());

In the ``step()`` function, the plugin copies the ``state_truth()`` pointer,

.. code-block:: c++
   :linenos:

   StateWithCovariance ns(*(parent_->state_truth()));

adds noise to the copied state values, and then assigns the ``state()`` pointer
with the noisy values:

.. code-block:: c++
   :linenos:

   *(parent_->state()) = static_cast<sc::State>(msg->data);

This allows the autonomy and controller plugins to only have to consider the
``state_`` pointer without having to consider whether ground truth or noisy
state information is being used in the simulation. However, the developer has
to determine whether they need to access the possibly noisy ``state()`` or the
``state_truth()`` pointers when writing sensor, interaction, metric, and
network plugins. Motion model plugins only need to consider the ``state_``
pointer since they are always initialized with the ``ground_truth_`` pointer in
``Entity.cpp``.

Testing a Real INS System
~~~~~~~~~~~~~~~~~~~~~~~~~

A real inertial navigation system (INS) will take inputs from GPS, LIDAR,
accelerometers, and gyroscopes. If the researcher wants to develop and simulate
an INS implementation, GPS, LIDAR, accelerometer, and gyroscope sensor plugins
should be configured to publish their respective data to an INS sensor
plugin. The INS sensor plugin should be the only the sensor plugin that makes
and reassigns the ``state()`` pointer in its ``init()`` function. After each
step of the INS sensor plugin, it should assign the ``state()`` pointer with
its best estimate of the entity's state. This will allow the entity's autonomy
and controller plugins to use this best estimated state for later calculations.


Noisy Contacts
--------------

For now, the developer can use the NoisyContacts sensor plugin. However, we
will soon be developing a noisy / ground truth interface to contacts that is
similar to the entity's interface.
