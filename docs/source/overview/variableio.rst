.. _variableio:

Variable IO
-----------

SCRIMMAGE exists to allow plugins to be easily interchangeable
with each other. One example of this is swapping out motion models.
A user can start with a low fidelity motion model (e.g., a single integrator)
where it is fast to lots of batch runs and optimizations. After
this initial step, a higher fidelity motion model (e.g., 6-dof) can be used.
However, this creates an issue because different motion models may 
require different inputs. The ``VariableIO`` class exists
to resolve this interfacing problem.

In a plugin's ``init`` function, it will declare what it expects
to receive as well as what it outputs. Since a motion model
only needs inputs (it always outputs an updated state), this 
may look something like (example for a ``SingleIntegrator``):
    
.. code-block:: c++
    :linenos:

    vel_x_idx_ = vars_.declare(VariableIO::Type::velocity_x, VariableIO::Direction::In);
    vel_y_idx_ = vars_.declare(VariableIO::Type::velocity_y, VariableIO::Direction::In);
    vel_z_idx_ = vars_.declare(VariableIO::Type::velocity_z, VariableIO::Direction::In);

These lines say the motion model expects a velocity in the `x`, `y`, and `z` directions.
The second argument to ``declare`` is the direction (in this case it is an input).
The first argument is an enum that will internally be converted
to a string. SCRIMMAGE offers the enum option for the first argument
so that typing errors can be found at compile rather than runtime. However,
if a type is not available (see the list in ``scrimmage/common/VariableIO.h``),
a user can also specify a string:

.. code-block:: c++
    :linenos:

    vel_x_idx_ = vars_.declare("velocity_x", VariableIO::Direction::In);
    vel_y_idx_ = vars_.declare("velocity_y", VariableIO::Direction::In);
    vel_z_idx_ = vars_.declare("velocity_z", VariableIO::Direction::In);

The output of ``declare`` is an index into a vector internal to the
``VariableIO`` object. A controller plugin will output values to 
this vector, also through a  ``VariableIO`` object. The controller
should expect an input from an autonomy and will output to a 
motion model, so in its ``init`` function the declaration will look like this:

.. code-block:: c++
    :linenos:

    input_pos_x_idx_ = vars_.declare(VariableIO::Type::position_x, VariableIO::Direction::In);
    input_pos_y_idx_ = vars_.declare(VariableIO::Type::position_y, VariableIO::Direction::In);
    input_pos_z_idx_ = vars_.declare(VariableIO::Type::position_z, VariableIO::Direction::In);

    output_vel_x_idx_ = vars_.declare(VariableIO::Type::velocity_x, VariableIO::Direction::Out);
    output_vel_y_idx_ = vars_.declare(VariableIO::Type::velocity_y, VariableIO::Direction::Out);
    output_vel_z_idx_ = vars_.declare(VariableIO::Type::velocity_z, VariableIO::Direction::Out);

In other words, it expects to take in a position and outputs a velocity (this comes 
from ``SingleIntegratorControllerWaypoint`` which is a proportional controller).
Finally, an autonomy will provide the desired position:

.. code-block:: c++
    :linenos:

    output_pos_x_idx_ = vars_.declare(VariableIO::Type::position_x, VariableIO::Direction::Out);
    output_pos_y_idx_ = vars_.declare(VariableIO::Type::position_y, VariableIO::Direction::Out);
    output_pos_z_idx_ = vars_.declare(VariableIO::Type::position_z, VariableIO::Direction::Out);

Now that the declarations have been made, it is time to use them.
If the autonomy wants to go to position ``(5, 10, 0)``
it will execute the following:

.. code-block:: c++
    :linenos:
    
    vars_.output(output_pos_x_idx_, 5);
    vars_.output(output_pos_y_idx_, 10);
    vars_.output(output_pos_z_idx_, 0);

The ``vars_`` variable is available to all SCRIMMAGE plugins. Similar
lines exist in the controller.

SCRIMMAGE will also attempt to figure out if a motion or controller
needs an input not provided by a controller or autonomy, respectively.
In particular, if you get the following message, all you need to 
do is follow the prompts::

    VariableIO Error: no autonomies provide inputs required by Controller "SimpleAircraftControllerPID". Add VariableIO output declarations in Straight, as follows
    First, place the following in its initializer:
        altitude_idx_ = vars_.declare("altitude", VariableIO::Direction::Out);
        heading_idx_ = vars_.declare("heading", VariableIO::Direction::Out);
        velocity_idx_ = vars_.declare("velocity", VariableIO::Direction::Out);
    Second, place the following in its step function:
        vars_.output(altitude_idx_, value_to_output);
        vars_.output(heading_idx_, value_to_output);
        vars_.output(velocity_idx_, value_to_output);
    where value_to_output is what you want SimpleAircraftControllerPID to receive as its input.
    Third, place following in the class declaration:
        uint8_t altitude_idx_ = 0;
        uint8_t heading_idx_ = 0;
        uint8_t velocity_idx_ = 0;
    Failed to parse entity at start position: x=-900, y=0
    Failed to generate entity


