.. _draw_shapes:

Draw Shapes in the Viewer
=========================

High-Level Shape API
~~~~~~~~~~~~~~~~~~~~

The high-level Shape API provides helper functions for generating shapes with
sensible default values. Helper functions for drawing spheres, text, circles,
cones, cubes, point clouds, etc. are defined in ``scrimmage/common/Shape.h``
. In a plugin, the following code will draw the white text, "Hello,
SCRIMMAGE!", at the location (10, 0, 0).

.. code-block:: c++

   // include the Shape.h header
   #include <scrimmage/common/Shape.h>

   ...

   // Generate the persistent shape object
   auto text_shape = scrimmage::shape::make_text("Hello, SCRIMMAGE!",
                                                 Eigen::Vector3d(10, 0, 0),       // Position
                                                 Eigen::Vector3d(255, 255, 255)); // Color

   // Draw the shape in the viewer
   draw_shape(text_shape);


Low-Level Shape API
~~~~~~~~~~~~~~~~~~~

The high-level Shape API provides access to most of the shape drawing
functionality. However, you may want to directly construct a shape object or
modify specific shape properties that are not modifiable through the high-level
Shape API. In this case, the low-level Shape API should be used. The low-level
Shape API requires the developer to directly use the Shape Protobuf class'
accessors and getters.

You can draw shapes in the SCRIMMAGE Viewer from any plugin. For example, you
could draw the text, "Hello, SCRIMMAGE!", with the following code in a plugin:

.. code-block:: c++
   :linenos:

   // Include the Shape header and some protobuf helper functions
   #include <scrimmage/proto/Shape.pb.h>         // scrimmage_proto::Shape
   #include <scrimmage/proto/ProtoConversions.h> // scrimmage::set()

Later in the init() or step() functions you can create a shape:

.. code-block:: c++
   :linenos:

   // Create a shared pointer to a scrimmage protobuf Shape instance
   auto text_shape = std::make_shared<scrimmage_proto::Shape>();

   // Set generic shape properties (opacity and color)
   text_shape->set_opacity(1.0);
   scrimmage::set(text_shape->mutable_color(), 255, 255, 255); // r, g, b

   // Set the shape's persistence to "true". This means the shape will not be
   // removed from the Viewer unless the plugin specifically removes it.
   text_shape->set_persistent(true);

   // Set the text's position
   scrimmage::set(text_shape->mutable_text()->mutable_center(), in_front);

   // Set the string text
   text_shape->mutable_text()->set_text("Hello, SCRIMMAGE!");

   // Draw the shape in the 3D viewer. The draw_shape function sets the Shape's
   // ID with a hash and appends it to the Plugin's private shapes_ member
   // variable. The simulation controller queries the Plugin for new shapes to
   // draw and then sends them to the Viewer.
   draw_shape(text_shape);

The scrimmage_proto::Shape protocol buffer class makes use of protobuf's
`oneof`_ feature. See the `Shape.proto`_ file for the types of shapes that can
be drawn. Note, you can only set the parameters for a single shape within the
oneof_type block in Shape.proto, otherwise, previous parameters will be
cleared.

Removing a Shape
----------------

To remove a shape after drawing it with the ``draw_shape()`` function, just set
the shape's persistence to false and call the ``draw_shape()`` function again:

.. code-block:: c++
   :linenos:

   text_shape_->set_persistent(false);
   draw_shape(text_shape);

You can also change the shape's color, opacity, and ttl after it has been
drawn by changing the value and calling ``draw_shape()`` again.

Shape Persistence and TTL
--------------------------

If you know that you want a shape to be visible for a specific number of frames
and then have it removed automatically, you can use the shape's peristence and
ttl (time-to-live, measured in simulation steps) parameters. Let's create a
cuboid that will only exist for five frames before it is removed from the
viewer.

.. code-block:: c++
   :linenos:

   // Create a shape instance
   auto wall = std::make_shared<scrimmage_proto::Shape>();

   // Set the color and opacity
   sc::set(wall->mutable_color(), 255, 0, 0); // r, g, b
   wall->set_opacity(1.0);

   // The shape will be removed after five simulation steps
   wall->set_persistent(false);
   wall->set_ttl(5);

   // Set the cube's center position to (10, 10, 10)
   sc::set(wall->mutable_cube()->mutable_center(), Eigen::Vector3d(10, 10, 10));

   // Set the x, y, and z lengths of the cuboid (3x3x3) (meters)
   wall->mutable_cube()->set_x_length(3);
   wall->mutable_cube()->set_y_length(3);
   wall->mutable_cube()->set_z_length(3);

   // Set the cube's rotation to zero:
   sc::Quaternion quat(0, 0, 0); // roll, pitch, yaw (radians)
   sc::set(wall->mutable_cube()->mutable_quat(), quat);

   // Draw the shape in the viewer
   draw_shape(wall);

We know which parameters to specify for the cube because they are explicitly
listed in `Shape.proto`_ in the ``Cube`` protobuf message:

.. code-block:: c++

   message Cube {
      double x_length = 1;
      double y_length = 2;
      double z_length = 3;
      Vector3d center = 4;
      Quaternion quat = 5;
   }

Additional Examples
-------------------

Draw a Sphere
~~~~~~~~~~~~~

The following will draw a persistent blue sphere with an opacity of 0.25, a
radius of 5 meters, at the position (1, 2, 3):

.. code-block:: c++
   :linenos:

   auto sphere = std::make_shared<scrimmage_proto::Shape>();
   sphere->set_opacity(0.25);
   sphere->set_persistent(true);
   sc::set(sphere->mutable_color(), 0, 0, 255); // r, g, b

   sphere->mutable_sphere()->set_radius(5);
   sc::set(sphere->mutable_sphere()->mutable_center(), 1, 2, 3);
   draw_shape(sphere);

Draw a Line
~~~~~~~~~~~

The following will draw a red line from (0, 0, 0) to (10, 10, 10).

.. code-block:: c++
   :linenos:

   auto line = std::make_shared<scrimmage_proto::Shape>();
   sc::set(line->mutable_color(), 255, 0, 0);
   line->set_opacity(0.75);
   sc::set(line->mutable_line()->mutable_start(), Eigen::Vector3d(0, 0, 0));
   sc::set(line->mutable_line()->mutable_end(), Eigen::Vector3d(10, 10, 10));
   draw_shape(line);


.. _oneof: https://developers.google.com/protocol-buffers/docs/proto#oneof
.. _Shape.proto: https://github.com/gtri/scrimmage/blob/master/src/proto/scrimmage/proto/Shape.proto
