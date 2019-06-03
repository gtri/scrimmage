.. _gen_entities:

Generate Entities at Run-Time
=============================

There are two options for generating entities during run-time:

1. Make use of the ``generate_rate``, ``generate_count``,
   ``generate_start_time``, and ``generate_time_variance`` XML tags defined in
   :ref:`scrimmage_xml`.

2. Create a plugin that publishes the ``scrimmage_msgs::GenerateEntity``
   message on the "GenerateEntity" topic.

This document describes how to accomplish the second option.

Mission File Setup
------------------

The first step is to create a valid entity block in your mission XML file with
an entity ``tag`` and a ``count`` of zero. For example, the start of the
following entity block has a ``tag`` of "gen_straight" and a ``count`` of zero,
which prevents it from being generated at the start of the simulation.

.. code-block:: xml

   <!-- Entity that is generated during runtime by other plugins -->
   <entity tag="gen_straight">
     <count>0</count>
     ...
   </entity>

Create the GenerateEntity Message
---------------------------------

The second step is to publish a ``scrimmage_msgs::GenerateEntity`` message on
the "GenerateEntity" topic (GlobalNetwork) in a plugin (any type of plugin).

Your plugin should include the appropriate protobuf header files:

.. code-block:: c++

   #include <scrimmage/pubsub/Publisher.h>
   #include <scrimmage/proto/State.pb.h>
   #include <scrimmage/msgs/Event.pb.h>
   #include <scrimmage/proto/ProtoConversions.h>

In the plugin's ``init()`` function, create the publisher:

.. code-block:: c++

   // PublisherPtr pub_gen_ents_; is declared in the header file
   pub_gen_ents_ = advertise("GlobalNetwork", "GenerateEntity");

When you want to generate an entity during the simulation, you refer to the
entity's XML ``tag`` and create a state for this entity when constructing the
``GenerateEntity`` message.

.. code-block:: c++

   // Create a state for the entity and place it at position (10, 10, 100) with
   // a roll, pitch, and yaw of (0 deg, 45 deg, 45 deg).
   State s;
   s.pos() << 10, 10, 100;
   s.quat() = scrimmage::Quaternion(0, M_PI / 4.0, M_PI / 4.0);

   // Create the GenerateEntity message
   auto msg = std::make_shared<Message<scrimmage_msgs::GenerateEntity>>();

   // Copy the new state into the message
   sc::set(msg->data.mutable_state(), s);

   // Set the entity_tag that references the entity to be generated in the
   // mission XML file.
   msg->data.set_entity_tag("gen_straight");

   // Publish the GenerateEntity message
   pub_gen_ents_->publish(msg);

Modify Entity Block Properties
------------------------------

Before publishing the message, you can modify other properties of the entity
block, such as the ``autonomy``, ``color``, ``health``, ``visual_model``, etc.,
by adding an entity block key-value pair to the ``GenerateEntity`` message:

.. code-block:: c++

   // Modify the entity's color
   auto kv_color = msg->data.add_entity_param();
   kv_color->set_key("color");
   kv_color->set_value("255, 255, 0");

   // Modify the entity's visual model
   auto kv_visual = msg->data.add_entity_param();
   kv_visual->set_key("visual_model");
   kv_visual->set_value("sphere");

   // Publish the GenerateEntity message
   pub_gen_ents_->publish(msg);

Modify Plugin Parameters
------------------------

Currently, this entity generation interface doesn't allow you to modify the XML
attributes of specific plugins. If you need to generate entities with different
plugin parameters you can either use multiple entity blocks with different
entity ``tags`` or you can create new plugin XML files that have differently
configured default parameters. For example, you could copy and rename the
``Straight.xml`` file to ``MyStraight.xml`` and then modify the plugin
parameters in ``MyStraight.xml``. In your entity, you can load the
``MyStraight`` autonomy plugin by referencing it directly as long as it is in
your ``SCRIMMAGE_PLUGIN_PATH``:

.. code-block:: xml

   <entity tag="gen_my_straight">
     <count>0</count>
     ...
     <autonomy>MyStraight</autonomy>
     ...
   </entity>

In your entity generation plugin, you can modify the ``autonomy`` tag before
publishing the ``GenerateEntity`` message:

.. code-block:: c++

   auto kv_autonomy = msg->data.add_entity_param();
   kv_autonomy->set_key("autonomy");
   kv_autonomy->set_value("MyStraight2"); // Load parameters from MyStraight2.xml
