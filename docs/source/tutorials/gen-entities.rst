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

Plugin Parameter Overrides
--------------------------

When one entity spawns others at runtime, each spawned entity can receive
different plugin parameters. Without this, spawning many entities with slight
parameter variations would require duplicating entity blocks in the mission XML
(resulting in huge files) or writing complex nested XML by hand.

A spawner entity (e.g., a carrier, hive, or RL controller) publishes
``GenerateEntity`` messages referencing a template. Each message can include
parameter overrides, so one template supports thousands of parameter variations.

For example, a carrier entity might spawn drones with varying speeds:

.. code-block:: xml

   <!-- The spawner entity (runs your spawning logic) -->
   <entity>
     <count>1</count>
     <autonomy>MySpawnerPlugin</autonomy>
     ...
   </entity>

   <!-- Template for spawned entities (not created at start) -->
   <entity tag="drone">
     <count>0</count>
     <autonomy speed="20">Straight</autonomy>
     ...
   </entity>

The spawner plugin publishes messages that override parameters per-spawn:

.. code-block:: c++

   auto msg = std::make_shared<Message<scrimmage_msgs::GenerateEntity>>();
   msg->data.set_entity_tag("drone");  // Reference the template

   // Override speed for the Straight autonomy plugin
   auto auto_override = msg->data.add_plugin_override();
   auto_override->set_plugin_type("autonomy");
   auto_override->set_plugin_name("Straight");

   auto speed = auto_override->add_params();
   speed->set_key("speed");
   speed->set_value("30");  // Each spawn can use a different value

   pub_gen_ents_->publish(msg);

Migrating from Ubuntu-24.04
~~~~~~~~~~~~~~~~~~~~~~~~~~~

If your external project used the older ``plugin_param`` API from
``Ubuntu-24.04``:

.. code-block:: c++

   auto old_param = msg->data.add_plugin_param();
   old_param->set_plugin_type("autonomy0");
   old_param->set_tag_name("start");
   old_param->set_tag_value("zone_a");

the equivalent runtime override now looks like this:

.. code-block:: c++

   auto auto_override = msg->data.add_plugin_override();
   auto_override->set_plugin_type("autonomy");
   auto_override->set_plugin_index(0);  // "autonomy0" in the old API

   auto start = auto_override->add_params();
   start->set_key("start");
   start->set_value("zone_a");

Key differences:

1. ``plugin_type`` is now only the plugin category: ``autonomy``,
   ``controller``, ``motion_model``, or ``sensor``.
2. The old numeric suffix (for example ``autonomy0``) moved to
   ``plugin_index``.
3. The old ``tag_name`` / ``tag_value`` pair became ``params`` entries with
   ``key`` / ``value``.

If you previously sent multiple overrides such as ``start`` and ``end`` for
the same autonomy plugin, you can either put them in one
``plugin_override`` block or append multiple blocks that target the same
plugin. SCRIMMAGE merges them in message order, and later values win if the
same key appears more than once.

.. code-block:: c++

   auto auto_override = msg->data.add_plugin_override();
   auto_override->set_plugin_type("autonomy");
   auto_override->set_plugin_index(0);

   auto start = auto_override->add_params();
   start->set_key("start");
   start->set_value("zone_a");

   auto end = auto_override->add_params();
   end->set_key("end");
   end->set_value("zone_b");

Both styles below are valid. One block is shorter; multiple blocks can be more
convenient when building messages across nested conditionals.

Alternatively, use ``plugin_index`` to target by position (0 = first, 1 = second, etc.):

.. code-block:: c++

   auto_override->set_plugin_type("autonomy");
   auto_override->set_plugin_index(0);

This works from any plugin type — autonomy, sensor, controller, etc. External
projects just publish the message; all validation happens in base SCRIMMAGE.

The ``entity_tag`` still selects which entity template from the mission file is
spawned. That part did not change. In the example above, ``start`` and ``end``
are plugin parameter keys for the selected template's autonomy plugin; they are
not alternative entity template tags.

Validation
~~~~~~~~~~

By default, override keys must already exist in the plugin XML configuration or
as inline mission attributes for that plugin instance. Unknown keys are rejected
to catch typos early.

For example, if your autonomy expects runtime keys such as ``start`` and
``end``, either declare them in the plugin's XML config, add placeholder values
inline in the mission template, or disable strict checking as shown below.

For loose runtime overrides (e.g., prototyping), add to the mission:

.. code-block:: xml

   <strict_runtime_plugin_params>false</strict_runtime_plugin_params>

.. warning::

   Attempting to override plugin names (``autonomy``, ``controller``,
   ``motion_model``, ``sensor``) via ``entity_param`` will be ignored.
   Plugins are determined by the entity template, not at runtime.

.. note::

   Use ``plugin_name`` to target plugins by name — clearer and refactor-safe.
   Use ``plugin_index`` when multiple plugins share the same name, or when
   you specifically need positional addressing. Index uses declaration order:
   the first ``sensor`` is index ``0``, the second is index ``1``, etc.

.. note::

   SCRIMMAGE runs **all** plugins of each type every timestep — all autonomies,
   all controllers, and all sensors execute in declaration order. Only
   ``motion_model`` is singular (one per entity, always index ``0``).

.. note::

   For a higher-volume example, see
   ``missions/test/test_generate_entity_runtime_override_stress.xml``.
   That mission spawns 120 entities from one template, each with distinct
   runtime parameter overrides.
