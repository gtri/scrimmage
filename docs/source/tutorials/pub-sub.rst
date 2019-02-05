.. _pub_sub:

Publishers and Subscribers
==========================

SCRIMMAGE provides a light-weight publish-and-subscribe system that allows for
arbitrary data to be transmitted between plugins. In order to setup a publisher
or a subscriber, the developer needs to identify the data type of the message,
the topic name (std::string), and the network name (std::string).

Network Plugins
---------------

The network name refers to Network plugin that must be specified in the mission
xml file.

- **GlobalNetwork** : By default, the GlobalNetwork is initialized in all
  SCRIMMAGE plugins because it is used by SimControl to publish simulation
  event information. Use the GlobalNetwork to transfer simulation-level data.

- **LocalNetwork** : The LocalNetwork only allows transmission of messages
  between plugins that are attached to the same entity. Use this network to
  transmit messages that shouldn't leave the current physical entity. For
  example, use the LocalNetwork to publish accelerometer data, own-vehicle pose
  data, and other state information that you don't want to conflict with
  another entity's data.


- **SphereNetwork** : The SphereNetwork is a probabilistic transmission model
  that is parameterized by communication range. (perfect comms within range or
  probabilistic comms within a range).

Multiple network tags can be specified in a single mission file. For example,
you may want to use the GlobalNetwork, LocalNetwork, and SphereNetwork in the
same mission.

.. code-block:: xml

   <network>GlobalNetwork</network>
   <network>LocalNetwork</network>
   <network>SphereNetwork</network>

Create a Publisher
------------------

Creating a SCRIMMAGE publisher is easy. First, declare a publisher in your
plugin's header file:

.. code-block:: c++

   sc::PublisherPtr pub_boundary_;

Now, intialize the publisher in the init() method of the plugin's \*.cpp
file. In this case, we are creating a publisher in the "GlobalNetwork" with a
topic name of "Boundary".

.. code-block:: c++

   pub_boundary_ = advertise("GlobalNetwork", "Boundary");

We can publish a message by creating a scrimmage::Message (templated with the
data type that we want to publish), filling in any data that is required, and
calling the ``publish`` method.

.. code-block:: c++

   auto msg = std::make_shared<sc::Message<BoundaryInfo>>();
   msg->data.type = BoundaryInfo::Type::Cuboid;
   pub_boundary_->publish(msg);

In the previous example, ``msg->data`` provides direct access to an instance of
the BoundaryInfo class. See the `Boundary
<https://github.com/gtri/scrimmage/blob/master/src/plugins/interaction/Boundary/Boundary.cpp/>`_
entity interaction plugin for the complete example.

If you wanted to limit the number of messages in the publisher output queue,
you can call the advertise method with a maximum queue size argument.

.. code-block:: c++

   pub_boundary_ = advertise("GlobalNetwork", "Boundary", 10);

.. note::

   You may have to include additional header files in your plugins that are
   not automatically included in the plugin templates. For example you may
   need::

     #include <scrimmage/pubsub/Message.h>
     #include <scrimmage/pubsub/Publisher.h>
     #include <scrimmage/pubsub/Subscriber.h>

Create a Subscriber
-------------------

We need to setup a callback function that will be called when a SCRIMMAGE
plugin receives a new message. This callback can be a class method, a static
function, or a lambda function. In this example, we will use a lambda function
due to its simplicity. Define the lambda callback function in the init() method
of your plugin:

.. code-block:: c++

   auto callback = [&] (scrimmage::MessagePtr<sci::BoundaryInfo> msg) {
       cout << "Time: " << time_->t() << endl;
       cout << "Received a boundary info message!" << endl;
   };

This lambda function consumes a scrimmage::MessagePtr templated with the
BoundaryInfo class. This lambda function has access to the plugin's member
variables because its closure was declared with ``[&]``. Thus, the lambda
function can access plugin member variables, such as the ``time_`` member
variable. Finally, you need to initialize the subscriber:

.. code-block:: c++

   subscribe<sci::BoundaryInfo>("GlobalNetwork", "Boundary", callback);

The subscriber is templated with the message data type. This subscriber will
listen to the "Boundary" topic on the "GlobalNetwork" network. The name of the
lambda callback function was also passed to the subscribe method. If you wanted
to limit the number of messages in the receive queue, you can call the
subscribe method with a maximum queue size argument.

.. code-block:: c++

   subscribe<sci::BoundaryInfo>("GlobalNetwork", "Boundary", callback, 10);

In this case, only the 10 most recent messages will remain in the queue if more
than 10 messages have been received since the autonomy last ran.

See the `Straight
<https://github.com/gtri/scrimmage/blob/master/src/plugins/autonomy/Straight/Straight.cpp/>`_
autonomy plugin for the complete example.

.. note::

   The subscriber callbacks are not called immediately when a message is
   published to the subscriber's topic. Instead, the published messages are
   filtered through the Network plugins and possibly delivered to the
   subscribers' message input queues during the next simulation time
   step. Before each plugins' ``step()`` method is called, the messages that
   were successfully delivered to the subscribers' input queues are delivered
   to their respective callback functions. This allows for complete determinism
   in the publishing and receiving of messages.

Subscribing to Another Entity's LocalNetwork
--------------------------------------------

The ``LocalNetwork`` is often used to keep sensor data and internal state
information isolated between entities. This can be an issue if a metrics or
entity interaction plugin wants to subscribe to a topic in the ``LocalNetwork``
of an entity. This can be accomplished in two different ways:

1. Create an autonomy plugin that subscribes to the ``LocalNetwork`` topic and
   re-publishes the data on a ``GlobalNetwork`` topic. In this case, you
   probably want to augment the ``GlobalNetwork`` topic name with the ID of the
   entity (e.g., "/3/AltitudeAboveTerrain").

2. Programmatically generate a plugin/subscriber in the metrics or entity
   interaction plugin and attach it to the entity that you wish to monitor. The
   code-block below provides an example for achieving this effect.

In the following code-block, an "empty" autonomy plugin is created during the
first iteration of ``step_entity_interaction()`` and the autonomy plugin is
added to the entity's autonomies list. The empty autonomy plugin is used to
create a subscriber on the entity's ``LocalNetwork``. When the callback
function is called, the entity's ID is used as the key of a map to store the
received data.

.. code-block:: c++
   :linenos:

   bool MyInteractionPlugin::step_entity_interaction(std::list<EntityPtr> &ents,
                                                     double t, double dt) {
    if (!sub_setup_) {
        // Only setup the subscribers one time.
        sub_setup_ = true;

        // For all entities in the simulation
        for (auto &kv : *id_to_ent_map_) {
            // Construct an empty plugin that we will use to create a
            // subscriber to a specific entity's LocalNetwork.
            AutonomyPtr plugin = std::make_shared<Autonomy>();

            // In order for plugins to communicate on the LocalNetwork they
            // need the same parent and pubsub pointers.
            plugin->set_parent(kv.second);
            plugin->set_pubsub(plugin->parent()->pubsub());

            int id = kv.first; // The ID is used in the lambda callback

            // Create the subscriber in the LocalNetwork
            auto state_cb = [&, id](auto &msg) {
                // Save the most recent published data in a map that can be
                // used later. The entity ID is the map's key.
                states_with_covs_[id] = msg->data;
            };
            plugin->subscribe<StateWithCovariance>("LocalNetwork", "StateWithCovariance", state_cb);

            // Add the empty plugin that we created to the entity's autonomies
            // list, so that the simulation controller calls its callbacks when
            // messages are received.
            kv.second->autonomies().push_back(plugin);
        }
    }

In the plugin's header file, the following member variables were declared:

.. code-block:: c++
   :linenos:

   bool sub_setup_ = false;
   std::unordered_map<int, StateWithCovariance> states_with_covs_;
