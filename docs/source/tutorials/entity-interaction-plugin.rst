.. _entity_interaction_plugin:

Create an Entity Interaction Plugin
===================================

An Entity Interaction plugin is used to apply constraints and physical
interactions between entities in the simulation. For example, simple collisions
between entities are implemented through the Entity Interaction plugin
interface. Also, ground collisions for flying aircraft are implemented using
this interface. Given a high-fidelity motion model, the Bullet entity
interaction SCRIMMAGE plugin implements mesh-based collision detection between
entities. To create an Entity Interaction plugin, enter the following at the
terminal: ::

  $ cd /path/to/scrimmage/scripts
  $ ./generate-plugin.sh interaction MyInteraction ~/scrimmage/my-scrimmage-plugins

Build the Entity Interaction plugin: ::

  $ cd ~/scrimmage/my-scrimmage-plugins/build
  $ cmake ..
  $ make

Examine Entity Interaction Plugin
---------------------------------

The Entity Interaction plugin implements the ``init`` method like all
plugins. It also implements the ``step_entity_interaction`` method, which takes
a list of SCRIMMAGE entities as input. Let's take a look at the
"GroundCollision" Entity Interaction that ships with SCRIMMAGE. The
GroundCollision plugin simply determines whether an entity is below a certain
z-position threshold and calls the entity's ``collision`` method if it is below
that threshold.

.. code-block:: c++
   :linenos:

   bool GroundCollision::step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                              double t, double dt)
   {
       // Account for entities colliding with
       for (sc::EntityPtr ent : ents) {
           if (ent->state()->pos()(2) < ground_collision_z_) {
               ent->collision();
   
               auto msg = std::make_shared<sc::Message<sm::GroundCollision>>();
               msg->data.set_entity_id(ent->id().id());
               publish_immediate(t, collision_pub_, msg);
           }
       }
       return true;
   }

The GroundCollision plugin also makes use of SCRIMMAGE's internal
publish-and-subscribe system. When the plugin determines that a ground
collision exists, it creates a GroundCollision message and publishes it using
the ``publish_immediate`` method. The ``collision_pub_`` member variable was
initialized in the ``init`` method with the ``create_subscriber`` method, which
takes a string topic name as input.

.. code-block:: c++

   collision_pub_ = create_publisher("GroundCollision");             
   
As a rule-of-thumb, entity interaction and metrics plugins should use
``publish_immediate`` and autonomy plugins should use ``publish``. This will be
described further in future publish-and-subscribe documentation.

The GroundCollision Entity Interaction plugin is used in a SCRIMMAGE mission by
adding the following XML tag:

.. code-block:: xml
                
   <entity_interaction order="0">GroundCollision</entity_interaction>
                

Multiple Entity Interactions can be used in the same mission file. By setting
the ``order`` attribute, the order in which the plugins are executed is
defined. For example, it is common to use both the GroundCollision and
SimpleCollision (collision between entities) in the same simulation. This would
be accomplished with the following XML lines:

.. code-block:: xml

   <entity_interaction order="0">SimpleCollision</entity_interaction>
   <entity_interaction order="1">GroundCollision</entity_interaction>

Since the Entity Interaction plugin isn't applied to a single entity, the
``entity_interaction`` tag should exist outside of the ``entity`` block.
