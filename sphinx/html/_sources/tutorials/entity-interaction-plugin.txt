.. _entity_interaction_plugin:

Create an Entity Interaction Plugin
===================================

An entity interaction plugin is used to apply constraints and physical
interactions between entities in the simulation. For example, simple collisions
between entities are implemented through the entity interaction plugin
interface. Also, ground collisions for flying aircraft are implemented using
this interface. Given a high-fidelity motion model, the Bullet entity
interaction SCRIMMAGE plugin implements mesh-based collision detection between
entities. To create an entity interaction plugin, enter the following at the
terminal: ::

  $ cd /path/to/scrimmage/scripts
  $ ./generate-plugin.sh interaction MyInteraction ~/scrimmage/my-scrimmage-plugins

Build the entity interaction plugin: ::

  $ cd ~/scrimmage/my-scrimmage-plugins/build
  $ cmake ..
  $ make

Examine Entity Interaction Plugin
---------------------------------

The entity interaction plugin implements the ``init`` method like all
plugins. It also implements the ``step_entity_interaction`` method, which takes
a list of SCRIMMAGE entities as input. Let's take a look at the
"GroundCollision" entity interaction that ships with SCRIMMAGE. The
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
               collision_pub_->publish(msg);
           }
       }
       return true;
   }

The GroundCollision plugin also makes use of SCRIMMAGE's internal
publish-and-subscribe system. When the plugin determines that a ground
collision exists, it creates a GroundCollision message and publishes it using
the ``collision_pub_`` variable. The ``collision_pub_`` member variable was
initialized in the ``init`` method with the ``advertise`` method, which
takes a string network name and a string topic name as arguments.

.. code-block:: c++

   collision_pub_ = advertise("GlobalNetwork", "GroundCollision");
   
The GroundCollision entity interaction plugin is used in a SCRIMMAGE mission by
adding the following XML tag:

.. code-block:: xml
                
   <entity_interaction>GroundCollision</entity_interaction>
                

Multiple entity interactions can be used in the same mission file. The order in
which the entity interaction plugins are executed is based on their relative
positions from the top of the XML file. Plugins that are listed first are
executed first. It is common to use both the GroundCollision and
SimpleCollision (collision between entities) in the same simulation. This would
be accomplished with the following XML lines:

.. code-block:: xml

   <entity_interaction>SimpleCollision</entity_interaction>
   <entity_interaction>GroundCollision</entity_interaction>

Since the entity interaction plugin isn't applied to a single entity, the
``entity_interaction`` tag should exist outside of the ``entity`` block.
