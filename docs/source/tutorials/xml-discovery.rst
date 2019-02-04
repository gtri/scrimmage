SCRIMMAGE Plugin / XML File Discovery
=====================================

This tutorial assumes you have completed the section
:ref:`install_scrimmage`.  Let's start with the command we ran to run the
``straight.xml`` mission::

    $ cd ~/scrimmage/scrimmage
    $ scrimmage ./missions/straight.xml

When executing this line, scrimmage first reads in the mission file
``straight.xml``. This file allows for quite a few customizations such as how
to visualize terrain and entities (aircraft in this case), how long we want the
simulation to run, where it takes place, and quite a few other items. For our
purposes though, the ``entity`` block is the one of interest. In the scenario,
we have 2 teams each with fixed-wing aircraft. Thus, we have two entity blocks.
The team is set with the ``team_id`` block and the entity type is set with the
``autonomy`` block. [#f1]_ When ``scrimmage`` sees the ``autonomy`` block
pointing to ``Straight`` it does the following:

1. Looks for the file ``Straight.xml`` in your ``SCRIMMAGE_PLUGIN_PATH`` and
   loads it.
2. Looks for the ``params`` xml tag in ``Straight.xml``.
3. It loads the custom autonomy specified by the ``library`` tag (we will
   discuss how to create these libraries below)
4. It saves all other items in the ``params`` block and passes these variables
   to the ``init`` method of the ``library`` as a map of strings pointing to
   strings (we will discuss this below as well).

Later tutorials will discuss how to implement custom autonomy plugins.


Finding Plugins
---------------

A command line utility called ``scrimmage-plugin`` can be used to find a plugin
and print its properties. For example, to find the ``Straight`` autonomy
plugin, type the following:

.. code-block:: bash

   $ scrimmage-plugin Straight

This utility is useful for debugging file path problems.


.. [#f1] We actually fully specify an entity with the ``motion_model``,
        ``visual_model``, ``controller``, and ``autonomy``. The first item
        concerns how a vehicle moves. In the case of a fixed-wing vehicle it
        can only move forward, roll, and pitch so we specify ``SimpleAircraft``
        as the motion model. Quadrotors have more degrees of freedom so they
        have the ``SimpleQuadrotor`` motion model. As the names imply, there
        are higher fidelity motion models available.
