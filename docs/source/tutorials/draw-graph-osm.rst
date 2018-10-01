.. _draw_osm:

Draw Graphs from OpenStreetMap Data
===================================

The GraphInteraction
:ref:`entity interaction plugin<entity_interaction_plugin>` provides the
ability to parse and visualize a graph, particularly street data from
`OpenStreetMap <https://www.openstreetmap.org>`_. The graph
data is published on a topic, ready to be used for path planning.


Acquire and Pre-Process OpenStreetMap Data
------------------------------------------

To acquire map data to turn into a graph, go to the `OpenStreetMap Export page <https://www.openstreetmap.org/export>`_,
draw a bounding box around your desired area, and click the
Export button (or a mirror link, if needed) to save the map to an .osm file.
Make a note of the latitude and longitude of the center of this bounding box to
use in the ``<latitude_origin>`` and ``<longitude_origin>`` of your mission file later.

.. _osm-data-download-details:

.. Tip::
   The latitude and longitude values of the bounding box for the map you downloaded
   are in the ``<bounds>`` tag in the .osm file.

To provide SCRIMMAGE the necessary information to plot the nodes and edges of the graph,
use `this tool <https://github.com/AndGem/OsmToRoadGraph>`_ to convert the osm
file to an easily parsable format.
For the purposes of this tutorial, we will use the *car* network type; additional network
types and other options are explained `here <https://github.com/AndGem/OsmToRoadGraph#usage>`_.
The following command creates a graph of the driveable streets in ``mymap.osm``, as well as a
smaller graph (the *contracted graph*) that contracts nodes of degree 2.::

   $ python run.py -f mymap.osm -n c -c

The output of this command will consist of four files:
1. ``mymap.pycgr`` (contains nodes and edges of map)
2. ``mymap.pycgr_names`` (contains street names of edges)
3. ``mymap.pycgrc`` (contracted graph's nodes and edges)
4. ``mymap.pycgrc_names`` (empty file with as many lines as edges)

Both the ``.pycgr`` or ``.pycgrc`` files contain enough information
for GraphInteraction to draw a simple graph.
Using ``.pycgr`` results in a more detailed graph than ``.pycgrc``, but 
``.pycgrc`` files are smaller than their corresponding ``.pycgr`` files.
The ``.pycgr*_names`` files are optional, and in the case of
``.pycgrc_names``, contain no useful information.

Graph File Location Structure
-----------------------------

For the rest of this tutorial, we will assume the use of ``mymap.pycgr``
and ``mymap.pycgr_names``, but the files defining the contracted graph,
``mymap.pycgrc`` and ``mymap.pycgrc_names``, could be used instead if desired.

Place the selected ``.pycgr*`` files in a directory in your
SCRIMMAGE project's data directory, e.g. ``path/to/scrimmage-project/data/maps/mymap``
Ensure that the path to the data directory is stored in the ``DATA_PATH`` variable
in your project's CMakeLists.txt file so that it can be found by FileSearch
in GraphInteraction.

Suppose we have the files ``mymap.pycgr`` and ``mymap.pycgr_names``
in ``path/to/scrimmage-project/data/maps/mymap``.
Next, we create an XML file, ``path/to/scrimmage-project/data/maps/mymap/mymap.xml``,
which tells SCRIMMAGE which files to use to draw the ``mymap`` graph:

.. code-block:: xml
   :linenos:

   <?xml version="1.0"?>
   <?xml-stylesheet type="text/xsl" href="http://gtri.gatech.edu"?>
   <params>
     <graph_file>mymap.pycgr</graph_file>
     <labels_file>mymap.pycgr_names</labels_file>
   </params>


Now, to use the mymap graph in a mission, include GraphInteraction in
your mission file with the data parameter set as follows: 

.. code-block:: xml

   <entity_interaction data="mymap.xml">GraphInteraction</entity_interaction>

The `parse_autonomy_data <https://github.com/gtri/scrimmage/blob/master/src/parse/ParseUtils.cpp>`_ function
allows GraphInteraction to pull the locations of the ``.pycgr`` and ``.pycgr_names`` files from ``mymap.xml``.
This allows us to change which location's map is drawn by changing only the name of the XML data file
and the latitude and longitude origin points.

.. Note::
   If the XML file is not found when you run the mission, double-check that the data
   directory has been added to the ``DATA_PATH`` in the main CMakeLists.txt file of the project.

.. Note::
   Remember to change the <latitude_origin> and <longitude_origin> tags in your mission
   file to reflect the location of your graph! Tips for finding this information from
   the ``.osm`` file :ref:`here<osm-data-download-details>`.

When the mission is run, GraphInteraction will visualize the graph and publish
a Graph message that can easily be parsed into an object.

.. code-block:: c++
   :linenos:

   #include <scrimmage/msgs/Graph.pb.h>
   namespace sm = scrimmage_msgs;
   auto callback = [&](scrimmage::MessagePtr<sm::Graph> msg) {
       scrimmage::Graph test_graph(msg);
   };
   subscribe<sm::Graph>("GlobalNetwork", "Graph", callback);

