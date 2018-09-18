
# GraphInteraction

## Functionality

This *entity interaction* provides the ability to parse and visualize a graph, e.g. street data from [OpenStreetMap](https://www.openstreetmap.org). The graph data is published on a topic, ready to be used for path planning.

## How to Load OpenStreetMap Data
First, use OpenStreetMap to define the map you wish to use. In the browser, draw the desired bounding box and click Export to save the map to a .osm file (use a mirror if needed). Write down the bounds of the box to be used by the underlaying image if desired. The map image can be saved by clicking Share on the right menu. 

The .osm file is a large xml file that we need to convert to nodes and edges. Use [this tool](https://github.com/AndGem/OsmToRoadGraph) to convert the osm file to an easily parsable format. The following settings create a graph of the driveable streets and contracts any nodes of degree 2. This command will then create a .pyXgr file containing a list of the nodes and edges, and a .pyXgr_names file containing each edges street name if available.
```
python run.py -f <map_name>.osm -n c -c
```
In your mission file, set the origin to the latitude and longitude of the center of the map. Point GraphInteraction in your mission file to the generated .ptXgr files. The labels file is optional.

GraphInteraction will then visualize the graph and publish a Graph message that can easily be parsed into an object.
```
#include <scrimmage/msgs/Graph.pb.h>
namespace sm = scrimmage_msgs;
auto callback = [&](scrimmage::MessagePtr<sm::Graph> msg) {
	scrimmage::Graph test_graph(msg);
};
subscribe<sm::Graph>("GlobalNetwork", "Graph", callback);
```
