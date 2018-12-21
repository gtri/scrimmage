.. _ros_plugin:

Create a ROS Node
=================

SCRIMMAGE allows quick prototyping for debugging and optimization. After this
process is complete users can easily wrap their working code for implementation
on in a ROS system. In this tutorial we will look at how to do this for the
``AuctionAssign`` plugin. You can run this plugin in SCRIMMAGE with the following::

  $ scrimmage missions/auction_assign.xml

  Agent (1) starting auction
  -----------------------------
  Time: 0.1
  StartAuction: entity ID (1) received message from entity ID: 1
  Sending bid of 4.02381
  -----------------------------
  Time: 0.1
  StartAuction: entity ID (2) received message from entity ID: 1
  Sending bid of 5.24396
  -----------------------------
  Time: 0.2
  BidAuction: entity ID (1) received message from entity ID (1),  bid: 4.02381
  -----------------------------
  Time: 0.2
  BidAuction: entity ID (1) received message from entity ID (2),  bid: 5.24396
  -----------------------------
  Time: 0.2
  BidAuction: entity ID (2) received message from entity ID (1),  bid: 4.02381
  -----------------------------
  Time: 0.2
  BidAuction: entity ID (2) received message from entity ID (2),  bid: 5.24396
  ======================================
  Auction Complete
  Max Bidder: 2
  Bid: 5.24396
  ======================================

In this scenario there are two entities. The entity with id 1 starts an
auction on topic ``StartAuction`` and both entities bid on ``BidAuction``. We
would like to place this into a ROS system with minimal effort. First, note
that on embedded systems we may want only a minimal installation of SCRIMMAGE
and its dependencies (e.g., no gui or simulation loop is needed) so one could
have done::

  # we don't need to do this step for the tutorial
  # only run this if you are space constrained on an embedded system
  $ sudo ./setup/install-binaries.sh -e 0
  $ cmake .. -DEXTERNAL=1

Let's build and run the wrapped ROS node available from the scrimmage_ros_ ROS
package. Follow the instructions in that repo's README to setup your catkin
workspace. Once it is installed we can run the ``auctioneer`` node as follows::

  $ cd catkin_ws
  $ catkin_make
  $ source devel/setup.bash # or source devel/setup.zsh
  $ roslaunch scrimmage_ros auction.launch

  SUMMARY
  ========

  PARAMETERS
   * /entity_1/entity_id: 1
   * /entity_1/entity_name: agent
   * /entity_1/max_contacts: 100
   * /entity_1/mission_file: ~/scrimmage/scrim...
   * /entity_2/entity_id: 2
   * /entity_2/entity_name: agent
   * /entity_2/max_contacts: 100
   * /entity_2/mission_file: ~/scrimmage/scrim...
   * /rosdistro: kinetic
   * /rosversion: 1.12.12

  NODES
    /
      entity_1 (scrimmage_ros/auctioneer)
      entity_2 (scrimmage_ros/auctioneer)

  auto-starting new master
  process[master]: started with pid [31656]
  ROS_MASTER_URI=http://localhost:11311

  setting /run_id to d001a062-2233-11e8-9392-484d7ed51daf
  process[rosout-1]: started with pid [31669]
  started core service [/rosout]
  process[entity_1-2]: started with pid [31677]
  process[entity_2-3]: started with pid [31687]
  Agent (1) starting auction
  -----------------------------
  Time: 1.52045e+09
  StartAuction: entity ID (1) received message from entity ID: 1
  Sending bid of -6.84253
  -----------------------------
  Time: 1.52045e+09
  StartAuction: entity ID (2) received message from entity ID: 1
  Sending bid of -3.40187
  -----------------------------
  Time: 1.52045e+09
  BidAuction: entity ID (2) received message from entity ID (1),  bid: -6.84253
  -----------------------------
  Time: 1.52045e+09
  BidAuction: entity ID (1) received message from entity ID (1),  bid: -6.84253
  -----------------------------
  Time: 1.52045e+09
  BidAuction: entity ID (1) received message from entity ID (2),  bid: -3.40187
  -----------------------------
  Time: 1.52045e+09
  BidAuction: entity ID (2) received message from entity ID (2),  bid: -3.40187
  ======================================
  Auction Complete
  Max Bidder: 2
  Bid: -3.40187
  ======================================
  [entity_1-2] process has finished cleanly
  log file: /home/syllogismrxs/.ros/log/d001a062-2233-11e8-9392-484d7ed51daf/entity_1-2*.log
  [entity_2-3] process has finished cleanly
  log file: /home/syllogismrxs/.ros/log/d001a062-2233-11e8-9392-484d7ed51daf/entity_2-3*.log

Note that this output is very similar to when we ran it in the SCRIMMAGE
simulation. This is because it is running the exactly same code [#f1]_.  The goal of
the design is to have as much code as possible run through simulation so that
bugs are caught relatively cheaply and quickly in front of a computer rather
than in front of an actual robot where debugging can take significantly
longer. Let's now examine how to wrap the ``AuctionAssign`` plugin. The
SCRIMMAGE plugin code can be found in the following locations::

  include/scrimmage/plugins/autonomy/AuctionAssign/AuctionAssign.h
  src/plugins/autonomy/AuctionAssign/AuctionAssign.cpp

and the ROS node that wraps this plugin can be found online (`auctioneer.cpp
<https://github.com/SyllogismRXS/scrimmage_ros/blob/master/src/auctioneer.cpp/>`_).

The most important aspect of the SCRIMMAGE plugin files is that they have
publishers and subscribers on topics ``StartAuction`` and ``BidAuction`` that
we want to get into ROS. To do this the user must do is convert between ROS
and SCRIMMAGE messaging formats [#f2]_ . The rest is handled by the
``External`` class provided by SCRIMMAGE. We start by initializing a ROS node as usual::

    // Initialize the ROS node
    ros::init(argc, argv, "auctioneer");
    ros::NodeHandle nh;

    // Get a private node handle to parse ros params
    ros::NodeHandle private_nh("~");

Next, we need to get the mission file that contains our entity's
description. We also need to specify the entity ID for the ROS environment, the
entity name, and the maximum number of contacts. We use ROS' parameter server
API to load the parameters from `auction.launch
<https://github.com/SyllogismRXS/scrimmage_ros/blob/master/launch/auction.launch/>`_.::

  // Get the mission file, which holds overrides for plugin parameters
  std::string mission_file;
  private_nh.param("mission_file", mission_file, std::string(""));

  // Get the entity ID
  int entity_id;
  private_nh.param("entity_id", entity_id, 1);

  // Get the entity name. This name should match the <name> tag in the
  // mission file for the entity block we want to use.
  std::string entity_name;
  private_nh.param("entity_name", entity_name, std::string("UNDEFINED"));

  // Specify the maximum number of contacts that we will encounter.
  int max_contacts;
  private_nh.param("max_contacts", max_contacts, 100);

The entity name is used to specify which entity block in the SCRIMMAGE mission
file should be used to initialize the entity, since the mission file could
define multiple entity blocks. In order to use an entity block from the mission
file, the entity block needs to define the ``<name>`` tag. In our
`auction_assign.xml
<https://github.com/gtri/scrimmage/blob/master/missions/auction_assign.xml>`_
mission file, we set the name to "agent" (e.g. ``<name>agent</name>``).

We can now create the entity::

  sc::External external;
  external.create_entity(mission_file, max_contacts, entity_id, entity_name);

Now that our entity has been initalized, we have to setup the publishers and
subscribers that shuttle data between the ROS and SCRIMMAGE environments. The
SCRIMMAGE auction plugins publish and subscribe to the "StartAuction" topic and
they publish and subscribe on the "BidAuction" topic. Thus, we need to setup
ROS publishers and subscribers for both of these topics. Let's setup a ROS
publisher for the "StartAuction" topic.::

  using scrimmage_ros::RosBidAuction;
  using scrimmage_ros::RosStartAuction;
  
  ros::Publisher pub_start_auction =
      nh.advertise<RosStartAuction>("StartAuction", 1000);
  
Now, we need to link the SCRIMMAGE publisher on the "StartAuction" topic to
this ROS publisher::

  external.pub_cb<auction::StartAuction>("SphereNetwork", "StartAuction",
                                         sc2ros_start_auction, pub_start_auction);

The ``auction::StartAuction`` type is a SCRIMMAGE message type. We want to use
the SCRIMMAGE publisher that is on the "SphereNetwork" and using topic
"StartAuction". Note, that we passed the ROS publisher (``pub_start_auction``)
to the ``pub_cb`` method. The ``sc2ros_start_auction`` variable is a function
that converts a SCRIMMAGE message type to a ROS message type. We have to define
this function before the ``pub_cb`` line:::

  RosStartAuction sc2ros_start_auction(const auction::StartAuction &sc_msg) {
      RosStartAuction ros_msg;
      ros_msg.sender_id = sc_msg.sender_id();
      return ros_msg;
  }

A similar publisher is setup for the "BidAuction" topic. Let's setup a
subscriber for the "StartAuction" topic. First, we need to define the function
that will convert from a ROS message type to a SCRIMMAGE message type:::

  auction::StartAuction ros2sc_start_auction(const RosStartAuction &ros_msg) {
      auction::StartAuction sc_msg;
      sc_msg.set_sender_id(ros_msg.sender_id);
      return sc_msg;
  }

  
Now, let's setup a ROS subscriber:::
    
  ros::Subscriber sub_start_auction = nh.subscribe("StartAuction", 1000,
        external.sub_cb<RosStartAuction>("SphereNetwork", "StartAuction",
        ros2sc_start_auction));

Here the ``External`` class is providing a ROS callback so we don't have to
write one ourselves. In ``External::sub_cb`` the template is the ROS message
type and the parameters are the SCRIMMAGE network name ("SphereNetwork"), the
topic name ("StartAuction"), and the ROS to SCRIMMAGE conversion function.

We can then enter the main loop as with a normal ROS node::

    while (ros::ok()) {
        external.step(ros::Time::now().toSec());
        loop_rate.sleep();
        ros::spinOnce();        
    }

The only new line above is the ``External::step`` call. This method performs
logging, updates any autonomies or controllers, and publishes messages on the
ROS system. Callbacks are handled as messages are received.

.. _scrimmage_ros: https://github.com/SyllogismRXS/scrimmage_ros 

Advanced Usage
--------------

There are a few additional cases that the ``External`` class can handle that
are not addressed in this tutorial.

  1. ``update_contacts`` - Often entities need to know that state of their
     neighbors. If this is the case, ``update_contacts`` is a callback
     called in the ``External::step`` that will set the
     ``External::entity()->contacts()`` variable.

  2. Services - SCRIMMAGE allows multiple autonomies to exist on a single
     entity similar to decoupled ROS nodes. In particular, these autonomies
     can provide services to each other and ``External`` provides wrapping
     similar to ``sub_cb`` and ``pub_cb`` for wrapping these service calls.

.. [#f1]
   the random seed is different in the ROS simulation which causes the bids to be different.
.. [#f2]
   A user can use ROS messages within SCRIMMAGE. In this case the conversions are trivial.
   
