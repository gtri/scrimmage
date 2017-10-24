.. _ros_plugin:

Create a ROS Node
=================

SCRIMMAGE allows quick prototyping for debugging and optimization. After this
process is complete users can easily wrap their working code for implementation
on in a ROS system. In this tutorial we will look at how to do this for the
``AuctionAssign`` plugin. You can run this plugin in SCRIMMAGE with the following::

  $ scrimmage missions/auction_assign.xml

  StartAuction: 1 received message from 1
   sending back bid of -7.36924
  StartAuction: 2 received message from 1
   sending back bid of -0.826997
  BidAuction: 1 received message from 1 bid: -7.36924
  BidAuction: 1 received message from 2 bid: -0.826997
  ======================================
  Auction Complete
  Max Bidder: 2 - Bid=-0.826997
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

Let's build and run the wrapped ROS node available from the `scrimmage_ros
<https://github.com/SyllogismRXS/scrimmage_ros.git>`_ ROS package. Follow the instructions
in that repo's README to setup your catkin workspace. Once it is installed
we can run the ``auctioneer`` node as follows::
  
  $ cd catkin_ws
  $ catkin_make
  $ source devel/setup.bash # or source devel/setup.zsh
  $ roslaunch scrimmage_ros auction.launch

  SUMMARY
  ========

  PARAMETERS
   * /rosdistro: kinetic
   * /rosversion: 1.12.7

  NODES
    /
      auctioneer1 (scrimmage_ros/auctioneer)
      auctioneer2 (scrimmage_ros/auctioneer)

  auto-starting new master
  process[master]: started with pid [10877]
  ROS_MASTER_URI=http://localhost:11311

  setting /run_id to 21639512-b4d6-11e7-a519-247703637410
  process[rosout-1]: started with pid [10910]
  started core service [/rosout]
  process[auctioneer1-2]: started with pid [10914]
  process[auctioneer2-3]: started with pid [10920]
  origin: 1
  Starting!
  StartAuction: 1 received message from 1
   sending back bid of 0.736862
  StartAuction: 2 received message from 1
   sending back bid of 0.55305
  BidAuction: 1 received message from 1 bid: 0.736862
  BidAuction: 1 received message from 2 bid: 0.55305
  ======================================
  Auction Complete
  Max Bidder: 1 - Bid=0.736862
  ======================================
  [auctioneer1-2] process has finished cleanly
  log file: /home/.ros/log/21639512-b4d6-11e7-a519-247703637410/auctioneer1-2*.log
  [auctioneer2-3] process has finished cleanly
  log file: /home/.ros/log/21639512-b4d6-11e7-a519-247703637410/auctioneer2-3*.log

Note that this output is very similar to when we ran it in the SCRIMMAGE
simulation. This is because it is running the exactly same code [#f1]_.  The goal of
the design is to have as much code as possible run through simulation so that
bugs are caught relatively cheaply and quickly in front of a computer rather
than in front of an actual robot where debugging can take significantly
longer. Let's now examine how to wrap the ``AuctionAssign`` plugin. The
SCRIMMAGE plugin code can be found in the following locations::

  include/scrimmage/plugins/autonomy/AuctionAssign/AuctionAssign.h
  src/plugins/autonomy/AuctionAssign/AuctionAssign.cpp

and the ROS node that wraps this plugin can be found here::

  catkin_ws/src/scrimmage_ros/src/auctioneer.cpp

The most important aspect of the SCRIMMAGE plugin files is that they have
publishers and subscribers on topics ``StartAuction`` and ``BidAuction`` that
we want to get into ROS. To do this the user must do is convert between ROS
and SCRIMMAGE messaging formats [#f2]_ . The rest is handled by the
``External`` class provided by SCRIMMAGE. We start by initializing an
``Entity``::

    ros::NodeHandle nh;

    std::map<std::string, std::string> info
        {{"x", "0"}, {"y", "0"}, {"autonomy0", "AuctionAssign"}};

    sc::External external;
    external.create_entity(100, id, info, "/tmp");

The ``info`` map holds basic initialization information that is often
contained in a mission file. This initializes the ``x/y`` position and gives
the name of the autonomy plugin [#f3]_ :: 

    auto pubs = external.entity()->autonomies().front()->pubs();

    // create a ros publisher  
    ros::Publisher pub_start_auction =
        nh.advertise<std_msgs::Int16>("StartAuction", 1000);

    // link the ros publisher to scrimmage
    external.pub_cb(sc2ros_start_auction,
        pubs["StartAuction"], pub_start_auction);

The last line provides the conversion between ROS and SCRIMMAGE with the
function ``sc2ros_start_auction``. In this case the only information in 
a ``StartAuction`` message is the sender information::

    std_msgs::Int16 sc2ros_start_auction(const sc::MessageBasePtr &sc_msg) {
        std_msgs::Int16 ros_msg;
        ros_msg.data = sc_msg->sender;
        return ros_msg;
    }

The same is done for ``BidAuction``. Now we can setup the subscribers::

    auto subs = external.entity()->autonomies().front()->subs();

    ros::Subscriber sub_start_auction = nh.subscribe("StartAuction", 1000,
        external.sub_cb<std_msgs::Int16>(ros2sc_start_auction, subs["StartAuction"]));

Here the ``External`` class is providing a ROS callback so we don't have to
write one ourselves. In ``External::sub_cb`` the template is the ROS message
type and the parameters are the ROS to SCRIMMAGE conversion function and the
SCRIMMAGE subscriber to use. The conversion function looks like this::

    sc::MessageBase ros2sc_start_auction(const std_msgs::Int16 &ros_msg) {
        sc::MessageBase sc_msg;
        sc_msg.sender = ros_msg.data;
        return sc_msg;
    }

We can then enter the main loop as with a normal ROS node::

    int ct = 0;
    while (ros::ok() && ct < runtime * loop_rate_hz) {
        external.step(ros::Time::now().toSec());
        loop_rate.sleep();
        ros::spinOnce();
        ++ct;
    }

The only new line above is the ``External::step`` call. This method performs
logging, updates any autonomies or controllers, and publishes messages on the
ROS system. Callbacks are handled as messages are received.

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
.. [#f3]
  Additional common items that could be given are:

    1. ``z``, ``vx``, ``vy``, ``vz``, ``roll``, ``pitch``, ``yaw`` for the initial
       state.
    2. ``autonomy#`` an arbitrary number of autonomies can be on an entity. In
       this case we only need one autonomy.
    3. ``controller0`` if an entity has a controller. In this case we are not
       moving so it is not necessary.
    4. ``latitude``, ``longitude`` if an entity needs to convert between
       coordinate systems a ``GeographicLib`` projection class will be
       initialized.
