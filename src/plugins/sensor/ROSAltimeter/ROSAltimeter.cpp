/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Natalie Rakoski <natalie.rakoski@gtri.gatech.edu>
 * @date 27 January 2020
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/plugins/sensor/ROSAltimeter/ROSAltimeter.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/proto/State.pb.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/math/Angles.h>

#define meters2feet 3.28084
#define feet2meters (1.0 / meters2feet)

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Sensor, scrimmage::sensor::ROSAltimeter, ROSAltimeter_plugin)

namespace scrimmage {
namespace sensor {

ROSAltimeter::ROSAltimeter() {}

void ROSAltimeter::init(std::map<std::string, std::string> &params) {

    if (!ros::isInitialized()) {
        int argc = 0;
        // scrimmage handles it's own SIGINT/SIGTERM shutdown in main.cpp
        ros::init(argc, NULL, "scrimmage", ros::init_options::NoSigintHandler);
    }
    nh_ = std::make_shared<ros::NodeHandle>();

    // Setup robot namespace
    ros_namespace_ = sc::get<std::string>("ros_namespace_prefix", params, "robot");
    ros_namespace_ += std::to_string(parent_->id().id());

    // Create Publisher
    altimeter_pub_ = nh_->advertise<mavros_msgs::Altitude>(ros_namespace_ + "/altimeter", 1);

    // Scrimmage is in East North Up (ENU)
    // For you to get the ROS data in North East Down (NED): switch x and y, and negate z outside scrimmage.
    // For more information on the MavROS message format view this page:
    // https://mavlink.io/en/messages/common.html#ALTITUDE

    // Monotonic Altitude should only be set on initialization and not changed.
    // this altitude value = above sea level (meters)
    ////// MavROS website on Monotonic: //////
    // This altitude measure is initialized on system boot and monotonic (it is never reset, but represents
    // the local altitude change). The only guarantee on this field is that it will never be reset and is
    // consistent within a flight. The recommended value for this field is the uncorrected barometric altitude
    // at boot time. This altitude will also drift and vary between flights.
    sc::StatePtr &state = parent_->state_truth();
    double lat_init, lon_init, alt_init;
    parent_->projection()->Reverse(state->pos()(0), state->pos()(1), state->pos()(2), lat_init, lon_init, alt_init);
    monotonic_ = static_cast<float>(alt_init);
}

bool ROSAltimeter::step() {
    // Obtain current state information
    sc::StatePtr &state = parent_->state_truth();

    // Scrimmage is in East North Up (ENU)
    // For you to get the ROS data in North East Down (NED): switch x and y, and negate z outside scrimmage.
    // For more information on the MavROS message format view this page:
    // https://mavlink.io/en/messages/common.html#ALTITUDE

    // Fill Altitude Message
    mavros_msgs::Altitude alt_msg;
    // Scrimmage is in cartesian, use Geographic lib to get lat, long, alt for monotonic
    double lat, lon, alt;
    // cartesian(x, y, z) to (lat, long, alt) using Geographic lib
    parent_->projection()->Reverse(state->pos()(0), state->pos()(1), state->pos()(2), lat, lon, alt);
    alt_msg.monotonic = monotonic_;
    // If you print here monotonic will look like it's changing, but if you ROSTopic echo it will be a static number.
    // cout << alt_msg.monotonic << endl;

    // amsl = above sea level (meters)
    // get amsl from the same place as monotonic altitude, but this DOES change throughout the simulation
    ////// MavROS website on AMSL: //////
    // This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on
    // events like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude
    // waypoints are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already
    // output MSL by default and not the WGS84 altitude.
    alt_msg.amsl = static_cast<float>(alt);

    // local = scrimmage z position of state
    ////// MavROS website on Local: //////
    // This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference
    // to the coordinate origin (0, 0, 0). It is up-positive.
    alt_msg.local = state->pos()(2);

    // local - monotonic = relative
    ////// MavROS website on Relative: //////
    // This is the altitude above the home position. It resets on each change of the current home position.
    alt_msg.relative = alt_msg.local - alt_msg.monotonic;

    // alt_terrain = amsl
    ////// MavROS website on Altitude Terrain: //////
    // This is the altitude above terrain. It might be fed by a terrain database or an altimeter.
    // Values smaller than -1000 should be interpreted as unknown.
    alt_msg.terrain = alt_msg.amsl;

    // clearance = same as terrain // laser alt in scrimmage one day?
    ////// MavROS website on Clearance: //////
    // This is not the altitude, but the clear space below the system according to the fused clearance estimate.
    // It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving target.
    // A negative value indicates no measurement available.
    alt_msg.bottom_clearance = alt_msg.amsl;

    // Header
    std_msgs::Header header; // empty header
    // TODO: header.frame = ? // No system for ROS Frame ID's yet
    header.stamp = ros::Time::now(); // time
    alt_msg.header = header;

    // Publish Altimeter information
    altimeter_pub_.publish(alt_msg);

    return true;
}
} // namespace sensor
} // namespace scrimmage
