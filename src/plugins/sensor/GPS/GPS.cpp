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
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/plugins/sensor/GPS/GPS.h>

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
#include <scrimmage/msgs/GPS.pb.h>
#include <scrimmage/plugins/interaction/Boundary/BoundaryBase.h>
#include <scrimmage/plugins/interaction/Boundary/Boundary.h>

#include <GeographicLib/LocalCartesian.hpp>


using std::cout;
using std::endl;

namespace sc = scrimmage;
namespace sp = scrimmage_proto;
namespace sci = scrimmage::interaction;
namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(scrimmage::Sensor, scrimmage::sensor::GPS, GPS_plugin)

namespace scrimmage {
namespace sensor {

GPS::GPS(): gps_found_(false), boundary_id_(-1) {}

void GPS::init(std::map<std::string, std::string> &params) {
    // Loading in params
    if (params.count("gps_denied_ids") > 0) {
       gps_found_ = str2container(params["gps_denied_ids"], ",", gps_denied_ids_);
    }

    pub_ = advertise("GlobalNetwork", "GPSStatus");

    auto bd_cb = [&](auto &msg) {
        boundary_id_ = msg->data.id().id();
        boundary_ = sci::Boundary::make_boundary(msg->data);
    };
    subscribe<sp::Shape>("GlobalNetwork", "Boundary", bd_cb);
}

bool GPS::step() {
    // Obtain current state information
    sc::State ns = *(parent_->state_truth());

    auto msg = std::make_shared<Message<sm::GPSStatus>>();

    msg->data.set_sender_id(parent_->id().id());

    double lat = 0;
    double lon = 0;
    double alt = 0;
    parent_->projection()->Reverse(ns.pos()(0), ns.pos()(1), ns.pos()(2),
            lat, lon, alt);
    msg->data.set_lat(lat);
    msg->data.set_lon(lon);
    msg->data.set_alt(alt);

    msg->data.set_time(time_->t());

    bool gps_fix = true;
    if (boundary_ != nullptr && gps_found_) {
       if (std::find(gps_denied_ids_.begin(), gps_denied_ids_.end(), boundary_id_) !=
                gps_denied_ids_.end() && boundary_->contains(ns.pos())) {
                gps_fix = false;
        }
    }

    msg->data.set_fixed(gps_fix);

    // Publish GPS information
    pub_->publish(msg);

    return true;
}
} // namespace sensor
} // namespace scrimmage
