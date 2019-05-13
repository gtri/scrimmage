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

#include <scrimmage/common/WaypointListProcessor.h>

#include <GeographicLib/LocalCartesian.hpp>

namespace sc = scrimmage;
namespace sp = scrimmage_proto;

namespace scrimmage {
namespace autonomy {

void WaypointListProcessor::set_waypoint_list(
    const scrimmage_msgs::WaypointList &wp_list) {
    returning_stage_ = false;

    // Construct a list of Waypoints from protobuf waypoint list
    wp_list_.clear();
    for (int i = 0; i < wp_list.waypoints().size(); i++) {
        wp_list_.push_back(Waypoint(wp_list.waypoints(i)));
    }

    // Filter the waypoint list to ensure that two waypoints in succession
    // are not equal. Some planners generate duplicate waypoints.
    auto wps_are_close = [] (auto &wp0, auto &wp1) {
        auto numbers_are_close = [] (const double &a, const double &b) {
            return std::abs(a - b) < std::numeric_limits<double>::epsilon();
        };
        return numbers_are_close(wp0.latitude(), wp1.latitude())
                && numbers_are_close(wp0.longitude(), wp1.longitude())
                && numbers_are_close(wp0.altitude(), wp1.altitude());
    };
    wp_list_.unique(wps_are_close);

    // Set the wp iterator to point to the beginning of the list
    prev_wp_it_ = wp_list_.begin();
    curr_wp_it_ = wp_list_.begin();
    next_wp_it_ = ++(wp_list_.begin());
}

WaypointListProcessor::Status
WaypointListProcessor::process(
    const scrimmage::StatePtr &state,
    const std::shared_ptr<GeographicLib::LocalCartesian> &proj) {
    if (wp_list_.size() == 0) {
        return Status::Empty;
    }

    Waypoint &curr_wp_lla = *curr_wp_it_;

    // If the current state isn't within tolerance, the waypoint is unchanged
    if (not curr_wp_lla.is_within_tolerance(state, proj)) {
        return Status::Unchanged;
    }

    // bool done = false;
    switch (mode_) {
        case scrimmage_msgs::WaypointList::FOLLOW_ONCE:
            prev_wp_it_ = curr_wp_it_;
            ++curr_wp_it_;

            if (curr_wp_it_ == wp_list_.end()) {
                curr_wp_it_ = prev_wp_it_;
                // done = true;
            }
            break;

        case scrimmage_msgs::WaypointList::BACK_AND_FORTH:
            prev_wp_it_ = curr_wp_it_;
            curr_wp_it_ = returning_stage_ ? --curr_wp_it_ : ++curr_wp_it_;

            if (curr_wp_it_ == wp_list_.begin()) {
                returning_stage_ = false;
            } else if (curr_wp_it_ == wp_list_.end()) {
                curr_wp_it_ = prev_wp_it_;
                returning_stage_ = true;
            }
            break;

        case scrimmage_msgs::WaypointList::LOITER:
            // TODO : Not implemented yet
            break;

        case scrimmage_msgs::WaypointList::LOOP:
            // Continuous repeat of waypoints
            prev_wp_it_ = curr_wp_it_;
            ++curr_wp_it_;

            if (curr_wp_it_ == wp_list_.end()) {
                curr_wp_it_ = wp_list_.begin();
            }
            break;

        default :
            std::string msg = "Waypoint mode not defined yet.";
            throw std::runtime_error(msg);
    }
    return Status::Changed;
}

const Waypoint & WaypointListProcessor::previous_waypoint() {
    return *prev_wp_it_;
}

const Waypoint & WaypointListProcessor::current_waypoint() {
    return *curr_wp_it_;
}

const Waypoint & WaypointListProcessor::next_waypoint() {
    return *next_wp_it_;
}
} // namespace autonomy
} // namespace scrimmage
