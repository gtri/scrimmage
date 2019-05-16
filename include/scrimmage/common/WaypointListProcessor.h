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

#ifndef INCLUDE_SCRIMMAGE_COMMON_WAYPOINTLISTPROCESSOR_H_
#define INCLUDE_SCRIMMAGE_COMMON_WAYPOINTLISTPROCESSOR_H_

#include <scrimmage/msgs/Waypoint.pb.h>
#include <scrimmage/common/Waypoint.h>
#include <scrimmage/math/State.h>

#include <list>
#include <memory>

#include <boost/optional.hpp>

namespace scrimmage {

namespace autonomy {
class WaypointListProcessor {
 public:
    enum class Status { Empty, Invalid, Changed, Unchanged  };
    void set_waypoint_list(const scrimmage_msgs::WaypointList &wp_list);
    Status process(const scrimmage::StatePtr &state,
                   const std::shared_ptr<GeographicLib::LocalCartesian> &proj);

    boost::optional<const Waypoint&> previous_waypoint();
    boost::optional<const Waypoint&> current_waypoint();
    boost::optional<const Waypoint&> next_waypoint();

 protected:
    std::shared_ptr<GeographicLib::LocalCartesian> proj_;
    std::list<Waypoint> wp_list_;

    std::list<Waypoint>::iterator prev_wp_it_ = wp_list_.end();
    std::list<Waypoint>::iterator curr_wp_it_ = wp_list_.end();
    std::list<Waypoint>::iterator next_wp_it_ = wp_list_.end();
    scrimmage_msgs::WaypointList proto_wp_list_;
    scrimmage_msgs::WaypointList::Mode mode_;
    bool returning_stage_ = false;

    std::list<Waypoint>::iterator
    next_waypoint(const std::list<Waypoint>::iterator &it_wp);
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_COMMON_WAYPOINTLISTPROCESSOR_H_
