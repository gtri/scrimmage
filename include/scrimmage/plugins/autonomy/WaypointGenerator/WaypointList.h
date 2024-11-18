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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_WAYPOINTGENERATOR_WAYPOINTLIST_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_WAYPOINTGENERATOR_WAYPOINTLIST_H_

#include <scrimmage/common/Waypoint.h>

#include <list>

namespace scrimmage {
namespace autonomy {

class WaypointList {
 public:
    enum class WaypointMode { follow_once = 0, back_and_forth = 1, loiter = 2, racetrack = 3 };

    WaypointList() {
    }

    std::list<Waypoint>& waypoints() {
        return waypoints_;
    }
    void set_mode(WaypointMode mode) {
        mode_ = mode;
    }
    WaypointMode mode() {
        return mode_;
    }

    friend std::ostream& operator<<(std::ostream& os, WaypointList& wp_list) {
        for (Waypoint wp : wp_list.waypoints()) {
            os << wp << std::endl;
        }
        return os;
    }

    void set_cycles(unsigned int cycles) {
        cycles_ = cycles;
    }

    unsigned int cycles() {
        return cycles_;
    }

    unsigned int size() {
        return waypoints_.size();
    }

 protected:
    std::list<Waypoint> waypoints_;
    WaypointMode mode_ = WaypointMode::follow_once;
    unsigned int cycles_ = 1;
};
}  // namespace autonomy
}  // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_WAYPOINTGENERATOR_WAYPOINTLIST_H_
