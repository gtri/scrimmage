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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_GOTOWAYPOINT_GOTOWAYPOINT_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_GOTOWAYPOINT_GOTOWAYPOINT_H_
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/plugins/autonomy/WaypointGenerator/WaypointList.h>

#include <vector>
#include <string>
#include <map>

namespace scrimmage {
namespace autonomy {
class GoToWaypoint : public scrimmage::Autonomy {
 public:
    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;
    void publish_waypoint(double t);

 protected:
    bool waypoint_status_;
    WaypointList wp_list_;
    std::vector<std::string> waypoint_;
    scrimmage::PublisherPtr waypoint_list_pub_;
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_GOTOWAYPOINT_GOTOWAYPOINT_H_
