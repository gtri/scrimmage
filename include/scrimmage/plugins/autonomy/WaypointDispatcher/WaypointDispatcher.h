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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_WAYPOINTDISPATCHER_WAYPOINTDISPATCHER_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_WAYPOINTDISPATCHER_WAYPOINTDISPATCHER_H_
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/plugins/autonomy/WaypointGenerator/Waypoint.h>
#include <scrimmage/plugins/autonomy/WaypointGenerator/WaypointList.h>

#include <string>
#include <map>
#include <list>

namespace scrimmage {
namespace autonomy {
class WaypointDispatcher : public scrimmage::Autonomy {
 public:
    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;

 protected:
    Eigen::Vector3d lla_to_xyz(const Waypoint &wpt);

    WaypointList wp_list_;
    std::list<Waypoint>::iterator wp_it_;
    std::list<Waypoint>::iterator prev_wp_it_;
    unsigned int cycles_ = 0;
    bool returning_stage_ = false;
    double lead_distance_ = 50;
    double filter_dist_ = 1;

    scrimmage_proto::ShapePtr track_sphere_shape_ = std::make_shared<scrimmage_proto::Shape>();
    scrimmage_proto::ShapePtr curr_wp_sphere_shape_ = std::make_shared<scrimmage_proto::Shape>();
    scrimmage_proto::ShapePtr prev_wp_sphere_shape_ = std::make_shared<scrimmage_proto::Shape>();
    bool show_shapes_ = false;

    scrimmage::PublisherPtr wp_pub_;
    scrimmage::PublisherPtr wp_pub_status_;
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_WAYPOINTDISPATCHER_WAYPOINTDISPATCHER_H_
