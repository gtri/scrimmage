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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_WAYPOINTFOLLOWER_WAYPOINTFOLLOWER_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_WAYPOINTFOLLOWER_WAYPOINTFOLLOWER_H_

#include <scrimmage/autonomy/Autonomy.h>

#include <Eigen/Dense>
#include <map>
#include <vector>
#include <string>

class WayPointFollower : public scrimmage::Autonomy {
 public:
    class WayPoint {
     public:
        WayPoint() : point(0, 0, 0), tolerance(10.0) {}
        WayPoint(Eigen::Vector3d p, double t) : point(p), tolerance(t) {}
        Eigen::Vector3d point;
        double tolerance;
    };

    WayPointFollower();
    virtual void init(std::map<std::string, std::string> &params);
    virtual bool step_autonomy(double t, double dt);

 protected:
    std::vector<WayPoint> wps_;

    bool staging_achieved_;
    unsigned int wp_idx_;
    double max_alt_change_;
    double wp_tolerance_;

    std::string waypoint_type_;
    bool loiter_mode_;

 private:
};

#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_WAYPOINTFOLLOWER_WAYPOINTFOLLOWER_H_
