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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_WAYPOINTGENERATOR_WAYPOINT_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_WAYPOINTGENERATOR_WAYPOINT_H_

#include <scrimmage/math/Quaternion.h>

#include <iomanip>
#include <string>
#include <map>
#include <limits>

namespace scrimmage {
namespace autonomy {

class Waypoint {
 public:
    Waypoint(double latitude, double longitude, double altitude)
        : latitude_(latitude), longitude_(longitude), altitude_(altitude) {
    }

    void set_time(double time) { time_ = time; }
    void set_latitude(double latitude) { latitude_ = latitude; }
    void set_longitude(double longitude) { longitude_ = longitude; }
    void set_altitude(double altitude) { altitude_ = altitude; }
    void set_quat(scrimmage::Quaternion &quat) { quat_ = quat; }
    void set_position_tolerance(double pos_tol) {
        if (pos_tol < 0.0) {
            position_tolerance_ = std::numeric_limits<double>::max();
        } else {
            position_tolerance_ = pos_tol;
        }
    }

    void set_quat_tolerance(double quat_tol) {
        if (quat_tol < 0.0) {
            quat_tolerance_ = std::numeric_limits<double>::max();
        } else {
            quat_tolerance_ = quat_tol;
        }
    }

    double time() { return time_; }
    double latitude() { return latitude_; }
    double longitude() { return longitude_; }
    double altitude() { return altitude_; }
    scrimmage::Quaternion &quat() { return quat_; }
    double position_tolerance() { return position_tolerance_; }
    double quat_tolerance() { return quat_tolerance_; }

    friend std::ostream& operator<<(std::ostream& os, Waypoint& wp) {
        os << std::setprecision(10)
           << wp.time() << ", "
           << wp.latitude() << ", " << wp.longitude() << ", " << wp.altitude()
           << wp.quat() << ", "
           << wp.position_tolerance() << ", "
           << wp.quat_tolerance();
        return os;
    }

    friend bool operator==(Waypoint &lhs, Waypoint &rhs) {
        if (std::abs(lhs.latitude() - rhs.latitude()) > std::numeric_limits<double>::epsilon()) {
            return false;
        }
        if (std::abs(lhs.longitude() - rhs.longitude()) > std::numeric_limits<double>::epsilon()) {
            return false;
        }
        if (std::abs(lhs.altitude() - rhs.altitude()) > std::numeric_limits<double>::epsilon()) {
            return false;
        }
        return true;
    }

 protected:
    double time_ = 0.0;
    double latitude_ = 0.0;
    double longitude_ = 0.0;
    double altitude_ = 0.0;
    scrimmage::Quaternion quat_;

    double position_tolerance_ = 100.0;
    double quat_tolerance_ = 100.0;
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_WAYPOINTGENERATOR_WAYPOINT_H_
