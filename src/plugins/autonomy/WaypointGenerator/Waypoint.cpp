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
#include <scrimmage/plugins/autonomy/WaypointGenerator/Waypoint.h>

#include <iomanip>
#include <limits>

namespace scrimmage {
namespace autonomy {

Waypoint::Waypoint(double latitude, double longitude, double altitude)
    : latitude_(latitude), longitude_(longitude), altitude_(altitude) {}

void Waypoint::set_position_tolerance(double pos_tol) {
    position_tolerance_ = pos_tol >= 0 ? pos_tol : std::numeric_limits<double>::max();
}

void Waypoint::set_quat_tolerance(double quat_tol) {
    quat_tolerance_ = quat_tol >= 0 ? quat_tol : std::numeric_limits<double>::max();
}

std::ostream& operator<<(std::ostream& os, Waypoint& wp) {
    os << std::setprecision(10)
       << wp.time() << ", "
       << wp.latitude() << ", " << wp.longitude() << ", " << wp.altitude()
       << wp.quat() << ", "
       << wp.position_tolerance() << ", "
       << wp.quat_tolerance();
    return os;
}

bool operator==(const Waypoint &lhs, const Waypoint &rhs) {
    auto far = [&](double a, double b) {return std::abs(a - b) > 1.0e-6;};

    return !far(lhs.altitude(), rhs.latitude()) &&
           !far(lhs.longitude(), rhs.longitude()) &&
           !far(lhs.altitude(), rhs.altitude());
}

} // namespace autonomy
} // namespace scrimmage
