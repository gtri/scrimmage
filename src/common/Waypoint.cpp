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
#include <scrimmage/common/Waypoint.h>

#include <iomanip>
#include <limits>

#include <GeographicLib/LocalCartesian.hpp>

namespace scrimmage {
namespace autonomy {

Waypoint::Waypoint(const double& latitude, const double& longitude,
                   const double& altitude)
    : latitude_(latitude), longitude_(longitude), altitude_(altitude) {}

Waypoint::Waypoint(const double &x, const double& y, const double& z,
                   const std::shared_ptr<GeographicLib::LocalCartesian> &proj) {
    proj->Reverse(x, y, z, latitude_, longitude_, altitude_);
}

Waypoint::Waypoint(const Eigen::Vector3d &xyz,
                   const std::shared_ptr<GeographicLib::LocalCartesian> &proj) {
    proj->Reverse(xyz(0), xyz(1), xyz(2), latitude_, longitude_, altitude_);
}

void Waypoint::set_position_tolerance(const double& pos_tol) {
    position_tolerance_ = pos_tol >= 0 ? pos_tol : std::numeric_limits<double>::max();
}

void Waypoint::set_quat_tolerance(const double& quat_tol) {
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

Eigen::Vector3d Waypoint::xyz(
    const std::shared_ptr<GeographicLib::LocalCartesian> &proj) {
    Eigen::Vector3d p;
    proj->Forward(latitude_, longitude_, altitude_, p(0), p(1), p(2));
    return p;
}

} // namespace autonomy
} // namespace scrimmage
