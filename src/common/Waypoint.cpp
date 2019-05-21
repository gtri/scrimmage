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
#include <scrimmage/math/State.h>
#include <scrimmage/proto/ProtoConversions.h>

#include <iomanip>
#include <limits>
#include <iostream>

#include <GeographicLib/LocalCartesian.hpp>

namespace sc = scrimmage;
namespace sc_msgs = scrimmage_msgs;

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

Waypoint::Waypoint(const scrimmage_msgs::Waypoint &wp)
        : id_(wp.id()), time_(wp.time()),
          position_tolerance_(wp.position_tolerance()),
          quat_tolerance_(wp.quat_tolerance()),
          tolerance_in_2d_(wp.tolerance_in_2d()) {
    sc::set(quat_, wp.quat());

    auto get_alt = [] (const sc_msgs::PositionLLA &lla) {
        double alt = 0;
        if (lla.alt_oneof_case() == sc_msgs::PositionLLA::kAltMsl) {
            alt = lla.alt_msl();
        } else {
            std::cout << "Waypoints only support alt_msl()!" << std::endl;
        }
        return alt;
    };

    if (wp.pos_oneof_case() == sc_msgs::Waypoint::kLla) {
        latitude_ = wp.lla().latitude();
        longitude_ = wp.lla().longitude();
        altitude_ = get_alt(wp.lla());
    } else if (wp.pos_oneof_case() == sc_msgs::Waypoint::kXyz) {
        // Convert to lat/lon/alt given origin
        GeographicLib::LocalCartesian proj(wp.xyz().origin().latitude(),
                                           wp.xyz().origin().longitude(),
                                           get_alt(wp.xyz().origin()));
        proj.Reverse(wp.xyz().xyz().x(), wp.xyz().xyz().y(), wp.xyz().xyz().z(),
                     latitude_, longitude_, altitude_);
    }
}

scrimmage_msgs::Waypoint Waypoint::lla_proto() {
    scrimmage_msgs::Waypoint wp;
    wp.set_id(id_);
    wp.set_time(time_);
    sc::set(wp.mutable_quat(), quat_);
    wp.mutable_lla()->set_latitude(latitude_);
    wp.mutable_lla()->set_longitude(longitude_);
    wp.mutable_lla()->set_alt_msl(altitude_);
    wp.set_position_tolerance(position_tolerance_);
    wp.set_quat_tolerance(quat_tolerance_);
    wp.set_tolerance_in_2d(tolerance_in_2d_);
    return wp;
}

bool Waypoint::is_within_tolerance(
    const scrimmage::StatePtr &state,
    const std::shared_ptr<GeographicLib::LocalCartesian> &proj) {

    Eigen::Vector3d pos;
    proj->Forward(latitude_, longitude_, altitude_, pos(0), pos(1), pos(2));
    double dist = tolerance_in_2d_ ?
            (state->pos().head<2>() - pos.head<2>()).norm() :
            (state->pos() - pos).norm();
    return dist <= position_tolerance_;
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
    const std::shared_ptr<GeographicLib::LocalCartesian> &proj) const {
    Eigen::Vector3d p;
    proj->Forward(latitude_, longitude_, altitude_, p(0), p(1), p(2));
    return p;
}

} // namespace autonomy
} // namespace scrimmage
