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

#ifndef INCLUDE_SCRIMMAGE_COMMON_WAYPOINT_H_
#define INCLUDE_SCRIMMAGE_COMMON_WAYPOINT_H_

#include <scrimmage/math/State.h>
#include <scrimmage/msgs/Waypoint.pb.h>

#include <string>
#include <map>
#include <memory>

namespace GeographicLib {
class LocalCartesian;
}

namespace scrimmage {
namespace autonomy {

class Waypoint {
 public:
    Waypoint() = default;
    Waypoint(const double& latitude, const double& longitude,
             const double& altitude);
    Waypoint(const double &x, const double& y, const double& z,
             const std::shared_ptr<GeographicLib::LocalCartesian> &proj);
    Waypoint(const Eigen::Vector3d &xyz,
             const std::shared_ptr<GeographicLib::LocalCartesian> &proj);
    explicit Waypoint(const scrimmage_msgs::Waypoint &wp);

    void set_id(const size_t& id) {id_ = id;}
    void set_time(const double& time) { time_ = time; }
    void set_latitude(const double& latitude) { latitude_ = latitude; }
    void set_longitude(const double& longitude) { longitude_ = longitude; }
    void set_altitude(const double& altitude) { altitude_ = altitude; }
    void set_quat(const scrimmage::Quaternion& quat) { quat_ = quat; }
    void set_position_tolerance(const double& pos_tol);
    void set_quat_tolerance(const double& quat_tol);

    scrimmage_msgs::Waypoint lla_proto();

    const size_t& id() const { return id_; }
    const double& time() const { return time_; }
    const double& latitude() const { return latitude_; }
    const double& longitude() const { return longitude_; }
    const double& altitude() const { return altitude_; }
    const scrimmage::Quaternion &quat() const { return quat_; }
    const double& position_tolerance() const { return position_tolerance_; }
    const double& quat_tolerance() const { return quat_tolerance_; }

    friend std::ostream& operator<<(std::ostream& os, Waypoint& wp);

    friend bool operator==(const Waypoint &lhs, const Waypoint &rhs);

    Eigen::Vector3d xyz(
        const std::shared_ptr<GeographicLib::LocalCartesian> &proj) const;

    bool is_within_tolerance(
        const scrimmage::StatePtr &state,
        const std::shared_ptr<GeographicLib::LocalCartesian> &proj);

 protected:
    size_t id_ = 0;
    double time_ = 0.0;
    double latitude_ = 0.0;
    double longitude_ = 0.0;
    double altitude_ = 0.0;
    scrimmage::Quaternion quat_;

    double position_tolerance_ = 100.0;
    double quat_tolerance_ = 100.0;

    bool tolerance_in_2d_ = false;
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_COMMON_WAYPOINT_H_
