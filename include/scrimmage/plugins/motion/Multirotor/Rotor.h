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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_MOTION_MULTIROTOR_ROTOR_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_MOTION_MULTIROTOR_ROTOR_H_

#include <scrimmage/math/Quaternion.h>
#include <scrimmage/math/Angles.h>

#include <Eigen/Dense>

namespace sc = scrimmage;

namespace scrimmage {
namespace motion {
class Rotor {
 public:
    enum Direction {
        CW = 1,
        CCW = -1
    };

    Rotor() : dir_(Direction::CW), offset_length_(1.0), xy_angle_(0.0) { }

    void set_direction(Direction dir) { dir_ = dir; }
    void set_offset(Eigen::Vector3d offset) {
        offset_ = offset;
        offset_length_ = offset_.norm();
        xy_angle_ = atan2(offset_(1), offset_(0));
    }
    void set_quat(scrimmage::Quaternion quat) { quat_ = quat; }

    Direction &direction() { return dir_; }
    Eigen::Vector3d &offset() { return offset_; }
    scrimmage::Quaternion &quat() { return quat_; }
    double offset_length() { return offset_length_; }
    double xy_angle() { return xy_angle_; }

 protected:
    Direction dir_;
    Eigen::Vector3d offset_;
    double offset_length_;
    double xy_angle_;
    scrimmage::Quaternion quat_;

 private:
};
} // namespace motion
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_MOTION_MULTIROTOR_ROTOR_H_
