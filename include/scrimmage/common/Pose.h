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
 * @author Christopher Richardson <christopher.richardson@gtri.gatech.edu>
 * @date 29 May 2019
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#ifndef INCLUDE_SCRIMMAGE_COMMON_POSE_H_
#define INCLUDE_SCRIMMAGE_COMMON_POSE_H_

#include <scrimmage/math/Quaternion.h>
#include <Eigen/Dense>

#include <iostream>

namespace scrimmage {

class Pose {
 public:
    Pose();
    virtual ~Pose();
    explicit Pose(const Eigen::Vector3d &pos);
    explicit Pose(const scrimmage::Quaternion &quat);
    explicit Pose(const Eigen::Vector3d &pos,
                  const scrimmage::Quaternion &quat);

    Eigen::Vector3d& pos() { return pos_; }
    scrimmage::Quaternion &quat() { return quat_; }

    const Eigen::Vector3d& pos() const { return pos_; }
    const scrimmage::Quaternion &quat() const { return quat_; }

    friend std::ostream& operator<<(std::ostream& os, Pose& wp);

 protected:
    Eigen::Vector3d pos_;
    scrimmage::Quaternion quat_;

    // indirection for polymorphism of << operator
    virtual void to_ostream(std::ostream& os) const;
};

}  // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_COMMON_POSE_H_
