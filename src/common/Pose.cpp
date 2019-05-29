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
#include <scrimmage/common/Pose.h>

#include <iomanip>
#include <limits>
#include <iostream>

namespace scrimmage {

Pose::Pose()
    : pos_(0, 0, 0),
      quat_(0, 0, 0) {}

Pose::Pose(const Eigen::Vector3d &pos)
    : pos_(pos),
      quat_(0, 0, 0) {}

Pose::Pose(const scrimmage::Quaternion &quat)
    : pos_(0, 0, 0),
      quat_(quat) {}

Pose::Pose(const Eigen::Vector3d &pos,
              const scrimmage::Quaternion &quat)
    : pos_(pos),
      quat_(quat) {}


void Pose::to_ostream(std::ostream& os) const {
    os << "Pose {\n"
       << "  pos: " << this->pos()[0] << ", "
                    << this->pos()[1] << ", "
                    << this->pos()[2] << "\n"
       << "  quat: " << this->quat() << "\n"
       << "}";
}

std::ostream& operator<<(std::ostream& os, Pose& wp) {
    wp.to_ostream(os);
    return os;
}

}  // namespace scrimmage
