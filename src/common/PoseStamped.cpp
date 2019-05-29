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
#include <scrimmage/common/PoseStamped.h>

#include <iomanip>
#include <limits>
#include <iostream>

namespace scrimmage {

PoseStamped::PoseStamped()
    : Pose(),
      time_(0) {}

PoseStamped:: PoseStamped(const Eigen::Vector3d &pos)
    : Pose(pos),
      time_(0) {}

PoseStamped:: PoseStamped(const scrimmage::Quaternion &quat)
    : Pose(quat),
      time_(0) {}

PoseStamped:: PoseStamped(const Eigen::Vector3d &pos,
                          const scrimmage::Quaternion &quat)
    : Pose(pos, quat),
      time_(0) {}

void PoseStamped::to_ostream(std::ostream& os) const {
    os << "PoseStamped {\n"
       << "  time: " << this->time() << "\n"
       << "  pos: " << this->pos()[0] << ", "
                    << this->pos()[1] << ", "
                    << this->pos()[2] << "\n"
       << "  quat: " << this->quat() << "\n"
       << "}";
}

}  // namespace scrimmage
