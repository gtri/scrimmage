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
#include <scrimmage/common/TwistStamped.h>

#include <iomanip>
#include <limits>
#include <iostream>

namespace scrimmage {

TwistStamped::TwistStamped()
    : Twist(),
      time_(0) {}

TwistStamped:: TwistStamped(const Eigen::Vector3d &vel)
    : Twist(vel),
      time_(0) {}

TwistStamped:: TwistStamped(const Eigen::Vector3d &vel,
                            const Eigen::Vector3d &ang_vel)
    : Twist(vel, ang_vel),
      time_(0) {}

void TwistStamped::to_ostream(std::ostream& os) const {
    os << "TwistStamped {\n"
       << "  time: " << this->time() << "\n"
       << "  vel: " << this->vel()[0] << ", "
                    << this->vel()[1] << ", "
                    << this->vel()[2] << "\n"
       << "  ang_vel: " << this->ang_vel()[0] << ", "
                        << this->ang_vel()[1] << ", "
                        << this->ang_vel()[2] << "\n"
       << "}";
}

}  // namespace scrimmage
