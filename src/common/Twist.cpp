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
 *   FITNESS FOR A PARTICULAR PURTWIST.  See the GNU Lesser General Public
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
#include <scrimmage/common/Twist.h>

#include <iomanip>
#include <limits>
#include <iostream>

namespace scrimmage {

Twist::Twist()
    : vel_(0, 0, 0),
      ang_vel_(0, 0, 0) {}

Twist::Twist(const Eigen::Vector3d &vel)
    : vel_(vel),
      ang_vel_(0, 0, 0) {}

Twist::Twist(const Eigen::Vector3d &vel,
             const Eigen::Vector3d &ang_vel)
    : vel_(vel),
      ang_vel_(ang_vel) {}

Twist::~Twist() {}


void Twist::to_ostream(std::ostream& os) const {
    os << "Twist {\n"
       << "  vel: " << this->vel()[0] << ", "
                    << this->vel()[1] << ", "
                    << this->vel()[2] << "\n"
       << "  ang_vel: " << this->ang_vel()[0] << ", "
                        << this->ang_vel()[1] << ", "
                        << this->ang_vel()[2] << "\n"
       << "}";
}

std::ostream& operator<<(std::ostream& os, Twist& wp) {
    wp.to_ostream(os);
    return os;
}

}  // namespace scrimmage
