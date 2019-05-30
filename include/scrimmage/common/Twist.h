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

#ifndef INCLUDE_SCRIMMAGE_COMMON_TWIST_H_
#define INCLUDE_SCRIMMAGE_COMMON_TWIST_H_

#include <Eigen/Dense>

#include <iostream>

namespace scrimmage {

class Twist {
 public:
    Twist();
    virtual ~Twist();
    explicit Twist(const Eigen::Vector3d &vel);
    explicit Twist(const Eigen::Vector3d &vel,
                   const Eigen::Vector3d &ang_vel);

    Eigen::Vector3d &vel() { return vel_; }
    Eigen::Vector3d &ang_vel() { return ang_vel_; }

    const Eigen::Vector3d &vel() const { return vel_; }
    const Eigen::Vector3d &ang_vel() const { return ang_vel_; }

    friend std::ostream &operator<<(std::ostream &os, Twist &wp);

 protected:
    Eigen::Vector3d vel_;
    Eigen::Vector3d ang_vel_;

    // indirection for polymorphism of << operator
    virtual void to_ostream(std::ostream &os) const;
};

}  // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_COMMON_TWIST_H_
