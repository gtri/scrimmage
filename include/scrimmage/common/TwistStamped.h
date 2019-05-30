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
 *   FITNESS FOR A PARTICULAR PURTWISTSTAMPED.  See the GNU Lesser General Public
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

#ifndef INCLUDE_SCRIMMAGE_COMMON_TWISTSTAMPED_H_
#define INCLUDE_SCRIMMAGE_COMMON_TWISTSTAMPED_H_

#include <scrimmage/common/Twist.h>
#include <Eigen/Dense>

namespace scrimmage {

class TwistStamped : public Twist {
 public:
    TwistStamped();
    explicit TwistStamped(const Eigen::Vector3d &vel);
    explicit TwistStamped(const Eigen::Vector3d &vel,
                          const Eigen::Vector3d &ang_vel);

    double &time() { return time_; }
    const double &time() const { return time_; }

 protected:
    void to_ostream(std::ostream &os) const override;
    double time_;
};

}  // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_COMMON_TWISTSTAMPED_H_
