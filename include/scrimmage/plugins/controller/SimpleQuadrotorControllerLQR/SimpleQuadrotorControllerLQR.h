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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_SIMPLEQUADROTORCONTROLLERLQR_SIMPLEQUADROTORCONTROLLERLQR_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_SIMPLEQUADROTORCONTROLLERLQR_SIMPLEQUADROTORCONTROLLERLQR_H_

#include <scrimmage/plugins/motion/SimpleQuadrotor/SimpleQuadrotor.h>

#include <map>
#include <string>

namespace scrimmage {
namespace controller {
class SimpleQuadrotorControllerLQR : public motion::SimpleQuadrotor::Controller {
 public:
    void init(std::map<std::string, std::string> &params) override;
    bool step(double t, double dt) override;
    Eigen::Vector4d &u() override { return u_; }

 protected:
    Eigen::Vector4d u_;
    scrimmage::PID vel_pid_;
    double prev_yaw_ = NAN;
    double max_vel_;

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace controller
}  // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_SIMPLEQUADROTORCONTROLLERLQR_SIMPLEQUADROTORCONTROLLERLQR_H_
