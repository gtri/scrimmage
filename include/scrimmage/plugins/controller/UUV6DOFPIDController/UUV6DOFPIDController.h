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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_UUV6DOFPIDCONTROLLER_UUV6DOFPIDCONTROLLER_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_UUV6DOFPIDCONTROLLER_UUV6DOFPIDCONTROLLER_H_

#include <scrimmage/common/PID.h>
#include <scrimmage/motion/Controller.h>

#include <Eigen/Dense>
#include <map>
#include <string>

namespace scrimmage {
namespace controller {

class UUV6DOFPIDController : public scrimmage::Controller {
 public:
    UUV6DOFPIDController();
    void init(std::map<std::string, std::string>& params) override;
    bool step(double t, double dt) override;

 protected:
    // Inputs
    int desired_altitude_idx_ = 0;
    int desired_speed_idx_ = 0;
    int desired_heading_idx_ = 0;

    // Outputs
    int throttle_idx_ = 0;
    int elevator_idx_ = 0;
    int rudder_idx_ = 0;

    scrimmage::PID heading_pid_;
    scrimmage::PID pitch_pid_;
    scrimmage::PID speed_pid_;
};
}  // namespace controller
}  // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_UUV6DOFPIDCONTROLLER_UUV6DOFPIDCONTROLLER_H_
