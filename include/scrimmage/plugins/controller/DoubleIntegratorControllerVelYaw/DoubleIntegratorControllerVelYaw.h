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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_DOUBLEINTEGRATORCONTROLLERVELYAW_DOUBLEINTEGRATORCONTROLLERVELYAW_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_DOUBLEINTEGRATORCONTROLLERVELYAW_DOUBLEINTEGRATORCONTROLLERVELYAW_H_

#include <scrimmage/common/PID.h>
#include <scrimmage/plugins/motion/DoubleIntegrator/DoubleIntegrator.h>

#include <map>
#include <vector>
#include <string>

namespace scrimmage {
namespace controller {
class DoubleIntegratorControllerVelYaw : public scrimmage::Controller {
 public:
    void init(std::map<std::string, std::string> &params) override;
    bool step(double t, double dt) override;

 protected:
    scrimmage::PID speed_pid_;
    scrimmage::PID alt_pid_;

    scrimmage::PID yaw_pid_;

    int desired_alt_idx_ = 0;
    int desired_speed_idx_ = 0;
    int desired_heading_idx_ = 0;

    int acc_x_idx_ = 0;
    int acc_y_idx_ = 0;
    int acc_z_idx_ = 0;
    int turn_rate_idx_ = 0;
};
} // namespace controller
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_DOUBLEINTEGRATORCONTROLLERVELYAW_DOUBLEINTEGRATORCONTROLLERVELYAW_H_
