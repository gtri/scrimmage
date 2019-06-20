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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_AIRCRAFTPIDCONTROLLER_AIRCRAFTPIDCONTROLLER_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_AIRCRAFTPIDCONTROLLER_AIRCRAFTPIDCONTROLLER_H_

#include <scrimmage/motion/Controller.h>
#include <scrimmage/common/PID.h>

#include <Eigen/Dense>

#include <map>
#include <string>

namespace scrimmage {
namespace controller {

class AircraftPIDController : public scrimmage::Controller {
 public:
    AircraftPIDController();
    void init(std::map<std::string, std::string> &params) override;
    bool step(double t, double dt) override;

 protected:
    // Input variables
    uint8_t desired_roll_idx_ = 0;
    uint8_t desired_heading_idx_ = 0;
    uint8_t desired_altitude_idx_ = 0;
    uint8_t desired_speed_idx_ = 0;

    // Output variables
    uint8_t throttle_idx_ = 0;
    uint8_t elevator_idx_ = 0;
    uint8_t aileron_idx_ = 0;
    uint8_t rudder_idx_ = 0;

    scrimmage::PID pitch_pid_;
    scrimmage::PID roll_pid_;

    scrimmage::PID heading_pid_;
    scrimmage::PID altitude_pid_;
    scrimmage::PID speed_pid_;

    double max_pitch_ = 45.0;
    double max_roll_ = 45.0;

    bool use_roll_control_ = false;
};
} // namespace controller
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_AIRCRAFTPIDCONTROLLER_AIRCRAFTPIDCONTROLLER_H_
