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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_SIMPLEAIRCRAFTCONTROLLERPID_SIMPLEAIRCRAFTCONTROLLERPID_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_SIMPLEAIRCRAFTCONTROLLERPID_SIMPLEAIRCRAFTCONTROLLERPID_H_

#include <scrimmage/common/PID.h>
#include <scrimmage/motion/Controller.h>

#include <map>
#include <string>

namespace scrimmage {
namespace controller {

/**
 * Designed to control the simple aircraft model.
 * Horizontal channel controlled through roll rate or heading rate
 * Vertical channel controll through altitude or glide slope (tan(z_vel / sqrt(x_vel^2 + y_vel^2)))
 * Note that pitch for the simple aircraft model is actually in the direction of the velocity vector in the vertical plane (angle of attack assumed to be zero)
 * Speed controlled through a separate channel.
 * A glider can be controlled through here if the speed is always set to match the glide speed based on the roll and pitch angles.
 */
class SimpleAircraftControllerPID : public Controller {
 public:
    virtual void init(std::map<std::string, std::string> &params);
    virtual bool step(double t, double dt);

 protected:
    scrimmage::PID heading_pid_;
    scrimmage::PID alt_pid_;
    scrimmage::PID vel_pid_;
    bool use_roll_ = false;
    bool use_glide_slope_ = false;
    int alt_idx_ = 0;

    uint8_t input_roll_or_heading_idx_ = 0;
    uint8_t input_altitude_or_glide_slope_idx_ = 0;
    uint8_t input_velocity_idx_ = 0;

    uint8_t output_throttle_idx_ = 0;
    uint8_t output_roll_rate_idx_ = 0;
    uint8_t output_pitch_rate_idx_ = 0;
};
}  // namespace controller
}  // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_SIMPLEAIRCRAFTCONTROLLERPID_SIMPLEAIRCRAFTCONTROLLERPID_H_
