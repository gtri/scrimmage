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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_MOTION_UNICYCLE_UNICYCLE_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_MOTION_UNICYCLE_UNICYCLE_H_

#include <scrimmage/motion/MotionModel.h>

#include <map>
#include <string>

namespace scrimmage {
namespace motion {
class Unicycle : public scrimmage::MotionModel {
 public:
    bool init(std::map<std::string, std::string> &info,
                      std::map<std::string, std::string> &params) override;

    bool step(double t, double dt) override;

    void model(const vector_t &x , vector_t &dxdt , double t) override;

 protected:
    double turn_rate_max_ = 1.0;
    double pitch_rate_max_ = 1.0;
    double velocity_z_max_ = 0.0;
    double vel_max_ = 1.0;
    bool enable_roll_ = false;
    bool use_pitch_ = true;

    uint8_t speed_idx_ = 0;
    uint8_t turn_rate_idx_ = 0;
    uint8_t pitch_rate_idx_ = 0;
    uint8_t velocity_z_idx_ = 0;

    double velocity_ = 0;
    double turn_rate_ = 0;
    double pitch_rate_ = 0;
    double velocity_z_ = 0;
};
} // namespace motion
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_MOTION_UNICYCLE_UNICYCLE_H_
