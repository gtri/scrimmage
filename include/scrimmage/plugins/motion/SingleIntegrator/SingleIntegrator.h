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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_MOTION_SINGLEINTEGRATOR_SINGLEINTEGRATOR_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_MOTION_SINGLEINTEGRATOR_SINGLEINTEGRATOR_H_

#include <scrimmage/motion/MotionModel.h>

#include <map>
#include <string>

namespace scrimmage {
namespace motion {
class SingleIntegrator : public scrimmage::MotionModel {
 public:
    bool init(std::map<std::string, std::string> &info,
              std::map<std::string, std::string> &params) override;
    bool step(double t, double dt) override;

 protected:
    bool override_heading_;

    uint8_t vel_x_idx_ = 0;
    uint8_t vel_y_idx_ = 0;
    uint8_t vel_z_idx_ = 0;
    uint8_t desired_heading_idx_ = 0;
};
} // namespace motion
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_MOTION_SINGLEINTEGRATOR_SINGLEINTEGRATOR_H_
