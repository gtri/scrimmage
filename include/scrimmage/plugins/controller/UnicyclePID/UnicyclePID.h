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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_UNICYCLEPID_UNICYCLEPID_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_UNICYCLEPID_UNICYCLEPID_H_

#include <scrimmage/motion/Controller.h>
#include <scrimmage/common/PID.h>

#include <map>
#include <string>
#include <memory>

namespace scrimmage {
namespace controller {

class UnicyclePID : public scrimmage::Controller {
 public:
    UnicyclePID();
    void init(std::map<std::string, std::string> &params) override;
    bool step(double t, double dt) override;

 protected:
    // Input variables
    int desired_heading_idx_ = 0;
    int desired_alt_idx_ = 0;
    int desired_speed_idx_ = 0;

    // Output variables
    int speed_idx_ = 0;
    int turn_rate_idx_ = 0;
    int pitch_rate_idx_ = 0;
    int roll_rate_idx_ = 0;

    scrimmage::PID heading_pid_;
    scrimmage::PID speed_pid_;
    scrimmage::PID pitch_pid_;
    scrimmage::PID roll_pid_;

    bool show_shapes_ = false;
    scrimmage_proto::ShapePtr line_shape_ = std::make_shared<scrimmage_proto::Shape>();

    bool use_accel_ = false;
};
} // namespace controller
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_UNICYCLEPID_UNICYCLEPID_H_
