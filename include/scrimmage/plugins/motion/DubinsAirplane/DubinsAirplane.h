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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_MOTION_DUBINSAIRPLANE_DUBINSAIRPLANE_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_MOTION_DUBINSAIRPLANE_DUBINSAIRPLANE_H_

#include <scrimmage/math/State.h>
#include <scrimmage/motion/MotionModel.h>
#include <scrimmage/motion/Controller.h>
#include <scrimmage/common/PID.h>
#include <scrimmage/common/CSV.h>

#include <map>
#include <string>
#include <limits>

namespace scrimmage {
namespace motion {
class DubinsAirplane : public scrimmage::MotionModel {
 public:
    bool init(std::map<std::string, std::string> &info,
              std::map<std::string, std::string> &params) override;
    bool step(double t, double dt) override;
    void model(const vector_t &x , vector_t &dxdt , double t) override;

 protected:
    double speed_max_ = +std::numeric_limits<double>::infinity();
    double speed_min_ = -std::numeric_limits<double>::infinity();
    double pitch_max_ = +90;
    double pitch_min_ = -90;
    double roll_max_  = +90;
    double roll_min_  = -90;

    bool write_csv_ = false;
    CSV csv_;

    int desired_speed_idx_ = 0;
    int desired_pitch_idx_ = 0;
    int desired_roll_idx_ = 0;

    double speed_ = 0;
    double pitch_ = 0;
    double roll_ = 0;
};
} // namespace motion
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_MOTION_DUBINSAIRPLANE_DUBINSAIRPLANE_H_
