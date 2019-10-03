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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_MOTION_SIMPLEAIRCRAFT_SIMPLEAIRCRAFT_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_MOTION_SIMPLEAIRCRAFT_SIMPLEAIRCRAFT_H_

#include <scrimmage/motion/MotionModel.h>
#include <scrimmage/motion/Controller.h>
#include <scrimmage/common/PID.h>

#include <map>
#include <string>
#include <tuple>
#include <memory>

namespace scrimmage {
class State;
using StatePtr = std::shared_ptr<State>;
}

namespace scrimmage {
namespace motion {
class SimpleAircraft : public scrimmage::MotionModel{
 public:
    virtual std::tuple<int, int, int> version();

    bool init(std::map<std::string, std::string> &info,
                      std::map<std::string, std::string> &params) override;
    bool step(double time, double dt) override;

    void model(const vector_t &x , vector_t &dxdt , double t) override;

    void teleport(scrimmage::StatePtr &state) override;

 protected:
    scrimmage::PID heading_pid_;
    scrimmage::PID alt_pid_;
    scrimmage::PID vel_pid_;

    double length_{50.0};  // Turning radius
    double speedTarget_{50.0};  // The speed at which a full turn will result in the turning radius being met
    // The slope at which the turning radius is adjusted based on speed.  Positive means larger turning radius at higher speeds.
    double lengthSlopePerSpeed_{0.0};

    double min_velocity_{15.0};
    double max_velocity_{40.0};
    double max_roll_{30.0};
    double max_pitch_{30.0};
    double max_pitch_rate_{57.3};
    double max_roll_rate_{57.3};

    uint8_t throttle_idx_{0};
    uint8_t roll_rate_idx_{0};
    uint8_t pitch_rate_idx_{0};
};
} // namespace motion
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_MOTION_SIMPLEAIRCRAFT_SIMPLEAIRCRAFT_H_
