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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_MOTIONBATTERY_MOTIONBATTERY_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_MOTIONBATTERY_MOTIONBATTERY_H_

#include <scrimmage/motion/Controller.h>
#include <scrimmage/common/Battery.h>

#include <map>
#include <string>
#include <memory>

namespace scrimmage {
namespace controller {

class MotionBattery : public scrimmage::Controller {
 public:
    // enum class DepletionEquation {LINEAR};

    void init(std::map<std::string, std::string> &params) override;
    bool step(double t, double dt) override;

 protected:
    bool get_battery_charge(scrimmage::MessageBasePtr request,
                            scrimmage::MessageBasePtr &response);
    double calculate_charge_usage(const double &throttle, const double &dt);

    Battery battery_;

    class VarLimit {
     public:
        VarLimit(const int &input, const int &output, const double &rate,
                 const double &value, const bool &lim) :
            input_idx(input), output_idx(output), depletion_rate(rate),
            value_when_depleted(value), limit_when_depleted(lim) {}
        int input_idx = 0;
        int output_idx = 0;
        double depletion_rate = 0;
        double value_when_depleted = 0;
        bool limit_when_depleted = false;

        double calc_depletion(const double &input, const double &dt) {
            return input * depletion_rate * dt;
        }
    };
    // Key: variable name
    // Value: VarLimit class unique_ptr with properties
    std::map<std::string, std::unique_ptr<VarLimit>> io_map_;

    bool publish_charge_ = false;
    PublisherPtr pub_charge_percentage_;
};
} // namespace controller
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_MOTIONBATTERY_MOTIONBATTERY_H_
