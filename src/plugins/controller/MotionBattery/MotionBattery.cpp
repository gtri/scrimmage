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

#include <scrimmage/plugins/controller/MotionBattery/MotionBattery.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/msgs/Battery.pb.h>
#include <scrimmage/pubsub/Publisher.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;
namespace sm = scrimmage_msgs;
namespace pl = std::placeholders;

REGISTER_PLUGIN(scrimmage::Controller,
                scrimmage::controller::MotionBattery,
                MotionBattery_plugin)

namespace scrimmage {
namespace controller {

void MotionBattery::init(std::map<std::string, std::string> &params) {
    // Discover the output variables that point to the input variables for the
    // motion model. Setup input variables that mirror the output variables.
    for (auto &kv : vars_.output_variable_index()) {
        int out_idx = vars_.declare(kv.first, VariableIO::Direction::Out);
        int in_idx = vars_.declare(kv.first, VariableIO::Direction::In);
        io_map_[kv.first] = std::make_unique<VarLimit>(in_idx, out_idx, 0.0, 0.0, false);
    }

    double charge = sc::get<double>("charge_initial", params, 1.0);
    double charge_max = sc::get<double>("charge_max", params, 1.0);
    double charge_min = sc::get<double>("charge_min", params, 0.0);
    battery_ = Battery(charge_min, charge_max, charge);

    std::vector<std::vector<std::string>> vecs;
    if (get_vec_of_vecs(sc::get<std::string>("depletion_map", params, ""),
                        vecs)) {
        for (auto &vec : vecs) {
            if (vec.size() != 5) {
                cout << "Invalid depletion mapping: " << endl;
                for (std::string s : vec) {
                    cout << s << " ";
                }
                continue;
            }

            std::string name = vec[0];
            auto it_name = io_map_.find(name);
            if (it_name == io_map_.end()) {
                cout << "MotionBattery: Invalid variable name: " << name << endl;
            } else {
                it_name->second->depletion_rate = std::stod(vec[1]);
                it_name->second->limit_when_depleted = str2bool(vec[3]);
                it_name->second->value_when_depleted = std::stod(vec[4]);
            }
        }
    } else {
        cout << "Failed to parse depletion_map" << endl;
    }

    // Charging subscriber
    auto callback_charge_added = [&] (scrimmage::MessagePtr<sm::Charge> msg) {
        if (parent_->id().id() == msg->data.id()) {
            battery_.add_charge(msg->data.charge_amount());
        }
    };
    subscribe<sm::Charge>("GlobalNetwork", "ChargeAdded", callback_charge_added);

    // Service call to get battery charge
    parent_->services()["get_battery_charge"] =
        std::bind(&MotionBattery::get_battery_charge, this, pl::_1, pl::_2);

    publish_charge_ = sc::get<bool>("publish_charge", params, publish_charge_);
    pub_charge_percentage_ = advertise("LocalNetwork", "ChargePercentage");
}

bool MotionBattery::step(double t, double dt) {
    if (publish_charge_) {
        auto msg = std::make_shared<sc::Message<double>>(battery_.charge_percentage());
        pub_charge_percentage_->publish(msg);
    }

    // Loop over all variables in the io_map_, apply charge calculation, and
    // limit output if required
    for (auto &kv : io_map_) {
        double value = vars_.input(kv.second->input_idx);
        double depletion = kv.second->calc_depletion(value, time_->dt());
        if (!battery_.deplete(depletion)) {
            // If deplete() returns false, the battery is depleted.
            parent()->collision();
            if (kv.second->limit_when_depleted) {
                // Limit the output value if specified
                value = kv.second->value_when_depleted;
            }
        }
        vars_.output(kv.second->output_idx, value);
    }
    return true;
}

bool MotionBattery::get_battery_charge(scrimmage::MessageBasePtr request,
                                       scrimmage::MessageBasePtr &response) {
    response =
        std::make_shared<sc::Message<double>>(battery_.current_charge());
    return true;
}

double MotionBattery::calculate_charge_usage(const double &throttle, const double &dt) {
    return 0.0;
}

} // namespace controller
} // namespace scrimmage
