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

#include <pybind11/pybind11.h>

#include <scrimmage/common/Time.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/ScrimmageOpenAIAutonomy.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/math/State.h>
#include <scrimmage/sensor/Sensor.h>

#include <iostream>
#include <limits>

#include <boost/algorithm/string.hpp>

#include <chrono> // NOLINT
#include <thread> // NOLINT

namespace sp = scrimmage_proto;
namespace py = pybind11;

REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::ScrimmageOpenAIAutonomy, ScrimmageOpenAIAutonomy_plugin)

namespace scrimmage {
namespace autonomy {

ScrimmageOpenAIAutonomy::ScrimmageOpenAIAutonomy() :
    reward_range(-std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity()),
    nonlearning_mode_(false) {}

void ScrimmageOpenAIAutonomy::init(std::map<std::string, std::string> &params) {
    init_helper(params);

    nonlearning_mode_ = get("nonlearning_mode_openai_plugin", params, true);

    if (nonlearning_mode_) {
        // get the actor func
        const std::string module = params.at("module");
        actor_func_ = py::module::import(module.c_str());

        std::vector<std::string> tokens;
        split(tokens, params.at("actor_func"), ".");
        for (std::string s : tokens) {
            actor_func_ = actor_func_.attr(py::str(s));
        }

        // there is only 1 entity in this case so it would make no sense
        // to combine actors
        const size_t num_entities = 1;

        auto p = std::dynamic_pointer_cast<ScrimmageOpenAIAutonomy>(shared_from_this());
        actions_.ext_ctrl_vec().push_back(p);
        actions_.create_action_space(false);

        observations_.set_combine_actors(false);
        observations_.set_global_sensor(false);

        observations_.add_sensors(parent_->sensors());

        observations_.create_observation_space(num_entities);
    }

    pub_reward_ = advertise("GlobalNetwork", "reward");
}

bool ScrimmageOpenAIAutonomy::step_autonomy(double /*t*/, double /*dt*/) {
    if (nonlearning_mode_) {
        const size_t num_entities = 1;
        observations_.update_observation(num_entities);
        py::object temp_action = actor_func_(observations_.observation);

        bool combine_actors = false;
        actions_.distribute_action(temp_action, combine_actors);

        if (first_step_) {
            // there is no reward on the first step
            first_step_ = false;
        } else {
            bool done;
            double reward;
            std::tie(done, reward) = calc_reward();
            if (done) return false;
            if (reward != 0) {
                auto msg = std::make_shared<Message<std::pair<size_t, double>>>();
                msg->data.first = parent_->id().id();
                msg->data.second = reward;
                pub_reward_->publish(msg);
            }
        }
    }

    return step_helper();
}

std::pair<bool, double> ScrimmageOpenAIAutonomy::calc_reward() {
    return {false, 0.0};
}

} // namespace autonomy
} // namespace scrimmage
