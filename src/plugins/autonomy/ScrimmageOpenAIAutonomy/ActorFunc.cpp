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
#include <pybind11/stl.h>

#include <scrimmage/entity/Entity.h>
#include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/ActorFunc.h>
#include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/OpenAIActions.h>
#include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/OpenAIObservations.h>
#include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/ScrimmageOpenAIAutonomy.h>

#include <boost/range/algorithm/copy.hpp>

namespace py = pybind11;
namespace br = boost::range;

namespace scrimmage {
namespace autonomy {

std::tuple<OpenAIActions, OpenAIObservations, pybind11::object>
init_actor_func(
        std::vector<std::shared_ptr<ScrimmageOpenAIAutonomy>> autonomies,
        const std::map<std::string, std::string> &params,
        CombineActors combine_actors,
        UseGlobalSensor global_sensor,
        bool grpc_mode) {

    if (autonomies.empty()) {
        return std::make_tuple(OpenAIActions(), OpenAIObservations(), pybind11::none());
    }

    OpenAIActions actions;
    br::copy(autonomies, std::back_inserter(actions.ext_ctrl_vec()));
    actions.create_action_space(combine_actors == CombineActors::YES);

    OpenAIObservations observations;
    observations.set_combine_actors(combine_actors == CombineActors::YES);
    observations.set_global_sensor(global_sensor == UseGlobalSensor::YES);

    for (auto autonomy : autonomies) {
        observations.add_sensors(autonomy->parent()->sensors());
        if (global_sensor == UseGlobalSensor::YES) break;
    }

    observations.create_observation_space(autonomies.size());
    py::object actor_func = py::none();

    if (!grpc_mode) {
        // get the actor func
        const std::string module_str = params.at("module");
        const std::string actor_init_func_str = params.at("actor_init_func");
        py::object m = py::module::import(module_str.c_str());
        py::object actor_init_func =
            m.attr(actor_init_func_str.c_str());

        actor_func = actor_init_func(
            actions.action_space, observations.observation_space, params);
    }
    return std::make_tuple(actions, observations, actor_func);
}
} // namespace autonomy
} // namespace scrimmage
