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
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/ScrimmageOpenAIAutonomy.h>
#include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/ActorFunc.h>
#include <scrimmage/plugins/sensor/ScrimmageOpenAISensor/ScrimmageOpenAISensor.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/sensor/Sensor.h>
#include <scrimmage/parse/MissionParse.h>

#include <limits>

#if ENABLE_GRPC
#include <grpc++/impl/codegen/status_code_enum.h>
using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
#include <chrono> //NOLINT
#include <thread> //NOLINT
#endif

namespace sp = scrimmage_proto;
namespace py = pybind11;

REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::ScrimmageOpenAIAutonomy, ScrimmageOpenAIAutonomy_plugin)

namespace scrimmage {
namespace autonomy {

ScrimmageOpenAIAutonomy::ScrimmageOpenAIAutonomy() :
    reward_range(-std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity()),
    tuple_space_(get_gym_space("Tuple")),
    box_space_(get_gym_space("Box")),
    nonlearning_mode_(false) {}

void ScrimmageOpenAIAutonomy::init(std::map<std::string, std::string> &params) {
    init_helper(params);
    nonlearning_mode_ = get("nonlearning_mode_openai_plugin", params, true);
    self_id_ = parent_->id().id();
    #if ENABLE_GRPC
    // In case we have multiple agents, add the id to the port number
    grpc_mode_ = get("grpc_mode", params, grpc_mode_);
    std::string module = get("module", params, "my_module");
    std::string actor_func_str = get("actor_init_func", params, "my_func");

    if (grpc_mode_) {
        const size_t port = get("port", params, 50050);
        const std::string grpc_address_ = get("grpc_address", params, "127.0.0.1");

        const std::string port_str = std::to_string(port + parent_->id().id());
        const std::string address = grpc_address_ + ":" + port_str;

        python_cmd_ = std::string("openai_grpc_link.py") + " --port " + port_str
            + " --actor " + module + ":" + actor_func_str
            + " --ip " + grpc_address_;
        auto channel = grpc::CreateChannel(address, grpc::InsecureChannelCredentials());
        openai_stub_ = sp::OpenAI::NewStub(channel);
    }
    #endif
    params_ = params;
}

bool ScrimmageOpenAIAutonomy::step_autonomy(double t, double /*dt*/) {
    if (nonlearning_mode_) {
        if (pub_reward_ == nullptr) {
            // need contacts size to be set before calling init
            auto p = std::dynamic_pointer_cast<ScrimmageOpenAIAutonomy>(shared_from_this());
            std::tie(actions_, observations_, actor_func_) =
                init_actor_func({p}, params_, CombineActors::NO, UseGlobalSensor::NO,
                grpc_mode_);

            pub_reward_ = advertise("GlobalNetwork", "reward");
            #if ENABLE_GRPC
            if (grpc_mode_) {
                int attempts = 0;
                bool done = false;
                while (attempts++ < 10 && !done) {
                    std::cout << "Connecting to gRPC Server attempt: " << attempts << "/10" << std::endl;
                    done = send_env();
                }
                // Return an exception saying we could not connect to grpc
                if (!done) {
                    const std::string err_msg =
                        std::string("Didn't connect to the grpc server. Make sure")
                        + " the grpc client is running and your python"
                        + " bindings are up to date."
                        + "You can start a grpc server with the followning command: "
                        + python_cmd_;
                    throw std::runtime_error(err_msg);
                }
            }
            #endif
        }
        const size_t num_entities = 1;
        observations_.update_observation(num_entities);
        py::object temp_action;
        if (grpc_mode_) {
            #if ENABLE_GRPC
            boost::optional<sp::Action> proto_act;

            // Convert observation to proto message
            sp::Obs proto_obs = obs_to_proto();
            // Send over GRPC
            proto_act = get_action(proto_obs);
            // Convert proto action back to python action
            if (proto_act) {
                temp_action = convert_proto_action(*proto_act);
            } else {
                std::cout << "Bad Action received!" << std::endl;
            }
            #endif
        } else {
            // Get action from actor function called through pybind
            temp_action = actor_func_(observations_.observation);
        }
        bool combine_actors = false;
        actions_.distribute_action(temp_action, combine_actors);

        if (first_step_) {
            // there is no reward on the first step
            first_step_ = false;
        } else {
            bool done;
            double reward;
            py::dict info;
            std::tie(done, reward, info) = calc_reward();
            if (done) {
                return false;
            }
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

#if ENABLE_GRPC
boost::optional<sp::Action> ScrimmageOpenAIAutonomy::get_action(sp::Obs &observation) {
    if (!openai_stub_) return boost::none;

    sp::Action action;
    grpc::ClientContext context;
    auto deadline = std::chrono::system_clock::now() +
                          std::chrono::milliseconds(500);
    context.set_deadline(deadline);
    grpc::Status status = openai_stub_->GetAction(&context, observation, &action);
    if (!(status.error_code() == grpc::StatusCode::OK ||
        status.error_code() == grpc::StatusCode::UNKNOWN)) {
        return boost::none;
    } else if (action.done()) {
        return boost::none;
    } else {
        return boost::optional<sp::Action>(action);
    }
}

bool ScrimmageOpenAIAutonomy::send_env() {
    sp::Environment env;
    // Setup action space parameters
    auto discrete_space = env.mutable_action_spaces()->mutable_discrete();
    *(discrete_space) = {action_space.discrete_count.begin(), action_space.discrete_count.end()};
    for (std::pair<double, double> cont_extrema : action_space.continuous_extrema) {
        sp::ContExtrema * proto_extrema = env.mutable_action_spaces()->add_continuous();
        proto_extrema->set_min(cont_extrema.first);
        proto_extrema->set_max(cont_extrema.second);
    }
    // Setup observation space parameters
    // This for loop should only occur once as the agent should only know about its own sensors
    for (auto agent : observations_.ext_sensor_vec()) {
        // For every sensor the agent has
        for (auto sensor : agent) {
            auto obs_space = sensor->observation_space;
            // Add discrete observation spaces to the protobuf environment
            for (int discrete_obs : obs_space.discrete_count) {
                env.mutable_observation_spaces()->add_discrete(discrete_obs);
            }
            // Add continuous/ observation spaces to the protobuf environment
            for (std::pair<double, double> cont_extrema : obs_space.continuous_extrema) {
                sp::ContExtrema * proto_extrema = env.mutable_observation_spaces()->add_continuous();
                proto_extrema->set_min(cont_extrema.first);
                proto_extrema->set_max(cont_extrema.second);
            }
        }
    }
    // Send over params through GRPC
    std::vector<std::string> necessary_params = {"logdir", "hyperparams_file"};
    auto &proto_map = *env.mutable_params();
    for (std::string param_key : necessary_params) {
        auto search = params_.find(param_key);
        if (search != params_.end()) {
            proto_map[param_key] = search->second;
        }
    }

    if (!openai_stub_) return false;
    sp::Empty reply;

    grpc::ClientContext context;
    auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(2);
    context.set_deadline(deadline);
    grpc::Status status = openai_stub_->SendEnvironment(&context, env, &reply);
    if (!status.ok()) {
        std::cout << "Can't Connect: message " << status.error_code()
                  << ", " << status.error_message() << std::endl;
    }
    return status.ok();
}

sp::Obs ScrimmageOpenAIAutonomy::obs_to_proto() {
    // Convert observation to proto message
    sp::Obs proto_obs;

    if (PyObject_IsInstance(observations_.observation_space.ptr(), tuple_space_.ptr())) {
        for (py::handle disc_val : observations_.observation.cast<py::list>()[0]) {
            proto_obs.add_discrete(disc_val.cast<int>());
        }

        for (py::handle cont_val : observations_.observation.cast<py::list>()[1]) {
            proto_obs.add_continuous(cont_val.cast<double>());
        }
    } else if (PyObject_IsInstance(observations_.observation_space.ptr(), box_space_.ptr())) {
        for (py::handle cont_val : observations_.observation.cast<py::list>()) {
            proto_obs.add_continuous(cont_val.cast<double>());
        }
    } else {
        for (py::handle disc_val : observations_.observation.cast<py::list>()) {
            proto_obs.add_discrete(disc_val.cast<int>());
        }
    }
    return proto_obs;
}

pybind11::object ScrimmageOpenAIAutonomy::convert_proto_action(const sp::Action &proto_act) {
    pybind11::list py_act;
    std::vector<int> disc(proto_act.discrete_size(), 0);
    std::vector<double> cont(proto_act.continuous_size(), 0);

    for (int i = 0; i < proto_act.discrete_size(); i++) {
        disc[i] = proto_act.discrete(i);
    }

    for (int i = 0; i < proto_act.continuous_size(); i++) {
        cont[i] = proto_act.continuous(i);
    }

    if (PyObject_IsInstance(actions_.action_space.ptr(), tuple_space_.ptr())) {
        py_act.attr("append")(py::cast(disc));
        py_act.attr("append")(py::cast(cont));
    } else if (PyObject_IsInstance(actions_.action_space.ptr(), box_space_.ptr())) {
        py_act = py::cast(cont);
    } else {
        py_act = py::cast(disc);
    }
    return py_act;
}
#endif

std::tuple<bool, double, pybind11::dict> ScrimmageOpenAIAutonomy::calc_reward() {
    return std::make_tuple(false, 0.0, pybind11::dict());
}

} // namespace autonomy
} // namespace scrimmage
