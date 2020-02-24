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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_SCRIMMAGEOPENAIAUTONOMY_SCRIMMAGEOPENAIAUTONOMY_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_SCRIMMAGEOPENAIAUTONOMY_SCRIMMAGEOPENAIAUTONOMY_H_

#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/common/Visibility.h>
#include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/OpenAIObservations.h>
#include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/OpenAIActions.h>
#include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/OpenAIUtils.h>

#include <pybind11/pybind11.h>

#if ENABLE_GRPC
#include <grpc++/grpc++.h>
#include <scrimmage/network/ScrimmageServiceImpl.h>
#include <scrimmage/proto/OpenAI.grpc.pb.h>
#endif

#include <map>
#include <vector>
#include <string>
#include <utility>
#include <tuple>
#include <memory>

namespace scrimmage {

struct EnvParams {
    std::vector<int> discrete_count;
    std::vector<std::pair<double, double>> continuous_extrema;
};

struct EnvValues {
    std::vector<int> discrete;
    std::vector<double> continuous;
};

namespace autonomy {

class DLL_PUBLIC ScrimmageOpenAIAutonomy : public scrimmage::Autonomy {
 public:
    ScrimmageOpenAIAutonomy();

    // normal overrides
    void init(std::map<std::string, std::string> &params) final;
    bool step_autonomy(double t, double dt) final;

    // new overrides
    virtual void init_helper(std::map<std::string, std::string> &/*params*/) {}
    virtual bool step_helper() {return true;}

    virtual void set_environment() {}
    virtual std::tuple<bool, double, pybind11::dict> calc_reward();
    std::pair<double, double> reward_range;
    EnvParams action_space;
    EnvValues action;
    int self_id() {return self_id_;}

 protected:
#if ENABLE_GRPC
    std::unique_ptr<scrimmage_proto::OpenAI::Stub> openai_stub_;
    std::unique_ptr<grpc::Server> server_;
    boost::optional<scrimmage_proto::Action> get_action(scrimmage_proto::Obs &observation);
    pybind11::object convert_proto_action(const scrimmage_proto::Action &proto_act);
    scrimmage_proto::Obs obs_to_proto();
    bool send_env();

    std::string python_cmd_;
#endif
    std::map<std::string, std::string> params_;
    pybind11::object tuple_space_, box_space_;
    int self_id_ = 0;
    bool first_step_ = true;
    PublisherPtr pub_reward_ = nullptr;
    pybind11::object actor_func_ = pybind11::none();
    bool nonlearning_mode_ = false;
    OpenAIObservations observations_;
    OpenAIActions actions_;
#if ENABLE_GRPC
    bool grpc_mode_ = true;
#else
    bool grpc_mode_ = false;
#endif
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_SCRIMMAGEOPENAIAUTONOMY_SCRIMMAGEOPENAIAUTONOMY_H_
