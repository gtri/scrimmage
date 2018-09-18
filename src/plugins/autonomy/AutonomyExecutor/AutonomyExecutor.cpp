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

#include <scrimmage/plugins/autonomy/AutonomyExecutor/AutonomyExecutor.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ConfigParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/PluginManager.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/common/Time.h>

#include <iostream>
#include <limits>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::AutonomyExecutor,
                AutonomyExecutor_plugin)

namespace scrimmage {
namespace autonomy {

void AutonomyExecutor::init(std::map<std::string, std::string> &params) {
    show_shapes_ = sc::get("show_shapes", params, false);

    // Discover the output variables that point to the input variables for the
    // motion model. Setup input variables that mirror the output variables.
    for (auto &kv : vars_.output_variable_index()) {
        int out_idx = vars_.declare(kv.first, VariableIO::Direction::Out);
        int in_idx = vars_.declare(kv.first, VariableIO::Direction::In);
        io_map_[out_idx] = in_idx;
    }

    // Subscribe to state information
    std::string state_topic_name = sc::get<std::string>("state_topic_name", params, "State");
    std::string network_name = sc::get<std::string>("network_name", params, "LocalNetwork");

    auto state_callback = [&] (scrimmage::MessagePtr<std::string> msg) {
        current_state_ = msg->data;

        // Get the currently running autonomies
        auto autonomy_list = autonomies_.find(current_state_);
        if (autonomy_list != autonomies_.end()) {
            running_autonomies_ = autonomy_list->second;
        } else {
            running_autonomies_ = default_autonomies_;
        }
    };
    subscribe<std::string>(network_name, state_topic_name, state_callback);

    // Parse the autonomy plugins
    std::string autonomies_str = sc::get<std::string>("autonomies", params, "");
    std::vector<std::vector<std::string>> vecs_of_vecs;
    sc::get_vec_of_vecs(autonomies_str, vecs_of_vecs, " ");
    for (std::vector<std::string> vecs : vecs_of_vecs) {
        if (vecs.size() < 1) {
            std::cout << "Autonomy name missing." << std::endl;
            continue;
        }

        std::string autonomy_name = "";
        std::map<std::string, std::string> autonomy_params;
        int i = 0;
        for (std::string str : vecs) {
            if (i == 0) {
                autonomy_name = str;
            } else {
                // Parse the autonomy parameters (e.g., param_name='value')
                // Split the param_name and value, with equals sign in between
                std::vector<std::string> tokens;
                boost::split(tokens, str, boost::is_any_of("="));
                if (tokens.size() == 2) {
                    // Remove the quotes from the value
                    tokens[1].erase(
                        std::remove_if(tokens[1].begin(),
                                       tokens[1].end(),
                                       [](unsigned char x){
                                           return (x == '\"') || (x == '\'');
                                       }),
                        tokens[1].end());
                    autonomy_params[tokens[0]] = tokens[1];
                }
            }
            ++i;
        }

        sc::ConfigParse config_parse;
        PluginStatus<Autonomy> status =
            parent_->plugin_manager()->make_plugin<Autonomy>("scrimmage::Autonomy",
                                                             autonomy_name,
                                                             *(parent_->file_search()),
                                                             config_parse,
                                                             autonomy_params,
                                                             std::set<std::string>{});
        if (status.status == PluginStatus<Autonomy>::cast_failed) {
            cout << "AutonomyExecutor Failed to open autonomy plugin: "
                 << autonomy_name << endl;
        } else if (status.status == PluginStatus<Autonomy>::loaded) {
            // connect the initialized autonomy plugin's variable output with
            // the variable input to the controller. This is similar to
            // connect(autonomy->vars(), controller->vars());
            // but without the reference to the controller->vars()
            AutonomyPtr autonomy = status.plugin;
            autonomy->vars().output_variable_index() = vars_.output_variable_index();
            autonomy->vars().set_output(vars_.output());

            // Initialize the autonomy
            autonomy->set_rtree(rtree_);
            autonomy->set_parent(parent_);
            autonomy->set_projection(proj_);
            autonomy->set_pubsub(parent_->pubsub());
            autonomy->set_time(time_);
            autonomy->set_state(state_);
            autonomy->set_contacts(contacts_);
            autonomy->set_is_controlling(true);
            autonomy->set_name(autonomy_name);
            autonomy->init(config_parse.params());

            // Determine which states in which this autonomy is active
            std::vector<std::string> states;
            if (sc::get_vec("states", config_parse.params(), " ,", states)) {
                // This is a autonomy that only runs in these states
                for (std::string state : states) {
                    autonomies_[state].push_back(autonomy);
                }
            } else {
                default_autonomies_.push_back(autonomy);
            }
        }
    }

    // Add the default autonomies to each declared state
    for (scrimmage::AutonomyPtr autonomy : default_autonomies_) {
        for (auto &kv : autonomies_) {
            kv.second.push_back(autonomy);
        }
    }

    running_autonomies_ = default_autonomies_;
}

bool AutonomyExecutor::step_autonomy(double t, double dt) {
    for (sc::AutonomyPtr autonomy : running_autonomies_) {
        // cout << "Running... " << autonomy->name() << endl;
        autonomy->shapes().clear();

        for (SubscriberBasePtr &sub : autonomy->subs()) {
            for (auto msg : sub->pop_msgs<sc::MessageBase>()) {
                sub->accept(msg);
            }
        }
        if (!autonomy->step_autonomy(time_->t(), time_->dt())) {
            cout << "AutonomyExecutor: autonomy error- " << autonomy->name();
        }

        if (show_shapes_) {
            std::for_each(autonomy->shapes().begin(),
                          autonomy->shapes().end(), [&](auto &s) {
                              this->draw_shape(s);
                          });
        }
    }
    return true;
}
} // namespace autonomy
} // namespace scrimmage
