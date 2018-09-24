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

#include <Eigen/Geometry>

#include <scrimmage/common/Utilities.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/common/ParameterServer.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/parse/ConfigParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugin_manager/PluginManager.h>

#include <scrimmage/plugins/autonomy/MotorSchemas/MotorSchemas.h>
#include <scrimmage/plugins/autonomy/MotorSchemas/BehaviorBase.h>

#include <iostream>
#include <string>
#include <cmath>
#include <cfloat>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

using std::cout;
using std::endl;

namespace sc = scrimmage;
namespace sp = scrimmage_proto;
namespace ms = scrimmage::autonomy::motor_schemas;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::MotorSchemas,
                MotorSchemas_plugin)

namespace scrimmage {
namespace autonomy {

void MotorSchemas::init(std::map<std::string, std::string> &params) {
    show_shapes_ = sc::get("show_shapes", params, false);
    max_speed_ = sc::get<double>("max_speed", params, 21);

    auto max_speed_cb = [&] (const double &max_speed) {
        cout << "MotorSchemas Max speed set: " << max_speed << endl;
    };
    register_param<double>("max_speed", max_speed_, max_speed_cb);

    // Subscribe to state information
    std::string state_topic_name = sc::get<std::string>("state_topic_name", params, "State");
    std::string network_name = sc::get<std::string>("network_name", params, "LocalNetwork");
    auto state_callback = [&] (scrimmage::MessagePtr<std::string> msg) {
        current_state_ = msg->data;

        // Get the currently running behaviors
        auto behavior_list = behaviors_.find(current_state_);
        if (behavior_list != behaviors_.end()) {
            current_behaviors_ = behavior_list->second;
        } else {
            current_behaviors_ = default_behaviors_;
        }
    };
    subscribe<std::string>(network_name, state_topic_name, state_callback);

    // Parse the behavior plugins
    std::string behaviors_str = sc::get<std::string>("behaviors", params, "");
    std::vector<std::vector<std::string>> vecs_of_vecs;
    sc::get_vec_of_vecs(behaviors_str, vecs_of_vecs, " ");
    for (std::vector<std::string> vecs : vecs_of_vecs) {
        if (vecs.size() < 1) {
            std::cout << "Behavior name missing." << std::endl;
            continue;
        }

        std::string behavior_name = "";
        std::map<std::string, std::string> behavior_params;
        int i = 0;
        for (std::string str : vecs) {
            if (i == 0) {
                behavior_name = str;
            } else {
                // Parse the behavior parameters (e.g., param_name="value")
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
                    behavior_params[tokens[0]] = tokens[1];
                }
            }
            ++i;
        }

        sc::ConfigParse config_parse;
        PluginStatus<ms::BehaviorBase> status =
            parent_->plugin_manager()->make_plugin<ms::BehaviorBase>(
                "scrimmage::Autonomy",
                behavior_name,
                *(parent_->file_search()),
                config_parse,
                behavior_params,
                std::set<std::string>{});
        if (status.status == PluginStatus<ms::BehaviorBase>::cast_failed) {
            cout << "Failed to load MotorSchemas behavior: " << behavior_name << endl;
        } else if (status.status == PluginStatus<ms::BehaviorBase>::loaded) {
            ms::BehaviorBasePtr behavior = status.plugin;
            // Initialize the autonomy/behavior
            behavior->set_rtree(rtree_);
            behavior->set_parent(parent_);
            behavior->set_projection(proj_);
            behavior->set_pubsub(parent_->pubsub());
            behavior->set_time(time_);
            behavior->set_state(state_);
            behavior->set_contacts(contacts_);
            behavior->set_is_controlling(true);
            behavior->set_name(behavior_name);
            behavior->init(config_parse.params());

            // Extract the gain for this plugin and apply it
            behavior->set_gain(sc::get<double>("gain", behavior_params, 1.0));
            behavior->set_max_vector_length(max_speed_);

            // cout << "Behavior: " << behavior->name() << endl;

            // Determine which states in which this behavior is active
            std::vector<std::string> states;
            if (sc::get_vec("states", config_parse.params(), " ,", states)) {
                // cout << "adding to... " << endl;
                // This is a behavior that only runs in these states
                for (std::string state : states) {
                    // cout << state << " ";
                    behaviors_[state].push_back(behavior);
                }
                // cout << endl;
            } else {
                default_behaviors_.push_back(behavior);
            }
        }
    }

    // Add the default behaviors to each declared state
    for (motor_schemas::BehaviorBasePtr behavior : default_behaviors_) {
        for (auto &kv : behaviors_) {
            kv.second.push_back(behavior);
        }
    }

    current_behaviors_ = default_behaviors_;

    desired_alt_idx_ = vars_.declare(VariableIO::Type::desired_altitude, VariableIO::Direction::Out);
    desired_speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::Out);
    desired_heading_idx_ = vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::Out);
}

bool MotorSchemas::step_autonomy(double t, double dt) {
    // Run all sub behaviors
    double vec_w_gain_sum = 0;
    Eigen::Vector3d vec_w_gain(0, 0, 0);

    for (motor_schemas::BehaviorBasePtr &behavior : current_behaviors_) {
        behavior->shapes().clear();

        // cout << "Behavior: " << behavior->name() << endl;

        // Execute callbacks for received messages before calling
        // step_autonomy
        for (SubscriberBasePtr &sub : behavior->subs()) {
            for (auto msg : sub->pop_msgs<sc::MessageBase>()) {
                sub->accept(msg);
            }
        }
        if (!behavior->step_autonomy(time_->t(), time_->dt())) {
            cout << "MotorSchemas: behavior error" << endl;
        }

        // Grab the desired vector and normalize to max_speed if too large
        Eigen::Vector3d desired_vector = behavior->desired_vector();
        if (desired_vector.norm() > max_speed_) {
            desired_vector = desired_vector.normalized() * max_speed_;
        }

        if (desired_vector.hasNaN()) {
            cout << "Behavior error: " << behavior->name()
                 << ", desired vector has NaN" << endl;
            continue;
        }

        // cout << "desired_vector: " << desired_vector << endl;
        // cout << "gain: " << behavior->gain() << endl;
        // cout << "desired_vector.norm(): " << desired_vector.norm() << endl;

        // Keep a running sum of all vectors with gains
        vec_w_gain += desired_vector * behavior->gain();

        // Keep a running sum of all gains that have an effective vector
        Eigen::Vector3d desired_vector_normalized = desired_vector.normalized();
        if (!desired_vector_normalized.hasNaN()) {
            vec_w_gain_sum += desired_vector.normalized().norm() * behavior->gain();
        }

        if (show_shapes_) {
             std::for_each(behavior->shapes().begin(),
                           behavior->shapes().end(), [&](auto &s) {
                               this->draw_shape(s);
                           });
        }
        behavior->shapes().clear();
    }

    Eigen::Vector3d vel_result = vec_w_gain / vec_w_gain_sum;

    // If the vel_result has a NaN value, just go straight. NaNs occur during
    // initialization of some behaviors.
    if (vel_result.hasNaN()) {
        vel_result = state_->quat() * Eigen::Vector3d::UnitX() * max_speed_;
    }

    ///////////////////////////////////////////////////////////////////////////
    // Convert resultant vector into heading / speed / altitude command:
    ///////////////////////////////////////////////////////////////////////////

    double heading = sc::Angles::angle_2pi(atan2(vel_result(1), vel_result(0)));
    vars_.output(desired_alt_idx_, state_->pos()(2) + vel_result(2));
    vars_.output(desired_speed_idx_, vel_result.norm());
    vars_.output(desired_heading_idx_, heading);

    ///////////////////////////////////////////////////////////////////////////
    // Draw important shapes
    ///////////////////////////////////////////////////////////////////////////
    if (show_shapes_) {
        // Draw resultant vector:
        line_shape_->set_persistent(true);
        sc::set(line_shape_->mutable_color(), 255, 255, 0);
        line_shape_->set_opacity(0.75);
        sc::set(line_shape_->mutable_line()->mutable_start(), state_->pos());
        sc::set(line_shape_->mutable_line()->mutable_end(), vel_result + state_->pos());
        draw_shape(line_shape_);
    }
    return true;
}
} // namespace autonomy
} // namespace scrimmage
