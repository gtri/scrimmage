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

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::MotorSchemas,
                MotorSchemas_plugin)

namespace scrimmage {
namespace autonomy {

void MotorSchemas::init(std::map<std::string, std::string> &params) {
    // Parse the behavior plugins
    std::string behaviors_str = sc::get<std::string>("behaviors", params, "");
    std::vector<std::vector<std::string>> vecs_of_vecs;
    sc::get_vec_of_vecs(behaviors_str, vecs_of_vecs);
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
        motor_schemas::BehaviorBasePtr behavior =
            std::dynamic_pointer_cast<motor_schemas::BehaviorBase>(
                parent_->plugin_manager()->make_plugin("scrimmage::Autonomy",
                                                       behavior_name,
                                                       *(parent_->file_search()),
                                                       config_parse,
                                                       behavior_params));
        if (behavior == nullptr) {
            cout << "Failed to load MotorSchemas behavior: " << behavior_name << endl;
        } else {
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
            behaviors_.push_back(behavior);
        }
    }

    show_shapes_ = sc::get("show_shapes", params, false);
    max_speed_ = sc::get<double>("max_speed", params, 21);

    desired_alt_idx_ = vars_.declare(VariableIO::Type::desired_altitude, VariableIO::Direction::Out);
    desired_speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::Out);
    desired_heading_idx_ = vars_.declare("heading", VariableIO::Direction::Out);
}

bool MotorSchemas::step_autonomy(double t, double dt) {
    shapes_.clear();

    // Run all sub behaviors
    std::list<Eigen::Vector3d> vecs_with_gains;
    double sum_norms = 0;
    Eigen::Vector3d vec_sums(0, 0, 0);
    for (motor_schemas::BehaviorBasePtr &behavior : behaviors_) {
        behavior->shapes().clear();

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

        // Grab the desired vector
        Eigen::Vector3d vec_with_gain = behavior->desired_vector() * behavior->gain();
        vecs_with_gains.push_back(vec_with_gain);

        // Keep a running sum of the norms of all vectors with gains
        sum_norms += vec_with_gain.norm();

        // Keep a running sum of all vectors with gains
        vec_sums += vec_with_gain;

        if (show_shapes_) {
            // Grab the behavior shapes:
            shapes_.insert(shapes_.end(), behavior->shapes().begin(),
                           behavior->shapes().end());
        }
        behavior->shapes().clear();
    }

    Eigen::Vector3d v_sum = vec_sums / sum_norms;

    // Scale velocity to max speed:
    Eigen::Vector3d vel_result = v_sum * max_speed_;

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
    // Draw sphere of influence:
    if (show_shapes_) {
        // Draw resultant vector:
        auto arrow = std::make_shared<scrimmage_proto::Shape>();
        arrow->set_type(scrimmage_proto::Shape::Line);
        sc::set(arrow->mutable_color(), 255, 255, 0);
        arrow->set_opacity(0.75);
        sc::add_point(arrow, state_->pos());
        sc::add_point(arrow, vel_result + state_->pos());
        shapes_.push_back(arrow);
    }

    return true;
}
} // namespace autonomy
} // namespace scrimmage
