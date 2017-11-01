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
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/sensor/Sensor.h>
#include <scrimmage/proto/ExternalControl.pb.h>
#include <scrimmage/common/RTree.h>


#include <scrimmage/parse/MissionParse.h>

#include <scrimmage/plugins/autonomy/TutorialOpenAIAutonomy/TutorialOpenAIAutonomy.h>

namespace sc = scrimmage;
namespace sp = scrimmage_proto;

REGISTER_PLUGIN(scrimmage::Autonomy, TutorialOpenAIAutonomy, TutorialOpenAIAutonomy_plugin)

void TutorialOpenAIAutonomy::init(std::map<std::string, std::string> &params) {
    desired_state_->pos()(2) = state_->pos()(2);
    ExternalControl::init(params);
}

double TutorialOpenAIAutonomy::calc_reward(double time) {
    double reward = 0.0;

    for (auto &kv : parent_->mp()->team_info()) {
        // same team
        if (kv.first == parent_->id().team_id()) {
            continue;
        }

        // For each base
        int i = 0;
        for (Eigen::Vector3d &base_pos : kv.second.bases) {
            Eigen::Vector3d base_2d_pos(base_pos.x(), base_pos.y(), state_->pos().z());
            double radius = kv.second.radii.at(i);
            if ((state_->pos()-base_2d_pos).norm() < radius) {
                reward += 1;
            }
            i++;
        }
    }
    return reward;
}

// Here is where handle the action received from OpenAI
// In this case, our action space is turn left, turn right, or straight ahead
bool TutorialOpenAIAutonomy::handle_action(double t, double dt, const scrimmage_proto::Action &action) {
    if (!check_action(action, 1, 0)) {
        return false;
    }

    double turn_rate = (action.discrete(0) - 1)*.2;
    Eigen::Vector3d velocity_cmd;
    velocity_cmd << 1, turn_rate, 0; // Constanlty moving forward
    desired_state_->set_vel(velocity_cmd);
    return true;
}

// Set up the action space for OpenAI
// In this example, our action space will be a Discrete space of 0,1,2
scrimmage_proto::SpaceParams TutorialOpenAIAutonomy::action_space_params() {
    sp::SpaceParams space_params;
    sp::SingleSpaceParams *single_space_params = space_params.add_params();
    single_space_params->set_discrete(true);
    single_space_params->set_num_dims(1);
    single_space_params->add_minimum(0);
    single_space_params->add_maximum(2);
    return space_params;
}
