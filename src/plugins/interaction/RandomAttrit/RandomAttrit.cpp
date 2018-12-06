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
 * @author Jeremy Feltracco <jeremy.feltracco@gtri.gatech.edu>
 * @date 06 December 2018
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/plugins/interaction/RandomAttrit/RandomAttrit.h>

#include <scrimmage/common/Utilities.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/common/Random.h>

#include <memory>
#include <limits>
#include <iostream>
#include <cmath>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::EntityInteraction,
                scrimmage::interaction::RandomAttrit,
                RandomAttrit_plugin)

namespace scrimmage {
namespace interaction {

RandomAttrit::RandomAttrit() {
}

bool RandomAttrit::init(std::map<std::string, std::string> &mission_params,
                               std::map<std::string, std::string> &plugin_params) {
    team_id_ = sc::get("team", plugin_params, -1);
    decay_method_ = sc::get("decay_method", plugin_params, "linear");
    start_wait_time_s_ = sc::get("start_after", plugin_params, 0.0);
    duration_s_ = sc::get("duration", plugin_params, 10.0);
    std::vector<std::string> stay_alive_ids_str;
    sc::get_vec("leave_alive", plugin_params, " ", stay_alive_ids_str);
    std::transform(stay_alive_ids_str.begin(), stay_alive_ids_str.end(),
                   std::back_inserter(stay_alive_ids_),
                   [](const std::string& str) { return std::stoi(str); });

    attrit_pub_ = advertise("GlobalNetwork", "AttritAnnounce");
    return true;
}


bool RandomAttrit::step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                                  double t, double dt) {
    if (ents.empty() || num_ent_ < 0.0) {
        return true;
    }

    total_num_entities_ = 0;
    std::vector<sc::EntityPtr> cur_ents;
    std::copy_if(ents.begin(), ents.end(), std::back_inserter(cur_ents),
         [&](const sc::EntityPtr& ent) {
             bool is_stay_alive = std::find(stay_alive_ids_.begin(), stay_alive_ids_.end(),
                                            ent->id().id()) != stay_alive_ids_.end();
             bool team_match = ent->id().team_id() == team_id_;
             if (team_match) total_num_entities_++;
             return !is_stay_alive && team_match;
         });

    if (t < start_wait_time_s_) {
        return true;
    } else if (!started_) {
        // one time initialization
        start_num_ent_ = cur_ents.size();
        num_ent_ = start_num_ent_;
        started_ = true;
    }

    if (decay_method_ == "linear") {
        double slope = -start_num_ent_ / duration_s_;
        num_ent_ = slope * (t - start_wait_time_s_) + start_num_ent_;
    } else {
        std::cout << "RandomAttrit: Decay method, " << decay_method_
                  << ", is not implemented." << std::endl;
    }

    // else if (decay_method_ == "exponential") {

    if (std::ceil(num_ent_) < cur_ents.size()) {
        // pick one to attrit
        int drop_id = random_.rng_uniform_int(0, cur_ents.size() - 1);
        cur_ents[drop_id]->set_health_points(-1); // kill entity

        auto msg = std::make_shared<sc::Message<EntityPtr>>();
        msg->data = cur_ents[drop_id];
        attrit_pub_->publish(msg);
    }

    return true;
}
} // namespace interaction
} // namespace scrimmage
