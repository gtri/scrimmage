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

#include <scrimmage/plugins/metrics/TeamCPA/TeamCPA.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/metrics/Metrics.h>

#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/msgs/Collision.pb.h>
#include <scrimmage/msgs/Event.pb.h>

#include <scrimmage/parse/MissionParse.h>

#include <iostream>
#include <sstream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;
namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(scrimmage::Metrics,
                scrimmage::metrics::TeamCPA,
                TeamCPA_plugin)

namespace scrimmage {
namespace metrics {

TeamCPA::TeamCPA() {
}

void TeamCPA::init(std::map<std::string, std::string> &params) {
    team_id_ = sc::get<int>("team_id", params, 1);
    after_time_s_ = sc::get<double>("after_time_s", params, 0.0);
}

bool TeamCPA::step_metrics(double t, double dt) {
    if (t >= after_time_s_) {

        if (!initialized_) {
            for (auto &kv : *id_to_ent_map_) {
                if (kv.second->id().team_id() == team_id_) {
                    cpa_map_[kv.first] = TeamCPAData();
                }
            }
            std::string log_dir = ((*id_to_ent_map_)[1])->mp()->log_dir();
            std::ostringstream filename;
            filename << log_dir << "/" << "team_" << team_id_ << "_cpa.csv";
            csv_.open_output(filename.str());
            csv_.set_column_headers(scrimmage::CSV::Headers{
                "team",
                "entity",
                "cpa",
                "closest_entity",
                "time"});
            initialized_ = true;
        }

        for (auto &kv : *id_to_ent_map_) {
            if (kv.second->id().team_id() == team_id_) {
                for (auto &kv2 : *id_to_ent_map_) {
                    if ((kv != kv2) && (kv2.second->id().team_id() == team_id_)) {
                        if (kv.second->state_truth() && kv2.second->state_truth()) {
                            double cur_distance = (kv.second->state_truth()->pos() -
                                    kv2.second->state_truth()->pos()).norm();
                            if (cur_distance < cpa_map_[kv.first].distance()) {
                                cpa_map_[kv.first].set_distance(cur_distance);
                                cpa_map_[kv.first].set_closest_entity(kv2.first);
                                cpa_map_[kv.first].set_time(t);
                            }
                        }
                    }
                }
            }
        }
    }
    return true;
}

void TeamCPA::calc_team_scores() {
    for (auto &kv: cpa_map_) {
        csv_.append(scrimmage::CSV::Pairs{
                {"team", team_id_},
                {"entity", kv.first},
                {"cpa", kv.second.distance()},
                {"closest_entity", kv.second.closest_entity()},
                {"time", kv.second.time()}});
    }
}

void TeamCPA::print_team_summaries() {
    for (auto &kv: cpa_map_) {
        cout << "Team: " << team_id_;
        cout << " | Entity: " << kv.first;
        cout << " | CPA: " << kv.second.distance() << "m";
        cout << " | Closest Entity: " << kv.second.closest_entity();
        cout << " | Time: " << kv.second.time() << "s" << std::endl;
    }
    cout << sc::generate_chars("-", 80) << endl;
}
} // namespace metrics
} // namespace scrimmage
