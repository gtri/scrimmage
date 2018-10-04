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

#include <scrimmage/plugins/metrics/OpenAIRewards/OpenAIRewards.h>

#include <scrimmage/common/CSV.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/metrics/Metrics.h>

#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/msgs/Collision.pb.h>
#include <scrimmage/msgs/Event.pb.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;
namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(scrimmage::Metrics,
                scrimmage::metrics::OpenAIRewards,
                OpenAIRewards_plugin)

namespace scrimmage {
namespace metrics {

OpenAIRewards::OpenAIRewards() : Metrics() {
    print_team_summary_ = false;
}

void OpenAIRewards::init(std::map<std::string, std::string> &/*params*/) {
    auto cb = [&](auto msg) {
        size_t id;
        double reward;
        std::tie(id, reward) = msg->data;
        auto it = rewards_.find(id);
        if (it == rewards_.end()) {
            rewards_[id] = reward;
        } else {
            it->second += reward;
        }
        print_team_summary_ = true;
    };
    subscribe<std::pair<size_t, double>>("GlobalNetwork", "reward", cb);
}

void OpenAIRewards::print_team_summaries() {
    for (auto &kv : rewards_) {
        std::cout << "Reward for id " << kv.first << " = " << kv.second << std::endl;
    }
}

void OpenAIRewards::calc_team_scores() {
    CSV csv;
    std::string filename = parent_->mp()->log_dir() + "/rewards.csv";

    if (!csv.open_output(filename)) {
       std::cout << "Couldn't create output file" << endl;
    }

    csv.set_column_headers("id, reward");
    for (auto &kv : rewards_) {
        csv.append(CSV::Pairs{{"id", kv.first}, {"reward", kv.second}});
    }
    csv.close_output();
}
} // namespace metrics
} // namespace scrimmage
