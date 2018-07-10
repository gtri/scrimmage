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
#include <scrimmage/common/Utilities.h>
#include <scrimmage/metrics/Metrics.h>

#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/msgs/Capture.pb.h>
#include <scrimmage/msgs/Event.pb.h>

#include <scrimmage/plugins/metrics/SimpleCaptureMetrics/SimpleCaptureMetrics.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;
namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(scrimmage::Metrics,
        scrimmage::metrics::SimpleCaptureMetrics, SimpleCaptureMetrics_plugin)

namespace scrimmage {
namespace metrics {

void SimpleCaptureMetrics::init(std::map<std::string, std::string> &params) {
    params_ = params;

    auto teamcapture_cb = [&] (scrimmage::MessagePtr<sm::TeamCapture> msg) {
        scores_[msg->data.source_id()].increment_count("TeamCapture");
    };
    subscribe<sm::TeamCapture>("GlobalNetwork", "TeamCapture", teamcapture_cb);

    auto nonteamcapture_cb = [&] (
            scrimmage::MessagePtr<sm::NonTeamCapture> msg) {
        scores_[msg->data.source_id()].increment_count("NonTeamCapture");
    };
    subscribe<sm::NonTeamCapture>("GlobalNetwork", "NonTeamCapture",
            nonteamcapture_cb);
}

bool SimpleCaptureMetrics::step_metrics(double t, double dt) {
    if (!initialized_) {
        for (auto & teamnum : *id_to_team_map_) {
            // get all the teams so we can initialize their scores
            teams_.emplace(teamnum.second);
        }
        initialized_ = true;
    }
    return true;
}

void SimpleCaptureMetrics::calc_team_scores() {
    // Create the score if there isn't one yet
    for (auto &team_id : teams_) {
        if (team_scores_map_.count(team_id) == 0) {
            Score score;
            score.set_weights(params_);
            team_scores_map_[team_id] = score;
        }
    }

    for (auto &kv : scores_) {
        Score &score = kv.second;

        int team_id = (*id_to_team_map_)[kv.first];

        team_scores_map_[team_id].add_count("TeamCapture",
                score.count("TeamCapture"));
        team_scores_map_[team_id].add_count("NonTeamCapture",
                score.count("NonTeamCapture"));
    }

    for (auto &kv : team_scores_map_) {
        int team_id = kv.first;
        Score &score = kv.second;
        team_metrics_[team_id]["TeamCapture"] = score.count("TeamCapture");
        team_metrics_[team_id]["NonTeamCapture"] =
            score.count("NonTeamCapture");

        double s = score.score();
        team_scores_[team_id] = s;
    }

    // list the headers we want put in the csv file
    headers_.push_back("TeamCapture");
    headers_.push_back("NonTeamCapture");
}

void SimpleCaptureMetrics::print_team_summaries() {
    for (auto kv : team_scores_map_) {
        int team_id = kv.first;
        Score &score = kv.second;

        cout << "Team ID: " << team_id << endl;
        cout << "Score: " << score.score() << endl;
        cout << "Team Captures: " << score.count("TeamCapture") << endl;
        cout << "Non Team Captures: " << score.count("NonTeamCapture") << endl;
        cout << sc::generate_chars("-", 70) << endl;
    }
}
}  // namespace metrics
}  // namespace scrimmage
