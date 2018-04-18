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

#include <scrimmage/metrics/Metrics.h>

#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/common/RTree.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>

#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/msgs/Collision.pb.h>
#include <scrimmage/msgs/Event.pb.h>

#include <scrimmage/plugins/metrics/SimpleCollisionMetrics/SimpleCollisionMetrics.h>

#include <iostream>
#include <fstream>

namespace sc = scrimmage;
namespace sm = scrimmage_msgs;
using std::cout;
using std::endl;

REGISTER_PLUGIN(scrimmage::Metrics, scrimmage::metrics::SimpleCollisionMetrics,
        SimpleCollisionMetrics_plugin)

namespace scrimmage {
namespace metrics {

SimpleCollisionMetrics::SimpleCollisionMetrics() {}

void SimpleCollisionMetrics::init(std::map<std::string, std::string> &params) {
    params_ = params;

    auto teamcoll_cb = [&] (scrimmage::MessagePtr<sm::TeamCollision> msg) {
        scores_[msg->data.entity_id_1()].increment_team_collisions();
        scores_[msg->data.entity_id_2()].increment_team_collisions();
    };
    subscribe<sm::TeamCollision>("GlobalNetwork", "TeamCollision", teamcoll_cb);

    auto nonteamcoll_cb =
        [&] (scrimmage::MessagePtr<sm::NonTeamCollision> msg) {
        scores_[msg->data.entity_id_1()].increment_non_team_collisions();
        scores_[msg->data.entity_id_2()].increment_non_team_collisions();
    };
    subscribe<sm::NonTeamCollision>("GlobalNetwork", "NonTeamCollision",
            nonteamcoll_cb);

    auto groundcoll_cb = [&] (scrimmage::MessagePtr<sm::GroundCollision> msg) {
        scores_[msg->data.entity_id()].increment_ground_collisions();
    };
    subscribe<sm::GroundCollision>("GlobalNetwork", "GroundCollision",
            groundcoll_cb);

    auto entitygen_cb = [&] (scrimmage::MessagePtr<sm::EntityGenerated> msg) {
        scores_[msg->data.entity_id()].set_flight_time_start(msg->time);
    };
    subscribe<sm::EntityGenerated>("GlobalNetwork", "EntityGenerated",
            entitygen_cb);

    auto entityrem_cb = [&] (scrimmage::MessagePtr<sm::EntityRemoved> msg) {
        scores_[msg->data.entity_id()].set_flight_time_end(msg->time);
    };
    subscribe<sm::EntityRemoved>("GlobalNetwork", "EntityRemoved",
            entityrem_cb);

    auto entity_pre_end_cb =
        [&] (scrimmage::MessagePtr<sm::EntityPresentAtEnd> msg) {
        scores_[msg->data.entity_id()].set_flight_time_end(time_->t());
        surviving_teams_[(*id_to_team_map_)[msg->data.entity_id()]] = true;
    };
    subscribe<sm::EntityPresentAtEnd>("GlobalNetwork", "EntityPresentAtEnd",
            entity_pre_end_cb);
}

bool SimpleCollisionMetrics::step_metrics(double t, double dt) {
    if (!initialized_) {
        for (auto & teamnum : *id_to_team_map_) {
            // get all the teams so we can initialize their scores
            teams_.emplace(teamnum.second);
        }
        initialized_ = true;
    }
    return true;
}

void SimpleCollisionMetrics::calc_team_scores() {
    double end_time = -std::numeric_limits<double>::infinity();
    double beg_time = std::numeric_limits<double>::infinity();
    for (auto &kv : scores_) {
        double temp_beg_time = kv.second.flight_time_start();
        double temp_end_time = kv.second.flight_time_end();
        if (temp_beg_time < beg_time) {
            beg_time = temp_beg_time;
        }
        if (temp_end_time > end_time) {
            end_time = temp_end_time;
        }
    }

    // If the end_time is <= beginning time, all entities survived, need
    // to use the ending simulation time.
    if (end_time <= beg_time) {
        end_time = time_->t();
    }

    double max_flight_time = end_time - beg_time;

    // Create the score, if necessary
    for (auto &team_id : teams_) {
        if (team_coll_scores_.count(team_id) == 0) {
            SimpleCollisionScore score;
            score.set_weights(params_);
            // Set the max flight time for each score:
            score.set_max_flight_time(max_flight_time);
            team_coll_scores_[team_id] = score;
        }
    }

    for (auto &kv : scores_) {
        SimpleCollisionScore &score = kv.second;

        // If the end time is less than start time, the flight time was never
        // set. Use the ending simulation time.
        if (score.flight_time_end() <= score.flight_time_start()) {
            score.set_flight_time_end(end_time);
        }

        int team_id = (*id_to_team_map_)[kv.first];

        team_coll_scores_[team_id].add_non_team_collisions(
                score.non_team_collisions());
        team_coll_scores_[team_id].add_team_collisions(
                score.team_collisions());
        team_coll_scores_[team_id].add_ground_collisions(
                score.ground_collisions());
        team_coll_scores_[team_id].add_flight_time(score.flight_time());
        team_coll_scores_[team_id].increment_entity_count();
    }

    for (auto &kv : team_coll_scores_) {
        int team_id = kv.first;
        SimpleCollisionScore &score = kv.second;
        team_metrics_[team_id]["entity_count"] = score.entity_count();
        team_metrics_[team_id]["flight_time"] = score.flight_time();
        team_metrics_[team_id]["flight_time_norm"] = score.flight_time_norm();
        team_metrics_[team_id]["non_team_coll"] = score.non_team_collisions();
        team_metrics_[team_id]["team_coll"] = score.team_collisions();
        team_metrics_[team_id]["ground_coll"] = score.ground_collisions();

        team_scores_[team_id] = score.score();
    }

    // list the headers we want put in the csv file
    headers_.push_back("entity_count");
    headers_.push_back("flight_time");
    headers_.push_back("flight_time_norm");
    headers_.push_back("non_team_coll");
    headers_.push_back("team_coll");
    headers_.push_back("ground_coll");
}

void SimpleCollisionMetrics::print_team_summaries() {
    for (std::map<int, SimpleCollisionScore>::iterator it =
            team_coll_scores_.begin(); it != team_coll_scores_.end(); ++it) {
        bool survived = false;
        auto it_survive = surviving_teams_.find(it->first);
        if (it_survive != surviving_teams_.end()) survived = true;

        cout << "Team ID: " << it->first;
        if (survived) {
            cout << "\t(Survived round)" << endl;
        } else {
            cout << "\t(Didn't survive round)" << endl;
        }
        cout << "Score: " << it->second.score() << endl;
        cout << "Entity Count: " << it->second.entity_count() << endl;
        cout << "Total Flight Time: " << it->second.flight_time() << endl;
        cout << "Total Normalized Flight Time: " <<
            it->second.flight_time_norm() << endl;
        cout << "Non-Team Collisions: " << it->second.non_team_collisions() <<
            endl;
        cout << "Team Collisions: " << it->second.team_collisions() << endl;
        cout << "Ground Collisions: " << it->second.ground_collisions() << endl;
        cout << sc::generate_chars("-", 70) << endl;
    }
}
}  // namespace metrics
}  // namespace scrimmage
