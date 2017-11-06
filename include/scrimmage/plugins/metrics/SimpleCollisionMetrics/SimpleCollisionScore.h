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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_METRICS_SIMPLECOLLISIONMETRICS_SIMPLECOLLISIONSCORE_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_METRICS_SIMPLECOLLISIONMETRICS_SIMPLECOLLISIONSCORE_H_

#include <map>
#include <string>

namespace scrimmage {
namespace metrics {
class SimpleCollisionScore {
 public:
    bool set_weights(std::map<std::string, std::string> &params);

    void set_flight_time_start(double t) { flight_time_start_ = t; }
    void set_flight_time_end(double t) { flight_time_end_ = t; }
    void increment_non_team_collisions() { non_team_collisions_++; }
    void increment_team_collisions() { team_collisions_++; }
    void increment_ground_collisions() { ground_collisions_++; }

    void add_non_team_collisions(int c) { non_team_collisions_ += c; }
    void add_team_collisions(int c) { team_collisions_ += c; }
    void add_ground_collisions(int c) { ground_collisions_ += c; }
    void add_flight_time(double t) { flight_time_end_ += t; }

    int non_team_collisions() { return non_team_collisions_; }
    int team_collisions() { return team_collisions_; }
    int ground_collisions() { return ground_collisions_; }

    void set_max_flight_time(double t) { max_flight_time_ = t; }
    void increment_entity_count() { entity_count_++; }

    int entity_count() { return entity_count_; }
    void set_entity_count(int entity_count) {entity_count_ = entity_count;}
    void set_non_team_collisions(int non_team_collisions) {non_team_collisions_ = non_team_collisions;}
    void set_team_collisions(int team_collisions) {team_collisions_ = team_collisions;}
    void set_ground_collisions(int ground_collisions) {ground_collisions_ = ground_collisions;}

    double flight_time() {return (flight_time_end_ - flight_time_start_);}
    double flight_time_end() {return flight_time_end_;}
    double flight_time_start() {return flight_time_start_;}
    double flight_time_norm() {return (flight_time() / max_flight_time_);}

    double score();

 protected:
    // 1.) Flight Time (FT)
    double flight_time_start_ = 0;
    double flight_time_end_ = 0;
    int non_team_collisions_ = 0;
    int team_collisions_ = 0;
    int ground_collisions_ = 0;

    double flight_time_w_ = 0;
    double non_team_collisions_w_ = 0;
    double team_collisions_w_ = 0;

    double max_flight_time_ = 0;
    int entity_count_ = 0;
};
} // namespace metrics
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_METRICS_SIMPLECOLLISIONMETRICS_SIMPLECOLLISIONSCORE_H_
