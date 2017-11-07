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

#include <scrimmage/parse/ConfigParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugins/metrics/SimpleCollisionMetrics/SimpleCollisionScore.h>

namespace sc = scrimmage;

namespace scrimmage {
namespace metrics {

bool SimpleCollisionScore::set_weights(std::map<std::string, std::string> &params) {
    flight_time_w_ = sc::get<double>("flight_time_w", params, 0.0);
    team_collisions_w_ = sc::get<double>("team_collisions_w", params, 0.0);
    non_team_collisions_w_ = sc::get<double>("non_team_collisions_w", params, 0.0);

    return true;
}

double SimpleCollisionScore::score() {
    double s = flight_time_norm() * flight_time_w_
        + non_team_collisions() * non_team_collisions_w_
        + team_collisions() * team_collisions_w_;
    return s;
}
} // namespace metrics
} // namespace scrimmage
