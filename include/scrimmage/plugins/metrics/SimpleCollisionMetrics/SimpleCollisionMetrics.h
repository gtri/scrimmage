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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_METRICS_SIMPLECOLLISIONMETRICS_SIMPLECOLLISIONMETRICS_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_METRICS_SIMPLECOLLISIONMETRICS_SIMPLECOLLISIONMETRICS_H_
#include <scrimmage/metrics/Metrics.h>

#include "SimpleCollisionScore.h"

#include <map>
#include <set>
#include <string>

namespace scrimmage {
namespace metrics {
class SimpleCollisionMetrics : public scrimmage::Metrics {
 public:
    SimpleCollisionMetrics();

    void init(std::map<std::string, std::string> &params) override;
    bool step_metrics(double t, double dt) override;
    void calc_team_scores() override;
    void print_team_summaries() override;

 protected:
    std::map<int, SimpleCollisionScore> scores_;
    std::map<int, SimpleCollisionScore> team_coll_scores_;
    std::map<int, bool> surviving_teams_;

    std::map<std::string, std::string> params_;
    bool initialized_ = false;
    std::set<int> teams_;
};
}  // namespace metrics
}  // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_PLUGINS_METRICS_SIMPLECOLLISIONMETRICS_SIMPLECOLLISIONMETRICS_H_
