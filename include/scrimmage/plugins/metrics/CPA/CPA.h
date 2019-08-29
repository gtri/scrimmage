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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_METRICS_CPA_CPA_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_METRICS_CPA_CPA_H_

#include <scrimmage/metrics/Metrics.h>
#include <scrimmage/common/CSV.h>

#include <map>
#include <string>
#include <vector>
#include <limits>

namespace sc = scrimmage;

namespace scrimmage {
namespace metrics {

class CPAData {
 public:
    CPAData() {}
    double distance() {return distance_;}
    int closest_entity() {return closest_entity_;}
    double time() {return time_;}
    void set_distance(double dist) {distance_ = dist;}
    void set_closest_entity(int id) {closest_entity_ = id;}
    void set_time(double t) {time_ = t;}

 protected:
    double distance_ = std::numeric_limits<double>::infinity();
    int closest_entity_ = -1;
    double time_ = -1;
};

class CPA : public scrimmage::Metrics {
 public:
    CPA();
    void init(std::map<std::string, std::string> &params) override;
    bool step_metrics(double t, double dt) override;
    void calc_team_scores() override;
    void print_team_summaries() override;
 protected:
    std::map<std::string, std::string> params_;

    // Entity Num: CPA, Closest Entity, Time
    std::map<int, CPAData> cpa_map_;
    CSV csv_;
    bool initialized_ = false;
 private:
};

} // namespace metrics
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_METRICS_CPA_CPA_H_
