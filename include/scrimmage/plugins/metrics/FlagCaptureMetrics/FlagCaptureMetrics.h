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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_METRICS_FLAGCAPTUREMETRICS_FLAGCAPTUREMETRICS_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_METRICS_FLAGCAPTUREMETRICS_FLAGCAPTUREMETRICS_H_

#include <scrimmage/metrics/Metrics.h>
#include <scrimmage/parse/ParseUtils.h>

#include <map>
#include <string>

namespace sc = scrimmage;

namespace scrimmage {
namespace metrics {

class Score {
 public:
    bool set_weights(std::map<std::string, std::string> &params) {
        flags_taken_w_ = sc::get<double>("flags_taken_w", params, 0.0);
        flags_captured_w_ = sc::get<double>("flags_captured_w", params, 0.0);
        return true;
    }

    void increment_flags_taken() { flags_taken_++; }
    void add_flags_taken(int c) { flags_taken_ += c; }
    int flags_taken() { return flags_taken_; }
    void set_flags_taken(int flags_taken) {
        flags_taken_ = flags_taken;
    }

    void increment_flags_captured() { flags_captured_++; }
    void add_flags_captured(int c) { flags_captured_ += c; }
    int flags_captured() { return flags_captured_; }
    void set_flags_captured(int flags_captured) {
        flags_captured_ = flags_captured;
    }

    double score() {
        double s = flags_taken() * flags_taken_w_ +
            flags_captured() * flags_captured_w_;
        return s;
    }

 protected:
    int flags_taken_ = 0;
    double flags_taken_w_ = 0.0;

    int flags_captured_ = 0;
    double flags_captured_w_ = 0.0;
};


class FlagCaptureMetrics : public scrimmage::Metrics {
 public:
    FlagCaptureMetrics();
    void init(std::map<std::string, std::string> &params) override;
    bool step_metrics(double t, double dt) override;
    void calc_team_scores() override;
    void print_team_summaries() override;
 protected:
    std::map<int, Score> scores_;
    std::map<int, Score> team_flag_scores_;
    std::map<std::string, std::string> params_;
 private:
};

} // namespace metrics
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_METRICS_FLAGCAPTUREMETRICS_FLAGCAPTUREMETRICS_H_
