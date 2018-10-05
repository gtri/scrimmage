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

#ifndef INCLUDE_SCRIMMAGE_METRICS_METRICS_H_
#define INCLUDE_SCRIMMAGE_METRICS_METRICS_H_
#include <scrimmage/plugin_manager/Plugin.h>

#include <map>
#include <list>
#include <string>

namespace scrimmage {

class Metrics : public Plugin{
 public:
    Metrics();
    virtual ~Metrics();

    std::string name() override;
    virtual void init();
    virtual void init(std::map<std::string, std::string> &params);
    virtual bool step_metrics(double t, double dt);

    virtual void calc_team_scores();
    virtual void print_team_summaries();

    // Key 1  : Team ID
    // Value 1: Map of..
    //                   Key 2  : Header string
    //                   Value 2: Metric value
    virtual std::map<int, std::map<std::string, double>> &team_metrics();

    virtual std::list<std::string> &headers();

    virtual std::map<int, double> & team_scores();

    bool get_print_team_summary() {return print_team_summary_;}

 protected:
    std::string weights_file_;
    std::map<int, std::map<std::string, double>> team_metrics_;
    std::map<int, double> team_scores_;
    std::list<std::string> headers_;
    bool print_team_summary_ = true;
};

using MetricsPtr = std::shared_ptr<Metrics>;

} // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_METRICS_METRICS_H_
