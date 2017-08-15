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
#include <scrimmage/proto/Frame.pb.h>

#include <limits>

namespace scrimmage {

Metrics::Metrics() {}

Metrics::~Metrics() {}

std::string Metrics::name() { return std::string("Metrics"); }

void Metrics::init() {}

void Metrics::init(std::map<std::string, std::string> &params) {}

bool Metrics::step_metrics(double t, double dt) { return false; }

void Metrics::set_team_lookup(std::shared_ptr<std::unordered_map<int, int> > &lookup)
{ team_lookup_ = lookup; }

void Metrics::calc_team_scores() {}

void Metrics::print_team_summaries() {}

std::map<int, std::map<std::string, double> > &Metrics::team_metrics()
{ return team_metrics_; }

std::list<std::string> &Metrics::headers() { return headers_; }

std::map<int, double> &Metrics::team_scores() { return team_scores_; }

} // namespace scrimmage
