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

#include <scrimmage/common/ExponentialFilter.h>

#include <cmath>
#include <limits>

namespace scrimmage {

ExponentialFilter::ExponentialFilter() :
    ExponentialFilter(1.0) {}

ExponentialFilter::ExponentialFilter(double time_constant) :
    estimate_(0),
    time_last_estimate_(-std::numeric_limits<double>::infinity()),
    time_constant_(time_constant) {}

double ExponentialFilter::get_estimate() const {
    return estimate_;
}

void ExponentialFilter::set_time_constant(double tau) {
    time_constant_ = tau;
}

double ExponentialFilter::add_estimate(double estimate, double t) {
    if (time_last_estimate_ < t) {
        double dt = t - time_last_estimate_;
        double alpha = 1 - exp(-dt / time_constant_);
        estimate_ = alpha * estimate + (1 - alpha) * estimate_;
        time_last_estimate_ = t;
    }
    return estimate_;
}

} // namespace scrimmage
