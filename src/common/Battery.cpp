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
#include <scrimmage/common/Battery.h>

#include <limits>
#include <cmath>

namespace scrimmage {
Battery::Battery(const double &min, const double &max, const double &current)
    : min_charge_(min), max_charge_(max), current_charge_(current) {
    if (current_charge_ > max_charge_) {
        current_charge_ = max_charge_;
    }
    if (current_charge_ < min_charge_) {
        current_charge_ = min_charge_;
    }
}
void Battery::add_charge(const double &amount) {
    current_charge_ += amount;
    if (current_charge_ > max_charge_) {
        current_charge_ = max_charge_;
    }
}

bool Battery::deplete(const double &amount) {
    current_charge_ -= amount;

    if (current_charge_ < min_charge_) {
        current_charge_ = min_charge_;
        return false;
    }
    return true;
}

bool Battery::is_full() {
    return std::abs(current_charge_ - max_charge_) <
        std::numeric_limits<double>::epsilon();
}

bool Battery::has_charge() {
    return current_charge_ > min_charge_;
}

const double & Battery::current_charge() {
    return current_charge_;
}

double Battery::charge_percentage() {
    return 100.0 * (current_charge_ - min_charge_) / (max_charge_ - min_charge_);
}
} // namespace scrimmage
