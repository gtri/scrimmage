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

#ifndef INCLUDE_SCRIMMAGE_COMMON_BATTERY_H_
#define INCLUDE_SCRIMMAGE_COMMON_BATTERY_H_

namespace scrimmage {
class Battery {
 public:
    Battery() {}
    Battery(const double &min, const double &max, const double &current);
    void add_charge(const double &amount);
    bool deplete(const double &amount);
    bool is_full();
    bool has_charge();
    const double &current_charge();
    double charge_percentage();

 protected:
    double min_charge_ = 0;
    double max_charge_ = 1;
    double current_charge_ = 1;
};
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_COMMON_BATTERY_H_
