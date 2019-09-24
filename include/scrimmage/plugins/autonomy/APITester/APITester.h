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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_APITESTER_APITESTER_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_APITESTER_APITESTER_H_
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/common/CSV.h>

#include <string>
#include <map>

namespace scrimmage {
namespace autonomy {
class APITester : public scrimmage::Autonomy {
 public:
    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;

 protected:
    bool my_test_bool_ = false;
    int my_test_int_ = 1;
    float my_test_float_ = 1.2345;
    double my_test_double_ = 9.87654321;

    uint8_t desired_heading_idx_ = 0;
    uint8_t desired_alt_idx_ = 0;
    uint8_t desired_speed_idx_ = 0;

    CSV csv_;

    void write_my_test_values();
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_APITESTER_APITESTER_H_
