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

#ifndef INCLUDE_SCRIMMAGE_COMMON_PID_H_
#define INCLUDE_SCRIMMAGE_COMMON_PID_H_

#include <string>

namespace scrimmage {
class PID {
 public:
    PID();
    void set_parameters(double kp, double ki, double kd);
    void set_setpoint(double setpoint);
    double step(double dt, double measurement);
    void set_integral_band(double integral_band);
    void set_is_angle(bool is_angle);
    bool init(const std::string &str, const bool &is_angle);

 protected:
    double kp_;
    double ki_;
    double kd_;

    double prev_error_;
    double integral_;

    double setpoint_;

    double integral_band_;
    bool is_angle_;
};
} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_COMMON_PID_H_
