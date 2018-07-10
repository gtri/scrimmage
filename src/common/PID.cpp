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

#include <scrimmage/math/Angles.h>
#include <scrimmage/common/PID.h>

#include <cmath>
#include <algorithm>

#include <boost/algorithm/string.hpp>

namespace scrimmage {

PID::PID() : kp_(0), ki_(0), kd_(0), prev_error_(0), integral_(0),
             setpoint_(0), integral_band_(0), is_angle_(false) {}

bool PID::init(const std::string &str, const bool &is_angle) {
    std::vector<std::string> str_vals;
    boost::split(str_vals, str, boost::is_any_of(","));

    if (str_vals.size() != 4) {
        return false;
    } else {
        double p = std::stod(str_vals[0]);
        double i = std::stod(str_vals[1]);
        double d = std::stod(str_vals[2]);
        set_parameters(p, i, d);

        set_is_angle(is_angle);
        if (is_angle) {
            double i_lim = Angles::deg2rad(std::stod(str_vals[3]));
            set_integral_band(i_lim);
        } else {
            double i_lim = std::stod(str_vals[3]);
            set_integral_band(i_lim);
        }
    }
    return true;
}

void PID::set_parameters(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PID::set_setpoint(double setpoint) {
    setpoint_ = setpoint;
}

void PID::set_integral_band(double integral_band) {
    integral_band_ = integral_band;
}

void PID::set_is_angle(bool is_angle) { is_angle_ = is_angle; }

double PID::step(double dt, double measurement) {

    double error = setpoint_ - measurement;
    if (is_angle_) {
        error = Angles::angle_pi(error);
    }

    if (std::abs(error) > integral_band_) {
        integral_ = 0;
    } else {
        integral_ += error * dt;
    }

    double derivative = (error - prev_error_) / std::max(1.0e-9, dt);
    double u = kp_*error + ki_*integral_ + kd_*derivative;
    prev_error_ = error;
    return u;
}
} // namespace scrimmage
