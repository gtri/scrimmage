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
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/clamp.hpp>

using boost::algorithm::clamp;

namespace scrimmage {

PID::PID() : kp_(0), ki_(0), kd_(0), prev_error_(0), integral_(0),
             setpoint_(0), integral_band_(0), is_angle_(false),
             derivative_(0), error_(0), output_min_(0), output_max_(0),
             enable_output_limits_(false) {}

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

void PID::set_output_limits(const double &min, const double &max,
                            const double &enable) {
    output_min_ = min;
    output_max_ = max;
    enable_output_limits_ = enable;
}

void PID::set_parameters(const double &kp, const double &ki, const double &kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PID::set_setpoint(const double &setpoint) {
    setpoint_ = setpoint;
}

void PID::set_integral_band(const double &integral_band) {
    integral_band_ = integral_band;
}

void PID::reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
    derivative_ = 0.0;
}

void PID::set_is_angle(const bool &is_angle) { is_angle_ = is_angle; }

const double & PID::setpoint() { return setpoint_; }
const double & PID::kp() { return kp_; }
const double & PID::ki() { return ki_; }
const double & PID::kd() { return kd_; }
const double & PID::integral_band() { return integral_band_; }
const double & PID::integral() { return integral_; }
const double & PID::error() { return error_; }
const double & PID::previous_error() { return prev_error_; }
const double & PID::derivative() { return derivative_; }

double PID::step(const double &dt, const double &measurement) {
    error_ = setpoint_ - measurement;
    if (is_angle_) {
        error_ = Angles::angle_pi(error_);
    }

    if (std::abs(error_) > integral_band_) {
        integral_ = 0;
    } else {
        integral_ += error_ * dt;
    }

    derivative_ = (error_ - prev_error_) / std::max(1.0e-9, dt);
    double u = kp_*error_ + ki_*integral_ + kd_*derivative_;
    prev_error_ = error_;

    return enable_output_limits_ ? clamp(u, output_min_, output_max_) : u;
}
} // namespace scrimmage
