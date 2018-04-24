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
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/motion/SimpleCar/SimpleCar.h>

#include <boost/algorithm/clamp.hpp>

using boost::algorithm::clamp;

REGISTER_PLUGIN(scrimmage::MotionModel, scrimmage::motion::SimpleCar, SimpleCar_plugin)

namespace scrimmage {
namespace motion {

enum ModelParams {
    X = 0,
    Y,
    Z,
    Z_dot,
    THETA,
    MODEL_NUM_ITEMS
};

enum ControlParams {
    FORWARD_VELOCITY = 0,
    TURN_RATE,
    CONTROL_NUM_ITEMS
};

bool SimpleCar::init(std::map<std::string, std::string> &info,
                     std::map<std::string, std::string> &params) {
    x_.resize(MODEL_NUM_ITEMS);
    x_[X] = std::stod(info["x"]);
    x_[Y] = std::stod(info["y"]);
    x_[Z] = std::stod(info["z"]);
    x_[Z_dot] = 0;
    x_[THETA] = Angles::deg2rad(std::stod(info["heading"]));

    length_ = get<double>("length", params, 100.0);
    mass_ = get<double>("mass", params, 1.0);
    enable_gravity_ = get<bool>("enable_gravity", params, false);
    max_velocity_ = get<double>("max_velocity", params, 30.0);

    /////////
    state_->vel() << 0, 0, 0;
    state_->pos() << x_[X], x_[Y], x_[Z];
    state_->quat().set(0, 0, x_[THETA]);

    input_speed_idx_ = vars_.declare(VariableIO::Type::speed, VariableIO::Direction::In);
    input_turn_rate_idx_ = vars_.declare(VariableIO::Type::turn_rate, VariableIO::Direction::In);

    return true;
}

bool SimpleCar::step(double time, double dt) {
    double prev_x = x_[X];
    double prev_y = x_[Y];
    double prev_z = x_[Z];

    ode_step(dt);

    ext_force_ = Eigen::Vector3d::Zero();

    /////////////////////
    // Save state
    // Simple velocity
    state_->vel() << (x_[X] - prev_x) / dt, (x_[Y] - prev_y) / dt,
        (x_[Z] - prev_z) / dt;

    state_->pos() << x_[X], x_[Y], x_[Z];
    state_->quat().set(0, 0, x_[THETA]);
    return true;
}

void SimpleCar::model(const vector_t &x , vector_t &dxdt , double t) {
    /// 0 : x-position
    /// 1 : y-position
    /// 2 : theta

    const double u_vel = clamp(vars_.input(input_speed_idx_), 0, max_velocity_);
    const double theta_lim = M_PI / 4 - 0.0001;
    const double u_theta = clamp(vars_.input(input_turn_rate_idx_), -theta_lim, theta_lim);

    dxdt[X] = u_vel*cos(x[THETA]);
    dxdt[Y] = u_vel*sin(x[THETA]);
    dxdt[THETA] = u_vel/length_*tan(u_theta);

    if (enable_gravity_) {
        dxdt[Z] = x[Z_dot];
        dxdt[Z_dot] = mass_ * -9.8;
    } else {
        dxdt[Z] = 0;
        dxdt[Z_dot] = 0;
    }

    // Saturate based on external force:
    if (std::abs(ext_force_(0)) > 0.1) {
        dxdt[X] = 0;
    }

    if (std::abs(ext_force_(1)) > 0.1) {
        dxdt[Y] = 0;
    }

    if (std::abs(ext_force_(2)) > 0.1) {
        dxdt[Z] = 0;
    }
}
} // namespace motion
} // namespace scrimmage
