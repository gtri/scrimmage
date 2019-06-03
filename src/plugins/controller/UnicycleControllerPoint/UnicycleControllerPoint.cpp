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

#include <scrimmage/math/State.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/controller/UnicycleControllerPoint/UnicycleControllerPoint.h>

REGISTER_PLUGIN(scrimmage::Controller, scrimmage::controller::UnicycleControllerPoint, UnicycleControllerPoint_plugin)

namespace scrimmage {
namespace controller {

void UnicycleControllerPoint::init(std::map<std::string, std::string> &params) {
    l_ = std::stod(params.at("l"));
    gain_ = std::stod(params.at("gain"));

    using Type = VariableIO::Type;
    using Dir = VariableIO::Direction;

    x_idx_in_ = vars_.declare(Type::position_x, Dir::In);
    y_idx_in_ = vars_.declare(Type::position_y, Dir::In);
    z_idx_in_ = vars_.declare(Type::position_z, Dir::In);

    speed_idx_out_ = vars_.declare(Type::speed, Dir::Out);
    turn_rate_idx_out_ = vars_.declare(Type::turn_rate, Dir::Out);
    pitch_rate_idx_out_ = vars_.declare(Type::pitch_rate, Dir::Out);
    velocity_z_idx_out_ = vars_.declare(Type::velocity_z, Dir::Out);
}

bool UnicycleControllerPoint::step(double t, double dt) {
    Eigen::Vector2d des_pos_xy(vars_.input(x_idx_in_), vars_.input(y_idx_in_));
    Eigen::Vector2d pos_xy = state_->pos().head<2>();
    Eigen::Vector2d des_vel = gain_ * (des_pos_xy - pos_xy);

    double th = state_->quat().yaw();

    Eigen::Matrix2d M;
    M << cos(th), sin(th), -sin(th) / l_, cos(th) / l_;
    Eigen::Vector2d u_2d = M * des_vel;

    vars_.output(speed_idx_out_, u_2d(0));
    vars_.output(turn_rate_idx_out_, u_2d(1));
    vars_.output(pitch_rate_idx_out_, 0);

    const double des_z = vars_.input(z_idx_in_);
    const double z = state_->pos()(2);
    vars_.output(velocity_z_idx_out_, gain_ * (des_z - z));
    return true;
}
} // namespace controller
} // namespace scrimmage
