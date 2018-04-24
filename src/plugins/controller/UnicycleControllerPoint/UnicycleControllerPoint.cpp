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

    speed_idx_ = vars_.declare(VariableIO::Type::speed, VariableIO::Direction::Out);
    turn_rate_idx_ = vars_.declare(VariableIO::Type::turn_rate, VariableIO::Direction::Out);
    pitch_rate_idx_ = vars_.declare(VariableIO::Type::pitch_rate, VariableIO::Direction::Out);
}

bool UnicycleControllerPoint::step(double t, double dt) {
    Eigen::Vector2d des_pos = desired_state_->pos().head<2>();
    Eigen::Vector2d pos = state_->pos().head<2>();
    Eigen::Vector2d des_vel = gain_ * (des_pos - pos);

    double th = state_->quat().yaw();

    // TODO: Extend from 2D to 3D point controller
    Eigen::Matrix2d M;
    M << cos(th), sin(th), -sin(th) / l_, cos(th) / l_;
    Eigen::Vector2d u_2d = M * des_vel;

    vars_.output(speed_idx_, u_2d(0));
    vars_.output(turn_rate_idx_, u_2d(1));
    vars_.output(pitch_rate_idx_, 0);
    return true;
}
} // namespace controller
} // namespace scrimmage
