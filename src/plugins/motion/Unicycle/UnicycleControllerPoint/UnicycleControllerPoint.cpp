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

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/motion/Unicycle/UnicycleControllerPoint/UnicycleControllerPoint.h>
REGISTER_PLUGIN(scrimmage::Controller,
                UnicycleControllerPoint,
                UnicycleControllerPoint_plugin)

void UnicycleControllerPoint::init(std::map<std::string, std::string> &params) {
    l_ = std::stod(params.at("l"));
    gain_ = std::stod(params.at("gain"));
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
    u_(0) = u_2d(0);
    u_(1) = u_2d(1);
    u_(2) = 0;

    return true;
}
