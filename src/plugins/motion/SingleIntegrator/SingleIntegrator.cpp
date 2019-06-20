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
#include <scrimmage/math/Angles.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugin_manager/PluginManager.h>

#include <scrimmage/plugins/motion/SingleIntegrator/SingleIntegrator.h>

#include <cmath>

REGISTER_PLUGIN(scrimmage::MotionModel, scrimmage::motion::SingleIntegrator, SingleIntegrator_plugin)

namespace scrimmage {
namespace motion {

bool SingleIntegrator::init(std::map<std::string, std::string> &info,
                            std::map<std::string, std::string> &params) {

    override_heading_ = scrimmage::get<bool>("override_heading", params, false);

    vel_x_idx_ = vars_.declare(VariableIO::Type::velocity_x, VariableIO::Direction::In);
    vel_y_idx_ = vars_.declare(VariableIO::Type::velocity_y, VariableIO::Direction::In);
    vel_z_idx_ = vars_.declare(VariableIO::Type::velocity_z, VariableIO::Direction::In);

    if (override_heading_) {
        desired_heading_idx_ = vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::In);
    }

    auto get = [&](auto s) {return std::stod(info.at(s));};
    state_->pos() << get("x"), get("y"), get("z");

    state_->vel() << 0, 0, 0;
    state_->quat().set(0, 0, Angles::deg2rad(get("heading")));

    return true;
}

bool SingleIntegrator::step(double /*t*/, double dt) {

    Eigen::Vector3d &vel = state_->vel();

    vel << vars_.input(vel_x_idx_), vars_.input(vel_y_idx_), vars_.input(vel_z_idx_);
    state_->pos() = state_->pos() + vel * dt;

    double yaw = override_heading_ ? vars_.input(desired_heading_idx_) : atan2(vel(1), vel(0));
    double pitch = atan2(vel(2), vel.head<2>().norm());
    state_->quat().set(0, pitch, yaw);

    return true;
}

} // namespace motion
} // namespace scrimmage
