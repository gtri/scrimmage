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

#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/plugins/motion/DoubleIntegrator/DoubleIntegrator.h>
#include <boost/algorithm/clamp.hpp>

REGISTER_PLUGIN(scrimmage::MotionModel, scrimmage::motion::DoubleIntegrator, DoubleIntegrator_plugin)

namespace scrimmage {
namespace motion {

namespace pl = std::placeholders;
using boost::algorithm::clamp;
namespace sc = scrimmage;

enum ModelParams {X, Y, Z, VX, VY, VZ, YAW, YAW_DOT, STATE_SIZE};

DoubleIntegrator::DoubleIntegrator() : motion_model_sets_yaw_(false) {
    x_.resize(STATE_SIZE);
}

bool DoubleIntegrator::init(std::map<std::string, std::string> &info,
                            std::map<std::string, std::string> &params) {
    max_vel_ = std::stod(params.at("max_vel"));
    max_acc_ = std::stod(params.at("max_acc"));
    motion_model_sets_yaw_ = sc::str2bool(params.at("motion_model_sets_yaw"));
    sim_copter_orientation_ = sc::get<bool>("sim_copter_orientation", params, false);
    sim_copter_max_roll_ = sc::Angles::deg2rad(sc::get<double>("sim_copter_max_roll", params, 30));
    sim_copter_max_pitch_ = sc::Angles::deg2rad(sc::get<double>("sim_copter_max_pitch", params, 30));
    max_yaw_vel_ = sc::get<double>("max_yaw_vel", params, 1.0);
    max_yaw_acc_ = sc::get<double>("max_yaw_acc", params, 1.0);

    x_[X] = state_->pos()(0);
    x_[Y] = state_->pos()(1);
    x_[Z] = state_->pos()(2);
    x_[YAW] = state_->quat().yaw();

    x_[VX] = clamp(state_->vel()(0), -max_vel_, max_vel_);
    x_[VY] = clamp(state_->vel()(1), -max_vel_, max_vel_);
    x_[VZ] = clamp(state_->vel()(2), -max_vel_, max_vel_);
    x_[YAW_DOT] = 0;

    state_->pos() << x_[X], x_[Y], x_[Z];
    state_->quat().set(0, 0, x_[YAW]);

    return true;
}

bool DoubleIntegrator::step(double t, double dt) {
    ctrl_u_ = std::static_pointer_cast<Controller>(parent_->controller())->u();
    ctrl_u_(0) = clamp(ctrl_u_(0), -max_acc_, max_acc_);
    ctrl_u_(1) = clamp(ctrl_u_(1), -max_acc_, max_acc_);
    ctrl_u_(2) = clamp(ctrl_u_(2), -max_acc_, max_acc_);

    ode_step(dt);

    x_[VX] = clamp(x_[VX], -max_vel_, max_vel_);
    x_[VY] = clamp(x_[VY], -max_vel_, max_vel_);
    x_[VZ] = clamp(x_[VZ], -max_vel_, max_vel_);

    state_->pos() << x_[X], x_[Y], x_[Z];
    state_->vel() << x_[VX], x_[VY], x_[VZ];
    if (motion_model_sets_yaw_) {
        if (x_[VY] != 0 || x_[VX] != 0) {
            state_->quat().set(0, 0, atan2(x_[VY], x_[VX]));
        }
    } else if (sim_copter_orientation_) {
        ctrl_u_(3) = clamp(ctrl_u_(3), -max_yaw_acc_, max_yaw_acc_);

        // Get global velocity vector into local coordinate frame
        Eigen::Vector3d vel_global(x_[VX], x_[VY], x_[VZ]);
        Eigen::Vector3d vel_local = state_->quat().rotate_reverse(vel_global);

        double pitch = vel_local(0) / max_vel_ * sim_copter_max_pitch_;
        double roll = vel_local(1) / max_vel_ * sim_copter_max_roll_;
        state_->quat().set(-roll, pitch, x_[YAW]);
    } else {
        state_->quat().set(0, 0, ctrl_u_(3));
    }
    return true;
}

double DoubleIntegrator::update_dvdt(double vel, double max_vel, double acc) {
    if (vel >= max_vel && acc > 0) {
        return 0;
    } else if (vel <= -max_vel && acc < 0) {
        return 0;
    } else {
        return acc;
    }
}

void DoubleIntegrator::model(const vector_t &x , vector_t &dxdt , double t) {
    dxdt[X] = clamp(x_[VX], -max_vel_, max_vel_);
    dxdt[Y] = clamp(x_[VY], -max_vel_, max_vel_);
    dxdt[Z] = clamp(x_[VZ], -max_vel_, max_vel_);
    dxdt[YAW] = clamp(x_[VZ], -max_yaw_vel_, max_yaw_vel_);

    dxdt[VX] = update_dvdt(x[VX], max_vel_, ctrl_u_(0));
    dxdt[VY] = update_dvdt(x[VY], max_vel_, ctrl_u_(1));
    dxdt[VZ] = update_dvdt(x[VZ], max_vel_, ctrl_u_(2));
    dxdt[YAW_DOT] = update_dvdt(x[YAW_DOT], max_yaw_vel_, ctrl_u_(3));
}

void DoubleIntegrator::teleport(sc::StatePtr &state) {
    x_[X] = state->pos()[0];
    x_[Y] = state->pos()[1];
    x_[Z] = state->pos()[2];
    x_[VX] = state->vel()[0];
    x_[VY] = state->vel()[1];
    x_[VZ] = state->vel()[2];
    x_[YAW] = state->quat().yaw();
    x_[YAW_DOT] = 0;
}
} // namespace motion
} // namespace scrimmage
