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
 * @author Christopher Richardson <christopher.richardson@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/plugins/motion/HarmonicOscillator/HarmonicOscillator.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/common/VariableIO.h>

#include <cmath>

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::MotionModel,
                scrimmage::motion::HarmonicOscillator,
                HarmonicOscillator_plugin)

namespace scrimmage {
namespace motion {

enum ModelParams {
    Z = 0,
    VZ,
    MODEL_NUM_ITEMS
};

bool HarmonicOscillator::init(std::map<std::string, std::string> &info,
                     std::map<std::string, std::string> &params) {
    // params
    damping_ratio = sc::get<double>("damping_ratio", params, 0.707);
    natural_frequency = sc::get<double>("natural_frequency", params, 1);

    // Declare variables for controllers
    acceleration_z_idx_ = vars_.declare(VariableIO::Type::acceleration_z, VariableIO::Direction::In);

    x_.resize(MODEL_NUM_ITEMS);
    x_[Z] = state_->pos()(2);
    x_[VZ] = state_->vel()(2);

    return true;
}

bool HarmonicOscillator::step(double time, double dt) {

    ode_step(dt);
    state_->pos()(2) = x_[Z];
    state_->vel()(2) = x_[VZ];

    return true;
}

void HarmonicOscillator::model(const vector_t &x , vector_t &dxdt ,
                                double t) {
    const double acc = vars_.input(acceleration_z_idx_);

    dxdt[Z] = x[VZ];
    dxdt[VZ] = -std::pow(natural_frequency, 2) * x[Z] - 2 * damping_ratio * natural_frequency * x[VZ] + acc;
}
} // namespace motion
} // namespace scrimmage
