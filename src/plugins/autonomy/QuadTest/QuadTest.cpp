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

#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>

#include <scrimmage/plugins/autonomy/QuadTest/QuadTest.h>

REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::QuadTest, QuadTest_plugin)

namespace scrimmage {
namespace autonomy {

void QuadTest::init(std::map<std::string, std::string> &params) {
     desired_state_->vel() = Eigen::Vector3d::Zero();
     desired_state_->quat().set(0, 0, state_->quat().yaw());
     desired_state_->pos() = state_->pos()(2)*Eigen::Vector3d::UnitZ();

     if (params.count("take_off_time") > 0) {
          take_off_time_ = std::stod(params["take_off_time"]);
     } else {
          take_off_time_ = 0;
     }
}

bool QuadTest::step_autonomy(double t, double dt) {


     if (t >= take_off_time_) {
          desired_state_->vel() = 15*Eigen::Vector3d::UnitX();
          desired_state_->quat().set(0, 0, scrimmage::Angles::deg2rad(90));
          desired_state_->pos() = 10*Eigen::Vector3d::UnitZ();
     }

     return true;
}
} // namespace autonomy
} // namespace scrimmage
