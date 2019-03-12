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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_ARDUPILOT_PWMSTATE_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_ARDUPILOT_PWMSTATE_H_

#include <scrimmage/math/State.h>

#include <Eigen/Dense>

#include <memory>

namespace scrimmage {
namespace motion {

class PwmState : public scrimmage::State {
 public:
    void set_pwm_input(Eigen::VectorXd pwm_input) {
        pwm_input_ = pwm_input;
    }

    Eigen::VectorXd &pwm_input() {
        return pwm_input_;
    }

    void set_pwm_min(double pwm_min) {
        pwm_min_ = pwm_min;
    }
    void set_pwm_max(double pwm_max) {
        pwm_max_ = pwm_max;
    }
    double &pwm_min() {
        return pwm_min_;
    }
    double &pwm_max() {
        return pwm_max_;
    }

 protected:
    Eigen::VectorXd pwm_input_;
    double pwm_min_ = 1000;
    double pwm_max_ = 2000;
};

using PwmStatePtr = std::shared_ptr<PwmState>;

} // namespace motion
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_ARDUPILOT_PWMSTATE_H_
