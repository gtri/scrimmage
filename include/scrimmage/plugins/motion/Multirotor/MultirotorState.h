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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_MOTION_MULTIROTOR_MULTIROTORSTATE_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_MOTION_MULTIROTOR_MULTIROTORSTATE_H_

#include <scrimmage/math/State.h>

#include <Eigen/Dense>

#include <memory>

namespace scrimmage {
namespace motion {

class MultirotorState : public scrimmage::State {
 public:
    enum InputType {
        OMEGA = 0,
        PWM
    };

    void set_input_type(InputType input_type) {
        input_type_ = input_type;
    }

    void set_prop_input(Eigen::VectorXd prop_input) {
        prop_input_ = prop_input;
    }

    InputType &input_type() {
        return input_type_;
    }

    Eigen::VectorXd &prop_input() {
        return prop_input_;
    }

 protected:
    Eigen::VectorXd prop_input_;
    InputType input_type_;
};

using MultirotorStatePtr = std::shared_ptr<MultirotorState>;

} // namespace motion
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_MOTION_MULTIROTOR_MULTIROTORSTATE_H_
