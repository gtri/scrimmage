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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_TRAJECTORYRECORDPLAYBACK_TRAJECTORYPOINT_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_TRAJECTORYRECORDPLAYBACK_TRAJECTORYPOINT_H_

#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/math/State.h>

namespace scrimmage {
namespace autonomy {
class TrajectoryPoint {
 public:
    TrajectoryPoint() : t_(0) {}
    TrajectoryPoint(double t, scrimmage::State &state, scrimmage::State &desired)
        : t_(t), state_(state), desired_state_(desired) { }

    double t() { return t_; }
    scrimmage::State &state() { return state_; }
    scrimmage::State &desired_state() { return desired_state_; }

    void set_t(double t) { t_ = t; }
    void set_state(scrimmage::State &state) { state_ = state; }
    void set_desired_state(scrimmage::State &state) { desired_state_ = state; }

 protected:
    double t_;
    scrimmage::State state_;
    scrimmage::State desired_state_;
 private:
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_TRAJECTORYRECORDPLAYBACK_TRAJECTORYPOINT_H_
