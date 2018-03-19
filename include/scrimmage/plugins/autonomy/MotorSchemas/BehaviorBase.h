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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_MOTORSCHEMAS_BEHAVIORBASE_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_MOTORSCHEMAS_BEHAVIORBASE_H_

#include <scrimmage/autonomy/Autonomy.h>

#include <map>
#include <string>

namespace scrimmage {
namespace autonomy {
namespace motor_schemas {
class BehaviorBase : public scrimmage::Autonomy {
 public:
    BehaviorBase() : desired_vector_(Eigen::Vector3d(0, 0, 0)), gain_(1.0) {
    }

    Eigen::Vector3d &desired_vector() {
        return desired_vector_;
    }

    void set_gain(const double &gain) { gain_ = gain; }
    const double &gain() { return gain_; }

 protected:
    Eigen::Vector3d desired_vector_;
    double gain_;
};
using BehaviorBasePtr = std::shared_ptr<BehaviorBase>;
} // namespace motor_schemas
} // namespace autonomy
} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_MOTORSCHEMAS_BEHAVIORBASE_H_
