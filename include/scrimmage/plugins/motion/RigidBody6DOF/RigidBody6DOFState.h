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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_MOTION_RIGIDBODY6DOF_RIGIDBODY6DOFSTATE_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_MOTION_RIGIDBODY6DOF_RIGIDBODY6DOFSTATE_H_

#include <scrimmage/math/State.h>

#include <Eigen/Dense>

#include <memory>

namespace scrimmage {
namespace motion {

class RigidBody6DOFState : public scrimmage::State {
 public:
RigidBody6DOFState() : linear_vel_body_(0, 0, 0), ang_vel_body_(0, 0, 0),
        linear_accel_body_(0, 0, 0), ang_accel_body_(0, 0, 0) {
    }

    Eigen::Vector3d &linear_vel_body() {
        return linear_vel_body_;
    }

    Eigen::Vector3d &ang_vel_body() {
        return ang_vel_body_;
    }

    Eigen::Vector3d &linear_accel_body() {
        return linear_accel_body_;
    }

    Eigen::Vector3d &ang_accel_body() {
        return ang_accel_body_;
    }

    const Eigen::Vector3d &linear_vel_body_const() const {
        return linear_vel_body_;
    }

    const Eigen::Vector3d &ang_vel_body_const() const {
        return ang_vel_body_;
    }

    const Eigen::Vector3d &linear_accel_body_const() const {
        return linear_accel_body_;
    }

    const Eigen::Vector3d &ang_accel_body_const() const {
        return ang_accel_body_;
    }


 protected:
    Eigen::Vector3d linear_vel_body_;
    Eigen::Vector3d ang_vel_body_;
    Eigen::Vector3d linear_accel_body_;
    Eigen::Vector3d ang_accel_body_;
};

using RigidBody6DOFStatePtr = std::shared_ptr<RigidBody6DOFState>;

} // namespace motion
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_MOTION_RIGIDBODY6DOF_RIGIDBODY6DOFSTATE_H_
