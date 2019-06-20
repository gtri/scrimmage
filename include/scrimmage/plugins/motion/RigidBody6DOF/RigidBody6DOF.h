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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_MOTION_RIGIDBODY6DOF_RIGIDBODY6DOF_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_MOTION_RIGIDBODY6DOF_RIGIDBODY6DOF_H_
#include <scrimmage/math/State.h>
#include <scrimmage/motion/MotionModel.h>
#include <scrimmage/motion/Controller.h>
#include <scrimmage/common/PID.h>

#include <map>
#include <string>
#include <tuple>
#include <memory>

namespace scrimmage {
namespace motion {
class RigidBody6DOF : public scrimmage::MotionModel{
 public:
    virtual std::tuple<int, int, int> version();

    bool init(std::map<std::string, std::string> &info,
                      std::map<std::string, std::string> &params) override;
    bool step(double time, double dt) override;

    void model(const vector_t &x , vector_t &dxdt , double t) override;

    void teleport(scrimmage::StatePtr &state) override;

    class Controller : public scrimmage::Controller {
     public:
        virtual std::shared_ptr<Eigen::Vector4d> u() = 0;
    };

    // cppcheck-suppress passedByValue
    void set_u(std::shared_ptr<Eigen::Vector4d> u) {ctrl_u_ = u;}

 protected:
    scrimmage::PID heading_pid_;
    scrimmage::PID alt_pid_;
    scrimmage::PID vel_pid_;

    double length_;
    std::shared_ptr<Eigen::Vector4d> ctrl_u_;

    double min_velocity_;
    double max_velocity_;
    double max_roll_;
    double max_pitch_;

    scrimmage::Quaternion quat_world_;
    scrimmage::Quaternion quat_local_;
};
} // namespace motion
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_MOTION_RIGIDBODY6DOF_RIGIDBODY6DOF_H_
