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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_MOTION_DOUBLEINTEGRATOR_DOUBLEINTEGRATOR_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_MOTION_DOUBLEINTEGRATOR_DOUBLEINTEGRATOR_H_

#include <scrimmage/math/State.h>
#include <scrimmage/motion/MotionModel.h>
#include <scrimmage/motion/Controller.h>
#include <scrimmage/common/PID.h>

#include <map>
#include <string>
#include <limits>

namespace scrimmage {
namespace motion {
class DoubleIntegrator : public scrimmage::MotionModel {
 public:
    DoubleIntegrator();

    bool init(std::map<std::string, std::string> &info,
                      std::map<std::string, std::string> &params) override;

    bool step(double t, double dt) override;

    void model(const vector_t &x , vector_t &dxdt , double t) override;

    void teleport(scrimmage::StatePtr &state) override;

 protected:
    double update_dvdt(double vel, double max_vel, double acc);
    double max_vel_ = std::numeric_limits<double>::infinity();
    double max_acc_ = std::numeric_limits<double>::infinity();
    double max_yaw_vel_ = std::numeric_limits<double>::infinity();
    double max_yaw_acc_ = std::numeric_limits<double>::infinity();
    bool motion_model_sets_yaw_ = true;
    bool sim_copter_orientation_ = false;
    double sim_copter_max_roll_ = 1.0;
    double sim_copter_max_pitch_ = 1.0;

    int acc_x_idx_ = 0;
    int acc_y_idx_ = 0;
    int acc_z_idx_ = 0;
    int turn_rate_idx_ = 0;

    Eigen::Vector3d acc_vec_;
    double turn_rate_ = 0;
};
} // namespace motion
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_MOTION_DOUBLEINTEGRATOR_DOUBLEINTEGRATOR_H_
