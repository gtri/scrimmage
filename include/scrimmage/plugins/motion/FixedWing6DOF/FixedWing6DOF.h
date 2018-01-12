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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_MOTION_FIXEDWING6DOF_FIXEDWING6DOF_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_MOTION_FIXEDWING6DOF_FIXEDWING6DOF_H_
#include <scrimmage/math/State.h>
#include <scrimmage/motion/MotionModel.h>
#include <scrimmage/motion/Controller.h>
#include <scrimmage/common/PID.h>
#include <scrimmage/common/CSV.h>

#include <map>
#include <string>
#include <tuple>

namespace scrimmage {
namespace motion {
class FixedWing6DOF : public scrimmage::MotionModel{
 public:
    enum ModelParams {
        U = 0,
        V,
        W,
        P,
        Q,
        R,
        U_dot,
        V_dot,
        W_dot,
        P_dot,
        Q_dot,
        R_dot,
        Uw,
        Vw,
        Ww,
        Xw,
        Yw,
        Zw,
        q0,
        q1,
        q2,
        q3,
        MODEL_NUM_ITEMS
    };

    FixedWing6DOF();

    virtual std::tuple<int, int, int> version();

    virtual bool init(std::map<std::string, std::string> &info,
                      std::map<std::string, std::string> &params);
    virtual bool step(double time, double dt);

    void model(const vector_t &x , vector_t &dxdt , double t);

    virtual void teleport(scrimmage::StatePtr &state);

    class Controller : public scrimmage::Controller {
     public:
        virtual Eigen::Vector4d u() = 0;
    };

    void set_u(Eigen::Vector4d u) {ctrl_u_ = u;}

 protected:
    scrimmage::PID heading_pid_;
    scrimmage::PID alt_pid_;
    scrimmage::PID vel_pid_;

    Eigen::Vector4d ctrl_u_;

    double min_velocity_;
    double max_velocity_;
    double max_roll_;
    double max_pitch_;

    scrimmage::Quaternion quat_world_;
    scrimmage::Quaternion quat_local_;

    Eigen::Matrix3d I_;
    Eigen::Matrix3d I_inv_;

    // Logging utility
    bool write_csv_;
    CSV csv_;
};
} // namespace motion
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_MOTION_FIXEDWING6DOF_FIXEDWING6DOF_H_
