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
#include <scrimmage/plugins/motion/RigidBody6DOF/RigidBody6DOFBase.h>

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
class FixedWing6DOF : public scrimmage::motion::RigidBody6DOFBase{
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

    bool init(std::map<std::string, std::string> &info,
                      std::map<std::string, std::string> &params) override;
    bool step(double time, double dt) override;

    void model(const vector_t &x , vector_t &dxdt , double t) override;

 protected:
    int throttle_idx_ = 0;
    int elevator_idx_ = 0;
    int aileron_idx_ = 0;
    int rudder_idx_ = 0;

    scrimmage::PID heading_pid_;
    scrimmage::PID alt_pid_;
    scrimmage::PID vel_pid_;

    Eigen::VectorXd ctrl_u_;

    Eigen::Matrix3d I_;
    Eigen::Matrix3d I_inv_;

    // drawing
    bool draw_vel_ = false;
    bool draw_ang_vel_ = false;

    // Logging utility
    bool write_csv_ = false;
    CSV csv_;

    scrimmage::Quaternion quat_body_;
    Eigen::Vector3d force_ext_body_;

    double thrust_ = 0;
    double throttle_ = 0;
    double delta_elevator_ = 0;
    double delta_aileron_ = 0;
    double delta_rudder_ = 0;

    double alpha_ = 0; // angle of attack
    // double alpha_prev_ = 0;
    double alpha_dot_ = 0;

    double rho_ = 1.2250; // air density

    // Control input limits
    double thrust_min_ = -100.0e3;
    double thrust_max_ = 100.0e3;
    double delta_elevator_min_ = -0.5236;
    double delta_elevator_max_ = 0.5236;
    double delta_aileron_min_ = -0.5236;
    double delta_aileron_max_ = 0.5236;
    double delta_rudder_min_ = -0.2618;
    double delta_rudder_max_ = 0.2618;

    double b_ = 8.382; // wing span
    double S_ = 24.1548; // surface area of wing
    double c_ = 3.29184; // chord length
    double e_ = .995; // induced drag effiency factor
    double AR_ = 2.5; // aspect ratio (computed)

    double C_D0_ = 0.03;
    double C_D_alpha_ = 0.3;
    double C_D_delta_elevator_ = 0.01;

    double C_L0_ = 0.28;
    double C_L_alpha_ = 3.45;
    double C_LQ_ = 0.0;
    double C_L_alpha_dot_ = 0.72;
    double C_L_delta_elevator_ = 0.36;

    double C_M0_ = 0.0;
    double C_MQ_ = -3.6;
    double C_M_alpha_ = -0.38;
    double C_M_alpha_dot_ = -1.1;
    double C_M_delta_elevator_ = -0.5;

    double C_Y_beta_ = -0.98;
    double C_Y_delta_rudder_ = 0.17;

    double C_L_beta_ = -0.12;
    double C_LP_ = -0.26;
    double C_LR_ = 0.14;
    double C_L_delta_aileron_ = 0.08;
    double C_L_delta_rudder_ = -0.105;

    double C_N_beta_ = 0.25;
    double C_NP_ = 0.022;
    double C_NR_ = -0.35;
    double C_N_delta_aileron_ = 0.06;
    double C_N_delta_rudder_ = 0.032;

    Eigen::Quaterniond rot_180_x_axis_;
};
} // namespace motion
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_MOTION_FIXEDWING6DOF_FIXEDWING6DOF_H_
