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

#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>

#include <scrimmage/plugins/motion/UUV6DOF/UUV6DOF.h>

#include <iostream>

#include <boost/algorithm/clamp.hpp>

using std::cout;
using std::endl;
using boost::algorithm::clamp;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::MotionModel,
                scrimmage::motion::UUV6DOF,
                UUV6DOF_plugin)

namespace scrimmage {
namespace motion {

UUV6DOF::UUV6DOF() {
    Eigen::AngleAxisd aa(M_PI, Eigen::Vector3d::UnitX());
    rot_180_x_axis_ = Eigen::Quaterniond(aa);
    x_.resize(MODEL_NUM_ITEMS);
}

bool UUV6DOF::init(std::map<std::string, std::string> &info,
                     std::map<std::string, std::string> &params) {
    throttle_idx_ = vars_.declare(VariableIO::Type::throttle, VariableIO::Direction::In);
    elevator_idx_ = vars_.declare(VariableIO::Type::elevator, VariableIO::Direction::In);
    rudder_idx_ = vars_.declare(VariableIO::Type::rudder, VariableIO::Direction::In);

    Eigen::Vector3d &pos = state_->pos();

    // Need to rotate axes by 180 degrees around X-axis SCRIMMAGE's global
    // frame uses Z-axis pointing up. Many aircraft equations of motion are
    // specified with Z-axis pointing down.
    quat_body_ = rot_180_x_axis_ * state_->quat();
    quat_body_.set(sc::Angles::angle_pi(quat_body_.roll()+M_PI),
                   quat_body_.pitch(), quat_body_.yaw());

    x_[U] = state_->vel()(0);
    x_[V] = 0;
    x_[W] = 0;

    x_[P] = 0;
    x_[Q] = 0;
    x_[R] = 0;

    x_[Uw] = state_->vel()(0);
    x_[Vw] = 0;
    x_[Ww] = 0;

    x_[Xw] = pos(0);
    x_[Yw] = pos(1);
    x_[Zw] = pos(2);

    x_[q0] = quat_body_.w();
    x_[q1] = quat_body_.x();
    x_[q2] = quat_body_.y();
    x_[q3] = quat_body_.z();

    x_[U_dot] = 0;
    x_[V_dot] = 0;
    x_[W_dot] = 0;
    x_[P_dot] = 0;
    x_[Q_dot] = 0;
    x_[R_dot] = 0;

    // Parse XML parameters
    g_ = sc::get<double>("gravity_magnitude", params, 9.81);
    mass_ = sc::get<double>("mass", params, 1.2);
    buoyancy_ = sc::get<double>("buoyancy", params, buoyancy_);
    surface_height_ = sc::get<double>("surface_height", params, surface_height_);

    {
        // Get the inertia matrix
        std::vector<std::vector<std::string>> vecs;
        std::string inertia_matrix = sc::get<std::string>("inertia_matrix",
                                                          params, "");
        // Parse inertia matrix
        bool valid_inertia = false;
        if (!sc::get_vec_of_vecs(inertia_matrix, vecs)) {
            cout << "Failed to parse inertia_matrix:" << inertia_matrix << endl;
        } else {
            int row = 0;
            for (std::vector<std::string> vec : vecs) {
                if (vec.size() != 3) {
                    cout << "Invalid vector size in: " << inertia_matrix << endl;
                    break;
                }
                for (int i = 0; i < 3; i++) {
                    I_(row, i) = std::stod(vec[i]);
                }
                row++;
            }
            if (row == 3) {
                valid_inertia = true;
            }
        }
        if (!valid_inertia) {
            cout << "Using identity matrix for inertia." << endl;
            I_ = Eigen::Matrix3d::Identity();
        }
        I_inv_ = I_.inverse();
    }

    // {
    //     // Parse Added Mass Matrix
    //     // Get the inertia matrix
    //     std::vector<std::vector<std::string>> vecs;
    //     std::string added_mass_matrix = sc::get<std::string>("added_mass",
    //                                                          params, "");
    //     bool valid_added_mass = false;
    //     if (!sc::get_vec_of_vecs(added_mass_matrix, vecs)) {
    //         cout << "Failed to parse added_mass:" << added_mass_matrix << endl;
    //     } else {
    //         int row = 0;
    //         for (std::vector<std::string> vec : vecs) {
    //             if (vec.size() != 6) {
    //                 cout << "Invalid vector size in: " << added_mass_matrix << endl;
    //                 break;
    //             }
    //             for (int i = 0; i < 6; i++) {
    //                 added_mass_(row, i) = std::stod(vec[i]);
    //             }
    //             row++;
    //         }
    //         if (row == 6) {
    //             valid_added_mass = true;
    //         }
    //     }
    //     if (!valid_added_mass) {
    //         cout << "Using identity matrix for added mass." << endl;
    //         added_mass_ = Eigen::Matrix<double, 6, 6>::Identity();
    //     }
    // }

    // Should we write a CSV file? What values should be written?
    write_csv_ = sc::get<bool>("write_csv", params, false);
    if (write_csv_) {
        csv_.open_output(parent_->mp()->log_dir() + "/"
                         + std::to_string(parent_->id().id())
                         + "-states.csv");
        cout << "Writing log to " + parent_->mp()->log_dir() + "/"
                         + std::to_string(parent_->id().id())
                         + "-states.csv" << endl;

        csv_.set_column_headers(sc::CSV::Headers{"t",
                    "x", "y", "z",
                    "U", "V", "W",
                    "P", "Q", "R",
                    "U_dot", "V_dot", "W_dot",
                    "P_dot", "Q_dot", "R_dot",
                    "roll", "pitch", "yaw",
                    "throttle", "thrust",
                    "elevator", "rudder"});
    }

    Xuu_ = sc::get<double>("Xuu", params, Xuu_);
    Yvv_ = sc::get<double>("Yvv", params, Yvv_);
    Zww_ = sc::get<double>("Zww", params, Zww_);
    Mww_ = sc::get<double>("Mww", params, Mww_);
    Yrr_ = sc::get<double>("Yrr", params, Yrr_);
    Mqq_ = sc::get<double>("Mqq", params, Mqq_);

    {
        // Parse center of gravity
        std::vector<double> c_g_vec;
        if (sc::get_vec<double>("c_g", params, ", ", c_g_vec, 3)) {
            c_g_ = sc::vec2eigen(c_g_vec);
        } else {
            cout << "Warning: Invalid center of gravity, c_g." << endl;
        }
    }

    {
        // Parse center of buoyancy
        std::vector<double> c_b_vec;
        if (sc::get_vec<double>("c_b", params, ", ", c_b_vec, 3)) {
            c_b_ = sc::vec2eigen(c_b_vec);
        } else {
            cout << "Warning: Invalid center of buoyancy, c_b." << endl;
        }
    }

    double m = mass_;
    double xg = c_g_(0);
    double yg = c_g_(1);
    double zg = c_g_(2);
    double Ixx = I_(0, 0);
    double Iyy = I_(1, 1);
    double Izz = I_(2, 2);

    masses_ <<
        m-Xu_dot, 0          , 0           , 0         , m*zg        , -m*yg      ,
        0       , m-Yv_dot   , 0           , -m*zg     , 0           , m*xg-Yr_dot,
        0       , 0          , m-Zw_dot    , m*yg      , -m*xg-Zq_dot, 0          ,
        0       , -m*zg      , m*yg        , Ixx-Kp_dot, 0           , 0          ,
        m*zg    , 0          , -m*xg-Mw_dot, 0         , Iyy-Mq_dot  , 0          ,
        -m*yg   , m*xg-Nv_dot, 0           , 0         , 0           , Izz-Nr_dot;

    masses_inverse_ = masses_.inverse();

    return true;
}

bool UUV6DOF::step(double time, double dt) {
    // Saturate inputs
    throttle_ = clamp(vars_.input(throttle_idx_), -1.0, 1.0);
    thrust_ = scale<double>(throttle_, -1.0, 1.0, thrust_min_, thrust_max_);
    Kprop_ = -scale<double>(throttle_, -1.0, 1.0, -Kprop_max_mag_, Kprop_max_mag_);

    delta_elevator_ = clamp(vars_.input(elevator_idx_), -1.0, 1.0);
    delta_elevator_ = scale<double>(delta_elevator_, -1.0, 1.0, delta_elevator_min_, delta_elevator_max_);

    delta_rudder_ = clamp(vars_.input(rudder_idx_), -1.0, 1.0);
    delta_rudder_ = scale<double>(delta_rudder_, -1.0, 1.0, delta_rudder_min_, delta_rudder_max_);

    // Cache values to calculate changes:
    Eigen::Vector3d prev_linear_vel_ENU(x_[Uw], x_[Vw], x_[Ww]);

    Eigen::Vector3d prev_linear_vel(x_[U], x_[V], x_[W]);
    Eigen::Vector3d prev_angular_vel(x_[P], x_[Q], x_[R]);

    force_ext_body_ = quat_body_.rotate_reverse(ext_force_);
    ext_force_ = Eigen::Vector3d::Zero(); // reset ext_force_ member variable

    ode_step(dt);

    // Limit depth/height
    if (x_[Zw] >= surface_height_) {
        x_[Zw] = surface_height_ - std::numeric_limits<double>::epsilon();
        if (x_[Q] > 0) {
            x_[Q] *= 0.90;
            x_[Q_dot] *= 0.10;
        }
    }

    quat_body_.set(x_[q0], x_[q1], x_[q2], x_[q3]);
    quat_body_.normalize();

    // Calculate change in velocity to populate acceleration elements
    Eigen::Vector3d linear_vel_ENU(x_[Uw], x_[Vw], x_[Ww]);
    Eigen::Vector3d linear_acc_ENU = (linear_vel_ENU - prev_linear_vel_ENU) / dt;
    Eigen::Vector3d linear_vel(x_[U], x_[V], x_[W]);
    Eigen::Vector3d angular_vel(x_[P], x_[Q], x_[R]);
    Eigen::Vector3d linear_acc = (linear_vel - prev_linear_vel) / dt;
    Eigen::Vector3d angular_acc = (angular_vel - prev_angular_vel) / dt;
    Eigen::Vector3d angular_acc_FLU(angular_acc(0), -angular_acc(1), -angular_acc(2));

    x_[U_dot] = linear_acc(0);
    x_[V_dot] = linear_acc(1);
    x_[W_dot] = linear_acc(2);
    x_[P_dot] = angular_acc(0);
    x_[Q_dot] = angular_acc(1);
    x_[R_dot] = angular_acc(2);

    // Rotate back to Z-axis pointing up
    state_->quat() = rot_180_x_axis_ * quat_body_;
    state_->quat().set(sc::Angles::angle_pi(state_->quat().roll()+M_PI),
                       state_->quat().pitch(), state_->quat().yaw());


    Eigen::Vector3d angvel_b_e_bodyRef = quat_body_.rotate(angular_vel);
    Eigen::Vector3d angvel_b_e_ENU;
    angvel_b_e_ENU << angvel_b_e_bodyRef[0], -angvel_b_e_bodyRef[1], -angvel_b_e_bodyRef[2];
    state_->set_ang_vel(angvel_b_e_ENU);

    state_->pos() << x_[Xw], x_[Yw], x_[Zw];
    state_->vel() << x_[Uw], x_[Vw], x_[Ww];

    linear_accel_body_ = state_->quat().rotate_reverse(linear_acc_ENU);
    ang_accel_body_ = angular_acc_FLU;

    if (write_csv_) {
        // Log state to CSV
        csv_.append(sc::CSV::Pairs{
                {"t", time},
                {"x", x_[Xw]},
                {"y", x_[Yw]},
                {"z", x_[Zw]},
                {"U", x_[U]},
                {"V", x_[V]},
                {"W", x_[W]},
                {"P", x_[P]},
                {"Q", x_[Q]},
                {"R", x_[R]},
                {"U_dot", x_[U_dot]},
                {"V_dot", x_[V_dot]},
                {"W_dot", x_[W_dot]},
                {"P_dot", x_[P_dot]},
                {"Q_dot", x_[Q_dot]},
                {"R_dot", x_[R_dot]},
                {"roll", quat_body_.roll()},
                {"pitch", quat_body_.pitch()},
                {"yaw", quat_body_.yaw()},
                {"throttle", throttle_},
                {"thrust", thrust_},
                {"elevator", delta_elevator_},
                {"rudder", delta_rudder_}});
    }

    return true;
}

void UUV6DOF::model(const vector_t &x, vector_t &dxdt, double t) {
    // Calculate force from weight in body frame:
    Eigen::Vector3d gravity_vector(0, 0, +mass_*g_);
    Eigen::Vector3d F_weight = quat_body_.rotate_reverse(gravity_vector);

    Eigen::Vector3d buoyancy_vector(0, 0, buoyancy_);
    Eigen::Vector3d F_buoyancy = quat_body_.rotate_reverse(buoyancy_vector);

    Eigen::Vector3d F_hydro = F_weight - F_buoyancy;
    Eigen::Vector3d Moments_hydro = c_g_.cross(F_weight) - c_b_.cross(F_buoyancy);

    double Xprop = thrust_;

    double X_ext = F_hydro(0) + Xuu_ * x[U] * std::abs(x[U]) + Xu_dot*x[U_dot]
        + Xwq*x[W]*x[Q] + Xqq*x[Q]*x[Q] + Xvr*x[V]*x[R] + Xrr*x[R]*x[R] + Xprop;

    double Y_ext = F_hydro(1) + Yvv_*x[V]*std::abs(x[V]) + Yrr*x[R]*std::abs(x[R])
        + Yv_dot*x[V_dot] + Yr_dot*x[R_dot] + Yur*x[U]*x[R] + Ywp*x[W]*x[P]
        + Ypq*x[P]*x[Q] + Yuv*x[U]*x[V] + Yuu_delta_r * pow(x[U], 2) * delta_rudder_;

    double Z_ext = F_hydro(2) + Zww_*x[W]*std::abs(x[W]) + Zqq*x[Q]*std::abs(x[Q])
        + Zw_dot*x[W_dot] + Zq_dot*x[Q_dot] + Zuq*x[U]*x[Q] + Zvp*x[V]*x[P]
        + Zrp*x[R]*x[P] + Zuw*x[U]*x[W] + Zuu_delta_s*pow(x[U], 2) * delta_elevator_;

    // Roll moment
    double K_ext = Moments_hydro(0) + Kpp*x[P]*std::abs(x[P]) + Kp_dot*x[P_dot] + Kprop_;

    // Pitch moment
    double M_ext = Moments_hydro(1) + Mww_*x[W]*std::abs(x[W]) + Mqq_*x[Q]*std::abs(x[Q])
        + Mw_dot*x[W_dot] + Mq_dot*x[Q_dot] + Muq*x[U]*x[Q] + Mvp*x[V]*x[P] + Mrp*x[R]*x[P]
        + Muw*x[U]*x[W] + Muu_delta_s*pow(x[U], 2) * delta_elevator_;

    // Yaw moment
    double N_ext = Moments_hydro(2) + Nvv*x[V]*std::abs(x[V]) + Nrr*x[R]*std::abs(x[R])
        + Nv_dot*x[V_dot] + Nr_dot*x[R_dot] + Nur*x[U]*x[R] + Nwp*x[W]*x[P] + Npq*x[P]*x[Q]
        + Nuv*x[U]*x[V] + Nuu_delta_r*pow(x[U], 2)*delta_rudder_;

    Eigen::Matrix<double, 6, 1> forces;
    forces << X_ext, Y_ext, Z_ext, K_ext, M_ext, N_ext;

    Eigen::Matrix<double, 6, 1> accelerations = masses_inverse_ * forces;

    dxdt[U] = accelerations(0, 0);
    dxdt[V] = accelerations(1, 0);
    dxdt[W] = accelerations(2, 0);
    dxdt[P] = accelerations(3, 0);
    dxdt[Q] = accelerations(4, 0);
    dxdt[R] = accelerations(5, 0);

    // Compute quaternion derivatives
    double lambda = 1 - (pow(x[q0], 2) + pow(x[q1], 2) + pow(x[q2], 2) + pow(x[q3], 2));
    dxdt[q0] = -0.5 * (x[q1]*x[P] + x[q2]*x[Q] + x[q3]*x[R]) + lambda * x[q0];
    dxdt[q1] = +0.5 * (x[q0]*x[P] + x[q2]*x[R] - x[q3]*x[Q]) + lambda * x[q1];
    dxdt[q2] = +0.5 * (x[q0]*x[Q] + x[q3]*x[P] - x[q1]*x[R]) + lambda * x[q2];
    dxdt[q3] = +0.5 * (x[q0]*x[R] + x[q1]*x[Q] - x[q2]*x[P]) + lambda * x[q3];

    // Normalize quaternion
    sc::Quaternion quat(x[q0], x[q1], x[q2], x[q3]);
    quat.w() = x[q0];
    quat.x() = x[q1];
    quat.y() = x[q2];
    quat.z() = x[q3];
    quat.normalize();

    // Integrate local velocities to compute local positions
    Eigen::Vector3d vel_local(x[U], x[V], x[W]);
    Eigen::Vector3d vel_world = quat.rotate(vel_local); // rot * vel_local;
    dxdt[Xw] = vel_world(0);
    dxdt[Yw] = -vel_world(1); // Due to rotated frame
    dxdt[Zw] = -vel_world(2); // Due to rotated frame

    // // Integrate local accelerations to compute global velocities
    Eigen::Vector3d acc_local = forces.head<3>() / mass_;
    Eigen::Vector3d acc_world = quat.rotate(acc_local); // rot * acc_local;
    dxdt[Uw] = acc_world(0);
    dxdt[Vw] = -acc_world(1); // Due to rotated frame
    dxdt[Ww] = -acc_world(2); // Due to rotated frame
    //
    // Accelerations get updated based on change in velocities
    dxdt[U_dot] = 0;
    dxdt[V_dot] = 0;
    dxdt[W_dot] = 0;
    dxdt[P_dot] = 0;
    dxdt[Q_dot] = 0;
    dxdt[R_dot] = 0;
}
} // namespace motion
} // namespace scrimmage
