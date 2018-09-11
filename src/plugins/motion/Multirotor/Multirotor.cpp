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

#include <scrimmage/plugins/motion/Multirotor/Multirotor.h>

#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/parse/MissionParse.h>

#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>

#include <iostream>

#include <boost/algorithm/clamp.hpp>

namespace sc = scrimmage;

using std::cout;
using std::endl;

REGISTER_PLUGIN(scrimmage::MotionModel,
                scrimmage::motion::Multirotor,
                Multirotor_plugin)

namespace scrimmage {
namespace motion {

Multirotor::Multirotor() : write_csv_(false) {
    x_.resize(MODEL_NUM_ITEMS);
}

bool Multirotor::init(std::map<std::string, std::string> &info,
                      std::map<std::string, std::string> &params) {

    show_shapes_ = sc::get<bool>("show_shapes", params, false);

    x_.resize(MODEL_NUM_ITEMS);
    Eigen::Vector3d &pos = state_->pos();

    x_[U] = 0;
    x_[V] = 0;
    x_[W] = 0;

    x_[P] = 0;
    x_[Q] = 0;
    x_[R] = 0;

    x_[Uw] = 0;
    x_[Vw] = 0;
    x_[Ww] = 0;

    x_[Xw] = pos(0);
    x_[Yw] = pos(1);
    x_[Zw] = pos(2);

    // Initial Local orientation
    x_[q0] = state_->quat().w();
    x_[q1] = state_->quat().x();
    x_[q2] = state_->quat().y();
    x_[q3] = state_->quat().z();

    // Parse XML parameters
    g_ = sc::get<double>("gravity_magnitude", params, 9.81);
    mass_ = sc::get<double>("mass", params, 1.2);

    // Parse inertia matrix
    std::vector<std::vector<std::string>> vecs;
    std::string inertia_matrix = sc::get<std::string>("inertia_matrix",
                                                      params, "");
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
            I_(row, 0) = std::stod(vec[0]);
            I_(row, 1) = std::stod(vec[1]);
            I_(row, 2) = std::stod(vec[2]);
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

    c_D_ = sc::get<double>("c_D", params, 0.058);
    c_T_ = sc::get<double>("c_T", params, 5.45e-6);
    c_Q_ = sc::get<double>("c_Q", params, 2.284e-7);
    wmax_ = sc::get<double>("omega_max", params, 1200.0);
    wmin_ = sc::get<double>("omega_min", params, 346.41);

    // Parse rotor configuration
    std::vector<std::vector<std::string>> vecs_r;
    std::string rotor_config = sc::get<std::string>("rotor_config",
                                                    params, "");
    if (!sc::get_vec_of_vecs(rotor_config, vecs_r)) {
        cout << "Failed to parse rotor_config:" << rotor_config << endl;
        return false;
    }

    for (std::vector<std::string> vec : vecs_r) {
        if (vec.size() != 7) {
            cout << "Invalid rotor_config: " << rotor_config << endl;
            return false;
        }

        Rotor r;
        if (vec[0] == "CW") {
            r.set_direction(Rotor::Direction::CW);
        } else if (vec[0] == "CCW") {
            r.set_direction(Rotor::Direction::CCW);
        } else {
            cout << "Invalid rotor direction: " << vec[0] << endl;
            r.set_direction(Rotor::Direction::CW);
        }

        r.set_offset(Eigen::Vector3d(std::stod(vec[1]),
                                     std::stod(vec[2]),
                                     std::stod(vec[3])));

        r.set_quat(sc::Quaternion(sc::Angles::deg2rad(std::stod(vec[4])),
                                  sc::Angles::deg2rad(std::stod(vec[5])),
                                  sc::Angles::deg2rad(std::stod(vec[6]))));
        rotors_.push_back(r);
    }


    motor_idx_vec_.resize(rotors_.size());
    ctrl_u_.resize(rotors_.size());
    for (unsigned int i = 0; i < rotors_.size(); i++) {
        std::string name = "motor_"+std::to_string(i);
        motor_idx_vec_(i) = vars_.declare(name, VariableIO::Direction::In);
    }


    write_csv_ = sc::get<bool>("write_csv", params, false);
    if (write_csv_) {
        csv_.open_output(parent_->mp()->log_dir() + "/"
                         + std::to_string(parent_->id().id())
                         + "-states.csv");

        csv_.set_column_headers(sc::CSV::Headers{"t",
                    "x", "y", "z",
                    "U", "V", "W",
                    "P", "Q", "R",
                    "AXb", "AYb", "AZb",
                    "WXDOTb", "WYDOTb", "WZDOTb",
                    "roll", "pitch", "yaw",
                    "w_1", "w_2", "w_3", "w_4"});
    }

    return true;
}

bool Multirotor::step(double time, double dt) {
    for (int i = 0; i < motor_idx_vec_.size(); i++) {
        ctrl_u_(i) = boost::algorithm::clamp(vars_.input(motor_idx_vec_(i)), wmin_, wmax_);
    }

    // TODO: convert global linear velocity and angular velocity into local
    // velocities

    x_[q0] = state_->quat().w();
    x_[q1] = state_->quat().x();
    x_[q2] = state_->quat().y();
    x_[q3] = state_->quat().z();

    x_[Uw] = state_->vel()(0);
    x_[Vw] = state_->vel()(1);
    x_[Ww] = state_->vel()(2);

    x_[Xw] = state_->pos()(0);
    x_[Yw] = state_->pos()(1);
    x_[Zw] = state_->pos()(2);

    // Cache values to calculate changes:
    Eigen::Vector3d prev_linear_vel_w(x_[Uw], x_[Vw], x_[Ww]);
    Eigen::Vector3d prev_angular_vel(x_[P], x_[Q], x_[R]);

    // Apply any external forces
    force_ext_body_ = state_->quat().rotate_reverse(ext_force_);
    ext_force_ = Eigen::Vector3d::Zero(); // reset ext_force_ member variable

    ode_step(dt); // step the motion model ODE solver


    state_->quat().set(x_[q0], x_[q1], x_[q2], x_[q3]);
    state_->quat().normalize();

    // Calculate change in velocity to populate acceleration elements
    Eigen::Vector3d linear_vel_w(x_[Uw], x_[Vw], x_[Ww]);
    Eigen::Vector3d linear_acc_w = (linear_vel_w - prev_linear_vel_w) / dt;
    Eigen::Vector3d linear_acc = state_->quat().rotate_reverse(linear_acc_w);
    Eigen::Vector3d angular_vel(x_[P], x_[Q], x_[R]);
    Eigen::Vector3d angular_acc = (angular_vel - prev_angular_vel) / dt;


    // Convert local coordinates to world coordinates
    state_->pos() << x_[Xw], x_[Yw], x_[Zw];
    state_->vel() << x_[Uw], x_[Vw], x_[Ww];
    state_->set_ang_vel(state_->quat().rotate(angular_vel));
    linear_accel_body_ = linear_acc;
    ang_accel_body_ = angular_acc;

    // draw velocity
    if (show_shapes_) {
        sc::ShapePtr line(new sp::Shape());
        line->set_opacity(1.0);
        sc::set(line->mutable_color(), 255, 255, 0);
        sc::set(line->mutable_line()->mutable_start(), state_->pos());
        sc::set(line->mutable_line()->mutable_end(), state_->pos() + state_->vel());
        draw_shape(line);
    }

    // draw angular velocity
    if (show_shapes_) {
        sc::ShapePtr line(new sp::Shape());
        line->set_opacity(1.0);
        sc::set(line->mutable_color(), 0, 255, 255);
        sc::set(line->mutable_line()->mutable_start(), state_->pos());
        sc::set(line->mutable_line()->mutable_end(), state_->pos() + state_->ang_vel()*10);
        draw_shape(line);
    }

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
                {"AXb", linear_accel_body_(0)},
                {"AYb", linear_accel_body_(1)},
                {"AZb", linear_accel_body_(2)},
                {"WXDOTb", ang_accel_body_(0)},
                {"WYDOTb", ang_accel_body_(1)},
                {"WZDOTb", ang_accel_body_(2)},
                {"roll", state_->quat().roll()},
                {"pitch", state_->quat().pitch()},
                {"yaw", state_->quat().yaw()},
                {"w_1", ctrl_u_(0)},
                {"w_2", ctrl_u_(1)},
                {"w_3", ctrl_u_(2)},
                {"w_4", ctrl_u_(3)}});
    }

    return true;
}

void Multirotor::model(const vector_t &x , vector_t &dxdt , double t) {
    // Omega values for each rotor
    Eigen::VectorXd omega = ctrl_u_;
    Eigen::VectorXd omega_sq = omega.cwiseProduct(omega);

    // Calculate force from combined rotor thrust
    Eigen::Vector3d F_thrust(0, 0, 0);
    Eigen::VectorXd rotor_thrust(rotors_.size());
    Eigen::VectorXd rotor_torque(rotors_.size());

    for (unsigned int i = 0; i < rotors_.size(); i++) {
        rotor_thrust(i) =  c_T_ * omega_sq(i);
        rotor_torque(i) = rotors_[i].direction() * c_Q_ * omega_sq(i);
        F_thrust(2) += rotor_thrust(i);
    }

    // Calculate force from weight in body frame:
    Eigen::Vector3d gravity_vector(0, 0, -mass_*g_);
    Eigen::Vector3d F_weight = state_->quat().rotate_reverse(gravity_vector);

    // Calculate force from drag (TODO: Check source)
    Eigen::Vector3d vel_body(x_[U], x_[V], x_[W]);
    double vel_mag = vel_body.norm();
    Eigen::Vector3d F_drag = vel_body * (-0.5 * c_D_ * vel_mag);

    // Calculate total force
    Eigen::Vector3d F_total = F_thrust + F_weight + F_drag + force_ext_body_;

    dxdt[U] = x[V]*x[R] - x[W]*x[Q] + F_total(0) / mass_;
    dxdt[V] = x[W]*x[P] - x[U]*x[R] + F_total(1) / mass_;
    dxdt[W] = x[U]*x[Q] - x[V]*x[P] + F_total(2) / mass_;

    // Calculate moments from thrust
    Eigen::Vector3d Moments_thrust(0, 0, 0); // L, M, N

    for (unsigned int i = 0; i < rotors_.size(); i++) {
        double length = rotors_[i].offset_length();
        double xy_angle = rotors_[i].xy_angle();

        Moments_thrust(0) += rotor_thrust(i) * length * sin(xy_angle);
        Moments_thrust(1) -= rotor_thrust(i) * length * cos(xy_angle);
        Moments_thrust(2) += rotor_torque(i);
    }

    Eigen::Vector3d Moments_total = Moments_thrust;

    Eigen::Vector3d pqr(x_[P], x_[Q], x_[R]);
    Eigen::Vector3d pqr_dot = I_inv_ * (Moments_total - pqr.cross(I_*pqr));
    dxdt[P] = pqr_dot(0);
    dxdt[Q] = pqr_dot(1);
    dxdt[R] = pqr_dot(2);

    // Update quaternion from angular accelerations
    double lambda = 1 - (pow(x[q0], 2) + pow(x[q1], 2) + pow(x[q2], 2) + pow(x[q3], 2));
    dxdt[q0] = -0.5 * (x[q1]*x[P] + x[q2]*x[Q] + x[q3]*x[R]) + lambda * x[q0];
    dxdt[q1] = +0.5 * (x[q0]*x[P] + x[q2]*x[R] - x[q3]*x[Q]) + lambda * x[q1];
    dxdt[q2] = +0.5 * (x[q0]*x[Q] + x[q3]*x[P] - x[q1]*x[R]) + lambda * x[q2];
    dxdt[q3] = +0.5 * (x[q0]*x[R] + x[q1]*x[Q] - x[q2]*x[P]) + lambda * x[q3];

    // Local position / velocity to global
    // Normalize quaternion
    sc::Quaternion quat(x[q0], x[q1], x[q2], x[q3]);
    quat.w() = x[q0];
    quat.x() = x[q1];
    quat.y() = x[q2];
    quat.z() = x[q3];
    quat.normalize();

    Eigen::Vector3d vel_local(x[U], x[V], x[W]);
    Eigen::Vector3d vel_world = quat.rotate(vel_local);
    dxdt[Xw] = vel_world(0);
    dxdt[Yw] = vel_world(1);
    dxdt[Zw] = vel_world(2);

    //// TODO: Should these be cached from previous run or should the current dxdt

    Eigen::Vector3d acc_local = F_total / mass_; // this is in body frame
    Eigen::Vector3d acc_world = quat.rotate(acc_local);
    dxdt[Uw] = acc_world(0);
    dxdt[Vw] = acc_world(1);
    dxdt[Ww] = acc_world(2);
}
} // namespace motion
} // namespace scrimmage
