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

#include <scrimmage/plugins/motion/FixedWing6DOF/FixedWing6DOF.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>

#include <Eigen/Dense>

#include <iostream>
#include <iomanip>

#include <boost/algorithm/clamp.hpp>

using std::cout;
using std::endl;

using boost::algorithm::clamp;

REGISTER_PLUGIN(scrimmage::MotionModel, scrimmage::motion::FixedWing6DOF, FixedWing6DOF_plugin)

namespace scrimmage {
namespace motion {

namespace sc = scrimmage;
namespace pl = std::placeholders;

FixedWing6DOF::FixedWing6DOF() {
    Eigen::AngleAxisd aa(M_PI, Eigen::Vector3d::UnitX());
    rot_180_x_axis_ = Eigen::Quaterniond(aa);
}

std::tuple<int, int, int> FixedWing6DOF::version() {
    return std::tuple<int, int, int>(0, 0, 1);
}

bool FixedWing6DOF::init(std::map<std::string, std::string> &info,
                          std::map<std::string, std::string> &params) {
    // Setup variable index for controllers
    throttle_idx_ = vars_.declare(VariableIO::Type::throttle, VariableIO::Direction::In);
    elevator_idx_ = vars_.declare(VariableIO::Type::elevator, VariableIO::Direction::In);
    aileron_idx_ = vars_.declare(VariableIO::Type::aileron, VariableIO::Direction::In);
    rudder_idx_ = vars_.declare(VariableIO::Type::rudder, VariableIO::Direction::In);

    x_.resize(MODEL_NUM_ITEMS);

    // Need to rotate axes by 180 degrees around X-axis SCRIMMAGE's global
    // frame uses Z-axis pointing up. Many aircraft equations of motion are
    // specified with Z-axis pointing down.
    quat_body_ = rot_180_x_axis_ * state_->quat();
    quat_body_.set(sc::Angles::angle_pi(quat_body_.roll()+M_PI),
                   quat_body_.pitch(), quat_body_.yaw());
    quat_body_.normalize();

    Eigen::Vector3d vel_body(state_->vel()(0), -state_->vel()(1), -state_->vel()(2));
    vel_body = quat_body_.rotate_reverse(vel_body);

    x_[U] = vel_body(0);
    x_[V] = vel_body(1);
    x_[W] = vel_body(2);

    Eigen::Vector3d local_ang_vel(state_->ang_vel()(0), -state_->ang_vel()(1), -state_->ang_vel()(2));
    local_ang_vel = quat_body_.rotate_reverse(local_ang_vel);

    x_[P] = local_ang_vel(0);
    x_[Q] = local_ang_vel(1);
    x_[R] = local_ang_vel(2);

    x_[Uw] = state_->vel()(0);
    x_[Vw] = state_->vel()(1);
    x_[Ww] = state_->vel()(2);

    x_[Xw] = state_->pos()(0);
    x_[Yw] = state_->pos()(1);
    x_[Zw] = state_->pos()(2);

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

    // Get the inertia matrix
    std::vector<std::vector<std::string>> vecs;
    std::string inertia_matrix = sc::get<std::string>("inertia_matrix",
                                                      params, "");

    // If the inertia matrix is described using slug_ft_sq, it has priority
    bool convert_slug = false;
    if (params.count("inertia_matrix_slug_ft_sq")) {
        convert_slug = true;
        inertia_matrix = sc::get<std::string>("inertia_matrix_slug_ft_sq",
                                              params, "");
    }

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
                // If the matrix was defined in slug*ft^2, convert it to SI
                if (convert_slug) {
                    I_(row, i) *= 1.35581795; // slug*ft^2 to kg*m^2
                }
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

    // Drawing
    draw_vel_ = sc::get<bool>("draw_vel", params, false);
    draw_ang_vel_ = sc::get<bool>("draw_ang_vel", params, false);

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
                    "U", "V", "W", "alpha", "alpha_dot", "beta",
                    "P", "Q", "R",
                    "Ax_b", "Ay_b", "Az_b",
                    "AngAccelx_b", "AngAccely_b", "AngAccelz_b",
                    "roll", "pitch", "yaw",
                    "throttle", "thrust", "elevator", "aileron", "rudder"});
    }

    rho_ = sc::get<double>("air_density", params, rho_); // air density

    // thrust and dimensional specs
    thrust_min_ = sc::get<double>("thrust_min", params, thrust_min_);
    thrust_max_ = sc::get<double>("thrust_max", params, thrust_max_);
    delta_elevator_min_ = sc::get<double>("delta_elevator_min", params, delta_elevator_min_);
    delta_elevator_max_ = sc::get<double>("delta_elevator_max", params, delta_elevator_max_);
    delta_aileron_min_ = sc::get<double>("delta_aileron_min", params, delta_aileron_min_);
    delta_aileron_max_ = sc::get<double>("delta_aileron_max", params, delta_aileron_max_);
    delta_rudder_min_ = sc::get<double>("delta_rudder_min", params, delta_rudder_min_);
    delta_rudder_max_ = sc::get<double>("delta_rudder_max", params, delta_rudder_max_);

    // Aircraft dimensional specs
    b_ = sc::get<double>("wing_span", params, b_);
    S_ = sc::get<double>("surface_area_of_wing", params, S_);
    c_ = sc::get<double>("chord_length", params, c_);
    e_ = sc::get<double>("efficiency_factor", params, e_);
    AR_ = b_/c_;

    // Drag coefficients
    C_D0_ = sc::get<double>("C_D0", params, C_D0_);
    C_D_alpha_ = sc::get<double>("C_D_alpha", params, C_D_alpha_);
    C_D_delta_elevator_ = sc::get<double>("C_D_delta_elevator_", params, C_D_delta_elevator_);

    // Lift coefficients
    C_L0_ = sc::get<double>("C_L0", params, C_L0_);
    C_L_alpha_ = sc::get<double>("C_L_alpha", params, C_L_alpha_);
    C_LQ_ = sc::get<double>("C_LQ", params, C_LQ_);
    C_L_alpha_dot_ = sc::get<double>("C_L_alpha_dot", params, C_L_alpha_dot_);
    C_L_delta_elevator_ = sc::get<double>("C_L_delta_elevator", params, C_L_delta_elevator_);

    // Side force coefficients
    C_Y_beta_ = sc::get<double>("C_Y_beta", params, C_Y_beta_);
    C_Y_delta_rudder_ = sc::get<double>("C_Y_delta_rudder", params, C_Y_delta_rudder_);

    // Roll moment coefficients
    C_L_beta_ = sc::get<double>("C_L_beta", params, C_L_beta_);
    C_LP_ = sc::get<double>("C_LP", params, C_LP_);
    C_LR_ = sc::get<double>("C_LR", params, C_LR_);
    C_L_delta_aileron_ = sc::get<double>("C_L_delta_aileron", params, C_L_delta_aileron_);
    C_L_delta_rudder_ = sc::get<double>("C_L_delta_rudder", params, C_L_delta_rudder_);

    // Pitch moment coefficients
    C_M0_ = sc::get<double>("C_M0", params, C_M0_);
    C_MQ_ = sc::get<double>("C_MQ", params, C_MQ_);
    C_M_alpha_ = sc::get<double>("C_M_alpha", params, C_M_alpha_);
    C_M_alpha_dot_ = sc::get<double>("C_M_alpha_dot", params, C_M_alpha_dot_);
    C_M_delta_elevator_ = sc::get<double>("C_M_delta_elevator", params, C_M_delta_elevator_);

    // Yaw moment coefficients
    C_N_beta_ = sc::get<double>("C_N_beta", params, C_N_beta_);
    C_NP_ = sc::get<double>("C_NP", params, C_NP_);
    C_NR_ = sc::get<double>("C_NR", params, C_NR_);
    C_N_delta_aileron_ = sc::get<double>("C_N_delta_aileron", params, C_N_delta_aileron_);
    C_N_delta_rudder_ = sc::get<double>("C_N_delta_rudder", params, C_N_delta_rudder_);

    return true;
}

bool FixedWing6DOF::step(double time, double dt) {
    // Get inputs and saturate
    throttle_ = clamp(vars_.input(throttle_idx_), -1.0, 1.0);
    thrust_ = scale<double>(throttle_, -1.0, 1.0, thrust_min_, thrust_max_);

    delta_elevator_ = clamp(vars_.input(elevator_idx_), delta_elevator_min_, delta_elevator_max_);
    delta_aileron_ = clamp(vars_.input(aileron_idx_), delta_aileron_min_, delta_aileron_max_);
    delta_rudder_ = clamp(vars_.input(rudder_idx_), delta_rudder_min_, delta_rudder_max_);

#if 0
    int prec = 5;
    cout<< "*************************" << endl;
    cout<< std::setprecision(prec) << "thrust_:         " << thrust_ << endl;
    cout<< std::setprecision(prec) << "delta_elevator_: " << delta_elevator_ << endl;
    cout<< std::setprecision(prec) << "delta_aileron_:  " << delta_aileron_ << endl;
    cout<< std::setprecision(prec) << "delta_rudder_:   " << delta_rudder_ << endl;
#endif

    quat_body_ = rot_180_x_axis_ * state_->quat();
    quat_body_.set(sc::Angles::angle_pi(quat_body_.roll()+M_PI),
                   quat_body_.pitch(), quat_body_.yaw());
    quat_body_.normalize();

    Eigen::Vector3d local_lin_vel(state_->vel()(0), -state_->vel()(1), -state_->vel()(2));
    local_lin_vel = quat_body_.rotate_reverse(local_lin_vel);

    x_[U] = local_lin_vel(0);
    x_[V] = local_lin_vel(1);
    x_[W] = local_lin_vel(2);

    Eigen::Vector3d local_ang_vel(state_->ang_vel()(0), -state_->ang_vel()(1), -state_->ang_vel()(2));
    local_ang_vel = quat_body_.rotate_reverse(local_ang_vel);

    x_[P] = local_ang_vel(0);
    x_[Q] = local_ang_vel(1);
    x_[R] = local_ang_vel(2);

    x_[Uw] = state_->vel()(0);
    x_[Vw] = state_->vel()(1);
    x_[Ww] = state_->vel()(2);

    x_[Xw] = state_->pos()(0);
    x_[Yw] = state_->pos()(1);
    x_[Zw] = state_->pos()(2);

    x_[q0] = quat_body_.w();
    x_[q1] = quat_body_.x();
    x_[q2] = quat_body_.y();
    x_[q3] = quat_body_.z();

    x_[U_dot] = linear_accel_body_(0);
    x_[V_dot] = linear_accel_body_(1);
    x_[W_dot] = linear_accel_body_(2);
    x_[P_dot] = ang_accel_body_(0);
    x_[Q_dot] = ang_accel_body_(1);
    x_[R_dot] = ang_accel_body_(2);

    // Cache values to calculate changes:
    Eigen::Vector3d prev_linear_vel_ENU(x_[Uw], x_[Vw], x_[Ww]);
    Eigen::Vector3d prev_angular_vel(x_[P], x_[Q], x_[R]);

    // Apply any external forces (todo)
    force_ext_body_ = quat_body_.rotate_reverse(ext_force_);
    ext_force_ = Eigen::Vector3d::Zero(); // reset ext_force_ member variable

    alpha_ = atan2(x_[W], x_[U]); // angle of attack
    // TODO: Need to compute alpha_dot without using alpha_prev_
    alpha_dot_ = 0;
    // alpha_dot_ = (alpha_ - alpha_prev_) / dt;
    // alpha_prev_ = alpha_;

    ode_step(dt);

    quat_body_.set(x_[q0], x_[q1], x_[q2], x_[q3]);
    quat_body_.normalize();

    // Calculate change in velocity to populate acceleration elements
    Eigen::Vector3d linear_vel_ENU(x_[Uw], x_[Vw], x_[Ww]);
    Eigen::Vector3d linear_acc_ENU = (linear_vel_ENU - prev_linear_vel_ENU) / dt;
    Eigen::Vector3d angular_vel(x_[P], x_[Q], x_[R]);
    Eigen::Vector3d angular_acc = (angular_vel - prev_angular_vel) / dt;
    Eigen::Vector3d angular_acc_FLU(angular_acc(0), -angular_acc(1), -angular_acc(2));

    // Rotate back to Z-axis pointing up
    state_->quat() = rot_180_x_axis_ * quat_body_;
    state_->quat().set(sc::Angles::angle_pi(state_->quat().roll()+M_PI),
                       state_->quat().pitch(), state_->quat().yaw());
    state_->quat().normalize();

    Eigen::Vector3d angvel_b_e_bodyRef = quat_body_.rotate(angular_vel);
    Eigen::Vector3d angvel_b_e_ENU;
    angvel_b_e_ENU << angvel_b_e_bodyRef(0), -angvel_b_e_bodyRef(1), -angvel_b_e_bodyRef(2);

    state_->set_pos(Eigen::Vector3d(x_[Xw], x_[Yw], x_[Zw]));
    state_->set_vel(linear_vel_ENU);
    state_->set_ang_vel(angvel_b_e_ENU);

    linear_accel_body_ = state_->quat().rotate_reverse(linear_acc_ENU);
    ang_accel_body_ = angular_acc_FLU;

    // draw velocity
    if (draw_vel_) {
        sc::ShapePtr line(new sp::Shape());
        line->set_opacity(1.0);
        sc::set(line->mutable_color(), 255, 0, 0);
        sc::set(line->mutable_line()->mutable_start(), state_->pos());
        sc::set(line->mutable_line()->mutable_end(), state_->pos() + state_->vel());
        draw_shape(line);
    }

    // draw angular velocity
    if (draw_ang_vel_) {
        sc::ShapePtr line(new sp::Shape());
        line->set_opacity(1.0);
        sc::set(line->mutable_color(), 0, 255, 0);
        sc::set(line->mutable_line()->mutable_start(), state_->pos());
        sc::set(line->mutable_line()->mutable_end(), state_->pos() + state_->ang_vel());
        draw_shape(line);
    }

    if (write_csv_) {
        double beta = atan2(x_[V], x_[U]); // side slip
        // Log state to CSV
        csv_.append(sc::CSV::Pairs{
                {"t", time},
                {"x", x_[Xw]},
                {"y", x_[Yw]},
                {"z", x_[Zw]},
                {"U", x_[U]},
                {"V", x_[V]},
                {"W", x_[W]},
                {"alpha", alpha_},
                {"alpha_dot", alpha_dot_},
                {"beta", beta},
                {"P", x_[P]},
                {"Q", x_[Q]},
                {"R", x_[R]},
                {"Ax_b", linear_accel_body_(0)},
                {"Ay_b", linear_accel_body_(1)},
                {"Az_b", linear_accel_body_(2)},
                {"AngAccelx_b", ang_accel_body_(0)},
                {"AngAccely_b", ang_accel_body_(1)},
                {"AngAccelz_b", ang_accel_body_(2)},
                {"roll", quat_body_.roll()},
                {"pitch", quat_body_.pitch()},
                {"yaw", quat_body_.yaw()},
                {"throttle", throttle_},
                {"thrust", thrust_},
                {"elevator", delta_elevator_},
                {"aileron", delta_aileron_},
                {"rudder", delta_rudder_}});
    }
    return true;
}

void FixedWing6DOF::model(const vector_t &x , vector_t &dxdt , double t) {
    // Calculate velocity magnitude (handle zero velocity)
    double V_tau = sqrt(pow(x_[U], 2) + pow(x_[V], 2) + pow(x_[W], 2));
    if (std::abs(V_tau) < std::numeric_limits<double>::epsilon()) {
        V_tau = 0.00001;
    }

    // Calculate commonly used quantities
    double beta = atan2(x_[V], x_[U]); // side slip
    // double delta_Ve = 0.05*V_tau; // wind velocity across tail of aircraft (approximation)
    // double VtauVe = pow((V_tau + delta_Ve)/V_tau, 2);
    double pVtS = rho_ * pow(V_tau, 2) * S_ / 2.0;


    // Calculate lift, drag, and side_force magnitudes
    double CL = (C_L0_ + C_L_alpha_*alpha_ + C_LQ_*x_[Q] * c_ / (2*V_tau) +
                   C_L_alpha_dot_ * alpha_dot_ * c_ / (2*V_tau) +
                   C_L_delta_elevator_ * delta_elevator_);
    double lift = CL * pVtS;

    double drag = (C_D0_ + C_D_alpha_*std::abs(alpha_) + CL*CL/(M_PI*AR_*e_) +
                   C_D_delta_elevator_*std::abs(delta_elevator_)) * pVtS;


    double side_force = (C_Y_beta_*beta + C_Y_delta_rudder_*delta_rudder_) * pVtS;

    // Bring lift, drag, side_force magnitudes into body frame
    Eigen::Vector3d F_aero(lift*sin(alpha_) - drag*cos(alpha_) - side_force*sin(beta),
                           side_force*cos(beta),
                           -lift*cos(alpha_) - drag*sin(alpha_));

    // Calculate force from weight in body frame:
    Eigen::Vector3d gravity_vector(0, 0, +mass_*g_);
    Eigen::Vector3d F_weight = quat_body_.rotate_reverse(gravity_vector);

    Eigen::Vector3d F_thrust(thrust_, 0, 0);
    Eigen::Vector3d F_total = F_weight + F_thrust + F_aero;

    // simple ground contact model
    Eigen::Vector3d F_ground_W(0, 0, 0);
    Eigen::Vector3d F_ground(0, 0, 0);
    if (x_[Zw] < 0) {
        double wn = 20;
        double Kp = wn*wn*mass_;
        double Kd = 2*wn;

        F_ground_W[2] = std::max(0.0, -Kp*(x_[Zw] - 0.0) - Kd*(x_[Ww] - 0.0));

        F_ground(0) =  F_ground_W(0);
        F_ground(1) = -F_ground_W(1);
        F_ground(2) = -F_ground_W(2);
        F_ground = quat_body_.rotate_reverse(F_ground);

        F_total += F_ground;
    }

#if 0
    int prec = 5;
    cout<< "*************************" << endl;
    cout<< std::setprecision(prec) << "alpha:    " << alpha_ << endl;
    cout<< std::setprecision(prec) << "lift:     " << lift << endl;
    cout<< std::setprecision(prec) << "drag:     " << drag << endl;
    cout<< std::setprecision(prec) << "F_ground: " << F_ground[0] << " " << F_ground[1] << " " << F_ground[2] << " " << endl;
    cout<< std::setprecision(prec) << "F_weight: " << F_weight[0] << " " << F_weight[1] << " " << F_weight[2] << " " << endl;
    cout<< std::setprecision(prec) << "thrust_:  " << thrust_ << endl;
    cout<< std::setprecision(prec) << "F_total:  " << F_total[0] << " " << F_total[1] << " " << F_total[2] << " " << endl;
#endif

    // Calculate body frame linear velocities
    dxdt[U] = x[V]*x[R] - x[W]*x[Q] + F_total(0) / mass_;
    dxdt[V] = x[W]*x[P] - x[U]*x[R] + F_total(1) / mass_;
    dxdt[W] = x[U]*x[Q] - x[V]*x[P] + F_total(2) / mass_;

    // Calculate moments;
    Eigen::Vector3d Moments_thrust(0, 0, 0); // no moment from thrust
    Eigen::Vector3d Moments_torque(0, 0, 0); // no moment from torque
    Eigen::Vector3d Moments_gyro(0, 0, 0);   // no moment from gyro effect

    // Calculate moments from aerodynamic forces
    Eigen::Vector3d Moments_aero((C_L_beta_*beta + C_LP_*x_[P]*b_/(2*V_tau) + C_LR_*x_[R]*b_/(2*V_tau) + C_L_delta_aileron_*delta_aileron_ + C_L_delta_rudder_*delta_rudder_) * pVtS*b_,
                                 (C_M0_ + C_M_alpha_*alpha_ + C_MQ_*x_[Q]*c_/(2*V_tau) + C_M_alpha_dot_*alpha_dot_*c_/(2*V_tau) + C_M_delta_elevator_*delta_elevator_) * pVtS*c_,
                                 (C_N_beta_*beta + C_NP_*x_[P]*b_/(2*V_tau) + C_NR_*x_[R]*b_/(2*V_tau) + C_N_delta_aileron_*delta_aileron_ + C_N_delta_rudder_*delta_rudder_) * pVtS*b_);
    // Sum moments
    Eigen::Vector3d Moments_total = Moments_aero + Moments_thrust +
        Moments_torque + Moments_gyro;

    // Calculate rotational velocites
    Eigen::Vector3d pqr(x_[P], x_[Q], x_[R]);
    Eigen::Vector3d pqr_dot = I_inv_ * (Moments_total - pqr.cross(I_*pqr));
    dxdt[P] = pqr_dot(0);
    dxdt[Q] = pqr_dot(1);
    dxdt[R] = pqr_dot(2);

    // Compute quaternion derivatives
    double lambda = 1 - (pow(x[q0], 2) + pow(x[q1], 2) + pow(x[q2], 2) + pow(x[q3], 2));
    dxdt[q0] = -0.5 * (x[q1]*x[P] + x[q2]*x[Q] + x[q3]*x[R]) + lambda * x[q0];
    dxdt[q1] = +0.5 * (x[q0]*x[P] + x[q2]*x[R] - x[q3]*x[Q]) + lambda * x[q1];
    dxdt[q2] = +0.5 * (x[q0]*x[Q] + x[q3]*x[P] - x[q1]*x[R]) + lambda * x[q2];
    dxdt[q3] = +0.5 * (x[q0]*x[R] + x[q1]*x[Q] - x[q2]*x[P]) + lambda * x[q3];

    // Normalize quaternion
    sc::Quaternion quat(x[q0], x[q1], x[q2], x[q3]);
    quat.normalize();

    // Integrate local velocities to compute local positions
    Eigen::Vector3d vel_local(x[U], x[V], x[W]);
    Eigen::Vector3d vel_world = quat.rotate(vel_local); // rot * vel_local;
    dxdt[Xw] = vel_world(0);
    dxdt[Yw] = -vel_world(1); // Due to rotated frame
    dxdt[Zw] = -vel_world(2); // Due to rotated frame

    // Integrate local accelerations to compute global velocities
    Eigen::Vector3d acc_local = F_total / mass_;
    Eigen::Vector3d acc_world = quat.rotate(acc_local); // rot * acc_local;
    dxdt[Uw] = acc_world(0);
    dxdt[Vw] = -acc_world(1); // Due to rotated frame
    dxdt[Ww] = -acc_world(2); // Due to rotated frame
}
} // namespace motion
} // namespace scrimmage
