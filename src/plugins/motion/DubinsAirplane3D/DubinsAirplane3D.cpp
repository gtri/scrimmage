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

#include <scrimmage/plugins/motion/DubinsAirplane3D/DubinsAirplane3D.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <boost/algorithm/clamp.hpp>

REGISTER_PLUGIN(scrimmage::MotionModel, scrimmage::motion::DubinsAirplane3D, DubinsAirplane3D_plugin)

namespace scrimmage {
namespace motion {

namespace sc = scrimmage;

namespace pl = std::placeholders;

enum ModelParams {
    U = 0,
    V,
    W,
    P, // roll rate
    Q, // pitch rate
    R, // yaw rate
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

bool DubinsAirplane3D::init(std::map<std::string, std::string> &info,
                            std::map<std::string, std::string> &params) {
    write_csv_ = sc::get<bool>("write_csv", params, false);

    // Model limits
    speed_max_ = sc::get<double>("speed_max", params, speed_max_);
    speed_min_ = sc::get<double>("speed_min", params, speed_min_);

    // Directly set speed, pitch, and roll
    desired_speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::In);
    desired_pitch_idx_ = vars_.declare(VariableIO::Type::desired_pitch, VariableIO::Direction::In);
    desired_roll_idx_ = vars_.declare(VariableIO::Type::desired_roll, VariableIO::Direction::In);

    x_.resize(MODEL_NUM_ITEMS);
    Eigen::Vector3d &pos = state_->pos();
    quat_world_ = state_->quat();
    quat_world_.normalize();

    quat_world_inverse_ = quat_world_.inverse();
    quat_world_inverse_.normalize();

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

    // Initial Local orientation (no rotation)
    quat_local_.w() = 1;
    quat_local_.x() = 0;
    quat_local_.y() = 0;
    quat_local_.z() = 0;
    quat_local_.normalize();
    x_[q0] = quat_local_.w();
    x_[q1] = quat_local_.x();
    x_[q2] = quat_local_.y();
    x_[q3] = quat_local_.z();

    if (write_csv_) {
        csv_.open_output(parent_->mp()->log_dir() + "/"
                         + std::to_string(parent_->id().id())
                         + "-dubins-airplane3d-states.csv");

        csv_.set_column_headers(sc::CSV::Headers{"t",
                        "x", "y", "z",
                        "U", "V", "W",
                        "P", "Q", "R",
                        "roll", "pitch", "yaw",
                        "speed",
                        "Uw", "Vw", "Ww"});
    }
    return true;
}

bool DubinsAirplane3D::step(double t, double dt) {
    // Get inputs and saturate
    speed_ = boost::algorithm::clamp(vars_.input(desired_speed_idx_), speed_min_, speed_max_);
    pitch_ = vars_.input(desired_pitch_idx_);
    roll_ = vars_.input(desired_roll_idx_);

    x_[Uw] = state_->vel()(0);
    x_[Vw] = state_->vel()(1);
    x_[Ww] = state_->vel()(2);

    x_[Xw] = state_->pos()(0);
    x_[Yw] = state_->pos()(1);
    x_[Zw] = state_->pos()(2);

    // state_->quat().normalize();
    state_->quat().set(roll_, pitch_, state_->quat().yaw());
    quat_local_ = state_->quat() * quat_world_inverse_;
    quat_local_.normalize();
    x_[q0] = quat_local_.w();
    x_[q1] = quat_local_.x();
    x_[q2] = quat_local_.y();
    x_[q3] = quat_local_.z();

    Eigen::Vector3d force_body = quat_local_.rotate_reverse(ext_force_);
    Eigen::Vector3d ext_moment_body = ext_moment_;
    ext_force_ = Eigen::Vector3d::Zero();
    ext_moment_ = Eigen::Vector3d::Zero();

    x_[U] = speed_ + force_body(0) / mass_;
    x_[V] = force_body(1) / mass_;
    x_[W] = force_body(2) / mass_;

    double turn_rate = 0;
    if (std::abs(speed_) >= std::numeric_limits<double>::epsilon()) {
        turn_rate = -g_ / speed_ * tan(roll_);
    }

    x_[P] = ext_moment_body(0) / mass_;
    x_[Q] = ext_moment_body(1) / mass_;
    x_[R] = ext_moment_body(2) / mass_ + turn_rate;

    ode_step(dt);

    // Normalize quaternion
    quat_local_.w() = x_[q0];
    quat_local_.x() = x_[q1];
    quat_local_.y() = x_[q2];
    quat_local_.z() = x_[q3];
    quat_local_.normalize();

    x_[q0] = quat_local_.w();
    x_[q1] = quat_local_.x();
    x_[q2] = quat_local_.y();
    x_[q3] = quat_local_.z();

    Eigen::Vector3d vel_local(x_[U], x_[V], x_[W]);

    // Convert local coordinates to world coordinates
    state_->quat() = quat_local_ * quat_world_;
    state_->quat().normalize();
    state_->pos() << x_[Xw], x_[Yw], x_[Zw];
    state_->vel() << state_->quat().toRotationMatrix() * vel_local;

    speed_ = vel_local.norm();

    if (write_csv_) {
        // Log state to CSV
        csv_.append(sc::CSV::Pairs{
                {"t", t},
                {"x", x_[Xw]},
                {"y", x_[Yw]},
                {"z", x_[Zw]},
                {"U", x_[U]},
                {"V", x_[V]},
                {"W", x_[W]},
                {"P", x_[P]},
                {"Q", x_[Q]},
                {"R", x_[R]},
                {"roll", state_->quat().roll()},
                {"pitch", state_->quat().pitch()},
                {"yaw", state_->quat().yaw()},
                {"speed", speed_},
                {"Uw", state_->vel()(0)},
                {"Vw", state_->vel()(1)},
                {"Ww", state_->vel()(2)}});
    }

    return true;
}

void DubinsAirplane3D::model(const vector_t &x , vector_t &dxdt , double t) {
    dxdt[U] = 0;
    dxdt[V] = 0;
    dxdt[W] = 0;

    dxdt[P] = 0;
    dxdt[Q] = 0;
    dxdt[R] = 0;

    double lambda = 1 - (pow(x[q0], 2) + pow(x[q1], 2) + pow(x[q2], 2) + pow(x[q3], 2));
    dxdt[q0] = -0.5 * (x[q1]*x[P] + x[q2]*x[Q] + x[q3]*x[R]) + lambda * x[q0];
    dxdt[q1] = +0.5 * (x[q0]*x[P] + x[q2]*x[R] - x[q3]*x[Q]) + lambda * x[q1];
    dxdt[q2] = +0.5 * (x[q0]*x[Q] + x[q3]*x[P] - x[q1]*x[R]) + lambda * x[q2];
    dxdt[q3] = +0.5 * (x[q0]*x[R] + x[q1]*x[Q] - x[q2]*x[P]) + lambda * x[q3];

    // Local position / velocity to global
    // Normalize quaternion
    sc::Quaternion quat(x[q0], x[q1], x[q2], x[q3]);
    quat.normalize();

    quat = quat * quat_world_;
    quat.normalize();

    // Convert local positions and velocities into global coordinates
    Eigen::Matrix3d rot = quat.toRotationMatrix();

    Eigen::Vector3d vel_local(x[U], x[V], x[W]);
    Eigen::Vector3d vel_world = rot * vel_local;
    dxdt[Xw] = vel_world(0);
    dxdt[Yw] = vel_world(1);
    dxdt[Zw] = vel_world(2);
}
} // namespace motion
} // namespace scrimmage
