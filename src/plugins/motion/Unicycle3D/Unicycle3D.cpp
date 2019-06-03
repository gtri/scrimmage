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

#include <scrimmage/plugins/motion/Unicycle3D/Unicycle3D.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <boost/algorithm/clamp.hpp>

REGISTER_PLUGIN(scrimmage::MotionModel, scrimmage::motion::Unicycle3D, Unicycle3D_plugin)

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

bool Unicycle3D::init(std::map<std::string, std::string> &info,
                      std::map<std::string, std::string> &params) {

    auto get_max_min = [&params] (const std::string &str, double &max,
                                  double &min) {
        max = sc::get<double>(str + "_max", params, max);
        min = sc::get<double>(str + "_min", params, -max);
        if (max < min) {
            std::swap(max, min);
        }
    };

    get_max_min("speed", speed_max_, speed_min_);
    get_max_min("pitch_rate", pitch_rate_max_, pitch_rate_min_);
    get_max_min("roll_rate", roll_rate_max_, roll_rate_min_);
    get_max_min("turn_rate", turn_rate_max_, turn_rate_min_);

    accel_max_ = sc::get<double>("accel_max", params, -1);
    accel_min_ = sc::get<double>("accel_max", params, -accel_max_);

    // If the acceleration max is less than zero, directly use speed
    // input. Otherwise, use acceleration input.
    if (accel_max_ < 0) {
        speed_idx_ = vars_.declare(VariableIO::Type::speed, VariableIO::Direction::In);
    } else {
        accel_idx_ = vars_.declare(VariableIO::Type::acceleration_x, VariableIO::Direction::In);
        use_accel_input_ = true;
    }

    // Setup turn_rate, pitch_rate, and roll_rate inputs
    turn_rate_idx_ = vars_.declare(VariableIO::Type::turn_rate, VariableIO::Direction::In);
    pitch_rate_idx_ = vars_.declare(VariableIO::Type::pitch_rate, VariableIO::Direction::In);
    roll_rate_idx_ = vars_.declare(VariableIO::Type::roll_rate, VariableIO::Direction::In);

    // Setup model size and quaternions
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

    write_csv_ = sc::get<bool>("write_csv", params, false);
    if (write_csv_) {
        csv_.open_output(parent_->mp()->log_dir() + "/"
                         + std::to_string(parent_->id().id())
                         + "-unicycle-states.csv");

        csv_.set_column_headers(sc::CSV::Headers{"t",
                        "x", "y", "z",
                        "U", "V", "W",
                        "P", "Q", "R",
                        "roll", "pitch", "yaw",
                        "speed",
                        "turn_rate", "pitch_rate", "roll_rate",
                        "Uw", "Vw", "Ww"});
    }
    return true;
}

bool Unicycle3D::step(double t, double dt) {
    // Get inputs and saturate
    turn_rate_ = boost::algorithm::clamp(vars_.input(turn_rate_idx_), turn_rate_min_, turn_rate_max_);
    pitch_rate_ = boost::algorithm::clamp(vars_.input(pitch_rate_idx_), pitch_rate_min_, pitch_rate_max_);
    roll_rate_ = boost::algorithm::clamp(vars_.input(roll_rate_idx_), roll_rate_min_, roll_rate_max_);

    x_[Uw] = state_->vel()(0);
    x_[Vw] = state_->vel()(1);
    x_[Ww] = state_->vel()(2);

    x_[Xw] = state_->pos()(0);
    x_[Yw] = state_->pos()(1);
    x_[Zw] = state_->pos()(2);

    state_->quat().normalize();
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

    if (use_accel_input_) {
        // Enforce acceleration input limits
        acceleration_ = boost::algorithm::clamp(vars_.input(accel_idx_), accel_min_, accel_max_);
        x_[U] += force_body(0) / mass_;
    } else {
        // Enforce velocity input limits
        speed_ = boost::algorithm::clamp(vars_.input(speed_idx_), speed_min_, speed_max_);
        x_[U] = speed_ + force_body(0) / mass_;
    }

    x_[V] = force_body(1) / mass_;
    x_[W] = force_body(2) / mass_;

    x_[P] = ext_moment_body(0) / mass_ + roll_rate_;
    x_[Q] = ext_moment_body(1) / mass_ + pitch_rate_;
    x_[R] = ext_moment_body(2) / mass_ + turn_rate_;

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
                {"turn_rate", turn_rate_},
                {"pitch_rate", pitch_rate_},
                {"roll_rate", roll_rate_},
                {"Uw", state_->vel()(0)},
                {"Vw", state_->vel()(1)},
                {"Ww", state_->vel()(2)}});
    }
    return true;
}

void Unicycle3D::model(const vector_t &x , vector_t &dxdt , double t) {
    dxdt[U] = acceleration_;
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
