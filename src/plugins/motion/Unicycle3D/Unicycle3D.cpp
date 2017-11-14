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

enum ControlParams {
    VELOCITY = 0,
    TURN_RATE,
    PITCH_RATE,
    CONTROL_NUM_ITEMS
};

Unicycle3D::Unicycle3D() : turn_rate_max_(1), pitch_rate_max_(1), vel_max_(1),
                           enable_roll_(false), write_csv_(false) {
}

Unicycle3D::~Unicycle3D() {
}

bool Unicycle3D::init(std::map<std::string, std::string> &info,
                    std::map<std::string, std::string> &params) {

    x_.resize(MODEL_NUM_ITEMS);
    Eigen::Vector3d &pos = state_->pos();
    quat_world_ = state_->quat();

    turn_rate_max_ = std::stod(params.at("turn_rate_max"));
    pitch_rate_max_ = std::stod(params.at("pitch_rate_max"));
    vel_max_ = std::stod(params.at("vel_max"));
    enable_roll_ = sc::get<bool>("enable_roll", params, false);
    write_csv_ = sc::get<bool>("write_csv", params, false);

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
    x_[q0] = 1;
    x_[q1] = 0;
    x_[q2] = 0;
    x_[q3] = 0;

    if (write_csv_) {
        csv_.open_output(parent_->mp()->log_dir() + "/"
                         + std::to_string(parent_->id().id())
                         + "-unicycle-states.csv");

        csv_.set_column_headers(sc::CSV::Headers{"t",
                    "U", "V", "W",
                    "P", "Q", "R",
                    "roll", "pitch", "yaw",
                    "vel", "yaw_rate", "pitch_rate"});
    }

    return true;
}

bool Unicycle3D::step(double t, double dt) {
    ctrl_u_ = std::static_pointer_cast<Controller>(parent_->controllers().back())->u();

    // Saturate input parameters
    double vel = boost::algorithm::clamp(ctrl_u_(0), -vel_max_, vel_max_);
    double yaw_rate = boost::algorithm::clamp(ctrl_u_(1), -turn_rate_max_, turn_rate_max_);
    double pitch_rate = boost::algorithm::clamp(ctrl_u_(2), -pitch_rate_max_, pitch_rate_max_);

    x_[U] = vel;
    x_[Q] = pitch_rate;
    x_[R] = yaw_rate;

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

    // Convert local coordinates to world coordinates
    state_->quat() = quat_world_ * quat_local_;
    state_->pos() << x_[Xw], x_[Yw], x_[Zw];
    state_->vel() << x_[Uw], x_[Vw], x_[Ww];

    if (write_csv_) {
        // Log state to CSV
        csv_.append(sc::CSV::Pairs{
                {"t", t},
                {"U", x_[U]},
                {"V", x_[V]},
                {"W", x_[W]},
                {"P", x_[P]},
                {"Q", x_[Q]},
                {"R", x_[R]},
                {"roll", state_->quat().roll()},
                {"pitch", state_->quat().pitch()},
                {"yaw", state_->quat().yaw()},
                {"vel", vel},
                {"yaw_rate", yaw_rate},
                {"pitch_rate", pitch_rate}});
    }

    return true;
}

void Unicycle3D::model(const vector_t &x , vector_t &dxdt , double t) {
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
    quat.w() = x[q0];
    quat.x() = x[q1];
    quat.y() = x[q2];
    quat.z() = x[q3];

    quat = quat_world_ * quat;
    quat.normalize();

    // Convert local positions and velocities into global coordinates
    Eigen::Matrix3d rot = quat.toRotationMatrix();

    Eigen::Vector3d vel_local(x[U], x[V], x[W]);
    Eigen::Vector3d vel_world = rot * vel_local;
    dxdt[Xw] = vel_world(0);
    dxdt[Yw] = vel_world(1);
    dxdt[Zw] = vel_world(2);

    Eigen::Vector3d acc_local(dxdt[U], dxdt[V], dxdt[W]);
    Eigen::Vector3d acc_world = rot * acc_local;
    dxdt[Uw] = acc_world(0);
    dxdt[Vw] = acc_world(1);
    dxdt[Ww] = acc_world(2);
}
} // namespace motion
} // namespace scrimmage
