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

#include <scrimmage/plugins/motion/SimpleBoat6DOF/SimpleBoat6DOF.h>
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

REGISTER_PLUGIN(scrimmage::MotionModel, scrimmage::motion::SimpleBoat6DOF, SimpleBoat6DOF_plugin)

namespace scrimmage {
namespace motion {

enum ModelParams {
    X = 0,
    Y,
    Z,
    Z_dot,
    THETA,
    angular_speed,
    speed,
    MODEL_NUM_ITEMS
};

enum ControlParams {
    FORWARD_VELOCITY = 0,
    TURN_RATE,
    CONTROL_NUM_ITEMS
};

bool SimpleBoat6DOF::init(std::map<std::string, std::string> &info,
                     std::map<std::string, std::string> &params) {
    x_.resize(MODEL_NUM_ITEMS);
    x_[X] = std::stod(info["x"]);
    x_[Y] = std::stod(info["y"]);
    x_[Z] = std::stod(info["z"]);
    x_[Z_dot] = 0;
    x_[THETA] = Angles::deg2rad(std::stod(info["heading"]));

    length_ = get<double>("length", params, 100.0);
    mass_ = get<double>("mass", params, 1.0);
    enable_gravity_ = get<bool>("enable_gravity", params, false);
    max_velocity_ = get<double>("max_velocity", params, 30.0);
    max_acceleration_ = get<double>("max_acceleration", params, 5);
    max_angular_accel_ = get<double>("max_angular_accel", params, .1);
    max_turn_rate_ = get<double>("max_turn_rate", params, .3);
    alpha_ = get<double>("damping", params, .1);

    /////////
    state_->vel() << 0, 0, 0;
    state_->pos() << x_[X], x_[Y], x_[Z];
    state_->quat().set(0, 0, x_[THETA]);

    acceleration_ = 0;
    angular_acceleration_ = 0;

    input_speed_idx_ = vars_.declare(VariableIO::Type::speed, VariableIO::Direction::In);
    input_turn_rate_idx_ = vars_.declare(VariableIO::Type::turn_rate, VariableIO::Direction::In);

    // Should we write a CSV file? What values should be written?
    write_csv_ = get<bool>("write_csv", params, false);
    if (write_csv_) {
        csv_.open_output(parent_->mp()->root_log_dir() + "/"
                         + std::to_string(parent_->id().id())
                         + "-states.csv");
        cout << "Writing log to " + parent_->mp()->root_log_dir() + "/"
                         + std::to_string(parent_->id().id())
                         + "-states.csv" << endl;

        csv_.set_column_headers(CSV::Headers{"t",
                    "x", "y", "z",
                    "Ax_b", "Ay_b", "Az_b",
                    "AngAccelx_b", "AngAccely_b", "AngAccelz_b",
                    "roll", "pitch", "yaw", "cmd_accel", "cmd_ang_accel",
                    "throttle", "steering", "theta_dot", "x_bdot", "y_bdot"});
    }

    return true;
}

bool SimpleBoat6DOF::step(double time, double dt) {
    double prev_x = x_[X];
    double prev_y = x_[Y];
    double prev_z = x_[Z];
    double prev_theta = x_[THETA];

    const double u_vel = clamp(vars_.input(input_speed_idx_), 0, max_velocity_);
    const double accel_cmd = clamp(((u_vel - x_[speed]) / dt), -1*max_acceleration_, max_acceleration_); // set fixed linear accel for time step
    acceleration_ = (1 - alpha_) * acceleration_ + alpha_ * accel_cmd;

    const double u_theta = clamp(vars_.input(input_turn_rate_idx_), -1*max_turn_rate_, max_turn_rate_);
    const double angular_accel_cmd = clamp(((u_theta - x_[angular_speed]) / dt), -1*max_angular_accel_, max_angular_accel_); // set fixed angular accel for time step
    angular_acceleration_ = (1 - alpha_) * angular_acceleration_ + alpha_ * angular_accel_cmd;


    ode_step(dt);

    ext_force_ = Eigen::Vector3d::Zero();

    /////////////////////
    // Save state
    // Simple velocity
    state_->vel() << (x_[X] - prev_x) / dt, (x_[Y] - prev_y) / dt,
        (x_[Z] - prev_z) / dt;

    state_->pos() << x_[X], x_[Y], x_[Z];
    state_->quat().set(0, 0, x_[THETA]);
    state_->ang_vel() << 0.0, 0.0, (x_[THETA] - prev_theta) / dt;

    // UPDATE THESE FOR 6DOF SENSOR
    linear_accel_body_ = (state_->quat().rotate_reverse(state_->vel()) - linear_vel_body_) / dt;
    ang_accel_body_ = (state_->ang_vel() - ang_vel_body_) / dt;
    linear_vel_body_ = state_->quat().rotate_reverse(state_->vel()); // converting to body frame
    ang_vel_body_ = state_->ang_vel();

    if (write_csv_) {
        // Log state to CSV
        csv_.append(CSV::Pairs{
                {"t", time},
                {"x", x_[X]},
                {"y", x_[Y]},
                {"z", x_[Z]},
                {"Ax_b", linear_accel_body_(0)},
                {"Ay_b", linear_accel_body_(1)},
                {"Az_b", linear_accel_body_(2)},
                {"AngAccelx_b", ang_accel_body_(0)},
                {"AngAccely_b", ang_accel_body_(1)},
                {"AngAccelz_b", ang_accel_body_(2)},
                {"roll", x_[THETA]},
                {"pitch", 0.0},
                {"yaw", 0.0},
                {"cmd_accel", acceleration_},
                {"cmd_ang_accel", angular_acceleration_},
                {"throttle", u_vel},
                {"steering", u_theta},
                {"theta_dot", (x_[THETA] - prev_theta) / dt},
                {"x_bdot", linear_vel_body_(0)},
                {"y_bdot", linear_vel_body_(1)}});
    }

    return true;
}

void SimpleBoat6DOF::model(const vector_t &x , vector_t &dxdt , double t) {
    /// 0 : x-position
    /// 1 : y-position
    /// 2 : z-position
    /// 3 : z dot
    /// 4 : theta
    /// 5 : angular speed
    /// 6 : linear speed

    dxdt[X] = x[speed]*cos(x[THETA]);
    dxdt[Y] = x[speed]*sin(x[THETA]);
    dxdt[THETA] = x[angular_speed];

    if (enable_gravity_) {
        dxdt[Z] = x[Z_dot];
        dxdt[Z_dot] = mass_ * -9.8;
    } else {
        dxdt[Z] = 0;
        dxdt[Z_dot] = 0;
    }

    dxdt[speed] = acceleration_;
    dxdt[angular_speed] = angular_acceleration_;

    // Saturate based on external force:
    if (std::abs(ext_force_(0)) > 0.1) {
        dxdt[X] = 0;
    }

    if (std::abs(ext_force_(1)) > 0.1) {
        dxdt[Y] = 0;
    }

    if (std::abs(ext_force_(2)) > 0.1) {
        dxdt[Z] = 0;
    }
}
} // namespace motion
} // namespace scrimmage
