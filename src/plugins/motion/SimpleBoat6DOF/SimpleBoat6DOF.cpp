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
    X_dot,
    Y_dot,
    Z_dot,
    THETA,
    THETA_dot,
    Xw,
    Yw,
    ACCEL_CENTRIP,
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

    // initial conditions
    x_[X] = std::stod(info["x"]);
    x_[Y] = std::stod(info["y"]);
    x_[Z] = std::stod(info["z"]);
    x_[Z_dot] = 0;
    x_[X_dot] = 0;
    x_[Y_dot] = 0;
    x_[THETA] = Angles::deg2rad(std::stod(info["heading"]));
    x_[THETA_dot] = 0;

    state_->vel() << 0, 0, 0;
    state_->pos() << x_[Xw], x_[Yw], x_[Z];
    state_->quat().set(0, 0, x_[THETA]);

    // get model params
    max_velocity_ = get<double>("max_velocity", params, 30.0);
    max_acceleration_ = get<double>("max_acceleration", params, 5);
    max_angular_accel_ = get<double>("max_angular_accel", params, .05);
    max_speed_ = get<double>("max_speed", params, 20); // m/s
    max_turn_rate_ = get<double>("max_turn_rate", params, .3);
    max_thrust_deflection_angle_deg_ = get<double>("deflection_angle", params, 20);

    // define drag constants = max accel/max speed^2
    linear_drag_ = max_acceleration_ / pow(max_speed_, 2);
    angular_drag_ = max_angular_accel_ / pow(max_turn_rate_, 2);
    angular_accel_factor_ = max_angular_accel_ / (max_acceleration_ * sin(max_thrust_deflection_angle_deg_*M_PI/180));

    // controller inputs
    input_throttle_idx_ = vars_.declare(VariableIO::Type::speed, VariableIO::Direction::In);
    input_steering_idx_ = vars_.declare(VariableIO::Type::turn_rate, VariableIO::Direction::In);

    // Should we write a CSV file? What values should be written?
    write_csv_ = get<bool>("write_csv", params, false);
    /*if (write_csv_) {
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
    }*/
    return true;
}

bool SimpleBoat6DOF::step(double time, double dt) {

    // get controller inputs
    throttle_in_ = clamp(vars_.input(input_throttle_idx_), 0, max_acceleration_);
    steering_in_ = clamp(vars_.input(input_steering_idx_),
        -1*max_thrust_deflection_angle_deg_*M_PI/180, 
        max_thrust_deflection_angle_deg_*M_PI/180); // max deflection angle on thruster
    /*
    std::cout << "THROTTLE IN: " << throttle_in_ << std::endl;
    std::cout << "STEERING IN: " << steering_in_ << std::endl;
    */
    ode_step(dt);
    /////////////////////
    // Save state
    // Simple velocity

    /*
    std::cout << "X world: " << x_[Xw] << std::endl;
    std::cout << "Y world: " << x_[Yw] << std::endl;
    std::cout << "X body: " << x_[X] << std::endl;
    std::cout << "Y body: " << x_[Y] << std::endl;
    std::cout << "THETA: " << x_[THETA] << std::endl;
    std::cout << "TURN RATE: " << x_[THETA_dot] << std::endl;
    */

    Eigen::Vector3d current_position(x_[Xw], x_[Yw], x_[Z]);
    // inertial frame -- ENU coords
    state_->vel() << (current_position - state_->pos()) / dt;
    state_->pos() << x_[Xw], x_[Yw], x_[Z];
    state_->quat().set(0, 0, x_[THETA]);
    state_->ang_vel() << 0.0, 0.0, x_[THETA_dot];

    // UPDATE THESE FOR 6DOF SENSOR -- body frame, x-axis out nose, z-axis up(I think?)
    Eigen::Vector3d current_body_frame_vel(x_[X_dot], x_[Y_dot], x_[Z_dot]);
    Eigen::Vector3d current_angular_vel(0.0, 0.0, x_[THETA_dot]);

    linear_accel_body_ = (current_body_frame_vel - linear_vel_body_) / dt;
    linear_accel_body_[1] = x_[X_dot] * x_[THETA_dot]; // centripetal accel
    ang_accel_body_ = (current_angular_vel - ang_vel_body_) / dt;
    linear_vel_body_ = current_body_frame_vel;
    ang_vel_body_ = current_angular_vel;
    /*
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
    }*/
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

    dxdt[X] = x[X_dot];
    dxdt[Y] = x[Y_dot];
    dxdt[Z] = x[Z_dot];
    dxdt[THETA] = x[THETA_dot];

    double x_drag, y_drag, theta_drag;

    // drag_force = Cv^2 + constant_term: adding constant_term so velocity can reach 0
    if (abs(x[X_dot]) > 0) {
        x_drag = (abs(x[X_dot])/x[X_dot]) * linear_drag_ *
        (pow(x[X_dot],2) + pow(x[Y_dot], 2)) + .5 * (abs(x[X_dot])/x[X_dot]);
    } else {
        x_drag = 0;
    }

    if (abs(x[Y_dot]) > 0) {
        y_drag = (abs(x[Y_dot])/x[Y_dot]) * linear_drag_ *
        (pow(x[X_dot],2) + pow(x[Y_dot], 2)) + .5 * (abs(x[Y_dot])/x[Y_dot]);
    } else {
        y_drag = 0;
    }

    if (abs(x[THETA_dot]) > 0) {
        theta_drag = (abs(x[THETA_dot])/x[THETA_dot]) * angular_drag_ *
        (pow(x[THETA_dot], 2)) + .05 * (abs(x[THETA_dot])/x[THETA_dot]);
    } else {
        theta_drag = 0;
    }

    // assume body frame y accel is negligable for now
    double y_accel = 0; //-1*(throttle_in_ * sin(steering_in_)) - y_drag;
    double x_accel = (throttle_in_ * cos(steering_in_)) - x_drag;
    double ang_accel = (throttle_in_+max_acceleration_*.2) *
        sin(steering_in_) * angular_accel_factor_ - theta_drag;
    dxdt[X_dot] = x_accel;
    dxdt[Y_dot] = y_accel;
    dxdt[Z_dot] = 0;
    dxdt[THETA_dot] = ang_accel;

    // world frame position -- ENU
    double speed = sqrt(pow(x[X_dot],2) + pow(x[Y_dot],2));
    dxdt[Xw] = speed * cos(x[THETA]);
    dxdt[Yw] = speed * sin(x[THETA]);

    x_[ACCEL_CENTRIP] = throttle_in_ * sin(steering_in_);
}
} // namespace motion
} // namespace scrimmage
