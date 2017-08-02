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

#include <iostream>

#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>

#include <(>>>PROJECT_NAME<<<)/plugins/motion/(>>>PLUGIN_NAME<<<)/(>>>PLUGIN_NAME<<<).h>

namespace sc = scrimmage;

using std::cout;
using std::endl;

REGISTER_PLUGIN(scrimmage::MotionModel, (>>>PLUGIN_NAME<<<), (>>>PLUGIN_NAME<<<)_plugin)

enum ModelParams
{
    X = 0,
    Y,
    Z,
    THETA,    
    MODEL_NUM_ITEMS
};

enum ControlParams
{
    FORWARD_VELOCITY = 0,
    TURN_RATE,
    CONTROL_NUM_ITEMS
};

(>>>PLUGIN_NAME<<<)::(>>>PLUGIN_NAME<<<)()
{   
    x_.resize(MODEL_NUM_ITEMS);
}

bool (>>>PLUGIN_NAME<<<)::init(std::map<std::string, std::string> &info,
                     std::map<std::string, std::string> &params)
{
    // Get the initial internal state values
    x_[X] = std::stod(info["x"]); //x
    x_[Y] = std::stod(info["y"]); //y
    x_[Z] = std::stod(info["z"]); //y
    x_[THETA] = sc::Angles::deg2rad(std::stod(info["heading"]));    
        
    length_ = sc::get<double>("length", params, 100.0);

    // Set the state_ variable that is used by the entity and the main
    // simulation controller
    state_->vel() << 0, 0, 0;
    state_->pos() << x_[X], x_[Y], x_[Z];
    state_->quat().set(0, 0, x_[THETA]);
    
    return true;
}

bool (>>>PLUGIN_NAME<<<)::step(double time, double dt)
{
    // Save the previous x, y, z to calculate velocities
    double prev_x = x_[X];
    double prev_y = x_[Y];
    double prev_z = x_[Z];
    
    ode_step(dt); // step the motion model ODE solver

    // Save state (position, velocity, orientation) used by simulation
    // controller
    state_->vel() << (x_[X] - prev_x) / dt,
        (x_[Y] - prev_y) / dt,
        (x_[Z] - prev_z) / dt;
    
    state_->pos() << x_[X], x_[Y], x_[Z];
    state_->quat().set(0, 0, x_[THETA]);
    return true;
}

void (>>>PLUGIN_NAME<<<)::model(const vector_t &x , vector_t &dxdt , double t)
{
    /// 0 : x-position
    /// 1 : y-position
    /// 2 : theta

    // Get the controller's actuator inputs
    Eigen::Vector2d &u = std::static_pointer_cast<Controller>(parent_->controllers().back())->u();
    double u_vel = u(FORWARD_VELOCITY);
    double u_theta = u(TURN_RATE);
    
    // Saturate wheel angle:
    if (u_theta >= M_PI/4) {
        u_theta = M_PI/4 - 0.0001;
    } else if (u_theta <= -M_PI/4) {
        u_theta = -M_PI/4 + 0.0001;
    }

    // Define the state differential
    dxdt[X] = u_vel*cos(x[THETA]);
    dxdt[Y] = u_vel*sin(x[THETA]);
    dxdt[THETA] = u_vel/length_*tan(u_theta);

    dxdt[Z] = 0; // Remain at initial z-position    
}
