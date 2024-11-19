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

#include <scrimmage/motion/MotionModel.h>
#include <functional>
#include <boost/numeric/odeint.hpp>

namespace pl = std::placeholders;

namespace scrimmage {

MotionModel::MotionModel() : ext_force_(0, 0, 0), ext_moment_(0, 0, 0),
                             mass_(1.0), g_(9.81) {}

std::string MotionModel::type() { return std::string("MotionModel"); }

bool MotionModel::init(std::map<std::string, std::string> &info, std::map<std::string, std::string> &params)
{ return false; }

bool MotionModel::step(double time, double dt) { return true; }

///////////////////////////////////////////////////////////////////////////////////////
// Natalie's step functions
///////////////////////////////////////////////////////////////////////////////////////

bool MotionModel::offset_step(double time, double dt){ return true; }
bool MotionModel::step(double time, double dt, vector_t odevect) { return true; }
bool MotionModel::step(double time, double dt, int iteration) { return true; } // Can be deleted, not being used

///////////////////////////////////////////////////////////////////////////////////////

std::vector<double> MotionModel::getOdeStepVal() { return odeVal_;}

bool MotionModel::posthumous(double t) { return true; }

StatePtr &MotionModel::state() {return state_;}

void MotionModel::set_state(StatePtr &state) {state_ = state;}

void MotionModel::teleport(StatePtr &state) {state_ = state;}

void MotionModel::ode_step(double dt) {
    auto sys = std::bind(&MotionModel::model, this, pl::_1, pl::_2, pl::_3);
    boost::numeric::odeint::runge_kutta4<std::vector<double>> stepper;
    stepper.do_step(sys, x_, 0, dt);

    // ABM Stepper
    // This did not result in a change of wall time...
    // boost::numeric::odeint::adams_bashforth< 5, std::vector<double>> abm;
    // double t = 0;
    // abm.initialize(sys , x_ , t , dt );
    // abm.do_step( sys , x_ , t , dt );

    //Adjustable stepper
    //boost::numeric::odeint::controlled_runge_kutta


}

void MotionModel::model(const MotionModel::vector_t &x, MotionModel::vector_t &dxdt, double t) {}

void MotionModel::set_external_force(const Eigen::Vector3d &force) {
    ext_force_ = force;
}

void MotionModel::set_external_moment(const Eigen::Vector3d &moment) {
    ext_moment_ = moment;
}

void MotionModel::close(double t) {
    state_ = nullptr;
}
} // namespace scrimmage
