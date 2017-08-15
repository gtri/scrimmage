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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_MOTION_DOUBLEINTEGRATOR_DOUBLEINTEGRATOR_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_MOTION_DOUBLEINTEGRATOR_DOUBLEINTEGRATOR_H_

#include <scrimmage/math/State.h>
#include <scrimmage/motion/MotionModel.h>
#include <scrimmage/motion/Controller.h>
#include <scrimmage/common/PID.h>

#include <map>
#include <string>
#include <limits>

class DoubleIntegrator : public scrimmage::MotionModel {
 public:
    DoubleIntegrator();

    virtual bool init(std::map<std::string, std::string> &info,
                      std::map<std::string, std::string> &params);

    virtual bool step(double t, double dt);

    virtual void model(const vector_t &x , vector_t &dxdt , double t);

    class Controller : public scrimmage::Controller {
     public:
        virtual Eigen::Vector3d &u() = 0;
    };

 protected:
    double update_dvdt(double vel, double acc);
    Eigen::Vector3d ctrl_u_;
    double max_vel_ = std::numeric_limits<double>::infinity();
    double max_acc_ = std::numeric_limits<double>::infinity();
};

#endif // INCLUDE_SCRIMMAGE_PLUGINS_MOTION_DOUBLEINTEGRATOR_DOUBLEINTEGRATOR_H_
