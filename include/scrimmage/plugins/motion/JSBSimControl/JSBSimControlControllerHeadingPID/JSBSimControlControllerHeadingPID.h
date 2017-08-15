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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_MOTION_JSBSIMCONTROL_JSBSIMCONTROLCONTROLLERHEADINGPID_JSBSIMCONTROLCONTROLLERHEADINGPID_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_MOTION_JSBSIMCONTROL_JSBSIMCONTROLCONTROLLERHEADINGPID_JSBSIMCONTROLCONTROLLERHEADINGPID_H_

#include <map>
#include <string>

#include "../JSBSimControl.h"

class JSBSimControlControllerHeadingPID : public JSBSimControl::Controller {
 public:
    virtual void init(std::map<std::string, std::string> &params);
    virtual bool step(double t, double dt);
    virtual Eigen::Vector3d &u() {return u_;}

 protected:
    Eigen::Vector3d u_;
    scrimmage::Angles angles_to_jsbsim_;
    scrimmage::Angles angles_from_jsbsim_;

    scrimmage::PID heading_pid_;
    double prev_desired_yaw_;
    bool heading_lag_initialized_;

    scrimmage::PID roll_pid_;
    scrimmage::PID pitch_pid_;
    scrimmage::PID yaw_pid_;
};

#endif // INCLUDE_SCRIMMAGE_PLUGINS_MOTION_JSBSIMCONTROL_JSBSIMCONTROLCONTROLLERHEADINGPID_JSBSIMCONTROLCONTROLLERHEADINGPID_H_
