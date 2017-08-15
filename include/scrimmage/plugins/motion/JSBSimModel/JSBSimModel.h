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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_MOTION_JSBSIMMODEL_JSBSIMMODEL_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_MOTION_JSBSIMMODEL_JSBSIMMODEL_H_

#include <scrimmage/math/State.h>
#include <scrimmage/motion/MotionModel.h>
#include <scrimmage/motion/Controller.h>
#include <scrimmage/common/PID.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/entity/Entity.h>

#if ENABLE_JSBSIM == 1
#include <FGFDMExec.h>
#include <models/FGAircraft.h>
#include <input_output/FGPropertyManager.h>
#include <initialization/FGInitialCondition.h>

typedef std::shared_ptr<JSBSim::FGFDMExec> FGFDMExecPtr;
#endif

#include <map>
#include <string>
#include <tuple>

class JSBSimModel : public scrimmage::MotionModel{
 public:
    virtual std::tuple<int, int, int> version();

    virtual bool init(std::map<std::string, std::string> &info,
                      std::map<std::string, std::string> &params);
    virtual bool step(double time, double dt);

    virtual void teleport(scrimmage::StatePtr &state);

    class Controller : public scrimmage::Controller {
     public:
        virtual Eigen::Vector3d &u() = 0;
    };

 protected:
#if ENABLE_JSBSIM == 1
    FGFDMExecPtr exec_;

    JSBSim::FGPropertyNode *longitude_node_;
    JSBSim::FGPropertyNode *latitude_node_;
    JSBSim::FGPropertyNode *altitude_node_;

    JSBSim::FGPropertyNode *roll_node_;
    JSBSim::FGPropertyNode *pitch_node_;
    JSBSim::FGPropertyNode *yaw_node_;

    JSBSim::FGPropertyNode *desired_heading_node_;
    JSBSim::FGPropertyNode *desired_altitude_node_;
    JSBSim::FGPropertyNode *desired_velocity_node_;
    JSBSim::FGPropertyNode *bank_setpoint_node_;

    JSBSim::FGPropertyNode *vel_north_node_;
    JSBSim::FGPropertyNode *vel_east_node_;
    JSBSim::FGPropertyNode *vel_down_node_;

    scrimmage::Angles angles_to_jsbsim_;
    scrimmage::Angles angles_from_jsbsim_;

    scrimmage::PID heading_pid_;
    double prev_desired_yaw_;
    bool heading_lag_initialized_;

    double dt_;
#endif
};

#endif // INCLUDE_SCRIMMAGE_PLUGINS_MOTION_JSBSIMMODEL_JSBSIMMODEL_H_
