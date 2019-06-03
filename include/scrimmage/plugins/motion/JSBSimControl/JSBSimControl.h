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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_MOTION_JSBSIMCONTROL_JSBSIMCONTROL_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_MOTION_JSBSIMCONTROL_JSBSIMCONTROL_H_

#include <scrimmage/plugins/motion/RigidBody6DOF/RigidBody6DOFBase.h>

#include <scrimmage/math/Angles.h>
#include <scrimmage/motion/MotionModel.h>
#include <scrimmage/motion/Controller.h>
#include <scrimmage/entity/Entity.h>
#include <Eigen/Dense>

#if ENABLE_JSBSIM == 1
#include <FGFDMExec.h>
#include <models/FGAircraft.h>
#include <input_output/FGPropertyManager.h>
#include <initialization/FGInitialCondition.h>
#include <models/FGOutput.h>

typedef std::shared_ptr<JSBSim::FGFDMExec> FGFDMExecPtr;
#endif

#include <map>
#include <string>
#include <tuple>
#include <memory>

namespace scrimmage {
namespace motion {
class JSBSimControl : public scrimmage::motion::RigidBody6DOFBase {
 public:
     JSBSimControl();

     virtual std::tuple<int, int, int> version();

     bool init(std::map<std::string, std::string> &info,
                       std::map<std::string, std::string> &params) override;
     bool step(double time, double dt) override;

 protected:
#if ENABLE_JSBSIM == 1
     FGFDMExecPtr exec_;

     JSBSim::FGPropertyNode *longitude_node_ = nullptr;
     JSBSim::FGPropertyNode *latitude_node_ = nullptr;
     JSBSim::FGPropertyNode *altitude_node_ = nullptr;
     JSBSim::FGPropertyNode *altitudeAGL_node_ = nullptr;

     JSBSim::FGPropertyNode *roll_node_ = nullptr;
     JSBSim::FGPropertyNode *pitch_node_ = nullptr;
     JSBSim::FGPropertyNode *yaw_node_ = nullptr;

     JSBSim::FGPropertyNode *ap_aileron_cmd_node_ = nullptr;
     JSBSim::FGPropertyNode *ap_elevator_cmd_node_ = nullptr;
     JSBSim::FGPropertyNode *ap_rudder_cmd_node_ = nullptr;
     JSBSim::FGPropertyNode *ap_throttle_cmd_node_ = nullptr;

     JSBSim::FGPropertyNode *vel_north_node_ = nullptr;
     JSBSim::FGPropertyNode *vel_east_node_ = nullptr;
     JSBSim::FGPropertyNode *vel_down_node_ = nullptr;
     JSBSim::FGPropertyNode *u_vel_node_ = nullptr;

     JSBSim::FGPropertyNode *p_node_ = nullptr;
     JSBSim::FGPropertyNode *q_node_ = nullptr;
     JSBSim::FGPropertyNode *r_node_ = nullptr;

     JSBSim::FGPropertyNode *ax_pilot_node_ = nullptr;
     JSBSim::FGPropertyNode *ay_pilot_node_ = nullptr;
     JSBSim::FGPropertyNode *az_pilot_node_ = nullptr;

     scrimmage::Angles angles_to_jsbsim_;
     scrimmage::Angles angles_from_jsbsim_;

     JSBSim::FGOutputType* output_fg_ = 0;
     bool fg_out_enable_ = false;

     int throttle_idx_ = 0;
     int elevator_idx_ = 0;
     int aileron_idx_ = 0;
     int rudder_idx_ = 0;

     double throttle_ = 0;
     double delta_elevator_ = 0;
     double delta_aileron_ = 0;
     double delta_rudder_ = 0;

     double draw_vel_ = 0;
     double draw_ang_vel_ = 0;
     double draw_acc_ = 0;
#endif
};
} // namespace motion
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_MOTION_JSBSIMCONTROL_JSBSIMCONTROL_H_
