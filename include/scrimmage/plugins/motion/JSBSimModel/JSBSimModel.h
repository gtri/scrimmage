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

#include <scrimmage/motion/MotionModel.h>
#include <scrimmage/common/PID.h>
#include <scrimmage/math/Angles.h>

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
class State;
using StatePtr = std::shared_ptr<State>;
}

namespace scrimmage {
namespace motion {

class JSBSimModel : public MotionModel {
 public:
    virtual std::tuple<int, int, int> version();

    virtual bool init(std::map<std::string, std::string> &info,
                      std::map<std::string, std::string> &params);
    virtual bool step(double time, double dt);

    virtual void teleport(StatePtr &state);

    bool use_pitch() { return use_pitch_; }

 protected:
#if ENABLE_JSBSIM == 1
    FGFDMExecPtr exec_;

    JSBSim::FGPropertyNode *longitude_node_;
    JSBSim::FGPropertyNode *latitude_node_;
    JSBSim::FGPropertyNode *altitude_node_;

    JSBSim::FGPropertyNode *roll_node_;
    JSBSim::FGPropertyNode *pitch_node_;
    JSBSim::FGPropertyNode *yaw_node_;

    JSBSim::FGPropertyNode *fcs_elevator_cmd_node_;

    JSBSim::FGPropertyNode *desired_heading_node_;
    JSBSim::FGPropertyNode *desired_altitude_node_;
    JSBSim::FGPropertyNode *desired_velocity_node_;
    JSBSim::FGPropertyNode *bank_setpoint_node_;

    JSBSim::FGPropertyNode *vel_north_node_;
    JSBSim::FGPropertyNode *vel_east_node_;
    JSBSim::FGPropertyNode *vel_down_node_;

    JSBSim::FGPropertyNode *p_node_ = nullptr;
    JSBSim::FGPropertyNode *q_node_ = nullptr;
    JSBSim::FGPropertyNode *r_node_ = nullptr;

    JSBSim::FGPropertyNode *ax_pilot_node_ = nullptr;
    JSBSim::FGPropertyNode *ay_pilot_node_ = nullptr;
    JSBSim::FGPropertyNode *az_pilot_node_ = nullptr;

    JSBSim::FGOutputType* output_fg_ = 0;

    Angles angles_to_jsbsim_;
    Angles angles_from_jsbsim_;

    PID heading_pid_;
    double prev_desired_yaw_;
    bool heading_lag_initialized_;

    double dt_;
#endif
    bool use_pitch_ = false;

    int speed_idx_ = 0;
    int roll_idx_ = 0;
    int alt_or_pitch_idx_ = 0;

    bool fg_out_enable_ = false;
};
} // namespace motion
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_MOTION_JSBSIMMODEL_JSBSIMMODEL_H_
