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

#include <scrimmage/plugins/motion/JSBSimModel/JSBSimModel.h>
#include <scrimmage/plugins/motion/JSBSimModel/FGOutputFGMod.h>

#include <scrimmage/common/VariableIO.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/math/State.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/math/Angles.h>

// c system
#include <JSBSim/initialization/FGTrim.h>

// c++ system
// <none>

// other
#include <JSBSim/simgear/misc/sg_path.hxx>
#include <GeographicLib/LocalCartesian.hpp>

#define meters2feet 3.28084
#define feet2meters (1.0 / meters2feet)

#define knts2mps 0.5144444
#define mps2knts (1.0 / knts2mps)

REGISTER_PLUGIN(scrimmage::MotionModel, scrimmage::motion::JSBSimModel, JSBSimModel_plugin)

namespace scrimmage {
namespace motion {

using ang = scrimmage::Angles;

std::tuple<int, int, int> JSBSimModel::version() {
    return std::tuple<int, int, int>(0, 0, 1);
}

bool JSBSimModel::init(std::map<std::string, std::string> &info,
                       std::map<std::string, std::string> &params) {
    angles_from_jsbsim_ = Angles(0, Angles::Type::GPS, Angles::Type::EUCLIDEAN);
    angles_to_jsbsim_ = Angles(0, Angles::Type::EUCLIDEAN, Angles::Type::GPS);

    use_pitch_ = str2bool(params.at("use_pitch"));
    std::string z_name =  use_pitch_ ?
        vars_.type_map().at(VariableIO::Type::desired_pitch) :
        vars_.type_map().at(VariableIO::Type::desired_altitude);

    speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::In);
    roll_idx_ = vars_.declare(VariableIO::Type::desired_roll, VariableIO::Direction::In);
    alt_or_pitch_idx_ = vars_.declare(z_name, VariableIO::Direction::In);

    JSBSim::FGJSBBase base;
    base.debug_lvl = 0;
    exec_ = std::make_shared<JSBSim::FGFDMExec>();

    fg_out_enable_ = get<bool>("flightgear_output_enable", params, false);
    if (fg_out_enable_) {
        output_fg_ = new JSBSim::FGOutputFGMod(&(*exec_));
        std::string ip = get<std::string>("flightgear_ip", params, "localhost");
        std::string port = get<std::string>("flightgear_port", params, "5600");
        std::string protocol = get<std::string>("flightgear_protocol", params, "UDP");
        std::string name = ip + ":" + protocol + "/" + port; // localhost:UDP/5600

        output_fg_->SetIdx(0);
        output_fg_->SetOutputName(name);
        output_fg_->SetRateHz(60);
        output_fg_->InitModel();
    }

    exec_->SetDebugLevel(0);

    std::string jsbsim_root_dir = get<std::string>("jsbsim_root", params, info["JSBSIM_ROOT"]);
    if (jsbsim_root_dir == "") {
        exec_->SetRootDir(SGPath(info["JSBSIM_ROOT"]));
    } else {
        exec_->SetRootDir(SGPath(jsbsim_root_dir));
    }

    exec_->SetAircraftPath(SGPath("aircraft"));
    exec_->SetEnginePath(SGPath("engine"));
    exec_->SetSystemsPath(SGPath("systems"));

    exec_->LoadScript(SGPath("scripts/"+info["script_name"]));

    JSBSim::FGInitialCondition *ic = exec_->GetIC();
    if (info.count("latitude") > 0) {
        ic->SetLatitudeDegIC(std::stod(info["latitude"]));
    }
    if (info.count("longitude") > 0) {
        ic->SetLongitudeDegIC(std::stod(info["longitude"]));
    }
    if (info.count("heading") > 0) {
        angles_to_jsbsim_.set_angle(std::stod(info["heading"]));
        ic->SetPsiDegIC(angles_to_jsbsim_.angle());
    }
    if (info.count("altitude") > 0) {
        double alt_asl_meters = std::stod(info["altitude"]);
        ic->SetAltitudeASLFtIC(alt_asl_meters * meters2feet);
    }

    if (info.count("dt")) {
        dt_ = std::stod(info["dt"]);
        if (info.count("motion_multiplier")) {
            double motion_multiplier = std::stod(info["motion_multiplier"]);
            dt_ /= motion_multiplier;
        }
    } else {
        dt_ = 0.0083333;
    }

    exec_->RunIC();
    exec_->Setdt(dt_);
    exec_->Run();

    // If we need to trim programmatically later
    // if (exec_->GetIC()->NeedTrim()) {
    //     cout << "Trimming..." << endl;
    //     JSBSim::FGTrim* trimmer = new JSBSim::FGTrim( &*exec_ );
    //     try {
    //          trimmer->DoTrim();
    //          delete trimmer;
    //     } catch (string& msg) {
    //          std::cerr << endl << msg << endl << endl;
    //          exit(1);
    //     }
    //}

    // Get references to each of the nodes that hold properties that we
    // care about
    JSBSim::FGPropertyManager* mgr = exec_->GetPropertyManager();
    longitude_node_ = mgr->GetNode("position/long-gc-deg");
    latitude_node_ = mgr->GetNode("position/lat-gc-deg");
    altitude_node_ = mgr->GetNode("position/h-sl-ft");

    roll_node_ = mgr->GetNode("attitude/roll-rad");
    pitch_node_ = mgr->GetNode("attitude/pitch-rad");
    yaw_node_ = mgr->GetNode("attitude/heading-true-rad");

    // desired_heading_node_ = mgr->GetNode("guidance/specified-heading-rad");
    desired_altitude_node_ = mgr->GetNode("ap/altitude_setpoint");
    desired_velocity_node_ = mgr->GetNode("ap/airspeed_setpoint");
    bank_setpoint_node_ = mgr->GetNode("ap/bank_setpoint");
    fcs_elevator_cmd_node_ = mgr->GetNode("fcs/elevator-cmd-norm");

    vel_north_node_ = mgr->GetNode("velocities/v-north-fps");
    vel_east_node_ = mgr->GetNode("velocities/v-east-fps");
    vel_down_node_ = mgr->GetNode("velocities/v-down-fps");

    // angular velocity in ECEF frame
    p_node_ = mgr->GetNode("velocities/p-rad_sec");
    q_node_ = mgr->GetNode("velocities/q-rad_sec");
    r_node_ = mgr->GetNode("velocities/r-rad_sec");

    // acceleration at pilot location in body frame
    ax_pilot_node_ = mgr->GetNode("accelerations/a-pilot-x-ft_sec2");
    ax_pilot_node_ = mgr->GetNode("accelerations/a-pilot-y-ft_sec2");
    ax_pilot_node_ = mgr->GetNode("accelerations/a-pilot-z-ft_sec2");

    // Save state
    parent_->projection()->Forward(latitude_node_->getDoubleValue(),
                                  longitude_node_->getDoubleValue(),
                                  altitude_node_->getDoubleValue() * feet2meters,
                                  state_->pos()(0), state_->pos()(1), state_->pos()(2));

    angles_from_jsbsim_.set_angle(ang::rad2deg(yaw_node_->getDoubleValue()));

    state_->quat().set(roll_node_->getDoubleValue(),
                      -pitch_node_->getDoubleValue(),
                      ang::deg2rad(angles_from_jsbsim_.angle()));

    state_->vel() << vel_east_node_->getDoubleValue() * feet2meters,
                    vel_north_node_->getDoubleValue() * feet2meters,
                    -vel_down_node_->getDoubleValue() * feet2meters;

    state_->ang_vel() << p_node_->getDoubleValue(),
                         q_node_->getDoubleValue(),
                         p_node_->getDoubleValue();

    return true;
}

bool JSBSimModel::step(double time, double dt) {
    double desired_velocity = vars_.input(speed_idx_);
    double roll_cmd = vars_.input(roll_idx_);

    // + : roll right, - : roll left
    bank_setpoint_node_->setDoubleValue(roll_cmd);
    if (use_pitch_) {
        double elevator_cmd = vars_.input(alt_or_pitch_idx_);

        // Negate altitude PID from the elevator control
        elevator_cmd -= exec_->GetPropertyValue("ap/elevator_cmd");
        fcs_elevator_cmd_node_->setDoubleValue(elevator_cmd);

        // Try to remove altitude control by putting setpoint to current altitude
        desired_altitude_node_->setDoubleValue(state_->pos()(2) * meters2feet);
    } else {
        double desired_alt = vars_.input(alt_or_pitch_idx_);
        // Set desired altitude (we just need the desired altitude, use the current
        // x,y as placeholders).
        double lat_curr, lon_curr, alt_result;
        parent_->projection()->Reverse(state_->pos()(0), state_->pos()(1), desired_alt,
                                       lat_curr, lon_curr, alt_result);

        desired_altitude_node_->setDoubleValue(alt_result * meters2feet);
    }
    // set desired velocity
    desired_velocity_node_->setDoubleValue(desired_velocity * mps2knts);

    /////////////////////
    exec_->Setdt(dt);
    exec_->Run();
    // Save state

    if (fg_out_enable_) {
        output_fg_->Print();
    }

    parent_->projection()->Forward(latitude_node_->getDoubleValue(),
                                   longitude_node_->getDoubleValue(),
                                   altitude_node_->getDoubleValue() * feet2meters,
                                   state_->pos()(0), state_->pos()(1), state_->pos()(2));

    angles_from_jsbsim_.set_angle(ang::rad2deg(yaw_node_->getDoubleValue()));

    state_->quat().set(roll_node_->getDoubleValue(),
                      -pitch_node_->getDoubleValue(),
                      ang::deg2rad(angles_from_jsbsim_.angle()));

    state_->vel() << vel_east_node_->getDoubleValue() * feet2meters,
                    vel_north_node_->getDoubleValue() * feet2meters,
                    -vel_down_node_->getDoubleValue() * feet2meters;

    // save what was used as the input
    return true;
}

void JSBSimModel::teleport(StatePtr &state) {
    double lat, lon, alt;
    parent_->projection()->Reverse(state->pos()(0),
                                   state->pos()(1),
                                   state->pos()(2),
                                   lat, lon, alt);

    JSBSim::FGInitialCondition *ic = exec_->GetIC();
    ic->SetLatitudeDegIC(lat);
    ic->SetLongitudeDegIC(lon);
    ic->SetAltitudeASLFtIC(alt * meters2feet);

    angles_to_jsbsim_.set_angle(ang::rad2deg(state->quat().yaw()));
    ic->SetPsiDegIC(angles_to_jsbsim_.angle());

    exec_->RunIC();
    exec_->Setdt(dt_);
    exec_->Run();
}
} // namespace motion
} // namespace scrimmage
