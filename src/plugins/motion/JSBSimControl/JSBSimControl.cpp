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

#include <scrimmage/plugins/motion/JSBSimControl/JSBSimControl.h>
#include <scrimmage/plugins/motion/JSBSimModel/FGOutputFGMod.h>

#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/math/State.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>

// c system
#include <JSBSim/initialization/FGTrim.h>

// c++ system
#include <iomanip>
#include <iostream>

// other
#include <JSBSim/simgear/misc/sg_path.hxx>
#include <GeographicLib/LocalCartesian.hpp>
#include <boost/algorithm/clamp.hpp>

using std::cout;
using std::cerr;
using std::endl;

#define meters2feet 3.28084
#define feet2meters (1.0 / meters2feet)

REGISTER_PLUGIN(scrimmage::MotionModel, scrimmage::motion::JSBSimControl, JSBSimControl_plugin)

namespace scrimmage {
namespace motion {

namespace sc = scrimmage;
using ang = scrimmage::Angles;

namespace ba = boost::algorithm;

JSBSimControl::JSBSimControl() {
    angles_from_jsbsim_.set_input_clock_direction(ang::Rotate::CW);
    angles_from_jsbsim_.set_input_zero_axis(ang::HeadingZero::Pos_Y);
    angles_from_jsbsim_.set_output_clock_direction(ang::Rotate::CCW);
    angles_from_jsbsim_.set_output_zero_axis(ang::HeadingZero::Pos_X);

    angles_to_jsbsim_.set_input_clock_direction(ang::Rotate::CCW);
    angles_to_jsbsim_.set_input_zero_axis(ang::HeadingZero::Pos_X);
    angles_to_jsbsim_.set_output_clock_direction(ang::Rotate::CW);
    angles_to_jsbsim_.set_output_zero_axis(ang::HeadingZero::Pos_Y);
}

std::tuple<int, int, int> JSBSimControl::version() {
    return std::tuple<int, int, int>(0, 0, 1);
}

bool JSBSimControl::init(std::map<std::string, std::string> &info,
                         std::map<std::string, std::string> &params) {
    draw_vel_ = sc::get<bool>("draw_vel", params, draw_vel_);
    draw_ang_vel_ = sc::get<bool>("draw_ang_vel", params, draw_ang_vel_);
    draw_acc_ = sc::get<bool>("draw_acc", params, draw_acc_);

    jsbsim_dt_ = std::stod(info["dt"])/std::stod(info["motion_multiplier"]);
    jsbsim_script_path_ = "scripts/" + info["script_name"];

    // Setup variable index for controllers
    throttle_idx_ = vars_.declare(VariableIO::Type::throttle, VariableIO::Direction::In);
    elevator_idx_ = vars_.declare(VariableIO::Type::elevator, VariableIO::Direction::In);
    aileron_idx_ = vars_.declare(VariableIO::Type::aileron, VariableIO::Direction::In);
    rudder_idx_ = vars_.declare(VariableIO::Type::rudder, VariableIO::Direction::In);

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

    exec_->LoadScript(SGPath(jsbsim_script_path_));

    // Get references to each of the nodes that hold properties that we
    // care about
    JSBSim::FGPropertyManager* mgr = exec_->GetPropertyManager();
    longitude_node_ = mgr->GetNode("position/long-gc-deg");
    latitude_node_ = mgr->GetNode("position/lat-gc-deg");
    altitude_node_ = mgr->GetNode("position/h-sl-ft");
    altitudeAGL_node_ = mgr->GetNode("position/h-agl-ft");

    roll_node_ = mgr->GetNode("attitude/roll-rad");
    pitch_node_ = mgr->GetNode("attitude/pitch-rad");
    yaw_node_ = mgr->GetNode("attitude/heading-true-rad");

    ap_aileron_cmd_node_ = mgr->GetNode("fcs/aileron-cmd-norm");
    ap_elevator_cmd_node_ = mgr->GetNode("fcs/elevator-cmd-norm");
    ap_rudder_cmd_node_ = mgr->GetNode("fcs/rudder-cmd-norm");
    ap_throttle_cmd_node_ = mgr->GetNode("fcs/throttle-cmd-norm");
    // ap_throttle_1_cmd_node_ = mgr->GetNode("fcs/throttle-cmd-norm[1]");

    vel_north_node_ = mgr->GetNode("velocities/v-north-fps");
    vel_east_node_ = mgr->GetNode("velocities/v-east-fps");
    vel_down_node_ = mgr->GetNode("velocities/v-down-fps");

    u_vel_node_ = mgr->GetNode("velocities/u-fps");

    // angular velocity in ECEF frame
    p_node_ = mgr->GetNode("velocities/p-rad_sec");
    q_node_ = mgr->GetNode("velocities/q-rad_sec");
    r_node_ = mgr->GetNode("velocities/r-rad_sec");

    // acceleration at pilot location in body frame
    ax_pilot_node_ = mgr->GetNode("accelerations/a-pilot-x-ft_sec2");
    ay_pilot_node_ = mgr->GetNode("accelerations/a-pilot-y-ft_sec2");
    az_pilot_node_ = mgr->GetNode("accelerations/a-pilot-z-ft_sec2");

    set_jsbsim_initial_state(*state_);

    // if (info.count("latitude") > 0) {
    //     ic->SetLatitudeDegIC(std::stod(info["latitude"]));
    // }
    // if (info.count("longitude") > 0) {
    //     ic->SetLongitudeDegIC(std::stod(info["longitude"]));
    // }
    // if (info.count("heading") > 0) {
    //     angles_to_jsbsim_.set_angle(std::stod(info["heading"]));
    //     ic->SetPsiDegIC(angles_to_jsbsim_.angle());
    // }
    // if (info.count("altitude") > 0) {
    //     double alt_asl_meters = std::stod(info["altitude"]);
    //     ic->SetAltitudeASLFtIC(alt_asl_meters * meters2feet);
    // }

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

    Eigen::Vector3d ang_vel_FLU(p_node_->getDoubleValue(),
                               -q_node_->getDoubleValue(),
                               -r_node_->getDoubleValue());
    state_->ang_vel() = state_->quat().rotate(ang_vel_FLU);

    Eigen::Vector3d a_FLU(ax_pilot_node_->getDoubleValue(),
                         -ay_pilot_node_->getDoubleValue(),
                         -az_pilot_node_->getDoubleValue());
    linear_accel_body_ = state_->quat().rotate(a_FLU);

    // exec_->PrintPropertyCatalog();
    // exec_->PrintSimulationConfiguration();

    return true;
}

bool JSBSimControl::step(double time, double dt) {
    throttle_       = vars_.input(throttle_idx_);
    delta_elevator_ = vars_.input(elevator_idx_);
    delta_aileron_  = vars_.input(aileron_idx_);
    delta_rudder_   = vars_.input(rudder_idx_);

    // TODO: for some reason, jsb sim does not like it when there is an immediate thottle input
    if (time < .05)
        throttle_ = 0;

    ap_aileron_cmd_node_->setDoubleValue(delta_aileron_);
    ap_elevator_cmd_node_->setDoubleValue(delta_elevator_);
    ap_rudder_cmd_node_->setDoubleValue(delta_rudder_);
    ap_throttle_cmd_node_->setDoubleValue(throttle_);
    // if (ap_throttle_1_cmd_node_ != nullptr) {
    //     ap_throttle_1_cmd_node_->setDoubleValue(throttle_);
    // }

    exec_->Setdt(dt);
    exec_->Run();

    if (fg_out_enable_) {
        output_fg_->Print();
    }

    ///////////////////////////////////////////////////////////////////////////
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


    Eigen::Vector3d ang_vel_FLU(p_node_->getDoubleValue(),
                               -q_node_->getDoubleValue(),
                               -r_node_->getDoubleValue());
    state_->ang_vel() = state_->quat().rotate(ang_vel_FLU);

    Eigen::Vector3d a_FLU(ax_pilot_node_->getDoubleValue() * feet2meters,
                         -ay_pilot_node_->getDoubleValue() * feet2meters,
                         -az_pilot_node_->getDoubleValue() * feet2meters);
    // TODO: jsbsim returns specific force, but need to populate this value with
    // acceleration. Need to make gravity not a hard-coded value or find a better
    // way to handle this.
    a_FLU = a_FLU + state_->quat().rotate_reverse(Eigen::Vector3d(0, 0, -9.81));
    linear_accel_body_ = a_FLU;

    Eigen::Vector3d a_ENU = state_->quat().rotate(a_FLU);

    // draw velocity
    if (draw_vel_) {
        sc::ShapePtr line(new sp::Shape());
        line->set_opacity(1.0);
        sc::set(line->mutable_color(), 255, 0, 0);
        sc::set(line->mutable_line()->mutable_start(), state_->pos());
        sc::set(line->mutable_line()->mutable_end(), state_->pos() + state_->vel());
        draw_shape(line);
    }

    // draw angular velocity
    if (draw_ang_vel_) {
        sc::ShapePtr line(new sp::Shape());
        line->set_opacity(1.0);
        sc::set(line->mutable_color(), 0, 255, 0);
        sc::set(line->mutable_line()->mutable_start(), state_->pos());
        sc::set(line->mutable_line()->mutable_end(), state_->pos() + state_->ang_vel());
        draw_shape(line);
    }

    // draw acceleration
    if (draw_acc_) {
        sc::ShapePtr line(new sp::Shape());
        line->set_opacity(1.0);
        sc::set(line->mutable_color(), 0, 0, 255);
        sc::set(line->mutable_line()->mutable_start(), state_->pos());
        sc::set(line->mutable_line()->mutable_end(), state_->pos() + a_ENU);
        draw_shape(line);
    }

#if 0
    JSBSim::FGPropertyManager* mgr = exec_->GetPropertyManager();
    cout << "--------------------------------------------------------" << endl;
    cout << "  State information in JSBSImControl" << endl;
    cout << "--------------------------------------------------------" << endl;
    int prec = 5;
    // std::cout << "processing time, ms: " << ((double)time_diff.total_microseconds())/1000 << std::endl;
    // cout << std::setprecision(prec) << "dt: " << dt << endl;
    // cout << std::setprecision(prec) << "time: " << time << endl;
    // cout << std::setprecision(prec) << "Altitude AGL: " << altitudeAGL_node_->getDoubleValue() * feet2meters << endl;
    // cout << std::setprecision(prec) << "WOW[0]: " << mgr->GetNode("gear/unit/WOW")->getDoubleValue() << endl;
    // cout << std::setprecision(prec) << "WOW[1]: " << mgr->GetNode("gear/unit[1]/WOW")->getDoubleValue() << endl;
    // cout << std::setprecision(prec) << "WOW[2]: " << mgr->GetNode("gear/unit[2]/WOW")->getDoubleValue() << endl;
    cout << "Speed: " << state_->vel().norm() << endl;
    cout << std::setprecision(prec) << "xAccel: " << linear_accel_body_(0) << endl;
    cout << std::setprecision(prec) << "yAccel: " << linear_accel_body_(1) << endl;
    cout << std::setprecision(prec) << "zAccel: " << linear_accel_body_(2) << endl;
    cout << std::setprecision(prec) << "desired aileron cmd: " << delta_aileron_ << endl;
    cout << std::setprecision(prec) << "desired elevator cmd: " << delta_elevator_ << endl;
    cout << std::setprecision(prec) << "desired rudder cmd: " << delta_rudder_ << endl;
    cout << std::setprecision(prec) << "desired throttle cmd: " << throttle_ << endl;
    cout << std::setprecision(prec) << "aileron jsb: " << mgr->GetNode("fcs/right-aileron-pos-rad")->getDoubleValue() << endl;
    cout << std::setprecision(prec) << "elevator jsb: " << mgr->GetNode("fcs/elevator-pos-rad")->getDoubleValue() << endl;
    cout << std::setprecision(prec) << "rudder jsb: " << mgr->GetNode("fcs/rudder-pos-rad")->getDoubleValue() << endl;
    cout << std::setprecision(prec) << "fcs/throttle-cmd-norm: " << mgr->GetNode("fcs/throttle-cmd-norm")->getDoubleValue() << endl;
    cout << std::setprecision(prec) << "fcs/aileron-cmd-norm: " << mgr->GetNode("fcs/aileron-cmd-norm")->getDoubleValue() << endl;
    cout << std::setprecision(prec) << "fcs/elevator-cmd-norm: " << mgr->GetNode("fcs/elevator-cmd-norm")->getDoubleValue() << endl;
    cout << std::setprecision(prec) << "fcs/rudder-cmd-norm: " << mgr->GetNode("fcs/rudder-cmd-norm")->getDoubleValue() << endl;

    cout << std::setprecision(prec) << "thrust (N): " << mgr->GetNode("propulsion/engine/thrust-lbs")->getDoubleValue()*4.44 << endl;
    cout << std::setprecision(prec) << "alpha: " << mgr->GetNode("aero/alpha-rad")->getDoubleValue() << endl;
    cout << std::setprecision(prec) << "drag: " << mgr->GetNode("forces/fwx-aero-lbs")->getDoubleValue() << endl;
    cout << std::setprecision(prec) << "lift: " << -mgr->GetNode("forces/fwz-aero-lbs")->getDoubleValue() << endl;

    cout << std::setprecision(prec) << "lat:   " << latitude_node_->getDoubleValue() << endl;
    cout << std::setprecision(prec) << "lon:   " << longitude_node_->getDoubleValue() << endl;
    cout << std::setprecision(prec) << "alt:   " << altitude_node_->getDoubleValue() << endl;
    cout << std::setprecision(prec) << "ve:    " << vel_east_node_->getDoubleValue() << endl;
    cout << std::setprecision(prec) << "vn:    " << vel_north_node_->getDoubleValue() << endl;
    cout << std::setprecision(prec) << "nd:    " << vel_down_node_->getDoubleValue() << endl;
    cout << std::setprecision(prec) << "phi:   " << roll_node_->getDoubleValue() << endl;
    cout << std::setprecision(prec) << "theta: " << pitch_node_->getDoubleValue() << endl;
    cout << std::setprecision(prec) << "psi:   " << yaw_node_->getDoubleValue() << endl;
    cout << std::setprecision(prec) << "p:     " << p_node_->getDoubleValue() << endl;
    cout << std::setprecision(prec) << "q:     " << q_node_->getDoubleValue() << endl;
    cout << std::setprecision(prec) << "r:     " << r_node_->getDoubleValue() << endl;
#endif

    return true;
}

void JSBSimControl::teleport(StatePtr &state) {
    // set_jsbsim_state(*state);
    set_jsbsim_initial_state(*state);
}

void JSBSimControl::set_jsbsim_initial_state(const scrimmage::State& state) {
    // exec_->LoadScript(SGPath(jsbsim_script_path_));

    JSBSim::FGInitialCondition *ic = exec_->GetIC();

    Quaternion q_ned_enu(M_PI, 0.0, M_PI/2.0);
    Quaternion q_flu_frd(M_PI, 0.0, 0.0);
    Quaternion q_frd_enu(q_ned_enu * state_->quat() * q_flu_frd);
    ic->SetPsiRadIC(q_frd_enu.yaw());
    ic->SetThetaRadIC(q_frd_enu.pitch());
    ic->SetPhiRadIC(q_frd_enu.roll());

    ic->SetVEastFpsIC(state_->vel()[0] * meters2feet);
    ic->SetVNorthFpsIC(state_->vel()[1] * meters2feet);
    ic->SetVDownFpsIC(-state_->vel()[2] * meters2feet);

    ic->SetTerrainElevationFtIC(parent_->projection()->HeightOrigin() * meters2feet);

    Eigen::Vector3d lla;
    parent_->projection()->Reverse(state_->pos()[0], state_->pos()[1], state_->pos()[2], lla[0], lla[1], lla[2]);
    ic->SetLatitudeDegIC(lla[0]);
    ic->SetLongitudeDegIC(lla[1]);
    ic->SetAltitudeASLFtIC(lla[2] * meters2feet);

    JSBSim::FGPropertyManager* mgr = exec_->GetPropertyManager();
    mgr->GetNode("fcs/left-aileron-pos-rad")->setDoubleValue(0.0);
    mgr->GetNode("fcs/right-aileron-pos-rad")->setDoubleValue(0.0);
    // mgr->GetNode("fcs/aileron-pos-norm")->setDoubleValue(0.0);

    mgr->GetNode("fcs/elevator-pos-norm")->setDoubleValue(0.0);
    mgr->GetNode("fcs/rudder-pos-norm")->setDoubleValue(0.0);
    mgr->GetNode("fcs/throttle-pos-norm")->setDoubleValue(0.0);

    mgr->GetNode("accelerations/pdot-rad_sec2")->setDoubleValue(0.0);
    mgr->GetNode("accelerations/qdot-rad_sec2")->setDoubleValue(0.0);
    mgr->GetNode("accelerations/rdot-rad_sec2")->setDoubleValue(0.0);

    mgr->GetNode("accelerations/udot-ft_sec2")->setDoubleValue(0.0);
    mgr->GetNode("accelerations/vdot-ft_sec2")->setDoubleValue(0.0);
    mgr->GetNode("accelerations/wdot-ft_sec2")->setDoubleValue(0.0);

    mgr->GetNode("accelerations/a-pilot-x-ft_sec2")->setDoubleValue(0.0);
    mgr->GetNode("accelerations/a-pilot-y-ft_sec2")->setDoubleValue(0.0);
    mgr->GetNode("accelerations/a-pilot-z-ft_sec2")->setDoubleValue(0.0);

    ap_aileron_cmd_node_->setDoubleValue(0);
    ap_elevator_cmd_node_->setDoubleValue(0);
    ap_rudder_cmd_node_->setDoubleValue(0);
    ap_throttle_cmd_node_->setDoubleValue(0);
    // ap_throttle_1_cmd_node_->setDoubleValue(0);


    if (not init_values_cached_) {
        init_values_cached_ = true;
        init_thrust_ = mgr->GetNode("propulsion/engine/thrust-lbs")->getDoubleValue();

        init_fuel_rate_gph_ = mgr->GetNode("propulsion/engine/fuel-flow-rate-gph")->getDoubleValue();
        init_fuel_used_lbs_ = mgr->GetNode("propulsion/engine/fuel-used-lbs")->getDoubleValue();
        init_total_fuel_lbs_ = mgr->GetNode("propulsion/total-fuel-lbs")->getDoubleValue();

    }
    mgr->GetNode("propulsion/engine/thrust-lbs")->setDoubleValue(init_thrust_);
    // mgr->GetNode("propulsion/engine[1]/thrust-lbs")->setDoubleValue(init_thrust_);

    mgr->GetNode("propulsion/engine/fuel-flow-rate-gph")->setDoubleValue(init_fuel_rate_gph_);
    mgr->GetNode("propulsion/engine/fuel-used-lbs")->setDoubleValue(init_fuel_used_lbs_);
    mgr->GetNode("propulsion/total-fuel-lbs")->setDoubleValue(init_total_fuel_lbs_);

    mgr->GetNode("velocities/h-dot-fps")->setDoubleValue(0);

    // exec_->RunIC();
    exec_->ResetToInitialConditions(0);

    exec_->Setdt(jsbsim_dt_);
    exec_->Run();

    mgr->GetNode("aero/alpha-deg")->setDoubleValue(0); // OK?
    mgr->GetNode("aero/beta-deg")->setDoubleValue(0); // OK?

}

void JSBSimControl::set_jsbsim_state(const scrimmage::State& state) {
    // JSBSim lat/lon/alt from scrimmage state
    double lat, lon, alt;
    parent_->projection()->Reverse(state.pos()(0), state.pos()(1), state.pos()(2),
                                   lat, lon, alt);
    latitude_node_->setDoubleValue(lat);
    longitude_node_->setDoubleValue(lon);
    altitude_node_->setDoubleValue(alt / feet2meters);

    // Set JSBSim orientation from scrimmage state
    roll_node_->setDoubleValue(state.quat().roll());
    pitch_node_->setDoubleValue(-state.quat().pitch());

    angles_to_jsbsim_.set_angle(ang::rad2deg(state.quat().yaw()));
    yaw_node_->setDoubleValue(ang::deg2rad(angles_to_jsbsim_.angle()));

    // set JSBSim linear velocity (NED) from scrimmage state (ENU)
    vel_north_node_->setDoubleValue(state.vel()(1) / feet2meters); // scrimmage y-axis
    vel_east_node_->setDoubleValue(state.vel()(0) / feet2meters); // scrimmage x-axis
    vel_down_node_->setDoubleValue(-state.vel()(2) / feet2meters); // scrimmage -z-axis

    // set JSBSim angular velocity (?) from scrimmage state (ENU)
    // TODO: What frame is JSBSim using? zeros for now
    p_node_->setDoubleValue(0);
    q_node_->setDoubleValue(0);
    r_node_->setDoubleValue(0);
}

std::shared_ptr<JSBSim::FGFDMExec> JSBSimControl::jsbsim_exec() {
    return exec_;
}

} // namespace motion
} // namespace scrimmage
