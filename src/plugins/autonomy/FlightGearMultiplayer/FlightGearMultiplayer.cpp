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

#include <scrimmage/plugins/autonomy/FlightGearMultiplayer/FlightGearMultiplayer.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/common/Time.h>

#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>

#include <iostream>
#include <limits>

#include <flightgear/MultiPlayer/mpmessages.hxx>
#include <flightgear/MultiPlayer/tiny_xdr.hxx>

#include <GeographicLib/LocalCartesian.hpp>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::FlightGearMultiplayer,
                FlightGearMultiplayer_plugin)

namespace scrimmage {
namespace autonomy {

FlightGearMultiplayer::FlightGearMultiplayer() :
    callsign_("scrimmage"),
    data_socket_(std::make_shared<netSocket>()),
    aircraft_model_(std::string("Aircraft/c172p/Models/c172p.xml")),
    earth_(std::make_shared<GeographicLib::Geocentric>(
               GeographicLib::Constants::WGS84_a(),
               GeographicLib::Constants::WGS84_f())),
    angles_to_jsbsim_(0, Angles::Type::EUCLIDEAN, Angles::Type::GPS) {
}

void FlightGearMultiplayer::init(std::map<std::string, std::string> &params) {

    std::string server_ip = sc::get<std::string>("server_ip", params, "127.0.0.1");
    int server_port = sc::get<int>("server_port", params, 5000);
    callsign_ = sc::get<std::string>("callsign", params, "scrimmage");
    aircraft_model_ = sc::get<std::string>("aircraft_model", params, aircraft_model_);

    net_address_.set(server_ip.c_str(), server_port);
    if (data_socket_->open(false)  == 0) { // Open UDP socket
        std::cout << "Failed to open connection to: " << endl;
        cout << "Server IP: " << net_address_.getIP() << endl;
        cout << "Server Port: " << net_address_.getPort() << endl;
    }

    // Create the flight gear header
    msg_size_ = sizeof(T_MsgHdr) + sizeof(T_PositionMsg);
    header_msg_.Magic = XDR_encode<uint32_t>(MSG_MAGIC);
    header_msg_.Version = XDR_encode<uint32_t>(PROTO_VER);
    header_msg_.MsgId = XDR_encode<uint32_t>(FGFS::POS_DATA);
    header_msg_.MsgLen = XDR_encode<uint32_t>(msg_size_);
    header_msg_.RadarRange = XDR_decode<float>(15);
    header_msg_.ReplyPort = XDR_decode<uint32_t>(0);
    snprintf(header_msg_.Name, MAX_CALLSIGN_LEN, "%s", callsign_.c_str());

    // Initialize parts of position message that aren't updated
    snprintf(pos_msg_.Model, MAX_MODEL_NAME_LEN, "%s", aircraft_model_.c_str());
    pos_msg_.linearVel[0] = XDR_encode<float>(0);
    pos_msg_.linearVel[1] = XDR_encode<float>(0);
    pos_msg_.linearVel[2] = XDR_encode<float>(0);

    pos_msg_.angularVel[0] = XDR_encode<float>(0);
    pos_msg_.angularVel[1] = XDR_encode<float>(0);
    pos_msg_.angularVel[2] = XDR_encode<float>(0);

    pos_msg_.linearAccel[0]= XDR_encode<float>(0);
    pos_msg_.linearAccel[1]= XDR_encode<float>(0);
    pos_msg_.linearAccel[2]= XDR_encode<float>(0);

    pos_msg_.angularAccel[0] = XDR_encode<float>(0);
    pos_msg_.angularAccel[1] = XDR_encode<float>(0);
    pos_msg_.angularAccel[2] = XDR_encode<float>(0);
}

scrimmage::Quaternion FlightGearMultiplayer::fromLonLatRad(float lon, float lat) {
    // TODO : This transformation was taken from the Flight Gear Multiplayer
    // server, we should convert this to using our own quaternion class, but
    // for now, this works.

    // Eigen::AngleAxisd R_a(sc::Angles::deg2rad(lon), Eigen::Vector3d::UnitZ());
    // Eigen::AngleAxisd R_b(sc::Angles::deg2rad(lat),
    // Eigen::Vector3d::UnitX()); Eigen::Quaterniond rot2 = R_a * R_b;

    float zd2 = static_cast<float>(0.5) * lon;
    float yd2 = static_cast<float>(-0.25) * static_cast<float>(M_PI) - static_cast<float>(0.5) * lat;
    float Szd2 = sin(zd2);
    float Syd2 = sin(yd2);
    float Czd2 = cos(zd2);
    float Cyd2 = cos(yd2);
    scrimmage::Quaternion quat(Czd2 * Cyd2, -Szd2 * Syd2, Czd2 * Syd2, Szd2 * Cyd2);
    return quat;
}

bool FlightGearMultiplayer::step_autonomy(double t, double dt) {
    // Create the flight gear position msg
    pos_msg_.time = XDR_encode64<float>(time_->t());
    pos_msg_.lag = XDR_encode64<float>(time_->t());

    // Compute ECEF coordinates from x/y/z lat/lon/alt
    double lat, lon, alt;
    parent_->projection()->Reverse(state_->pos()(0), state_->pos()(1),
                                   state_->pos()(2),
                                   lat, lon, alt);
    double ECEF_X, ECEF_Y, ECEF_Z;
    earth_->Forward(lat, lon, alt, ECEF_X, ECEF_Y, ECEF_Z);

    pos_msg_.position[0] = XDR_encode64<double>(ECEF_X);
    pos_msg_.position[1] = XDR_encode64<double>(ECEF_Y);
    pos_msg_.position[2] = XDR_encode64<double>(ECEF_Z);

    // Convert local heading to GPS heading
    angles_to_jsbsim_.set_angle(sc::Angles::rad2deg(state_->quat().yaw()));
    sc::Quaternion local_quat(state_->quat().roll(), state_->quat().pitch(),
                              sc::Angles::deg2rad(angles_to_jsbsim_.angle()));

    // Convert local quaternion to ECEF coordinate
    scrimmage::Quaternion rot = this->fromLonLatRad(sc::Angles::deg2rad(lon),
                                                    sc::Angles::deg2rad(lat));

    // The FlightGear Multiplayer packet expects the orientation to be in
    // angle-axis form, where the axis magnitude is the rotation angle.
    Eigen::AngleAxisd ang_axis(rot * local_quat);
    Eigen::Vector3d ang_axis_mod = ang_axis.axis().normalized() * ang_axis.angle();

    pos_msg_.orientation[0] = XDR_encode<float>(ang_axis_mod(0));
    pos_msg_.orientation[1] = XDR_encode<float>(ang_axis_mod(1));
    pos_msg_.orientation[2] = XDR_encode<float>(ang_axis_mod(2));

    // Copy header and position data structs into memory buffer
    char * msg = new char[msg_size_];
    std::memcpy(msg, &header_msg_, sizeof(T_MsgHdr));
    std::memcpy(msg+sizeof(T_MsgHdr), &pos_msg_, sizeof(T_PositionMsg));

    data_socket_->sendto(msg, msg_size_, 0, &net_address_);
    delete[] msg;

    return true;
}
} // namespace autonomy
} // namespace scrimmage
