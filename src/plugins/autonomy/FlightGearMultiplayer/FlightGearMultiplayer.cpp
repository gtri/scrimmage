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

#include <iostream>
#include <limits>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>

#include <time.h>

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
    earth_(std::make_shared<GeographicLib::Geocentric>(
               GeographicLib::Constants::WGS84_a(),
               GeographicLib::Constants::WGS84_f())) {
}

void FlightGearMultiplayer::init(std::map<std::string, std::string> &params) {

    std::string server_ip = sc::get<std::string>("server_ip", params, "127.0.0.1");
    int server_port = sc::get<int>("server_port", params, 5000);
    callsign_ = sc::get<std::string>("callsign", params, "scrimmage");

    net_address_.set(server_ip.c_str(), server_port);
    if(data_socket_->open(false)  == 0) { // Open UDP socket
        std::cout << "Failed to open connection to: " << endl;
        cout << "Server IP: " << net_address_.getIP() << endl;
        cout << "Server Port: " << net_address_.getPort() << endl;
    }
}

bool FlightGearMultiplayer::step_autonomy(double t, double dt) {
    double lat, lon, alt;
    parent_->projection()->Reverse(state_->pos()(0), state_->pos()(1),
                                   state_->pos()(2),
                                   lat, lon, alt);


    double X, Y, Z;
    earth_->Forward(lat, lon, alt, X, Y, Z);
    // X = std::floor(X / 1000 + 0.5);
    // Y = std::floor(Y / 1000 + 0.5);
    // Z = std::floor(Z / 1000 + 0.5);

    // float velx, vel2, vel3;
    // parent_->projection()->Reverse(state_->vel()(0), state_->vel()(1),
    //                                state_->vel()(2),
    //                                vel1, vel2, vel3);
    // float ang_vel1, ang_vel2, ang_vel3;
    // parent_->projection()->Reverse(state_->ang_vel()(0), state_->ang_vel()(1),
    //                                state_->ang_vel()(2),
    //                                lat, lon, alt);
    //float vel_x = 0, vel_y = 0, vel_z = 0;
    //float ang_vel_x = 0, ang_vel_y = 0, ang_vel_z = 0;

    int msg_size = sizeof(T_MsgHdr) + sizeof(T_PositionMsg);

    // Create the flight gear header
    struct T_MsgHdr header_msg;
    header_msg.Magic = XDR_encode<uint32_t>(MSG_MAGIC);
    header_msg.Version = XDR_encode<uint32_t>(PROTO_VER);
    header_msg.MsgId = XDR_encode<uint32_t>(FGFS::POS_DATA);
    header_msg.MsgLen = XDR_encode<uint32_t>(msg_size);
    header_msg.RadarRange = XDR_decode<float>(15);
    header_msg.ReplyPort = XDR_decode<uint32_t>(0);
    sprintf(header_msg.Name, "%s", callsign_.c_str());

    // Create the flight gear position msg
    struct T_PositionMsg pos_msg;
    sprintf(pos_msg.Model, "%s", "Aircraft/c172p/Models/c172p.xml");
    pos_msg.time = XDR_encode64<float>(time_->t());
    pos_msg.lag = XDR_encode64<float>(time_->t());

    // time_t current_time = time(0);
    // pos_msg.time = XDR_encode64<float>();
    // pos_msg.lag = XDR_encode64<float>(time_->t());

    pos_msg.position[0] = XDR_encode64<double>(X);
    pos_msg.position[1] = XDR_encode64<double>(Y);
    pos_msg.position[2] = XDR_encode64<double>(Z);

    // TODO conversion
    pos_msg.orientation[0] = XDR_encode<float>(-2.136887);
    pos_msg.orientation[1] = XDR_encode<float>(0.587624);
    pos_msg.orientation[2] = XDR_encode<float>(-0.304996);

    // TODO conversion
    pos_msg.linearVel[0] = XDR_encode<float>(state_->vel()(0));
    pos_msg.linearVel[1] = XDR_encode<float>(state_->vel()(0));
    pos_msg.linearVel[2] = XDR_encode<float>(state_->vel()(0));

    pos_msg.angularVel[0] = XDR_encode<float>(state_->ang_vel()(0));
    pos_msg.angularVel[1] = XDR_encode<float>(state_->ang_vel()(0));
    pos_msg.angularVel[2] = XDR_encode<float>(state_->ang_vel()(0));

    pos_msg.linearAccel[0]= XDR_encode<float>(0);
    pos_msg.linearAccel[1]= XDR_encode<float>(0);
    pos_msg.linearAccel[2]= XDR_encode<float>(0);

    pos_msg.angularAccel[0] = XDR_encode<float>(0);
    pos_msg.angularAccel[1] = XDR_encode<float>(0);
    pos_msg.angularAccel[2] = XDR_encode<float>(0);

    // Copy header and position data structs into memory buffer
    char * msg = new char[msg_size];
    std::memcpy(msg, &header_msg, sizeof(T_MsgHdr));
    std::memcpy(msg+sizeof(T_MsgHdr), &pos_msg, sizeof(T_PositionMsg));

    data_socket_->sendto(msg, msg_size, 0, &net_address_);
    delete[] msg;

    return true;
}
} // namespace autonomy
} // namespace scrimmage
