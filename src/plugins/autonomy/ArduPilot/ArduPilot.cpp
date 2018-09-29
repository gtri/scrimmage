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
 * @author Michael Day <michael.day@gtri.gatech.edu>
 * @date 20 Nov 2017
 * @version 0.1.0
 * @brief Class to interface with ArduPilot SIL executable.
 * @section DESCRIPTION
 * Class to interface with ArduPilot SIL executable.
 *
 */

#include <scrimmage/plugins/autonomy/ArduPilot/ArduPilot.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/sensor/Sensor.h>

#include <iomanip>
#include <iostream>
#include <limits>
#include <fstream>

#include <GeographicLib/LocalCartesian.hpp>

#include <boost/bind.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/basic_datagram_socket.hpp>

using std::cout;
using std::cerr;
using std::endl;

namespace ba = boost::asio;
namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::ArduPilot,
                ArduPilot_plugin)

namespace scrimmage {
namespace autonomy {

ArduPilot::ArduPilot() :
    to_ardupilot_ip_("127.0.0.1"),
    to_ardupilot_port_("5003"),
    angles_to_gps_(0, Angles::Type::EUCLIDEAN, Angles::Type::GPS),
    from_ardupilot_port_(5002) {

    for (int i = 0; i < MAX_NUM_SERVOS; i++) {
        servo_pkt_.servos[i] = 0;
    }
}

void ArduPilot::init(std::map<std::string, std::string> &params) {


    std::string servo_map = sc::get<std::string>("servo_map", params, "");
    std::vector<std::vector<std::string>> vecs;
    if (!sc::get_vec_of_vecs(servo_map, vecs)) {
        cout << "Failed to parse servo map:" << servo_map << endl;
    } else {
        for (std::vector<std::string> vec : vecs) {
            if (vec.size() != 7) {
                cout << "Invalid servo mapping: " << endl;
                for (std::string s : vec) {
                    cout << s << " ";
                }
                continue;
            }

            int servo = std::stod(vec[1]);
            if (servo >= MAX_NUM_SERVOS) {
                cout << "Warning: servo_map contains out-of-range servo index"
                     << endl;
            } else {
                scrimmage::controller::AxisScale at(servo, std::stod(vec[2]),
                             std::stod(vec[3]), std::stod(vec[4]),
                             std::stod(vec[5]), std::stod(vec[6]),
                             vars_.declare(vec[0], VariableIO::Direction::Out));
                servo_tfs_.push_back(at);

                // cout << "Mapping servo " << at.axis_index() << " to channel " <<
                //     vec[0] << "(" << at.vector_index() << ")" << endl;
            }
        }
    }

    // Get parameters for transmit socket (to ardupilot)
    to_ardupilot_ip_ = sc::get<std::string>("to_ardupilot_ip", params, "127.0.0.1");
    to_ardupilot_port_ = sc::get<std::string>("to_ardupilot_port", params, "5003");

    // Setup transmit socket
    tx_socket_ = std::make_shared<ba::ip::udp::socket>(tx_io_service_,
                                                       ba::ip::udp::endpoint(
                                                           ba::ip::udp::v4(),
                                                           0));

    tx_resolver_ = std::make_shared<ba::ip::udp::resolver>(tx_io_service_);
    tx_endpoint_ = *(tx_resolver_->resolve({ba::ip::udp::v4(),
                    to_ardupilot_ip_, to_ardupilot_port_}));

    // Get parameters for receive socket (from ardupilot)
    from_ardupilot_port_ = sc::get<int>("from_ardupilot_port", params, 5002);
    recv_socket_ = std::make_shared<ba::ip::udp::socket>(recv_io_service_,
                                                         ba::ip::udp::endpoint(
                                                             ba::ip::udp::v4(),
                                                             from_ardupilot_port_));
    start_receive();

    state_6dof_ = std::make_shared<motion::RigidBody6DOFState>();
    auto cb = [&](auto &msg){*state_6dof_ = msg->data;};
    subscribe<motion::RigidBody6DOFState>("LocalNetwork", "RigidBody6DOFState", cb);
}

void ArduPilot::start_receive() {
    // Enable the receive async callback
    recv_socket_->async_receive_from(
        ba::buffer(recv_buffer_), recv_remote_endpoint_,
        boost::bind(&ArduPilot::handle_receive, this,
                    ba::placeholders::error,
                    ba::placeholders::bytes_transferred));
}

void ArduPilot::close(double t) {
    tx_socket_->close();
    recv_socket_->close();
}

bool ArduPilot::step_autonomy(double t, double dt) {

    // Convert state6dof into ArduPilot fdm_packet and transmit
    fdm_packet fdm_pkt = state6dof_to_fdm_packet(t, *state_6dof_);
    try {
        // TODO: Michael, endian handling?
        tx_socket_->send_to(ba::buffer(&fdm_pkt, sizeof(fdm_packet)),
                            tx_endpoint_);
    } catch (std::exception& e) {
        cerr << "Exception: " << e.what() << "\n";
    }

    // Let async receive callback run
    recv_io_service_.poll();

    // Copy the received servo commands into the desired state
    servo_pkt_mutex_.lock();
    for (auto servo_tf : servo_tfs_) {
        vars_.output(servo_tf.vector_index(), servo_tf.scale(servo_pkt_.servos[servo_tf.axis_index()]));
        // int prec = 9;
        // cout << std::setprecision(prec) << "servo_out"<< servo_tf.axis_index() << ": " << servo_tf.scale(servo_pkt_.servos[servo_tf.axis_index()]) << endl;
    }
    servo_pkt_mutex_.unlock();

    return true;
}

void ArduPilot::handle_receive(const boost::system::error_code& error,
                               std::size_t num_bytes) {
#if 0
    cout << "--------------------------------------------------------" << endl;
    cout << "  Servo packets received from ArduPilot" << endl;
    cout << "--------------------------------------------------------" << endl;
#endif

    if (error) {
        cout << "error: handle_receive" << endl;
    } else if (num_bytes != sizeof(servo_packet)) {
        cout << "Received wrong number of bytes: " << num_bytes << endl;
        cout << "Expected number of bytes: " << sizeof(servo_packet) << endl;
    } else {
        servo_pkt_mutex_.lock();
        for (unsigned int i = 0; i < num_bytes / sizeof(uint16_t); i++) {
            servo_pkt_.servos[i] = (recv_buffer_[i*2+1] << 8) + recv_buffer_[i*2];
#if 0
            int prec = 9;
            cout << std::setprecision(prec) << "servo"<< i << ": " << servo_pkt_.servos[i] << endl;
#endif
        }
        servo_pkt_mutex_.unlock();
    }
    start_receive(); // enable async receive for next message
}

ArduPilot::fdm_packet ArduPilot::state6dof_to_fdm_packet(
    double t, sc::motion::RigidBody6DOFState &state) {

    fdm_packet fdm_pkt = {0};

    fdm_pkt.timestamp_us = t * 1e6;

    // x/y/z to lat/lon/alt conversion
    parent_->projection()->Reverse(state.pos()(0), state.pos()(1),
                                   state.pos()(2), fdm_pkt.latitude,
                                   fdm_pkt.longitude,
                                   fdm_pkt.altitude);

    // convert everything to NED-FRU world and body frames

    // Heading conversion
    angles_to_gps_.set_angle(sc::Angles::rad2deg(state.quat().yaw()));
    fdm_pkt.heading = sc::Angles::deg2rad(angles_to_gps_.angle());

    fdm_pkt.speedN = state.vel()(1);
    fdm_pkt.speedE = state.vel()(0);
    fdm_pkt.speedD = -state.vel()(2);

    // Body frame linear acceleration FRU
    fdm_pkt.xAccel = state.linear_accel_body()(0);
    fdm_pkt.yAccel = -state.linear_accel_body()(1);
    fdm_pkt.zAccel = -state.linear_accel_body()(2);

    // Body frame rotational velocities FRU
    fdm_pkt.rollRate = state.ang_vel_body()(0);
    fdm_pkt.pitchRate = -state.ang_vel_body()(1);
    fdm_pkt.yawRate = -state.ang_vel_body()(2);

    // Global frame, roll, pitch, yaw from NED to FRU
    fdm_pkt.roll = state.quat().roll();
    fdm_pkt.pitch = -state.quat().pitch();
    fdm_pkt.yaw = fdm_pkt.heading;

    // Airspeed is magnitude of velocity vector for now
    fdm_pkt.airspeed = state.vel().norm();

#if 0
    cout << "--------------------------------------------------------" << endl;
    cout << "  State information being sent to ArduPilot" << endl;
    cout << "--------------------------------------------------------" << endl;
    int prec = 9;
    cout << std::setprecision(prec) << "timestamp_us: " << fdm_pkt.timestamp_us << endl;
    cout << std::setprecision(prec) << "Latitude: " << fdm_pkt.latitude << endl;
    cout << std::setprecision(prec) << "Longitude: " << fdm_pkt.longitude << endl;
    cout << std::setprecision(prec) << "Altitude: " << fdm_pkt.altitude << endl;
    cout << std::setprecision(prec) << "Airspeed: " << fdm_pkt.airspeed << endl;
    cout << std::setprecision(prec) << "Roll: " << fdm_pkt.roll << endl;
    cout << std::setprecision(prec) << "Pitch: " << fdm_pkt.pitch << endl;
    cout << std::setprecision(prec) << "Yaw: " << fdm_pkt.yaw << endl;
    cout << std::setprecision(prec) << "xAccel: " << fdm_pkt.xAccel << endl;
    cout << std::setprecision(prec) << "yAccel: " << fdm_pkt.yAccel << endl;
    cout << std::setprecision(prec) << "zAccel: " << fdm_pkt.zAccel << endl;
    cout << std::setprecision(prec) << "speedN: " << fdm_pkt.speedN << endl;
    cout << std::setprecision(prec) << "speedE: " << fdm_pkt.speedE << endl;
    cout << std::setprecision(prec) << "speedD: " << fdm_pkt.speedD << endl;
    cout << std::setprecision(prec) << "heading: " << fdm_pkt.heading << endl;
    cout << std::setprecision(prec) << "rollRate: " << fdm_pkt.rollRate << endl;
    cout << std::setprecision(prec) << "pitchRate: " << fdm_pkt.pitchRate << endl;
    cout << std::setprecision(prec) << "yawRate: " << fdm_pkt.yawRate << endl;
#endif

    return fdm_pkt;
}

} // namespace autonomy
} // namespace scrimmage
