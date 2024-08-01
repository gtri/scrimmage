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

#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/autonomy/ArduPilot/ArduPilot.h>
#include <scrimmage/sensor/Sensor.h>

#include <chrono> // NOLINT
#include <iostream>

#include <GeographicLib/LocalCartesian.hpp>
#include <boost/asio/basic_datagram_socket.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/bind.hpp>

using std::cerr;
using std::cout;
using std::endl;

namespace ba = boost::asio;
namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::ArduPilot, ArduPilot_plugin)

namespace scrimmage {
namespace autonomy {

ArduPilot::ArduPilot()
    : angles_to_gps_(0, Angles::Type::EUCLIDEAN, Angles::Type::GPS) {
    for (int i = 0; i < MAX_NUM_SERVOS; i++) {
        servo_pkt_.servos[i] = 0;
    }
}

void ArduPilot::init(std::map<std::string, std::string>& params) {
    std::string servo_map = sc::get<std::string>("servo_map", params, "");
    std::vector<std::vector<std::string>> vecs;
    if (!sc::get_vec_of_vecs(servo_map, vecs)) {
        cout << "Failed to parse servo map:" << servo_map << endl;
    } else {
        for (const std::vector<std::string>& vec : vecs) {
            if (vec.size() != 7) {
                cout << "Invalid servo mapping: " << endl;
                for (const std::string& s : vec) {
                    cout << s << " ";
                }
                continue;
            }

            int servo = std::stod(vec[1]);
            if (servo >= MAX_NUM_SERVOS) {
                cout << "Warning: servo_map contains out-of-range servo index" << endl;
            } else {
                scrimmage::controller::AxisScale at(
                    servo,
                    std::stod(vec[2]),
                    std::stod(vec[3]),
                    std::stod(vec[4]),
                    std::stod(vec[5]),
                    std::stod(vec[6]),
                    vars_.declare(vec[0], VariableIO::Direction::Out));
                servo_tfs_.push_back(at);

                // cout << "Mapping servo " << at.axis_index() << " to channel " <<
                //     vec[0] << "(" << at.vector_index() << ")" << endl;
            }
        }
    }

    mavproxy_mode_ = sc::get<bool>("mavproxy_mode", params, false);
    asynchonous_mode_ = sc::get<bool>("asynchronous_mode", params, true);

    // Get parameters for transmit socket (to ardupilot)
    std::string to_ardupilot_ip = sc::get<std::string>("to_ardupilot_ip", params, "127.0.0.1");

    int to_ardupilot_port = sc::get<int>("to_ardupilot_port", params, 5003);
    int from_ardupilot_port = sc::get<int>("from_ardupilot_port", params, 5002);

    // If the ardupilot_port_base is provided, use the instance multiplier to
    // automatically compute the Rx and Tx port numbers:
    auto it_base_port = params.find("ardupilot_port_base");
    if (it_base_port != params.end()) {
        int multiplier = sc::get<int>("port_instance_multiplier", params, 10);

        int base_port = convert<int>(it_base_port->second);
        from_ardupilot_port = base_port + multiplier * parent_->id().id();
        to_ardupilot_port = from_ardupilot_port + 1;
    }

    cout << "ArduPilot: sending to udp: " << to_ardupilot_ip << ":" << to_ardupilot_port << endl;
    cout << "ArduPilot: listening to udp: " << to_ardupilot_ip << ":" << from_ardupilot_port
         << endl;

    // Setup transmit socket
    tx_socket_ = std::make_shared<ba::ip::udp::socket>(tx_io_service_,
                                                       ba::ip::udp::endpoint(ba::ip::udp::v4(), 0));
    tx_resolver_ = std::make_shared<ba::ip::udp::resolver>(tx_io_service_);
    tx_endpoint_ = *(tx_resolver_->resolve(
        {ba::ip::udp::v4(), to_ardupilot_ip, std::to_string(to_ardupilot_port)}));

    recv_socket_ = std::make_shared<ba::ip::udp::socket>(
        recv_io_service_, ba::ip::udp::endpoint(ba::ip::udp::v4(), from_ardupilot_port));
    start_receive();

    state_6dof_ = std::make_shared<motion::RigidBody6DOFState>();
    auto cb = [&](const auto& msg) { *state_6dof_ = msg->data; };
    subscribe<motion::RigidBody6DOFState>("LocalNetwork", "RigidBody6DOFState", cb);
}

static int nrx = 0;
static int ntx = 0;
static int last_ntx = 0;
static double last_print_t = 0;
static std::chrono::time_point<std::chrono::system_clock> last_print_wall_t;
void ArduPilot::start_receive() {
    // Enable the receive async callback
    recv_socket_->async_receive_from(ba::buffer(recv_buffer_),
                                     recv_remote_endpoint_,
                                     boost::bind(&ArduPilot::handle_receive,
                                                 this,
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
        tx_socket_->send_to(ba::buffer(&fdm_pkt, sizeof(fdm_packet)), tx_endpoint_);
        ntx++;
        last_ntx++;
    } catch (std::exception& e) {
        cerr << "Exception: " << e.what() << "\n";
    }

    // Let async receive callback run
    if (asynchonous_mode_) {
        recv_io_service_.poll();
    } else {
        boost::system::error_code error;
        ba::ip::udp::socket::message_flags flags = 0;
        std::size_t nbytes = recv_socket_->receive_from(
            ba::buffer(recv_buffer_), recv_remote_endpoint_, flags, error);

        parse_receive(error, nbytes);
    }

    // Copy the received servo commands into the desired state
    servo_pkt_mutex_.lock();
    for (auto servo_tf : servo_tfs_) {
        vars_.output(servo_tf.vector_index(),
                     servo_tf.scale(servo_pkt_.servos[servo_tf.axis_index()]));
        // int prec = 9;
        // cout << std::setprecision(prec) << "servo_out"<< servo_tf.axis_index() << ": " <<
        // servo_tf.scale(servo_pkt_.servos[servo_tf.axis_index()]) << endl;
    }
    servo_pkt_mutex_.unlock();

    if (t - last_print_t > 5) {
        const auto t0 = std::chrono::system_clock::now();
        auto d = std::chrono::duration_cast<std::chrono::microseconds>(t0 - last_print_wall_t);
        cout << "walltime: " << t0.time_since_epoch().count() << ", simtime: " << t
             << ", packets tx: " << ntx << ", rx: " << nrx
             << ", rate: " << double(last_ntx) / double(d.count()) * 1e6 << " hz" << endl;
        last_print_t = t;
        last_print_wall_t = t0;
        last_ntx = 0;
    }

    return true;
}

void ArduPilot::parse_receive(const boost::system::error_code& error, std::size_t num_bytes) {
#if 0
    cout << "--------------------------------------------------------" << endl;
    cout << "  Servo packets received from ArduPilot (ID: "
            << parent_->id().id() << ")" << endl;
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
            servo_pkt_.servos[i] = (recv_buffer_[i * 2 + 1] << 8) + recv_buffer_[i * 2];
#if 0
            int prec = 9;
            cout << std::setprecision(prec) << "servo"<< i << ": " << servo_pkt_.servos[i] << endl;
#endif
        }
        servo_pkt_mutex_.unlock();
    }
    nrx++;
}

void ArduPilot::handle_receive(const boost::system::error_code& error, std::size_t num_bytes) {
    parse_receive(error, num_bytes);
    start_receive();  // enable async receive for next message
}

ArduPilot::fdm_packet ArduPilot::state6dof_to_fdm_packet(double t,
                                                         sc::motion::RigidBody6DOFState& state) {
    fdm_packet fdm_pkt = {0};

    fdm_pkt.timestamp_us = t * 1e6;

    // x/y/z to lat/lon/alt conversion
    parent_->projection()->Reverse(state.pos()(0),
                                   state.pos()(1),
                                   state.pos()(2),
                                   fdm_pkt.latitude,
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
    if (mavproxy_mode_) {
        Eigen::Vector3d specific_force_body =
            state.linear_accel_body() - state.quat().rotate_reverse(Eigen::Vector3d(0, 0, -9.81));
        fdm_pkt.xAccel = specific_force_body(0);
        fdm_pkt.yAccel = -specific_force_body(1);
        fdm_pkt.zAccel = -specific_force_body(2);
    } else {
        fdm_pkt.xAccel = state.linear_accel_body()(0);
        fdm_pkt.yAccel = -state.linear_accel_body()(1);
        fdm_pkt.zAccel = -state.linear_accel_body()(2);
    }

    // Global frame, roll, pitch, yaw from NED to FRU
    fdm_pkt.roll = state.quat().roll();
    fdm_pkt.pitch = -state.quat().pitch();
    fdm_pkt.yaw = fdm_pkt.heading;

    if (mavproxy_mode_) {
        // world frame rotational velocities
        double p = state.ang_vel_body()(0);
        double q = -state.ang_vel_body()(1);
        double r = -state.ang_vel_body()(2);
        double sphi = sin(fdm_pkt.roll);
        double cphi = cos(fdm_pkt.roll);
        double tth = tan(fdm_pkt.pitch);
        double cth = std::max(.00001, cos(fdm_pkt.pitch));
        fdm_pkt.rollRate = p + tth * sphi * q + tth * cphi * r;
        fdm_pkt.pitchRate = cphi * q - sphi * r;
        fdm_pkt.yawRate = sphi / cth * q + cphi / cth * r;
    } else {
        // Body frame rotational velocities FRU
        fdm_pkt.rollRate = state.ang_vel_body()(0);
        fdm_pkt.pitchRate = -state.ang_vel_body()(1);
        fdm_pkt.yawRate = -state.ang_vel_body()(2);
    }

    // Airspeed is magnitude of velocity vector for now
    fdm_pkt.airspeed = (state.vel() + state.wind()).norm();
    if (std::isnan(fdm_pkt.airspeed)) {
        // The norm of a small vector can be NaN, so most likely, the airspeed
        // should be zero in this case.
        fdm_pkt.airspeed = 0.0;
    }

    if (std::isinf(fdm_pkt.airspeed)) {
        std::cout << "ArduPilot: Warning: velocity or wind contains infinite value." << std::endl;
        fdm_pkt.airspeed = 0.0;
    }

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

}  // namespace autonomy
}  // namespace scrimmage
