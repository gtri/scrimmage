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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_ARDUPILOT_ARDUPILOT_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_ARDUPILOT_ARDUPILOT_H_

#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/math/Angles.h>

#include <scrimmage/plugins/motion/RigidBody6DOF/RigidBody6DOFState.h>
#include <scrimmage/plugins/controller/JoystickController/AxisScale.h>

#include <thread> // NOLINT
#include <mutex> // NOLINT
#include <map>
#include <string>
#include <list>

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>
#include <boost/array.hpp>

namespace scrimmage {

namespace motion {
class RigidBody6DOFState;
}

namespace autonomy {
class ArduPilot : public scrimmage::Autonomy {
 private:
    static const int MAX_NUM_SERVOS = 16;
    // Packet received by Scrimmage with state of ArduPilot servos
    struct servo_packet {
        uint16_t servos[MAX_NUM_SERVOS];
    };

    // State packet sent from Scrimmage to ArduPilot
    struct fdm_packet {
        uint64_t timestamp_us; // simulation time in microseconds
        double latitude, longitude;
        double altitude;
        double heading;
        double speedN, speedE, speedD;
        double xAccel, yAccel, zAccel;
        double rollRate, pitchRate, yawRate;
        double roll, pitch, yaw;
        double airspeed;
    };

 public:
    ArduPilot();
    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;
    void close(double t) override;

 protected:
    std::string to_ardupilot_ip_;
    std::string to_ardupilot_port_;

    boost::asio::io_service tx_io_service_;
    std::shared_ptr<boost::asio::ip::udp::socket> tx_socket_;
    std::shared_ptr<boost::asio::ip::udp::resolver> tx_resolver_;
    boost::asio::ip::udp::endpoint tx_endpoint_;

    std::list<scrimmage::controller::AxisScale> servo_tfs_;

    servo_packet servo_pkt_;
    std::mutex servo_pkt_mutex_;

    scrimmage::Angles angles_to_gps_;

    fdm_packet state6dof_to_fdm_packet(double t,
                                       scrimmage::motion::RigidBody6DOFState &state);

    boost::asio::io_service recv_io_service_;
    std::shared_ptr<boost::asio::ip::udp::socket> recv_socket_;
    boost::asio::ip::udp::endpoint recv_remote_endpoint_;
    boost::array<unsigned char, 100> recv_buffer_;
    uint32_t from_ardupilot_port_;

    void start_receive();
    void handle_receive(const boost::system::error_code& error,
                        std::size_t num_bytes);

    std::shared_ptr<motion::RigidBody6DOFState> state_6dof_;
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_ARDUPILOT_ARDUPILOT_H_
