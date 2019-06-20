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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_FLIGHTGEARMULTIPLAYER_FLIGHTGEARMULTIPLAYER_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_FLIGHTGEARMULTIPLAYER_FLIGHTGEARMULTIPLAYER_H_

#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/math/Quaternion.h>
#include <plib/netSocket.h>

#include <string>
#include <map>
#include <memory>

#include <GeographicLib/Geocentric.hpp>
#include <flightgear/MultiPlayer/mpmessages.hxx>

namespace scrimmage {
namespace autonomy {
class FlightGearMultiplayer : public scrimmage::Autonomy {
 public:
    FlightGearMultiplayer();
    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;

 protected:
    std::string callsign_;
    std::shared_ptr<netSocket> data_socket_;
    netAddress net_address_;
    std::string aircraft_model_;

    std::shared_ptr<GeographicLib::Geocentric> earth_;

    Angles angles_to_jsbsim_;

    scrimmage::Quaternion fromLonLatRad(float lon, float lat);

    struct T_MsgHdr header_msg_;
    struct T_PositionMsg pos_msg_;
    int msg_size_ = 0;
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_FLIGHTGEARMULTIPLAYER_FLIGHTGEARMULTIPLAYER_H_
