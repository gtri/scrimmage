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
 */

#include "OgreNextViewerBackend.h"

namespace scrimmage {

void OgreNextViewerBackend::set_incoming_interface(InterfacePtr& incoming_interface) {
    incoming_interface_ = incoming_interface;
    viewer_.set_incoming_interface(incoming_interface_);
}

void OgreNextViewerBackend::set_outgoing_interface(InterfacePtr& outgoing_interface) {
    outgoing_interface_ = outgoing_interface;
    viewer_.set_outgoing_interface(outgoing_interface_);
}

void OgreNextViewerBackend::set_enable_network(bool enable) {
    enable_network_ = enable;
    viewer_.set_enable_network(enable_network_);
}

bool OgreNextViewerBackend::init(
    const MissionParsePtr& mp,
    const std::map<std::string, std::string>& camera_params) {
    viewer_.set_enable_network(enable_network_);
    return viewer_.init(mp, camera_params);
}

bool OgreNextViewerBackend::run() {
    return viewer_.run();
}

}  // namespace scrimmage