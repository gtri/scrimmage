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

#include "scrimmage/network/Interface.h"
#include "scrimmage/parse/ParseUtils.h"
#include "OgreNextBootstrap.h"

namespace scrimmage {

OgreNextViewerBackend::OgreNextViewerBackend() = default;

OgreNextViewerBackend::~OgreNextViewerBackend() = default;

void OgreNextViewerBackend::set_incoming_interface(InterfacePtr& incoming_interface) {
    incoming_interface_ = incoming_interface;
}

void OgreNextViewerBackend::set_outgoing_interface(InterfacePtr& outgoing_interface) {
    outgoing_interface_ = outgoing_interface;
}

void OgreNextViewerBackend::set_enable_network(bool enable) {
    enable_network_ = enable;
}

bool OgreNextViewerBackend::init(
    const MissionParsePtr& mp,
    const std::map<std::string, std::string>& camera_params) {
    camera_params_ = camera_params;
    local_ip_ = get<std::string>("local_ip", camera_params_, local_ip_);
    local_port_ = get<int>("local_port", camera_params_, local_port_);
    remote_ip_ = get<std::string>("remote_ip", camera_params_, remote_ip_);
    remote_port_ = get<int>("remote_port", camera_params_, remote_port_);

    bootstrap_.reset(new OgreNextBootstrap());
    return bootstrap_->init(mp, camera_params_);
}

bool OgreNextViewerBackend::run() {
    if (!bootstrap_) {
        return false;
    }

    if (enable_network_) {
        outgoing_interface_->init_network(Interface::client, remote_ip_, remote_port_);
        network_thread_ = std::thread(
            &Interface::init_network,
            &(*incoming_interface_),
            Interface::server,
            local_ip_,
            local_port_);
        network_thread_.detach();
    } else {
        if (incoming_interface_) {
            incoming_interface_->set_mode(Interface::shared);
        }
        if (outgoing_interface_) {
            outgoing_interface_->set_mode(Interface::shared);
        }
    }

    return bootstrap_->run();
}

void OgreNextViewerBackend::stop() {
    if (bootstrap_) {
        bootstrap_->stop();
    }
}

}  // namespace scrimmage