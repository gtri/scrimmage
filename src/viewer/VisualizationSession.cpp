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

#include "scrimmage/viewer/VisualizationSession.h"

#include "scrimmage/viewer/ViewerFactory.h"

namespace scrimmage {

VisualizationSession::VisualizationSession(ViewerBackendType backend)
    : backend_(create_viewer_backend(backend)) {}

VisualizationSession::~VisualizationSession() = default;

VisualizationSession::VisualizationSession(VisualizationSession&&) noexcept = default;

VisualizationSession& VisualizationSession::operator=(VisualizationSession&&) noexcept = default;

bool VisualizationSession::backend_available(ViewerBackendType backend) {
    return viewer_backend_available(backend);
}

const char* VisualizationSession::backend_name(ViewerBackendType backend) {
    return viewer_backend_name(backend);
}

void VisualizationSession::set_incoming_interface(InterfacePtr& incoming_interface) {
    if (backend_) {
        backend_->set_incoming_interface(incoming_interface);
    }
}

void VisualizationSession::set_outgoing_interface(InterfacePtr& outgoing_interface) {
    if (backend_) {
        backend_->set_outgoing_interface(outgoing_interface);
    }
}

void VisualizationSession::set_enable_network(bool enable) {
    if (backend_) {
        backend_->set_enable_network(enable);
    }
}

bool VisualizationSession::init(
    const MissionParsePtr& mp,
    const std::map<std::string, std::string>& camera_params) {
    if (!backend_) {
        return false;
    }

    return backend_->init(mp, camera_params);
}

bool VisualizationSession::run() {
    return backend_ ? backend_->run() : false;
}

}  // namespace scrimmage