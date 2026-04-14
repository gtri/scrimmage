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

#ifndef INCLUDE_SCRIMMAGE_VIEWER_VISUALIZATIONSESSION_H_
#define INCLUDE_SCRIMMAGE_VIEWER_VISUALIZATIONSESSION_H_

#include <map>
#include <memory>
#include <string>

#include "scrimmage/fwd_decl.h"
#include "scrimmage/viewer/ViewerBackend.h"

namespace scrimmage {

class ViewerBackend;

class VisualizationSession {
 public:
    explicit VisualizationSession(ViewerBackendType backend);
    ~VisualizationSession();

    VisualizationSession(const VisualizationSession&) = delete;
    VisualizationSession& operator=(const VisualizationSession&) = delete;
    VisualizationSession(VisualizationSession&&) noexcept;
    VisualizationSession& operator=(VisualizationSession&&) noexcept;

    static bool backend_available(ViewerBackendType backend);
    static const char* backend_name(ViewerBackendType backend);

    void set_incoming_interface(InterfacePtr& incoming_interface);
    void set_outgoing_interface(InterfacePtr& outgoing_interface);
    void set_enable_network(bool enable);

    bool init(
        const MissionParsePtr& mp,
        const std::map<std::string, std::string>& camera_params);
    bool run();

 protected:
    std::unique_ptr<ViewerBackend> backend_;
};

}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_VIEWER_VISUALIZATIONSESSION_H_