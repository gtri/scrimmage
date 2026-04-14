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

#ifndef SRC_VIEWER_OGRE_NEXT_OGRENEXTVIEWERBACKEND_H_
#define SRC_VIEWER_OGRE_NEXT_OGRENEXTVIEWERBACKEND_H_

#include <memory>
#include <thread>

#include "scrimmage/viewer/ViewerBackend.h"

namespace scrimmage {

class OgreNextBootstrap;

class OgreNextViewerBackend : public ViewerBackend {
 public:
    OgreNextViewerBackend();
    ~OgreNextViewerBackend() override;

    void set_incoming_interface(InterfacePtr& incoming_interface) override;
    void set_outgoing_interface(InterfacePtr& outgoing_interface) override;
    void set_enable_network(bool enable) override;

    bool init(
        const MissionParsePtr& mp,
        const std::map<std::string, std::string>& camera_params) override;
    bool run() override;

 protected:
    std::unique_ptr<OgreNextBootstrap> bootstrap_;
    InterfacePtr incoming_interface_;
    InterfacePtr outgoing_interface_;
    bool enable_network_ = false;
    std::thread network_thread_;

    std::map<std::string, std::string> camera_params_;
    std::string local_ip_ = "localhost";
    int local_port_ = 50051;
    std::string remote_ip_ = "localhost";
    int remote_port_ = 50052;
};

}  // namespace scrimmage

#endif  // SRC_VIEWER_OGRE_NEXT_OGRENEXTVIEWERBACKEND_H_