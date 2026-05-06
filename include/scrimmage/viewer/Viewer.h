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

#ifndef INCLUDE_SCRIMMAGE_VIEWER_VIEWER_H_
#define INCLUDE_SCRIMMAGE_VIEWER_VIEWER_H_

#include <map>
#include <memory>
#include <string>

namespace scrimmage {

class MissionParse;
class Interface;
using InterfacePtr = std::shared_ptr<Interface>;

class Viewer {
 public:
    virtual ~Viewer() = default;

    virtual void set_incoming_interface(InterfacePtr& incoming_interface) = 0;

    virtual void set_outgoing_interface(InterfacePtr& outgoing_interface) = 0;

    virtual void set_enable_network(bool enable) = 0;

    virtual bool init(
        const std::shared_ptr<MissionParse>& mp,
        const std::map<std::string, std::string>& camera_params) = 0;

    virtual bool run() = 0;
};

}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_VIEWER_VIEWER_H_