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

#ifndef SRC_VIEWER_OGRE_NEXT_OGRENEXTBOOTSTRAP_H_
#define SRC_VIEWER_OGRE_NEXT_OGRENEXTBOOTSTRAP_H_

#include <map>
#include <memory>
#include <string>

#include "scrimmage/fwd_decl.h"

namespace scrimmage {

class OgreNextBootstrap {
 public:
    OgreNextBootstrap();
    ~OgreNextBootstrap();

    OgreNextBootstrap(const OgreNextBootstrap&) = delete;
    OgreNextBootstrap& operator=(const OgreNextBootstrap&) = delete;

    bool init(
        const MissionParsePtr& mp,
        const std::map<std::string, std::string>& camera_params);
    bool run();

 protected:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace scrimmage

#endif  // SRC_VIEWER_OGRE_NEXT_OGRENEXTBOOTSTRAP_H_