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

#include <iostream>

#if __has_include(<OgrePrerequisites.h>)
#include <OgrePrerequisites.h>
#elif __has_include(<OGRE-Next/OgrePrerequisites.h>)
#include <OGRE-Next/OgrePrerequisites.h>
#endif

#ifndef OGRE_VERSION_MAJOR
#define OGRE_VERSION_MAJOR 0
#endif

#ifndef OGRE_VERSION_MINOR
#define OGRE_VERSION_MINOR 0
#endif

#ifndef OGRE_VERSION_PATCH
#define OGRE_VERSION_PATCH 0
#endif

namespace scrimmage {

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
    const MissionParsePtr& /*mp*/,
    const std::map<std::string, std::string>& /*camera_params*/) {
    std::cerr << "Ogre-Next backend selected ("
              << OGRE_VERSION_MAJOR << "."
              << OGRE_VERSION_MINOR << "."
              << OGRE_VERSION_PATCH
              << "), but only the build/runtime scaffolding is implemented in this phase."
              << std::endl;
    std::cerr << "Use the default VTK backend for GUI execution until the Ogre bootstrap lands."
              << std::endl;
    return false;
}

bool OgreNextViewerBackend::run() {
    return false;
}

}  // namespace scrimmage