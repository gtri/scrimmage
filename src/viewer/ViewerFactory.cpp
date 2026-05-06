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

#include "scrimmage/viewer/ViewerFactory.h"

#if ENABLE_VTK == 1
#include "scrimmage/viewer/vtk/VtkViewer.h"
#endif
#if ENABLE_OGRE == 1
#include "scrimmage/viewer/ogre/OgreViewer.h"
#endif

namespace scrimmage {

namespace {

ViewerBackend resolve_backend(ViewerBackend backend) {
    return backend == ViewerBackend::Default ? default_viewer_backend() : backend;
}

}  // namespace

ViewerBackend default_viewer_backend() {
#if ENABLE_VTK == 1
    return ViewerBackend::Vtk;
#elif ENABLE_OGRE == 1
    return ViewerBackend::Ogre;
#else
    return ViewerBackend::None;
#endif
}

bool is_viewer_backend_available(ViewerBackend backend) {
    switch (resolve_backend(backend)) {
        case ViewerBackend::Vtk:
#if ENABLE_VTK == 1
            return true;
#else
            return false;
#endif
        case ViewerBackend::Ogre:
#if ENABLE_OGRE == 1
            return true;
#else
            return false;
#endif
        case ViewerBackend::Default:
        case ViewerBackend::None:
        default:
            return false;
    }
}

std::shared_ptr<Viewer> create_viewer(ViewerBackend backend) {
    switch (resolve_backend(backend)) {
        case ViewerBackend::Vtk:
#if ENABLE_VTK == 1
            return std::make_shared<viewer::VtkViewer>();
#else
            return nullptr;
#endif
        case ViewerBackend::Ogre:
#if ENABLE_OGRE == 1
            return std::make_shared<viewer::OgreViewer>();
#else
            return nullptr;
#endif
        case ViewerBackend::Default:
        case ViewerBackend::None:
        default:
            return nullptr;
    }
}

}  // namespace scrimmage