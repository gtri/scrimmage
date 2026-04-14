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

#if SCRIMMAGE_HAVE_VTK_VIEWER
#include "vtk/VtkViewerBackend.h"
#endif

#if SCRIMMAGE_HAVE_OGRE_NEXT_VIEWER
#include "ogre_next/OgreNextViewerBackend.h"
#endif

namespace scrimmage {

std::unique_ptr<ViewerBackend> create_viewer_backend(ViewerBackendType backend) {
    switch (backend) {
        case ViewerBackendType::VTK:
#if SCRIMMAGE_HAVE_VTK_VIEWER
            return std::unique_ptr<ViewerBackend>(new VtkViewerBackend());
#else
            return nullptr;
#endif
        case ViewerBackendType::OGRE_NEXT:
#if SCRIMMAGE_HAVE_OGRE_NEXT_VIEWER
            return std::unique_ptr<ViewerBackend>(new OgreNextViewerBackend());
#else
            return nullptr;
#endif
    }

    return nullptr;
}

bool viewer_backend_available(ViewerBackendType backend) {
    return create_viewer_backend(backend) != nullptr;
}

const char* viewer_backend_name(ViewerBackendType backend) {
    switch (backend) {
        case ViewerBackendType::VTK:
            return "vtk";
        case ViewerBackendType::OGRE_NEXT:
            return "ogre-next";
    }

    return "unknown";
}

}  // namespace scrimmage