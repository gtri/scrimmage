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

#ifndef INCLUDE_SCRIMMAGE_VIEWER_VIEWERFACTORY_H_
#define INCLUDE_SCRIMMAGE_VIEWER_VIEWERFACTORY_H_

#include <memory>

#include "scrimmage/viewer/ViewerBackend.h"

namespace scrimmage {

std::unique_ptr<ViewerBackend> create_viewer_backend(ViewerBackendType backend);
bool viewer_backend_available(ViewerBackendType backend);
const char* viewer_backend_name(ViewerBackendType backend);

}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_VIEWER_VIEWERFACTORY_H_