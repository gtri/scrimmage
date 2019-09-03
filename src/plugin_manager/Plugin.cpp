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
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/entity/EntityPlugin.h>

#include <string>
#include <memory>

namespace scrimmage {
Plugin::Plugin() : name_("Plugin") {}
Plugin::~Plugin() {}
std::string Plugin::name() { return name_; }
std::string Plugin::type() { return std::string("Plugin"); }

void Plugin::set_name(std::string name) {
    if (name_set_ == true) {
        std::cout << "WARNING: Plugin name, " << name << ", being reset. "
                  << "Should only be set once." << std::endl;
    }
    name_ = name;
    name_set_ = true;  // Set only once before warning.
}

} // namespace scrimmage
