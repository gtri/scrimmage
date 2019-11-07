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

#include <scrimmage/entity/Entity.h>
#include <scrimmage/entity/EntityPlugin.h>
#include <scrimmage/pubsub/Publisher.h>

namespace scrimmage {

Publisher::Publisher() {
}

Publisher::Publisher(const std::string &topic, const unsigned int& max_queue_size,
                     const bool& enable_queue_size, EntityPluginPtr plugin) :
    NetworkDevice(topic, max_queue_size, enable_queue_size, plugin) {}

void Publisher::set_debug_info(MessageBasePtr msg, const std::string &type) {
      msg->debug_info = std::string("  publisher:  ")
        + "type (" + type
        + "), plugin (" + plugin_->name() + ")"
        + "), id (" + std::to_string(plugin_->parent()->id().id()) + ")";
}
} // namespace scrimmage
