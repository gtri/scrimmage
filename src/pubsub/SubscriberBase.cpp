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
#include <scrimmage/pubsub/SubscriberBase.h>

#include <iostream>
#include <iomanip>

namespace scrimmage {

void SubscriberBase::print_err(const std::string &type, MessageBasePtr msg) const {
    std::cout << "WARNING: could not cast on topic " << std::quoted(topic_) << std::endl;
    std::cout << "  subscriber: type (" << type
        << "), plugin (" << plugin_->name()
        << "), id (" << plugin_->parent()->id().id() << ")"
        << std::endl;
    if (msg->debug_info != "") {
        std::cout << msg->debug_info << std::endl;
    }
}
} // namespace scrimmage
