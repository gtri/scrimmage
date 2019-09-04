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

#ifndef INCLUDE_SCRIMMAGE_PUBSUB_SUBSCRIBERBASE_H_
#define INCLUDE_SCRIMMAGE_PUBSUB_SUBSCRIBERBASE_H_

#include <scrimmage/pubsub/NetworkDevice.h>

#include <string>
#include <memory>

namespace scrimmage {
class SubscriberBase : public NetworkDevice {
 public:
    SubscriberBase() = default;
    SubscriberBase(const std::string &topic, unsigned int &max_queue_size,
                   bool enable_queue_size, EntityPluginPtr plugin) :
        NetworkDevice(topic, max_queue_size, enable_queue_size, plugin) {}
    virtual void accept(scrimmage::MessageBasePtr msg) = 0;

 protected:
    void print_err(const std::string &type, MessageBasePtr msg) const;
};

using SubscriberBasePtr = std::shared_ptr<SubscriberBase>;
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PUBSUB_SUBSCRIBERBASE_H_
