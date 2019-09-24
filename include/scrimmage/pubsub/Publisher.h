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

#ifndef INCLUDE_SCRIMMAGE_PUBSUB_PUBLISHER_H_
#define INCLUDE_SCRIMMAGE_PUBSUB_PUBLISHER_H_

#include <scrimmage/pubsub/NetworkDevice.h>

#include <functional>
#include <string>
#include <memory>

#include <boost/type_index.hpp>

namespace scrimmage {

class MessageBase;
using MessageBasePtr = std::shared_ptr<MessageBase>;

class Publisher : public NetworkDevice {
 public:
    Publisher();
    Publisher(const std::string &topic, const unsigned int &max_queue_size,
              const bool& enable_queue_size, EntityPluginPtr plugin);

    template <class T> void publish(const std::shared_ptr<T> &msg, bool add_debug_info = true) {
        if (add_debug_info) {
            set_debug_info(msg, boost::typeindex::type_id<T>().pretty_name());
        }
        add_msg(msg);
    }
    std::function<void(MessageBasePtr)> callback;

 protected:
    void set_debug_info(MessageBasePtr msg, const std::string &type);
};

} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PUBSUB_PUBLISHER_H_
