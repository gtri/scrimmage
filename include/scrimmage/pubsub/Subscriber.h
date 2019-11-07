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

#ifndef INCLUDE_SCRIMMAGE_PUBSUB_SUBSCRIBER_H_
#define INCLUDE_SCRIMMAGE_PUBSUB_SUBSCRIBER_H_

#include <scrimmage/pubsub/SubscriberBase.h>
#include <scrimmage/pubsub/Message.h>

#include <string>

#include <boost/type_index.hpp>

namespace scrimmage {
template <class T, class CallbackFunc>
class Subscriber : public SubscriberBase {
 public:
    Subscriber(const std::string &topic, unsigned int &max_queue_size,
               bool enable_queue_size, EntityPluginPtr plugin,
               CallbackFunc callback)
        : SubscriberBase(topic, max_queue_size, enable_queue_size, plugin),
        callback_(callback) {
    }

    void accept(scrimmage::MessageBasePtr msg) override {
        auto msg_cast = std::dynamic_pointer_cast<Message<T>>(msg);
        if (msg_cast != nullptr) {
            callback_(msg_cast);
        } else {
            this->print_err(boost::typeindex::type_id<scrimmage::Message<T>>().pretty_name(), msg);
        }
    }

 protected:
    CallbackFunc callback_;
};
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PUBSUB_SUBSCRIBER_H_
