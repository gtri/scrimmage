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

#include <functional>
#include <iostream>

using std::cout;
using std::endl;

namespace scrimmage {
template <class T>
class Subscriber : public SubscriberBase {
 public:
    explicit Subscriber(std::function<void(scrimmage::MessagePtr<T>)> callback)
        : callback_(callback) {
    }

    virtual void accept(scrimmage::MessageBasePtr msg) {
        auto msg_cast = std::dynamic_pointer_cast<scrimmage::Message<T>>(msg);
        if (msg_cast != nullptr) {
            callback_(msg_cast);
        } else {
            cout << "WARNING: Failed to cast received message" << endl;
        }
    }

 protected:
    std::function <void(scrimmage::MessagePtr<T>)> callback_;
};
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PUBSUB_SUBSCRIBER_H_
