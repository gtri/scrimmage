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

#ifndef Network_Device_H_
#define Network_Device_H_

#include <scrimmage/fwd_decl.h>
#include <scrimmage/plugin_manager/Plugin.h>
#include <scrimmage/pubsub/MessageBase.h>

#include <list>
#include <memory>
#include <string>

namespace scrimmage {

class NetworkDevice {
 public:
    NetworkDevice();

    std::string get_topic() const;
    void set_topic(std::string topic);

    std::list<MessageBasePtr> &msg_list();
    void set_msg_list(std::list<MessageBasePtr> msg_list);

    void set_max_queue_size(unsigned int size);
    unsigned int max_queue_size();

    PluginPtr &plugin();

    template <class T>
    std::list<std::shared_ptr<T>> msgs(bool pop_msgs = true, bool exclude_self = true) {
        std::list<std::shared_ptr<T>> msg_list_cast;

        int id = plugin_->get_network_id();
        auto it = msg_list_.begin();
        while (it != msg_list_.end()) {

            if (!exclude_self || (*it)->sender != id) {
                auto msg_cast = std::dynamic_pointer_cast<T>(*it);
                if (msg_cast) {
                    msg_list_cast.push_back(msg_cast);
                }
            }

            it = pop_msgs ? msg_list_.erase(it) : std::next(it);
        }

        return msg_list_cast;
    }

 protected:
    std::string topic_;
    std::list<MessageBasePtr> msg_list_;
    PluginPtr plugin_;
    unsigned int max_queue_size_;
};

} // namespace scrimmage
#endif // Network_Device_H_
