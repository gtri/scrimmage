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

#include <scrimmage/pubsub/NetworkDevice.h>
#include <scrimmage/pubsub/MessageBase.h>
#include <scrimmage/plugin_manager/Plugin.h>

namespace scrimmage {

NetworkDevice::NetworkDevice(NetworkDevice &rhs) :
        topic_(rhs.topic_), msg_list_(rhs.msg_list_),
        plugin_(rhs.plugin()), max_queue_size_(rhs.max_queue_size_) {
}

NetworkDevice::NetworkDevice(NetworkDevice &&rhs) :
        topic_(rhs.topic_), msg_list_(rhs.msg_list_),
        plugin_(rhs.plugin()), max_queue_size_(rhs.max_queue_size_) {
}

std::string NetworkDevice::get_topic() const {return topic_;}
void NetworkDevice::set_topic(std::string topic) {topic_ = topic;}

void NetworkDevice::set_msg_list(std::list<MessageBasePtr> msg_list) {
    mutex_.lock();
    msg_list_ = msg_list;
    mutex_.unlock();
}

void NetworkDevice::set_max_queue_size(unsigned int size) {max_queue_size_ = size;}
unsigned int NetworkDevice::max_queue_size() {return max_queue_size_;}

PluginPtr &NetworkDevice::plugin() {return plugin_;}

void NetworkDevice::add_msg(MessageBasePtr msg) {
    mutex_.lock();
    msg_list_.push_back(msg);
    mutex_.unlock();
}

void NetworkDevice::clear_msg_list() {
    mutex_.lock();
    msg_list_.clear();
    mutex_.unlock();
}

} // namespace scrimmage
