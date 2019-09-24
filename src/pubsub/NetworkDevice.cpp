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
#include <scrimmage/entity/EntityPlugin.h>

#include <iostream>

namespace scrimmage {

NetworkDevice::NetworkDevice() : plugin_(std::make_shared<EntityPlugin>()) {}

NetworkDevice::NetworkDevice(const std::string &topic, const unsigned int& max_queue_size,
                             const bool& enable_queue_size, EntityPluginPtr plugin) :
    topic_(topic), max_queue_size_(max_queue_size),
    enable_queue_size_(enable_queue_size), plugin_(plugin) {
}

NetworkDevice::NetworkDevice(NetworkDevice &rhs) :
    topic_(rhs.topic_),
    max_queue_size_(rhs.max_queue_size_),
    plugin_(rhs.plugin_),
    msg_list_(rhs.msg_list_) {}

NetworkDevice::NetworkDevice(NetworkDevice &&rhs) :
    topic_(rhs.topic_), max_queue_size_(rhs.max_queue_size_),
    msg_list_(rhs.msg_list_) {
}

std::string NetworkDevice::get_topic() const {return topic_;}

void NetworkDevice::set_topic(const std::string &topic) {topic_ = topic;}

void NetworkDevice::set_msg_list(const std::list<MessageBasePtr> &msg_list) {
    mutex_.lock();
    msg_list_ = msg_list;
    mutex_.unlock();
}

void NetworkDevice::set_max_queue_size(const unsigned int& size) {
    max_queue_size_ = size;
}

unsigned int NetworkDevice::max_queue_size() {
    return max_queue_size_;
}

void NetworkDevice::enable_queue_size(const bool& enforce) {
    enable_queue_size_ = enforce;
}

bool NetworkDevice::enable_queue_size() {
    return enable_queue_size_;
}

void NetworkDevice::enforce_queue_size() {
    if (enable_queue_size_) {
        if (msg_list_.size() > max_queue_size_) {
            mutex_.lock();
            auto erase_end = msg_list_.begin();
            std::advance(erase_end, msg_list_.size() - max_queue_size_);
            msg_list_.erase(msg_list_.begin(), erase_end);

            // enforce size constraint on undelivered messages
            erase_end = undelivered_msg_list_.begin();
            std::advance(erase_end, undelivered_msg_list_.size() - max_queue_size_);
            undelivered_msg_list_.erase(undelivered_msg_list_.begin(), erase_end);

            mutex_.unlock();
        }
    }
}

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

void NetworkDevice::print_str(const std::string &msg) {
    std::cout << msg << std::endl;
}

EntityPluginPtr & NetworkDevice::plugin() {
    return plugin_;
}


/* added for delay handling */
void NetworkDevice::add_undelivered_msg(MessageBasePtr msg, const bool& is_stochastic_delay) {
    mutex_.lock();
    if (!is_stochastic_delay) {
        // in case of a single, deterministic delay, just add msg to end of queue
        undelivered_msg_list_.push_back(msg);
    } else {
        // if delay is stochastic, sort messages by delivery time, with first-to-deliver in front
        std::list<MessageBasePtr>::reverse_iterator it = undelivered_msg_list_.rbegin();
        for (; it != undelivered_msg_list_.rend();) {
            if ((*it)->time <= msg->time) {
                break;
            } else {
                ++it;
            }
        }
        undelivered_msg_list_.insert(it.base(), msg);
    }
    mutex_.unlock();
}


auto NetworkDevice::deliver_undelivered_msg(std::list<MessageBasePtr>::iterator it) {
    mutex_.lock();
    msg_list_.push_back(*it);
    it = undelivered_msg_list_.erase(it);
    mutex_.unlock();
    return it;
}


int NetworkDevice::deliver_undelivered_msg(const double& time_now, const bool& is_stochastic_delay) {
    // number of messages delivered
    int n_delivered = 0;

    for (auto it = undelivered_msg_list_.begin(); it != undelivered_msg_list_.end(); /**/) {
        if ( time_now >= (*it)->time ) {

            mutex_.lock();
            msg_list_.push_back(*it);
            it = undelivered_msg_list_.erase(it);
            mutex_.unlock();

            ++n_delivered;

        } else {
            break;
        }
    }

    return n_delivered;
}

//
} // namespace scrimmage
