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

#ifndef INCLUDE_SCRIMMAGE_PUBSUB_NETWORKDEVICE_H_
#define INCLUDE_SCRIMMAGE_PUBSUB_NETWORKDEVICE_H_

#include <scrimmage/pubsub/MessageBase.h>
#include <scrimmage/pubsub/Message.h>

#include <type_traits>
#include <list>
#include <memory>
#include <string>
#include <mutex> // NOLINT

namespace scrimmage {

class Plugin;
using PluginPtr = std::shared_ptr<Plugin>;

class NetworkDevice {
 public:
    NetworkDevice();
    NetworkDevice(NetworkDevice &rhs);
    NetworkDevice(NetworkDevice &&rhs);

    virtual ~NetworkDevice() = default; // Make NetworkDevice Polymorphic

    NetworkDevice(const std::string &topic, unsigned int &max_queue_size,
                  bool enable_queue_size, PluginPtr plugin);

    std::string get_topic() const;
    void set_topic(const std::string &topic);

    void set_msg_list(const std::list<MessageBasePtr> &msg_list);
    void clear_msg_list();

    unsigned int msg_list_size() {
        return msg_list_.size();
    }

    void set_max_queue_size(unsigned int size);
    unsigned int max_queue_size();
    void enable_queue_size(bool enforce);
    bool enable_queue_size();

    void enforce_queue_size();

    void add_msg(MessageBasePtr msg);

    template <class T = MessageBase,
              class = std::enable_if_t<std::is_same<T, MessageBase>::value, void>>
    std::list<MessageBasePtr> pop_msgs() {
        mutex_.lock();
        std::list<MessageBasePtr> msg_list_cast;

        auto it = msg_list_.begin();
        while (it != msg_list_.end()) {
            msg_list_cast.push_back(*it);
            it = msg_list_.erase(it);
        }
        mutex_.unlock();
        return msg_list_cast;
    }

    template <class T,
              class = std::enable_if_t<!std::is_same<T, MessageBase>::value &&
                                       std::is_base_of<MessageBase, T>::value, void>>
    std::list<std::shared_ptr<T>> pop_msgs() {
        mutex_.lock();
        std::list<std::shared_ptr<T>> msg_list_cast;

        auto it = msg_list_.begin();
        while (it != msg_list_.end()) {
            auto msg_cast = std::dynamic_pointer_cast<T>(*it);
            if (msg_cast) {
                msg_list_cast.push_back(msg_cast);
                } else {
                print_str(std::string("WARNING: could not cast message on topic \"")
                          + topic_);
            }
            it = msg_list_.erase(it);
        }
        mutex_.unlock();
        return msg_list_cast;
    }

    template <class T,
              class = std::enable_if_t<!std::is_same<T, MessageBase>::value &&
                                       !std::is_base_of<MessageBase, T>::value, void>>
    std::list<std::shared_ptr<Message<T>>> pop_msgs() {
        return pop_msgs<Message<T>>();
    }

    ////////////////////

    template<class T>
        struct always_false : std::false_type
    { };

    /*! \brief The msgs() method is no longer supported. Please use the new
     *  subscriber callback interface.*/
    template <class T = MessageBase,
              class = std::enable_if_t<std::is_same<T, MessageBase>::value, void>>
    std::list<MessageBasePtr> msgs(bool pop_msgs = true) {
        static_assert(always_false<T>::value, "You are using the old publisher/subscriber interface (i.e., msgs(bool)) in your plugin. Please use the new subscriber callback interface.");
        std::list<std::shared_ptr<T>> msg_list_cast;
        return msg_list_cast;
    }

    /*! \brief The msgs() method is no longer supported. Please use the new
     *  subscriber callback interface.*/
    template <class T,
              class = std::enable_if_t<!std::is_same<T, MessageBase>::value &&
                                       std::is_base_of<MessageBase, T>::value, void>>
    std::list<std::shared_ptr<T>> msgs(bool pop_msgs = true) {
        static_assert(always_false<T>::value, "You are using the old publisher/subscriber interface (i.e., msgs(bool)) in your plugin. Please use the new subscriber callback interface.");
        std::list<std::shared_ptr<T>> msg_list_cast;
        return msg_list_cast;
    }

    /*! \brief The msgs() method is no longer supported. Please use the new
     *  subscriber callback interface.*/
    template <class T,
              class = std::enable_if_t<!std::is_same<T, MessageBase>::value &&
                                       !std::is_base_of<MessageBase, T>::value, void>>
    std::list<std::shared_ptr<Message<T>>> msgs(bool pop_msgs = true) {
        static_assert(always_false<T>::value, "You are using the old publisher/subscriber interface (i.e., msgs(bool)) in your plugin. Please use the new subscriber callback interface.");
        std::list<std::shared_ptr<T>> msg_list_cast;
        return msg_list_cast;
    }

    PluginPtr & plugin();

 protected:
    std::string topic_ = "";
    unsigned int max_queue_size_ = 1;
    bool enable_queue_size_ = false;
    PluginPtr plugin_;
    void print_str(std::string msg);
    std::list<MessageBasePtr> msg_list_;
    std::mutex mutex_;
};
using NetworkDevicePtr = std::shared_ptr<NetworkDevice>;
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PUBSUB_NETWORKDEVICE_H_
