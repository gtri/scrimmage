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

#include <scrimmage/plugin_manager/Plugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/pubsub/Network.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>

#include <string>
#include <memory>

namespace scrimmage {

int Plugin::plugin_count_ = 0;

Plugin::Plugin() :
    state_(new State()),
    network_id_(plugin_count_++) {}

Plugin::~Plugin() {}

std::string Plugin::name() { return std::string("Plugin"); }

std::string Plugin::type() { return std::string("Plugin"); }

void Plugin::set_parent(std::weak_ptr<Entity> parent) {parent_ = parent;}

std::weak_ptr<Entity> &Plugin::parent() {return parent_;}

NetworkPtr &Plugin::network() {return network_;}

void Plugin::set_network(NetworkPtr network) {network_ = network;}

std::map<std::string, PublisherPtr> &Plugin::pubs() {return pubs_;}

void Plugin::set_pubs(std::map<std::string, PublisherPtr> pubs) {pubs_ = pubs;}

std::map<std::string, SubscriberPtr> &Plugin::subs() {return subs_;}

void Plugin::set_subs(std::map<std::string, SubscriberPtr> subs) {subs_ = subs;}

PublisherPtr Plugin::create_publisher(std::string topic) {
    PublisherPtr pub = std::make_shared<Publisher>();
    pub->set_topic(topic);
    pub->plugin() = shared_from_this();
    network_->add_publisher(network_id_, pub, topic);
    pubs_[topic] = pub;
    return pub;
}

void Plugin::stop_publishing(PublisherPtr &pub) {
    std::string topic = pub->get_topic();
    network_->rm_publisher(network_id_, pub, topic);

    auto it = pubs_.find(topic);
    if (it != pubs_.end()) {
        pubs_.erase(it);
    }

    pub->msg_list().clear();
    pub->set_topic("");
}

SubscriberPtr Plugin::create_subscriber(std::string topic) {
    SubscriberPtr sub = std::make_shared<Subscriber>();
    sub->set_topic(topic);
    network_->add_subscriber(network_id_, sub, topic);
    sub->plugin() = shared_from_this();
    subs_[topic] = sub;
    return sub;
}

void Plugin::stop_subscribing(SubscriberPtr &sub) {
    std::string topic = sub->get_topic();
    network_->rm_subscriber(network_id_, sub, topic);

    auto it = subs_.find(topic);
    if (it != subs_.end()) {
        subs_.erase(it);
    }

    sub->msg_list().clear();
    sub->set_topic("");
}

void Plugin::publish_immediate(double t, PublisherPtr pub, MessageBasePtr msg) {
    msg->sender = network_id_;
    msg->time = t;
    for (Network::DeviceMap::value_type &kv : network_->sub_map()[pub->get_topic()]) {
        kv.second->msg_list().push_back(msg);
    }
}

void Plugin::clear_subscribers() {
    for (auto &kv : subs_) {
        kv.second->msg_list().clear();
    }
}

std::unordered_set<int> Plugin::ping() {
    return network_->ping(shared_from_this());
}

bool Plugin::ping(int network_id) {
    return network_->ping(shared_from_this(), network_id);
}

void Plugin::set_contacts(ContactMapPtr &contacts) {
    contacts_ = contacts;
}

void Plugin::set_contacts_from_plugin(const PluginPtr &ptr) {
    contacts_ = ptr->contacts_;
}

ContactMapPtr &Plugin::get_contacts() {return contacts_;}

ContactMap &Plugin::get_contacts_raw() {return *contacts_;}

call_service_helper(MessageBasePtr req, const std::string &service_name) {
    EntityPtr parent_shared = parent_.lock();
    if (parent_shared == nullptr) {
        return boost::none;
    }

    auto it = services_.find(service_name);
    if (it == services_.end()) {
        std::cout << "request for service ("
            << service_name
            << ") that does not exist" << std::endl;
        std::cout << "services are: ";
        for (auto &kv : parent_shared->services()) {
            std::cout << kv.first << ", ";
        }
        std::cout << std::endl;
        return boost::none;
    }

    Service &service = it->second;
    bool success = service(req, res);

    if (!success) {
        std::cout << "call to " << service_name << " failed" << std::endl;
        return false;
    } else {
        return true;
    }
}

}
} // namespace scrimmage
