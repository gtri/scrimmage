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

Plugin::Plugin() : network_id_(plugin_count_++),
                   parent_(std::make_shared<Entity>()),
                   transform_(std::make_shared<State>()),
                   id_to_team_map_(std::make_shared<std::unordered_map<int, int>>()),
                   id_to_ent_map_(std::make_shared<std::unordered_map<int, EntityPtr>>()) {}

Plugin::~Plugin() {}

std::string Plugin::name() { return std::string("Plugin"); }

std::string Plugin::type() { return std::string("Plugin"); }

void Plugin::set_parent(EntityPtr parent) {parent_ = parent;}

EntityPtr Plugin::parent() { return parent_; }

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

    pub->clear_msg_list();
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

    sub->clear_msg_list();
    sub->set_topic("");
}

void Plugin::publish_immediate(double t, PublisherPtr pub, MessageBasePtr msg) {
    msg->sender = network_id_;
    msg->time = t;
    for (Network::DeviceMap::value_type &kv : network_->sub_map()[pub->get_topic()]) {
        kv.second->add_msg(msg);
    }
}

void Plugin::clear_subscribers() {
    for (auto &kv : subs_) {
        kv.second->clear_msg_list();
    }
}

std::unordered_set<int> Plugin::ping() {
    return network_->ping(shared_from_this());
}

bool Plugin::ping(int network_id) {
    return network_->ping(shared_from_this(), network_id);
}

void Plugin::set_scoped_property(const std::string &property_name, const MessageBasePtr &property) {
    parent_->properties()[name() + "/" + property_name] = property;
}

MessageBasePtr Plugin::get_scoped_property_helper(const std::string &property_name) {
    auto it = parent_->properties().find(name() + "/" + property_name);
    return it == parent_->properties().end() ? nullptr : it->second;
}

std::list<scrimmage_proto::ShapePtr> &Plugin::shapes() {
    return shapes_;
}

} // namespace scrimmage
