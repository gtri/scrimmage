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
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/PubSub.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/common/ParameterServer.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>

#include <string>
#include <memory>
#include <functional>

namespace scrimmage {

Plugin::Plugin() : name_("Plugin"), parent_(std::make_shared<Entity>()),
                   transform_(std::make_shared<State>()),
                   id_to_team_map_(std::make_shared<std::unordered_map<int, int>>()),
                   id_to_ent_map_(std::make_shared<std::unordered_map<int, EntityPtr>>()),
                   time_(std::make_shared<const Time>()),
                   param_server_(std::make_shared<ParameterServer>()) {
}

Plugin::~Plugin() {}

std::string Plugin::name() { return name_; }

std::string Plugin::type() { return std::string("Plugin"); }

void Plugin::set_parent(EntityPtr parent) {parent_ = parent;}

EntityPtr Plugin::parent() { return parent_; }

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

PublisherPtr Plugin::advertise(std::string network_name, std::string topic,
                               unsigned int max_queue_size) {
    return pubsub_->advertise(network_name, topic, max_queue_size, true,
                              shared_from_this());
}

PublisherPtr Plugin::advertise(std::string network_name, std::string topic) {
    return pubsub_->advertise(network_name, topic, 0, false,
                              shared_from_this());
}

void Plugin::draw_shape(scrimmage_proto::ShapePtr s) {
    if (!s->hash_set()) {
        // Hash function uses entity ID, current simulation time, plugin name,
        // and a random number.
        std::string str = this->name() + std::to_string(parent_->id().id())
            + std::to_string(time_->t())
            + std::to_string(parent_->random()->rng_uniform());

        std::size_t hash_id = std::hash<std::string>{}(str);
        s->set_hash(hash_id);
        s->set_hash_set(true);
    }
    shapes_.push_back(s);
}

void Plugin::set_param_server(const ParameterServerPtr &param_server) {
    param_server_ = param_server;
}

void Plugin::close_plugin(const double &t) {
    close(t); // allow subclass to close()

    param_server_->unregister_params(shared_from_this());
    parent_ = nullptr;
    transform_ = nullptr;
    id_to_ent_map_ = nullptr;
    pubsub_ = nullptr;
    subs_.clear();
    time_ = nullptr;
    shapes_.clear();
}
} // namespace scrimmage
