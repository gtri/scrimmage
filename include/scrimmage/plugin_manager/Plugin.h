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

#ifndef INCLUDE_SCRIMMAGE_PLUGIN_MANAGER_PLUGIN_H_
#define INCLUDE_SCRIMMAGE_PLUGIN_MANAGER_PLUGIN_H_

#include <Eigen/Dense>

#include <scrimmage/common/VariableIO.h>
#include <scrimmage/common/ParameterServer.h>
#include <scrimmage/pubsub/PubSub.h>
#include <scrimmage/pubsub/Subscriber.h>

#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <map>
#include <list>
#include <string>

namespace scrimmage_proto {
class Shape;
using ShapePtr = std::shared_ptr<Shape>;
}

namespace scrimmage {

class Entity;
using EntityPtr = std::shared_ptr<Entity>;

class Publisher;
using PublisherPtr = std::shared_ptr<Publisher>;

class State;
using StatePtr = std::shared_ptr<State>;

class Time;
using TimePtr = std::shared_ptr<Time>;

class Plugin : public std::enable_shared_from_this<Plugin> {
 public:
    Plugin();
    virtual ~Plugin();

    virtual void set_name(std::string name) { name_ = name; }
    virtual std::string name();
    virtual std::string type();
    virtual bool ready() { return true; }

    void close_plugin(const double &t);
    virtual void close(double /*t*/) {}

    virtual void set_parent(EntityPtr parent);
    virtual EntityPtr parent();

    void set_scoped_property(const std::string &property_name, const MessageBasePtr &property);
    MessageBasePtr get_scoped_property_helper(const std::string &property_name);

    /* Homogeneous transform from parent link */
    StatePtr transform() { return transform_; }

    virtual void set_id_to_team_map(std::shared_ptr<std::unordered_map<int, int>> lookup)
    { id_to_team_map_ = lookup; }

    virtual void set_id_to_ent_map(std::shared_ptr<std::unordered_map<int, EntityPtr>> lookup)
    { id_to_ent_map_ = lookup; }

    std::list<scrimmage_proto::ShapePtr> &shapes();

    VariableIO &vars() { return vars_; }

    template <class T, class CallbackFunc>
    SubscriberBasePtr subscribe(const std::string &network_name,
                                const std::string &topic,
                                CallbackFunc callback) {
        SubscriberBasePtr sub =
            pubsub_->subscribe<T>(network_name, topic, callback,
                                  0, false, shared_from_this());
        subs_.push_back(sub);
        return sub;
    }

    template <class T, class CallbackFunc>
    SubscriberBasePtr subscribe(const std::string &network_name,
                                const std::string &topic,
                                CallbackFunc callback,
                                unsigned int max_queue_size) {
        SubscriberBasePtr sub  =
            pubsub_->subscribe<T>(network_name, topic, callback,
                                  max_queue_size, true, shared_from_this());
        subs_.push_back(sub);
        return sub;
    }

    PublisherPtr advertise(std::string network_name, std::string topic);
    PublisherPtr advertise(std::string network_name, std::string topic,
                           unsigned int max_queue_size);

    void set_pubsub(PubSubPtr pubsub) { pubsub_ = pubsub; }

    std::list<SubscriberBasePtr> subs() { return subs_; }

    void set_time(const std::shared_ptr<Time> &time) { time_ = time; }
    // cppcheck-suppress passedByValue
    void set_time(std::shared_ptr<const Time> time) { time_ = time; }

    void draw_shape(scrimmage_proto::ShapePtr s);
    bool print_err_on_exit = true;

    void set_param_server(const ParameterServerPtr &param_server);

    template <class T>
    bool register_param(const std::string &name, T &variable,
                        std::function<void(const T &value)> callback =
                        [](const T &value){}) {
        return param_server_->register_param<T>(name, variable,
                                                callback,
                                                shared_from_this());
    }

    template <class T>
    bool set_param(const std::string &name, const T &value) {
        return param_server_->set_param<T>(name, value);
    }

    template <class T>
    bool unregister_param(const std::string &name) {
        return param_server_->unregister_param<T>(name, shared_from_this());
    }

 protected:
    std::string name_;
    EntityPtr parent_;

    StatePtr transform_;

    std::shared_ptr<std::unordered_map<int, int>> id_to_team_map_;
    std::shared_ptr<std::unordered_map<int, EntityPtr>> id_to_ent_map_;

    VariableIO vars_;
    PubSubPtr pubsub_;

    std::list<SubscriberBasePtr> subs_;
    std::shared_ptr<const Time> time_;

 private:
    std::list<scrimmage_proto::ShapePtr> shapes_;
    ParameterServerPtr param_server_;

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using PluginPtr = std::shared_ptr<Plugin>;
} // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_PLUGIN_MANAGER_PLUGIN_H_
