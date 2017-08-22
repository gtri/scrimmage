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

#include <scrimmage/fwd_decl.h>
#include <scrimmage/common/ID.h>

#include <unordered_set>
#include <memory>
#include <map>
#include <string>

namespace scrimmage {

class Plugin : public std::enable_shared_from_this<Plugin> {
 public:
    Plugin();
    virtual ~Plugin();

    virtual std::string name();
    virtual std::string type();
    virtual bool ready() { return true; }

    virtual void set_parent(std::weak_ptr<Entity> parent);

    /*! \name getters/setters */
    ///@{
    std::weak_ptr<Entity> &parent();

    NetworkPtr &network();
    void set_network(NetworkPtr network);

    std::map<std::string, PublisherPtr> &pubs();
    void set_pubs(std::map<std::string, PublisherPtr> pubs);

    std::map<std::string, SubscriberPtr> &subs();
    void set_subs(std::map<std::string, SubscriberPtr> subs);

    ContactMapPtr &get_contacts();
    ContactMap &get_contacts_raw();
    virtual void set_contacts(ContactMapPtr &contacts);
    virtual void set_contacts_from_plugin(const PluginPtr &ptr);
    StatePtr &state() {return state_;}
    ///@}

    /*! \name utilities */
    ///@{
    PublisherPtr create_publisher(std::string topic);
    void stop_publishing(PublisherPtr &pub);
    SubscriberPtr create_subscriber(std::string topic);
    void stop_subscribing(SubscriberPtr &sub);
    void publish_immediate(double t, PublisherPtr pub, MessageBasePtr msg);

    void clear_subscribers();
    int get_network_id() {return network_id_;}

    std::unordered_set<int> ping();
    bool ping(int network_id);

    template <class T>
    boost::optional<T>
    call_service(MessageBasePtr req, const std::string &service_name);
    ///@}

 protected:
    ID id_;
    RTreePtr rtree_;
    RandomPtr random_;
    std::shared_ptr<GeographicLib::LocalCartesian> proj_;
    ContactMapPtr contacts_;
    StatePtr state_;

    int network_id_;
    static int plugin_count_;
    NetworkPtr network_;

    std::weak_ptr<Entity> parent_;

    std::map<std::string, PublisherPtr> pubs_;
    std::map<std::string, SubscriberPtr> subs_;
};

using PluginPtr = std::shared_ptr<Plugin>;
} // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_PLUGIN_MANAGER_PLUGIN_H_
