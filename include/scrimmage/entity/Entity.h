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

#ifndef INCLUDE_SCRIMMAGE_ENTITY_ENTITY_H_
#define INCLUDE_SCRIMMAGE_ENTITY_ENTITY_H_

#include <scrimmage/fwd_decl.h>
#include <scrimmage/common/ID.h>
#include <scrimmage/entity/Contact.h>
#include <scrimmage/proto/Visual.pb.h>
#include <scrimmage/pubsub/Message.h>

#include <map>
#include <set>
#include <unordered_map>
#include <list>
#include <vector>
#include <string>
#include <functional>
#include <memory>

namespace scrimmage_proto {
using ContactVisualPtr = std::shared_ptr<ContactVisual>;
}

namespace scrimmage {

using Service = std::function<bool (scrimmage::MessageBasePtr, scrimmage::MessageBasePtr&)>;

typedef std::map<std::string, std::map<std::string, std::string>> AttributeMap;

class Entity : public std::enable_shared_from_this<Entity> {

 public:
    /*! \name utilities */
    ///@{

    bool init(AttributeMap &overrides,
              std::map<std::string, std::string> &info,
              ContactMapPtr &contacts,
              MissionParsePtr mp,
              const std::shared_ptr<GeographicLib::LocalCartesian> &proj,
              int id, int ent_desc_id,
              PluginManagerPtr plugin_manager,
              FileSearchPtr &file_search,
              RTreePtr &rtree,
              PubSubPtr &pubsub,
              TimePtr &time,
              const ParameterServerPtr &param_server,
              const std::set<std::string> &plugin_tags,
              std::function<void(std::map<std::string, std::string>&)> param_override_func
        );

    void print_plugins(std::ostream &out) const;

    bool parse_visual(std::map<std::string, std::string> &info,
                      MissionParsePtr mp,
                      std::map<std::string, std::string> &overrides);

    void close(double t);
    void collision();
    void hit();
    bool is_alive();

    bool posthumous(double t);
    void setup_desired_state();
    bool ready();

    bool call_service(MessageBasePtr req, MessageBasePtr &res, const std::string &service_name);

    bool call_service(MessageBasePtr &res, const std::string &service_name) {
        return call_service(std::make_shared<MessageBase>(), res, service_name);
    }

    template <class T = MessageBasePtr,
              class = typename std::enable_if<!std::is_same<T, MessageBasePtr>::value, void>::type>
    bool call_service(MessageBasePtr req, T &res, const std::string &service_name) {
        MessageBasePtr res_base;
        if (call_service(req, res_base, service_name)) {
            res = std::dynamic_pointer_cast<typename T::element_type>(res_base);
            if (res == nullptr) {
                print(std::string("could not cast for service ") + service_name);
                return false;
            } else {
                return true;
            }
        } else {
            return false;
        }
    }

    template <class T = MessageBasePtr,
              class = typename std::enable_if<!std::is_same<T, MessageBasePtr>::value, void>::type>
    bool call_service(T &res, const std::string &service_name) {
        return call_service(std::make_shared<MessageBase>(), res, service_name);
    }
    ///@}

    /*! \name getters/setters */
    ///@{
    StatePtr &state();
    std::vector<AutonomyPtr> &autonomies();
    MotionModelPtr &motion();
    std::vector<ControllerPtr> &controllers();

    void set_id(ID &id);
    ID &id();

    void set_health_points(int health_points);
    int health_points();

    std::shared_ptr<GeographicLib::LocalCartesian> projection();
    void set_projection(std::shared_ptr<GeographicLib::LocalCartesian> proj);

    void set_mp(MissionParsePtr mp);
    MissionParsePtr mp();

    void set_random(RandomPtr random);
    RandomPtr random();

    Contact::Type type();

    void set_visual_changed(bool visual_changed);
    bool visual_changed();

    scrimmage_proto::ContactVisualPtr &contact_visual();

    std::unordered_map<std::string, SensorPtr> &sensors();
    std::unordered_map<std::string, SensorPtr> sensors(const std::string &sensor_name);
    SensorPtr sensor(const std::string &sensor_name);

    std::unordered_map<std::string, Service> &services();

    std::unordered_map<std::string, MessageBasePtr> &properties();

    void set_active(bool active);
    bool active();

    ContactMapPtr &contacts() { return contacts_; }
    RTreePtr &rtree() { return rtree_; }

    PluginManagerPtr & plugin_manager() {
        return plugin_manager_;
    }

    FileSearchPtr & file_search() {
        return file_search_;
    }

    PubSubPtr & pubsub() {
        return pubsub_;
    }

    double radius() { return radius_; }
    void set_time_ptr(TimePtr t);
    ///@}

 protected:
    ID id_;

    scrimmage_proto::ContactVisualPtr visual_ =
        std::make_shared<scrimmage_proto::ContactVisual>();

    std::vector<ControllerPtr> controllers_;
    MotionModelPtr motion_model_;
    std::vector<AutonomyPtr> autonomies_;
    MissionParsePtr mp_;

    int health_points_ = 1;

    int max_hits_ = -1;

    Contact::Type type_ = Contact::Type::AIRCRAFT;

    std::shared_ptr<GeographicLib::LocalCartesian> proj_;

    RandomPtr random_;

    StatePtr state_;
    std::unordered_map<std::string, MessageBasePtr> properties_;
    std::unordered_map<std::string, SensorPtr> sensors_;

    bool active_ = true;
    bool visual_changed_ = false;
    std::unordered_map<std::string, Service> services_;

    ContactMapPtr contacts_;
    RTreePtr rtree_;

    double radius_ = 1;

    void print(const std::string &msg);
    PluginManagerPtr plugin_manager_;
    FileSearchPtr file_search_;
    PubSubPtr pubsub_;
    TimePtr time_;
};

using EntityPtr = std::shared_ptr<Entity>;
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_ENTITY_ENTITY_H_
