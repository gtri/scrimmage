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
#include <scrimmage/common/FileSearch.h>
#include <scrimmage/common/ID.h>
#include <scrimmage/entity/Contact.h>
#include <scrimmage/proto/Visual.pb.h>

#include <map>
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
              std::shared_ptr<GeographicLib::LocalCartesian> proj,
              int id, int sub_swarm_id,
              PluginManagerPtr plugin_manager,
              NetworkPtr network,
              FileSearch &file_search,
              RTreePtr &rtree);

    bool parse_visual(std::map<std::string, std::string> &info,
                      MissionParsePtr mp, FileSearch &file_search);

    void collision();
    void hit();
    bool is_alive();

    bool posthumous(double t);
    void setup_desired_state();
    bool ready();

    bool call_service(MessageBasePtr req, MessageBasePtr &res, std::string service_name);
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
    MissionParsePtr mp();

    void set_random(RandomPtr random);
    RandomPtr random();

    Contact::Type type();

    void set_visual_changed(bool visual_changed);
    bool visual_changed();

    scrimmage_proto::ContactVisualPtr &contact_visual();

    std::unordered_map<std::string, std::list<SensablePtr>> &sensables();
    std::unordered_map<std::string, SensorPtr> &sensors();
    std::unordered_map<std::string, Service> &services();

    void set_active(bool active);
    bool active();

    ContactMapPtr &contacts() { return contacts_; }
    RTreePtr &rtree() { return rtree_; }

    double radius() { return radius_; }
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
    std::unordered_map<std::string, std::list<SensablePtr>> sensables_;
    std::unordered_map<std::string, SensorPtr> sensors_;

    bool active_ = true;
    bool visual_changed_ = false;
    std::unordered_map<std::string, Service> services_;

    ContactMapPtr contacts_;
    RTreePtr rtree_;

    double radius_ = 1;
};

using EntityPtr = std::shared_ptr<Entity>;
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_ENTITY_ENTITY_H_
