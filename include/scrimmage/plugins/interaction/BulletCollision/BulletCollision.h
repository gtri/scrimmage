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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_BULLETCOLLISION_BULLETCOLLISION_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_BULLETCOLLISION_BULLETCOLLISION_H_

#include <scrimmage/simcontrol/EntityInteraction.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/plugins/sensor/RayTrace/RayTrace.h>

#include <btBulletDynamicsCommon.h>

#include <list>
#include <map>
#include <string>

namespace sc = scrimmage;

namespace scrimmage {
namespace interaction {
class BulletCollision : public scrimmage::EntityInteraction {
 public:
    BulletCollision();
    ~BulletCollision();
    bool init(std::map<std::string, std::string> &mission_params,
                      std::map<std::string, std::string> &plugin_params) override;

    bool step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                         double t, double dt) override;

    bool collision_exists(std::list<sc::EntityPtr> &ents,
                                  Eigen::Vector3d &p) override;

 protected:
    btCollisionConfiguration* bt_collision_configuration;
    btCollisionDispatcher* bt_dispatcher;
    btBroadphaseInterface* bt_broadphase;
    btCollisionWorld* bt_collision_world;


    sc::PublisherPtr team_collision_pub_;
    sc::PublisherPtr non_team_collision_pub_;

    double scene_size_;
    unsigned int max_objects_;

    std::map<int, btCollisionObject*> objects_;

    // Key 1: Entity ID
    // Value 2: map
    // Key 2: Sensor Name (sensor0)
    // Value 2: Point Cloud
    std::map<int, std::map<std::string, sensor::RayTrace::PointCloud>> pcls_;
    std::map<int, std::map<std::string, sc::PublisherPtr>> pcl_pubs_;

    bool show_rays_ = false;
    bool enable_collision_detection_ = true;
    bool enable_ray_tracing_ = true;
};
} // namespace interaction
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_BULLETCOLLISION_BULLETCOLLISION_H_
