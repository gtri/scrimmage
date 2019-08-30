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

#include <vector>
#include <list>
#include <map>
#include <string>
#include <memory>
#include <utility>

namespace sc = scrimmage;
class Interface;
using InterfacePtr = std::shared_ptr<Interface>;

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

    /** For the service call */
    bool get_ray_tracing(scrimmage::MessageBasePtr request,
                         scrimmage::MessageBasePtr &response);

 protected:
    std::pair<bool, scrimmage::EntityPtr> get_entity(const int &id);
    void remove_entity_object(const int &id);
    void entity_collision(const int &id);
    void remove_object(const int &id);

    btCollisionConfiguration* bt_collision_configuration;
    btCollisionDispatcher* bt_dispatcher;
    btBroadphaseInterface* bt_broadphase;
    btCollisionWorld* bt_collision_world;

    double scene_size_;
    unsigned int max_objects_;

    struct SceneObject {
        btCollisionObject* object = nullptr;
        scrimmage::ShapePtr shape = nullptr;
    };
    std::map<int, SceneObject> objects_;

    struct PointCloudDescription {
        sensor::RayTrace::PointCloud point_cloud;
        double last_update_time;
        sc::PublisherPtr pub;
        std::vector<scrimmage_proto::ShapePtr> shapes;
    };

    // Key 1: Entity ID
    // Value 2: map
    // Key 2: Sensor Name (sensor0)
    // Value 2: PointCloudDescription
    std::map<int, std::map<std::string, std::unique_ptr<PointCloudDescription>>> pc_descs_;

    bool show_rays_ = false;
    bool enable_collision_detection_ = true;
    bool enable_ray_tracing_ = true;

    std::string pcl_network_name_ = "LocalNetwork";
    std::string pcl_topic_name_ = "pointcloud";
    bool prepend_pcl_topic_with_id_ = false;
    bool publish_on_local_networks_ = true;

    scrimmage::PublisherPtr collision_pub_;
    scrimmage::PublisherPtr team_collision_pub_;
    scrimmage::PublisherPtr non_team_collision_pub_;
    bool enable_team_collisions_ = true;
    bool enable_non_team_collisions_ = true;

    bool remove_on_collision_ = false;
    bool show_collision_shapes_ = false;
    bool enable_ground_plane_ = false;
    bool enable_terrain_ = false;
    double ground_plane_height_ = 0;
};
}  // namespace interaction
}  // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_BULLETCOLLISION_BULLETCOLLISION_H_
