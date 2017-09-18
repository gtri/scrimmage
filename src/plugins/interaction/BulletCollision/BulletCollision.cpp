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

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>

#include <scrimmage/math/State.h>

#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/msgs/Collision.pb.h>
#include <scrimmage/msgs/Event.pb.h>
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/common/RTree.h>
#include <scrimmage/sensor/Sensor.h>

#include <scrimmage/motion/MotionModel.h>

#include <scrimmage/plugins/interaction/BulletCollision/BulletCollision.h>
#include <scrimmage/plugins/sensor/RayTrace/RayTrace.h>

#include <memory>

namespace sc = scrimmage;
namespace sm = scrimmage_msgs;
namespace sp = scrimmage_proto;

REGISTER_PLUGIN(scrimmage::EntityInteraction, BulletCollision,
                BulletCollision_plugin)

BulletCollision::BulletCollision() {
    scene_size_ = 500;
    max_objects_ = 16000;

    bt_collision_configuration = new btDefaultCollisionConfiguration();
    bt_dispatcher = new btCollisionDispatcher(bt_collision_configuration);

    btScalar sscene_size = (btScalar) scene_size_;
    btVector3 worldAabbMin(-sscene_size, -sscene_size, -sscene_size);
    btVector3 worldAabbMax(sscene_size, sscene_size, sscene_size);

    // This is one type of broadphase, bullet has others that might be faster
    // depending on the application. true for disabling raycast accelerator.
    bt_broadphase = new bt32BitAxisSweep3(worldAabbMin, worldAabbMax,
                                          max_objects_, 0, true);

    bt_collision_world = new btCollisionWorld(bt_dispatcher, bt_broadphase,
                                              bt_collision_configuration);
}

BulletCollision::~BulletCollision() {
    delete bt_collision_world;
    delete bt_broadphase;
    delete bt_dispatcher;
    delete bt_collision_configuration;
}

bool BulletCollision::init(std::map<std::string, std::string> &mission_params,
                           std::map<std::string, std::string> &plugin_params) {
    show_rays_ = sc::get<bool>("show_rays", plugin_params, false);
    enable_collision_detection_ = sc::get<bool>("enable_collision_detection", plugin_params, true);
    enable_ray_tracing_ = sc::get<bool>("enable_ray_tracing", plugin_params, true);

    sub_ent_gen_ = create_subscriber("EntityGenerated");
    sub_shape_gen_ = create_subscriber("ShapeGenerated");

    btCollisionObject* coll_object = new btCollisionObject();
    btCollisionShape* ground_shape = new btStaticPlaneShape(btVector3(0, 0, 1), 0);

    coll_object->setUserIndex(0);
    coll_object->setCollisionShape(ground_shape);
    coll_object->getWorldTransform().setOrigin(btVector3((btScalar) 0,
                                                         (btScalar) 0,
                                                         (btScalar) 0));

    bt_collision_world->addCollisionObject(coll_object);

    return true;
}

bool BulletCollision::step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                              double t, double dt) {
    shapes_.clear();

    // Get new static shapes
    for (auto msg : sub_shape_gen_->msgs<sc::Message<sp::Shapes>>()) {
        for (int i = 0; i < msg->data.shape_size(); i++) {
            if (msg->data.shape(i).type() == sp::Shape::Cube) {
                btVector3 xyz(btScalar(msg->data.shape(i).xyz_lengths().x()/2.0),
                              btScalar(msg->data.shape(i).xyz_lengths().y()/2.0),
                              btScalar(msg->data.shape(i).xyz_lengths().z()/2.0));

                btBoxShape *wall = new btBoxShape(xyz);

                btCollisionObject* coll_object = new btCollisionObject();
                coll_object->setCollisionShape(wall);
                coll_object->getWorldTransform().setOrigin(btVector3((btScalar) msg->data.shape(i).center().x(),
                                                                     (btScalar) msg->data.shape(i).center().y(),
                                                                     (btScalar) msg->data.shape(i).center().z()));
                bt_collision_world->addCollisionObject(coll_object);
            }
        }
    }

    // Get newly created objects
    for (auto msg : sub_ent_gen_->msgs<sc::Message<sm::EntityGenerated>>()) {
        int id = msg->data.entity_id();

        sc::EntityPtr &ent = (*id_to_ent_map_)[id];

        btCollisionObject* coll_object = new btCollisionObject();
        coll_object->setUserIndex(id);
        coll_object->getWorldTransform().setOrigin(btVector3((btScalar) ent->state()->pos()(0),
                                                             (btScalar) ent->state()->pos()(1),
                                                             (btScalar) ent->state()->pos()(2)));

        btSphereShape * sphere_shape = new btSphereShape(ent->radius()); // TODO: memory management
        coll_object->setCollisionShape(sphere_shape);
        bt_collision_world->addCollisionObject(coll_object);

        objects_[id] = coll_object;

        if (enable_ray_tracing_) {
            // What types of sensors need to be attached to this entity?
            for (auto &kv : ent->sensors()) {
                if (kv.second->type() == "Ray") {
                    // Create a publisher for this sensor
                    // "entity_id/RayTrace0/pointcloud"
                    pcl_pubs_[id][kv.first] =
                        create_publisher(std::to_string(id) + "/" +
                                         kv.first  +
                                         "/pointcloud");

                    std::shared_ptr<RayTrace> rs =
                        std::dynamic_pointer_cast<RayTrace>(kv.second);
                    if (rs) {
                        RayTrace::PointCloud pcl;
                        pcl.max_range = rs->max_range();
                        pcl.min_range = rs->min_range();
                        pcl.num_rays_vert = rs->num_rays_vert();
                        pcl.num_rays_horiz = rs->num_rays_horiz();
                        pcl.angle_res_vert = rs->angle_res_vert();
                        pcl.angle_res_horiz = rs->angle_res_horiz();

                        double fov_horiz = rs->angle_res_horiz() * (rs->num_rays_horiz()-1);
                        double fov_vert = rs->angle_res_vert() * (rs->num_rays_vert()-1);
                        double start_angle_horiz = -fov_horiz / 2.0;
                        double start_angle_vert = -fov_vert / 2.0;

                        double angle_vert = start_angle_vert;
                        for (int v = 0; v < rs->num_rays_vert(); v++) {
                            double angle_horiz = start_angle_horiz;
                            for (int h = 0; h < rs->num_rays_horiz(); h++) {
                                Eigen::Vector3d r(rs->max_range(), 0, 0);
                                sc::Quaternion rot_vert(Eigen::Vector3d(0, 1, 0), angle_vert);
                                r = rot_vert.rotate(r);

                                sc::Quaternion rot_horiz(Eigen::Vector3d(0, 0, 1), angle_horiz);
                                r = rot_horiz.rotate(r);

                                pcl.points.push_back(RayTrace::PCPoint(r));
                                angle_horiz += rs->angle_res_horiz();
                            }
                            angle_vert += rs->angle_res_vert();
                        }
                        pcls_[id][kv.first]= pcl;
                    }
                }
            }
        }
    }

    // Update positions of all objects
    for (auto &kv : objects_) {
        sc::EntityPtr &ent = (*id_to_ent_map_)[kv.first];
        kv.second->getWorldTransform().setOrigin(btVector3((btScalar) ent->state()->pos()(0),
                                                           (btScalar) ent->state()->pos()(1),
                                                           (btScalar) ent->state()->pos()(2)));
    }

    // For each entity's ray-based sensors, compute point clouds
    for (auto &kv : pcls_) {
        sc::EntityPtr &own_ent = (*id_to_ent_map_)[kv.first];
        Eigen::Vector3d own_pos = own_ent->state()->pos();

        // For each ray sensor on a single entity
        for (auto kv2 : kv.second) {
            auto msg = std::make_shared<sc::Message<RayTrace::PointCloud>>();
            msg->data.max_range = kv2.second.max_range;
            msg->data.min_range = kv2.second.min_range;
            msg->data.num_rays_vert = kv2.second.num_rays_vert;
            msg->data.num_rays_horiz = kv2.second.num_rays_horiz;
            msg->data.angle_res_vert = kv2.second.angle_res_vert;
            msg->data.angle_res_horiz = kv2.second.angle_res_horiz;

            // Compute transformation matrix from entity's frame to sensor's
            // frame.
            sc::SensorPtr &sensor = own_ent->sensors()[kv2.first];
            Eigen::Matrix4d tf_m = own_ent->state()->tf_matrix(false) *
                sensor->transform()->tf_matrix();

            // For each ray in the sensor
            for (RayTrace::PCPoint &pcpoint : kv2.second.points) {
                Eigen::Vector3d original_ray = pcpoint.point;
                // Transform sensor's origin to world coordinates
                Eigen::Vector4d sensor_pos = tf_m * Eigen::Vector4d(0, 0, 0, 1);
                Eigen::Vector3d sensor_pos_w = sensor_pos.head<3>() + own_pos;

                // Transform ray's end point to world coordinates
                Eigen::Vector4d ray = tf_m * Eigen::Vector4d(original_ray(0),
                                                             original_ray(1),
                                                             original_ray(2),
                                                             1);
                Eigen::Vector3d ray_w = ray.head<3>() + own_pos;

                // Create bullet vectors
                btVector3 btFrom(sensor_pos_w(0), sensor_pos_w(1), sensor_pos_w(2));
                btVector3 btTo(ray_w(0), ray_w(1), ray_w(2));

                // Perform ray casting
                btCollisionWorld::ClosestRayResultCallback res(btFrom, btTo);
                bt_collision_world->rayTest(btFrom, btTo, res);

                // Points in the RayTrace message are defined with respect to
                // the LIDAR sensor's coordinate frame. Use original ray's
                // direction, shorten to length of detection ray, if a
                // collision occurred.
                if (res.hasHit()) {
                    Eigen::Vector3d hit_point(res.m_hitPointWorld.x(),
                                              res.m_hitPointWorld.y(),
                                              res.m_hitPointWorld.z());

                    msg->data.points.push_back(
                        RayTrace::PCPoint(original_ray.normalized() *
                                          (hit_point-sensor_pos_w).norm(), 255));

                    if (show_rays_) {
                        std::shared_ptr<sp::Shape> arrow(new sp::Shape);
                        arrow->set_type(sp::Shape::Line);
                        sc::set(arrow->mutable_color(), 255, 0, 0);
                        arrow->set_opacity(1.0);
                        sc::add_point(arrow, sensor_pos_w);
                        sc::add_point(arrow, hit_point);
                        shapes_.push_back(arrow);
                    }
                } else {
                    msg->data.points.push_back(RayTrace::PCPoint(original_ray, 255));
                    if (show_rays_) {
                        std::shared_ptr<sp::Shape> arrow(new sp::Shape);
                        arrow->set_type(sp::Shape::Line);
                        sc::set(arrow->mutable_color(), 0, 0, 255);
                        arrow->set_opacity(0.5);
                        sc::add_point(arrow, sensor_pos_w);
                        sc::add_point(arrow, ray_w);
                        shapes_.push_back(arrow);
                    }
                }
            }
            pcl_pubs_[kv.first][kv2.first]->publish(msg, t);
        }
    }

    if (enable_collision_detection_) {
        bt_collision_world->performDiscreteCollisionDetection();

        int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();

        // For each contact manifold
        for (int i = 0; i < numManifolds; i++) {
            btPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
            const btCollisionObject* obA = static_cast<const btCollisionObject*>(contactManifold->getBody0());
            const btCollisionObject* obB = static_cast<const btCollisionObject*>(contactManifold->getBody1());
            contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());
            int numContacts = contactManifold->getNumContacts();

            // For each contact point in that manifold
            for (int j = 0; j < numContacts; j++) {

                // Get the contact information
                btManifoldPoint& pt = contactManifold->getContactPoint(j);

                Eigen::Vector3d normal_B(pt.m_normalWorldOnB.x(),
                                         pt.m_normalWorldOnB.y(),
                                         pt.m_normalWorldOnB.z());

                if (id_to_ent_map_->count(obB->getUserIndex()) > 0) {
                    (*id_to_ent_map_)[obB->getUserIndex()]->motion()->set_external_force(-normal_B);
                }
                if (id_to_ent_map_->count(obA->getUserIndex()) > 0) {
                    (*id_to_ent_map_)[obA->getUserIndex()]->motion()->set_external_force(normal_B);
                }
            }
        }
    }

    return true;
}

bool BulletCollision::collision_exists(std::list<sc::EntityPtr> &ents,
                                       Eigen::Vector3d &p) {
    return false;
}
