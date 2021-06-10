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
#include <scrimmage/common/Time.h>
#include <scrimmage/common/Shape.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/proto/Shape.pb.h>

#include <scrimmage/math/State.h>

#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/msgs/Collision.pb.h>
#include <scrimmage/msgs/Event.pb.h>
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/common/RTree.h>
#include <scrimmage/sensor/Sensor.h>

#include <scrimmage/motion/MotionModel.h>
#include <scrimmage/network/Interface.h>
#include <scrimmage/plugins/interaction/BulletCollision/BulletCollision.h>
#include <scrimmage/plugins/sensor/RayTrace/RayTrace.h>

#include <memory>

#if ENABLE_VTK == 1
#include <vtkSmartPointer.h>
#include <vtkBYUReader.h>
#include <vtkOBJReader.h>
#include <vtkPLYReader.h>
#include <vtkTriangle.h>
#include <vtkPolyDataReader.h>
#include <vtkSTLReader.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkSphereSource.h>
#include <vtkTriangleFilter.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtksys/SystemTools.hxx>
#endif

namespace sc = scrimmage;
namespace sm = scrimmage_msgs;
namespace sp = scrimmage_proto;
namespace pl = std::placeholders;
using scrimmage::sensor::RayTrace;

REGISTER_PLUGIN(scrimmage::EntityInteraction, scrimmage::interaction::BulletCollision, BulletCollision_plugin)

namespace scrimmage {
namespace interaction {

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

    remove_on_collision_ = sc::get<bool>("remove_on_collision", plugin_params,
                                         remove_on_collision_);
    show_collision_shapes_ = sc::get<bool>("show_collision_shapes", plugin_params,
                                           show_collision_shapes_);
    enable_ground_plane_ = sc::get<bool>("enable_ground_plane", plugin_params,
                                         enable_ground_plane_);
    ground_plane_height_ = sc::get<double>("ground_plane_height", plugin_params,
                                           ground_plane_height_);
    enable_terrain_ = sc::get<bool>("enable_terrain", plugin_params,
                                    enable_terrain_);

    enable_team_collisions_ = get<bool>("enable_team_collisions",
                                        plugin_params, enable_team_collisions_);
    enable_non_team_collisions_ = get<bool>("enable_non_team_collisions",
                                            plugin_params, enable_non_team_collisions_);

    collision_pub_ = advertise("GlobalNetwork", "GroundCollision");
    team_collision_pub_ = advertise("GlobalNetwork", "TeamCollision");
    non_team_collision_pub_ = advertise("GlobalNetwork", "NonTeamCollision");

    publish_on_local_networks_ = get("publish_on_local_networks", plugin_params, publish_on_local_networks_);
    pcl_network_name_ = get("pcl_network_name", plugin_params, pcl_network_name_);
    pcl_topic_name_ = get("pcl_topic_name", plugin_params, pcl_topic_name_);
    prepend_pcl_topic_with_id_ = get("prepend_pcl_topic_with_id", plugin_params, false);

    // Define the service call for ray tracing
    parent_->global_services()["get_ray_tracing"] =
        std::bind(&BulletCollision::get_ray_tracing, this, pl::_1, pl::_2);

    // Enable the service interface for ray tracing

    auto ent_gen_cb = [&] (scrimmage::MessagePtr<sm::EntityGenerated> msg) {
        auto pc_desc = std::make_unique<PointCloudDescription>();

        int id = msg->data.entity_id();

        sc::EntityPtr &ent = (*id_to_ent_map_)[id];

        btCollisionObject* coll_object = new btCollisionObject();
        coll_object->setUserIndex(id);
        coll_object->getWorldTransform().setOrigin(btVector3(
            (btScalar) ent->state_truth()->pos()(0),
            (btScalar) ent->state_truth()->pos()(1),
            (btScalar) ent->state_truth()->pos()(2)));

        btSphereShape * sphere_shape = new btSphereShape(ent->radius());  // TODO: memory management
        coll_object->setCollisionShape(sphere_shape);
        bt_collision_world->addCollisionObject(coll_object);

        objects_[id].object = coll_object;

        if (show_collision_shapes_) {
            objects_[id].shape = sc::shape::make_sphere(
                ent->state_truth()->pos(), ent->radius(),
                Eigen::Vector3d(0, 0, 255), 0.30);
            draw_shape(objects_[id].shape);
        }

        if (enable_ray_tracing_) {
            // What types of sensors need to be attached to this entity?
            for (auto &kv : ent->sensors()) {
                if (kv.second->type() == "Ray") {
                    std::shared_ptr<RayTrace> rs =
                        std::dynamic_pointer_cast<RayTrace>(kv.second);
                    // Check whether automatic ray tracing should be done
                    if (rs->automatic_ray_tracing() == true) {
                        std::string topic = kv.second->name() + "/" + pcl_topic_name_;
                        if (prepend_pcl_topic_with_id_) {
                            topic = std::to_string(id) + "/" + topic;
                        }
                        if (publish_on_local_networks_) {
                            // Create a plugin that can publish on the local network
                            AutonomyPtr plugin = std::make_shared<Autonomy>();
                            plugin->set_parent(ent);
                            plugin->set_pubsub(plugin->parent()->pubsub());

                            // Create a publisher for this sensor
                            pc_desc->pub = plugin->advertise(pcl_network_name_, topic, 10);

                            // Add the empty plugin to the entity's autonomies list
                            ent->autonomies().push_back(plugin);
                        } else {
                            pc_desc->pub = advertise(pcl_network_name_, topic, 10);
                        }

                        std::shared_ptr<RayTrace> rs =
                            std::dynamic_pointer_cast<RayTrace>(kv.second);
                        if (rs) {
                            RayTrace::PointCloud pcl;
                            pcl.max_range = rs->max_range();
                            pcl.min_range = rs->min_range();
                            pcl.max_sample_rate = rs->max_sample_rate();
                            // Get the rays (copied internally, so can just assign here), using the model name
                            //  so the dirty flag can be checked later.
                            pcl.rays = rs->rays();
                            for (auto &ray : pcl.rays) {
                                Eigen::Vector3d r(rs->max_range(), 0, 0);
                                sc::Quaternion rot_vert(Eigen::Vector3d(0, 1, 0), ray.elevation_rad);
                                r = rot_vert.rotate(r);

                                sc::Quaternion rot_horiz(Eigen::Vector3d(0, 0, 1), ray.azimuth_rad);
                                r = rot_horiz.rotate(r);

                                pcl.points.push_back(RayTrace::PCPoint(r));
                            }
                            pc_desc->point_cloud = pcl;
                            pc_desc->last_update_time = time_->t();

                            if (show_rays_) {
                                // Construct the ray shapes, but don't draw them
                                // yet.
                                for (unsigned int i = 0;
                                     i < pc_desc->point_cloud.points.size(); i++) {
                                    std::shared_ptr<sp::Shape> line(new sp::Shape);
                                    sc::set(line->mutable_color(), 255, 0, 0);
                                    line->set_opacity(1.0);
                                    sc::set(line->mutable_line()->mutable_start(), Eigen::Vector3d(0, 0, 0));
                                    sc::set(line->mutable_line()->mutable_end(), Eigen::Vector3d(0, 0, 0));
                                    pc_desc->shapes.push_back(line);
                                }
                            }

                            // Move the description into the map
                            pc_descs_[id][kv.first] = std::move(pc_desc);
                        }
                    }
                }
            }
        }
    };
    subscribe<sm::EntityGenerated>("GlobalNetwork", "EntityGenerated", ent_gen_cb);

    auto shape_gen_cb = [&] (scrimmage::MessagePtr<sp::Shapes> msg) {
        for (int i = 0; i < msg->data.shape_size(); i++) {
            if (msg->data.shape(i).oneof_type_case() == sp::Shape::kCuboid) {
                btVector3 xyz(btScalar(msg->data.shape(i).cuboid().x_length() / 2.0),
                              btScalar(msg->data.shape(i).cuboid().y_length() / 2.0),
                              btScalar(msg->data.shape(i).cuboid().z_length() / 2.0));

                btBoxShape *wall = new btBoxShape(xyz);

                btCollisionObject* coll_object = new btCollisionObject();
                coll_object->setCollisionShape(wall);
                coll_object->getWorldTransform().setOrigin(btVector3((btScalar) msg->data.shape(i).cuboid().center().x(),
                        (btScalar) msg->data.shape(i).cuboid().center().y(),
                        (btScalar) msg->data.shape(i).cuboid().center().z()));
                bt_collision_world->addCollisionObject(coll_object);
            }
        }
    };
    subscribe<sp::Shapes>("GlobalNetwork", "ShapeGenerated", shape_gen_cb);

    if (enable_ground_plane_) {
        btCollisionObject* coll_object = new btCollisionObject();
        btCollisionShape* ground_shape = new btStaticPlaneShape(btVector3(0, 0, 1), 0);

        coll_object->setUserIndex(0);
        coll_object->setCollisionShape(ground_shape);
        coll_object->getWorldTransform().setOrigin(
            btVector3((btScalar) 0, (btScalar) 0,
                      (btScalar) ground_plane_height_));
        bt_collision_world->addCollisionObject(coll_object);
    }

#if ENABLE_VTK == 1
    if (enable_terrain_) {
        std::shared_ptr<scrimmage_proto::UTMTerrain> utm_terrain = parent_->mp()->utm_terrain();
        if (utm_terrain->enable_terrain() == true) {
            // Read the terrain mesh file
            vtkSmartPointer<vtkPolyData> polyData;
            std::string extension = vtksys::SystemTools::GetFilenameLastExtension(std::string(utm_terrain->poly_data_file().c_str()));
            if (extension == ".ply") {
                vtkSmartPointer<vtkPLYReader> reader =
                        vtkSmartPointer<vtkPLYReader>::New();
                reader->SetFileName(utm_terrain->poly_data_file().c_str());
                reader->Update();
                polyData = reader->GetOutput();
            } else if (extension == ".vtp") {
                vtkSmartPointer<vtkXMLPolyDataReader> reader =
                        vtkSmartPointer<vtkXMLPolyDataReader>::New();
                reader->SetFileName(utm_terrain->poly_data_file().c_str());
                reader->Update();
                polyData = reader->GetOutput();
            } else if (extension == ".obj") {
                vtkSmartPointer<vtkOBJReader> reader =
                        vtkSmartPointer<vtkOBJReader>::New();
                reader->SetFileName(utm_terrain->poly_data_file().c_str());
                reader->Update();
                polyData = reader->GetOutput();
            } else if (extension == ".stl") {
                vtkSmartPointer<vtkSTLReader> reader =
                        vtkSmartPointer<vtkSTLReader>::New();
                reader->SetFileName(utm_terrain->poly_data_file().c_str());
                reader->Update();
                polyData = reader->GetOutput();
            } else if (extension == ".vtk") {
                vtkSmartPointer<vtkPolyDataReader> reader =
                        vtkSmartPointer<vtkPolyDataReader>::New();
                reader->SetFileName(utm_terrain->poly_data_file().c_str());
                reader->Update();
                polyData = reader->GetOutput();
            } else if (extension == ".g") {
                vtkSmartPointer<vtkBYUReader> reader =
                        vtkSmartPointer<vtkBYUReader>::New();
                reader->SetGeometryFileName(utm_terrain->poly_data_file().c_str());
                reader->Update();
                polyData = reader->GetOutput();
            } else {
                vtkSmartPointer<vtkSphereSource> source =
                        vtkSmartPointer<vtkSphereSource>::New();
                source->Update();
                polyData = source->GetOutput();
            }
            // Update the location
            vtkSmartPointer<vtkTransform> translation =
                    vtkSmartPointer<vtkTransform>::New();
            translation->Translate(-utm_terrain->x_translate(),
                                   -utm_terrain->y_translate(),
                                   -utm_terrain->z_translate());

            vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter =
                    vtkSmartPointer<vtkTransformPolyDataFilter>::New();
            transformFilter->SetInputData(polyData);
            transformFilter->SetTransform(translation);
            transformFilter->Update();
            vtkSmartPointer<vtkPolyData> transformedData = transformFilter->GetOutput();

            // Next, convert it to a triangular mesh
            vtkSmartPointer<vtkTriangleFilter> triFilter = vtkSmartPointer<vtkTriangleFilter>::New();
            triFilter->SetInputData(transformedData);
            triFilter->Update();
            vtkSmartPointer<vtkPolyData> triangularData = triFilter->GetOutput();

            // Get the results
            btTriangleMesh* triangles = new btTriangleMesh();
            for (vtkIdType index = 0; index < triangularData->GetNumberOfCells(); index++) {
                vtkCell* cell = triangularData->GetCell(index);

                vtkTriangle* triangle = dynamic_cast<vtkTriangle*>(cell);
                double p0[3];
                double p1[3];
                double p2[3];
                triangle->GetPoints()->GetPoint(0, p0);
                triangle->GetPoints()->GetPoint(1, p1);
                triangle->GetPoints()->GetPoint(2, p2);
                // Create the new triangle
                triangles->addTriangle(btVector3(p0[0], p0[1], p0[2]), btVector3(p1[0], p1[1], p1[2]), btVector3(p2[0], p2[1], p2[2]));
            }
            btCollisionShape* mesh = new btBvhTriangleMeshShape(triangles, true, true);
            btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
            btRigidBody::btRigidBodyConstructionInfo rigidBodyConstructionInfo(0.0f, motionState, mesh, btVector3(0, 0, 0));
            btRigidBody* rigidBodyTerrain = new btRigidBody(rigidBodyConstructionInfo);
            rigidBodyTerrain->setFriction(btScalar(0.9));
            bt_collision_world->addCollisionObject(rigidBodyTerrain);
        }
    }
#endif
    return true;
}

bool BulletCollision::step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                              double t, double dt) {
    // Update positions of all objects
    for (auto &kv : *id_to_ent_map_) {
        auto it_object = objects_.find(kv.first);
        if (it_object != objects_.end()) {
            sc::EntityPtr &ent = kv.second;
            it_object->second.object->getWorldTransform().setOrigin(btVector3(
                (btScalar) ent->state_truth()->pos()(0),
                (btScalar) ent->state_truth()->pos()(1),
                (btScalar) ent->state_truth()->pos()(2)));

            if (show_collision_shapes_) {
                sc::set(it_object->second.shape->mutable_sphere()->mutable_center(),
                        ent->state_truth()->pos());
                draw_shape(objects_[kv.first].shape);
            }
        }
    }

    // If an entity no longer exists in the ent map, remove it's shape from the
    // bullet environment
    for (auto &kv : objects_) {
        auto it_ent = id_to_ent_map_->find(kv.first);
        if (it_ent == id_to_ent_map_->end()) {
            remove_object(kv.first);
        }
    }

    // For each entity's ray-based sensors, compute point clouds
    for (auto &kv : pc_descs_) {
        sc::EntityPtr &own_ent = (*id_to_ent_map_)[kv.first];
        Eigen::Vector3d own_pos = own_ent->state_truth()->pos();

        // For each ray sensor on a single entity
        for (auto &kv2 : kv.second) {
            sensor::RayTrace::PointCloud &pc = kv2.second->point_cloud;

            if (t > (pc.max_sample_rate + kv2.second->last_update_time)) {
                kv2.second->last_update_time = time_->t();

                auto msg = std::make_shared<sc::Message<RayTrace::PointCloud>>();
                msg->data.max_range = pc.max_range;
                msg->data.min_range = pc.min_range;
                msg->data.rays = pc.get_rays();

                // Compute transformation matrix from entity's frame to sensor's
                // frame.
                sc::SensorPtr &sensor = own_ent->sensors()[kv2.first];
                Eigen::Matrix4d tf_m = own_ent->state_truth()->tf_matrix(false) *
                                       sensor->transform()->tf_matrix();

                // For each ray in the sensor
                unsigned int i = 0;
                for (RayTrace::PCPoint &pcpoint : pc.points) {
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
                    Eigen::Vector3d ray_end;
                    if (res.hasHit()) {
                        Eigen::Vector3d hit_point(res.m_hitPointWorld.x(),
                                                  res.m_hitPointWorld.y(),
                                                  res.m_hitPointWorld.z());
                        Eigen::Vector3d return_vec = original_ray.normalized() *
                                                     (hit_point - sensor_pos_w).norm();
                        msg->data.points.push_back(
                            RayTrace::PCPoint(return_vec, 255,
                                              ((return_vec.norm() > msg->data.max_range) ||
                                               (return_vec.norm() < msg->data.min_range))));
                        ray_end = hit_point;
                    } else {
                        msg->data.points.push_back(RayTrace::PCPoint(original_ray, 255, true));
                        ray_end = ray_w;
                    }

                    if (show_rays_) {
                        std::unique_ptr<PointCloudDescription> &pc_desc = kv2.second;
                        if (res.hasHit()) {
                            sc::set(pc_desc->shapes[i]->mutable_color(), 255, 0, 0);
                            pc_desc->shapes[i]->set_opacity(1.0);
                        } else {
                            sc::set(pc_desc->shapes[i]->mutable_color(), 0, 0, 255);
                            pc_desc->shapes[i]->set_opacity(0.5);
                        }
                        sc::set(pc_desc->shapes[i]->mutable_line()->mutable_start(), sensor_pos_w);
                        sc::set(pc_desc->shapes[i]->mutable_line()->mutable_end(), ray_end);
                        draw_shape(pc_desc->shapes[i]);
                    }
                    i++;
                }
                kv2.second->pub->publish(msg);
            }
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

            if (remove_on_collision_ && numContacts > 0) {
                bool ob_a_is_ent, ob_b_is_ent;
                int obj_a_id = obA->getUserIndex();
                int obj_b_id = obB->getUserIndex();

                sc::EntityPtr ent_a, ent_b;
                std::tie(ob_a_is_ent, ent_a) = get_entity(obj_a_id);
                std::tie(ob_b_is_ent, ent_b) = get_entity(obj_b_id);

                // We need to determine if a collision occured because two
                // entities collided or if the entity collided with a
                // non-entity (terrain, shape obstacle, etc.). If both objects
                // were removed, two entities collided with each other. If only
                // one object was removed, it collided with the terrain or
                // another static obstacle.
                if (ob_a_is_ent && ob_b_is_ent) {
                    bool same_team = ent_a->id().team_id() == ent_b->id().team_id();
                    if (enable_team_collisions_ && same_team) {
                        // Construct the TeamCollision message
                        auto msg = std::make_shared<Message<sm::TeamCollision>>();
                        msg->data.set_entity_id_1(ent_a->id().id());
                        msg->data.set_entity_id_2(ent_b->id().id());
                        team_collision_pub_->publish(msg);

                        // Remove the entities
                        remove_entity_object(obj_a_id);
                        remove_entity_object(obj_b_id);

                    } else if (enable_non_team_collisions_ && !same_team) {
                        // Construct the NonTeamCollision message
                        auto msg = std::make_shared<Message<sm::NonTeamCollision>>();
                        msg->data.set_entity_id_1(ent_a->id().id());
                        msg->data.set_entity_id_2(ent_b->id().id());
                        non_team_collision_pub_->publish(msg);

                        // Remove the entities
                        remove_entity_object(obj_a_id);
                        remove_entity_object(obj_b_id);
                    }

                } else if (ob_a_is_ent && ent_a->is_alive()) {
                    // Construct the GroundCollision message for entity A
                    auto msg = std::make_shared<sc::Message<sm::GroundCollision>>();
                    msg->data.set_entity_id(ent_a->id().id());
                    collision_pub_->publish(msg);

                    remove_entity_object(obj_a_id);

                } else if (ob_b_is_ent && ent_b->is_alive()) {
                    // Construct the GroundCollision message for entity B
                    auto msg = std::make_shared<sc::Message<sm::GroundCollision>>();
                    msg->data.set_entity_id(ent_b->id().id());
                    collision_pub_->publish(msg);

                    remove_entity_object(obj_b_id);
                }
            }
        }
    }
    return true;
}

bool BulletCollision::get_ray_tracing(scrimmage::MessageBasePtr request,
                                      scrimmage::MessageBasePtr &response) {
    auto request_cast = std::dynamic_pointer_cast<sc::Message<RayTrace::PointCloudWithId>>(request);

    if (request_cast == nullptr) {
        std::cout << "Could not cast to sc::Message<RayTrace::PointCloudId> request" << std::endl;
        return false;
    }

    int entity_id = request_cast->data.entity_id;
    std::string sensor_name = request_cast->data.sensor_name;
    sc::EntityPtr &own_ent = (*id_to_ent_map_)[entity_id];
    Eigen::Vector3d own_pos = own_ent->state_truth()->pos();
    // Compute transformation matrix from entity's frame to sensor's
    // frame.
    sc::SensorPtr &sensor = own_ent->sensors()[sensor_name];
    Eigen::Matrix4d tf_m = own_ent->state_truth()->tf_matrix(false) *
                           sensor->transform()->tf_matrix();
    auto response_cast = std::make_shared<sc::Message<RayTrace::PointCloud>>();
    response_cast->data.max_range = request_cast->data.max_range;
    response_cast->data.min_range = request_cast->data.min_range;
    response_cast->data.rays = request_cast->data.get_rays();

    // Create the points from the rays
    for (auto &ray : response_cast->data.rays) {
        Eigen::Vector3d r(response_cast->data.max_range, 0, 0);
        sc::Quaternion rot_vert(Eigen::Vector3d(0, 1, 0), ray.elevation_rad);
        r = rot_vert.rotate(r);

        sc::Quaternion rot_horiz(Eigen::Vector3d(0, 0, 1), ray.azimuth_rad);
        r = rot_horiz.rotate(r);

        request_cast->data.points.push_back(RayTrace::PCPoint(r));
    }

    // For each ray in the sensor
    for (RayTrace::PCPoint &pcpoint : request_cast->data.points) {
        Eigen::Vector3d original_ray = pcpoint.point;
        // Transform sensor's origin to world coordinates
        Eigen::Vector4d sensor_pos = tf_m * Eigen::Vector4d(0, 0, 0, 1);
        Eigen::Vector3d sensor_pos_w = sensor_pos.head<3>() + own_pos;

        Eigen::Vector3d ray_w;
        if (request_cast->data.world_frame == false) {
            // Transform ray's end point to world coordinates
            Eigen::Vector4d ray = tf_m * Eigen::Vector4d(original_ray(0),
                                  original_ray(1),
                                  original_ray(2),
                                  1);
            ray_w = ray.head<3>() + own_pos;
        } else {
            // Already in world frame
            ray_w = original_ray;
        }

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
        Eigen::Vector3d ray_end;
        if (res.hasHit()) {
            Eigen::Vector3d hit_point(res.m_hitPointWorld.x(),
                                      res.m_hitPointWorld.y(),
                                      res.m_hitPointWorld.z());
            Eigen::Vector3d return_vec = original_ray.normalized() *
                                         (hit_point - sensor_pos_w).norm();
            response_cast->data.points.push_back(
                RayTrace::PCPoint(return_vec, 255,
                                  ((return_vec.norm() > response_cast->data.max_range) ||
                                   (return_vec.norm() < response_cast->data.min_range))));
            ray_end = hit_point;
        } else {
            response_cast->data.points.push_back(RayTrace::PCPoint(original_ray, 255, true));
            ray_end = ray_w;
        }
    }
    // Save the response
    response = response_cast;

    // Success
    return true;
}

std::pair<bool, scrimmage::EntityPtr> BulletCollision::get_entity(const int &id) {
    auto it_ent = id_to_ent_map_->find(id);
    if (it_ent != id_to_ent_map_->end()) {
        return std::make_pair(true, it_ent->second);
    } else {
        return std::make_pair(false, nullptr);
    }
}

void BulletCollision::remove_entity_object(const int &id) {
    entity_collision(id);
    remove_object(id);
}

void BulletCollision::entity_collision(const int &id) {
    // Call the collision() function to remove entity
    auto it_ent = id_to_ent_map_->find(id);
    if (it_ent != id_to_ent_map_->end()) {
        it_ent->second->collision();
    }
}

void BulletCollision::remove_object(const int &id) {
    auto it_object = objects_.find(id);
    if (it_object != objects_.end()) {
        // Remove the object shape if it exists
        if (it_object->second.shape != nullptr) {
            it_object->second.shape->set_persistent(false);
            sc::set(it_object->second.shape->mutable_color(), Eigen::Vector3d(0, 0, 0));
            draw_shape(it_object->second.shape);
        }

        // Remove the bullet object from the scene
        bt_collision_world->removeCollisionObject(it_object->second.object);
        // Erase the Bullet Object from the map
        objects_.erase(it_object);
    }
}

bool BulletCollision::collision_exists(std::list<sc::EntityPtr> &ents,
                                       Eigen::Vector3d &p) {
    return false;
}
}  // namespace interaction
}  // namespace scrimmage
