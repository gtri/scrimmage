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

#if(ENABLE_TERRAIN == 1)
    // VTK
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

#include <memory>

namespace sc = scrimmage;
namespace sm = scrimmage_msgs;
namespace sp = scrimmage_proto;
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

    publish_on_local_networks_ = get("publish_on_local_networks", plugin_params, publish_on_local_networks_);
    pcl_network_name_ = get("pcl_network_name", plugin_params, pcl_network_name_);
    pcl_topic_name_ = get("pcl_topic_name", plugin_params, pcl_topic_name_);
    prepend_pcl_topic_with_id_ = get("prepend_pcl_topic_with_id", plugin_params, false);

    auto ent_gen_cb = [&] (scrimmage::MessagePtr<sm::EntityGenerated> msg) {
        auto pc_desc = std::make_unique<PointCloudDescription>();

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
                        pcl.num_rays_vert = rs->num_rays_vert();
                        pcl.num_rays_horiz = rs->num_rays_horiz();
                        pcl.angle_res_vert = rs->angle_res_vert();
                        pcl.angle_res_horiz = rs->angle_res_horiz();
                        pcl.max_sample_rate = rs->max_sample_rate();

                        double fov_horiz = rs->angle_res_horiz() * (rs->num_rays_horiz() - 1);
                        double fov_vert = rs->angle_res_vert() * (rs->num_rays_vert() - 1);
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

    btCollisionObject* coll_object = new btCollisionObject();
    btCollisionShape* ground_shape = new btStaticPlaneShape(btVector3(0, 0, 1), 0);

    coll_object->setUserIndex(0);
    coll_object->setCollisionShape(ground_shape);
    coll_object->getWorldTransform().setOrigin(btVector3((btScalar) 0,
            (btScalar) 0,
            (btScalar) 0));

    bt_collision_world->addCollisionObject(coll_object);

#if(ENABLE_TERRAIN == 1)
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
        translation->Translate(-utm_terrain->x_translate(), -utm_terrain->y_translate(),
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
        printf("BulletCollision: Loaded terrain data with %u triangles\n", (unsigned int)triangularData->GetNumberOfCells());

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
#endif

    return true;
}

bool BulletCollision::step_entity_interaction(std::list<sc::EntityPtr> &ents,
        double t, double dt) {
    // Update positions of all objects
    for (auto &kv : objects_) {
        sc::EntityPtr &ent = (*id_to_ent_map_)[kv.first];
        kv.second->getWorldTransform().setOrigin(btVector3((btScalar) ent->state()->pos()(0),
                (btScalar) ent->state()->pos()(1),
                (btScalar) ent->state()->pos()(2)));
    }

    // For each entity's ray-based sensors, compute point clouds
    for (auto &kv : pc_descs_) {
        sc::EntityPtr &own_ent = (*id_to_ent_map_)[kv.first];
        Eigen::Vector3d own_pos = own_ent->state()->pos();

        // For each ray sensor on a single entity
        for (auto &kv2 : kv.second) {
            sensor::RayTrace::PointCloud &pc = kv2.second->point_cloud;

            if (t > (pc.max_sample_rate + kv2.second->last_update_time)) {
                kv2.second->last_update_time = time_->t();

                auto msg = std::make_shared<sc::Message<RayTrace::PointCloud>>();
                msg->data.max_range = pc.max_range;
                msg->data.min_range = pc.min_range;
                msg->data.num_rays_vert = pc.num_rays_vert;
                msg->data.num_rays_horiz = pc.num_rays_horiz;
                msg->data.angle_res_vert = pc.angle_res_vert;
                msg->data.angle_res_horiz = pc.angle_res_horiz;

                // Compute transformation matrix from entity's frame to sensor's
                // frame.
                sc::SensorPtr &sensor = own_ent->sensors()[kv2.first];
                Eigen::Matrix4d tf_m = own_ent->state()->tf_matrix(false) *
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
        }
    }

    return true;
}

bool BulletCollision::collision_exists(std::list<sc::EntityPtr> &ents,
                                       Eigen::Vector3d &p) {
    return false;
}
} // namespace interaction
} // namespace scrimmage
