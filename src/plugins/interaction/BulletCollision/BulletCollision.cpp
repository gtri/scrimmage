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

#include <iostream>
#include <limits>
#include <memory>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <memory>

#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>

#include <scrimmage/pubsub/Message.h>
#include <scrimmage/msgs/Collision.pb.h>
#include <scrimmage/msgs/Event.pb.h>
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/common/RTree.h>

#include <scrimmage/motion/MotionModel.h>

#include <scrimmage/plugins/interaction/BulletCollision/BulletCollision.h>

namespace sc = scrimmage;
namespace sm = scrimmage_msgs;

using std::cout;
using std::endl;

REGISTER_PLUGIN(scrimmage::EntityInteraction, BulletCollision,
                BulletCollision_plugin)

BulletCollision::BulletCollision()
{
    scene_size_ = 500;
    max_objects_ = 16000;
    
    bt_collision_configuration = new btDefaultCollisionConfiguration();
    bt_dispatcher = new btCollisionDispatcher(bt_collision_configuration);

    btScalar sscene_size = (btScalar) scene_size_;
    btVector3 worldAabbMin(-sscene_size, -sscene_size, -sscene_size);
    btVector3 worldAabbMax(sscene_size, sscene_size, sscene_size);
    
    //This is one type of broadphase, bullet has others that might be faster
    //depending on the application. true for disabling raycast accelerator.
    bt_broadphase = new bt32BitAxisSweep3(worldAabbMin, worldAabbMax,
                                          max_objects_, 0, true);

    bt_collision_world = new btCollisionWorld(bt_dispatcher, bt_broadphase,
                                              bt_collision_configuration);

    // TODO:
    sphere_shape_ = new btSphereShape(1);
}

BulletCollision::~BulletCollision()
{
    delete bt_collision_world;
    delete bt_broadphase;
    delete bt_dispatcher;    
    delete bt_collision_configuration;
}

bool BulletCollision::init(std::map<std::string,std::string> &mission_params,
                           std::map<std::string,std::string> &plugin_params)
{       
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
                                              double t, double dt)
{
    // Maybe we should use a map by default
    std::map<int, sc::EntityPtr> int_to_ent_map;
    for (sc::EntityPtr &ent : ents) {
        int_to_ent_map[ent->id().id()] = ent;
    }

    // Get newly created objects
    for (auto msg : sub_ent_gen_->msgs<sc::Message<sm::EntityGenerated>>()) {
        int id = msg->data.entity_id();       
        
        btCollisionObject* coll_object = new btCollisionObject();
        coll_object->setUserIndex(id);
        coll_object->getWorldTransform().setOrigin(btVector3((btScalar) int_to_ent_map[id]->state()->pos()(0),
                                                             (btScalar) int_to_ent_map[id]->state()->pos()(1),
                                                             (btScalar) int_to_ent_map[id]->state()->pos()(2)));
        coll_object->setCollisionShape(sphere_shape_);
        bt_collision_world->addCollisionObject(coll_object);

        objects_[id] = coll_object;
    }

    // Update positions of all objects
    for (auto &kv : objects_) {        
        kv.second->getWorldTransform().setOrigin(btVector3((btScalar) int_to_ent_map[kv.first]->state()->pos()(0),
                                                           (btScalar) int_to_ent_map[kv.first]->state()->pos()(1),
                                                           (btScalar) int_to_ent_map[kv.first]->state()->pos()(2)));
    }

    bt_collision_world->performDiscreteCollisionDetection();

    int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();
    
    //For each contact manifold
    for (int i = 0; i < numManifolds; i++) {
        btPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
        const btCollisionObject* obA = static_cast<const btCollisionObject*>(contactManifold->getBody0());
        const btCollisionObject* obB = static_cast<const btCollisionObject*>(contactManifold->getBody1());
        contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());
        int numContacts = contactManifold->getNumContacts();        
        
        //For each contact point in that manifold
        for (int j = 0; j < numContacts; j++) {

            //cout << "Collision: " << obA->getUserIndex() << " - " << obB->getUserIndex() << endl;
            
            //Get the contact information
            btManifoldPoint& pt = contactManifold->getContactPoint(j);
            //btVector3 ptA = pt.getPositionWorldOnA();
            //btVector3 ptB = pt.getPositionWorldOnB();            
            //double ptdist = pt.getDistance();
            //cout << "ptdist: " << ptdist << endl;
            //cout << "impulse: " << pt.getAppliedImpulse() << endl;
            //cout << "normal: " << pt.m_normalWorldOnB.x() << ", " << pt.m_normalWorldOnB.y() << ", " << pt.m_normalWorldOnB.z() << endl;                        
        
            //sc::MotionModelPtr &motionA = int_to_ent_map[obA->getUserIndex()]->motion();
            //sc::MotionModelPtr &motionB = int_to_ent_map[obB->getUserIndex()]->motion();
            //            
            //double momentumA = motionA->mass() * motionA->state()->vel().norm();
            //double momentumB = motionB->mass() * motionB->state()->vel().norm();
            //double force = (momentumA + momentumB) / dt;
            //
            Eigen::Vector3d normal_B(pt.m_normalWorldOnB.x(),
                                     pt.m_normalWorldOnB.y(),
                                     pt.m_normalWorldOnB.z());

            //cout << "----" << endl;
            //cout << "Normal: " << obA->getUserIndex() << ", " << normal_B << endl;
            //cout << "Normal: " << obB->getUserIndex() << ", " << -normal_B << endl;            
            
            //Eigen::Vector3d force_B_dir = -normal_B * force;
            //
            //cout << "Force on " << obA->getUserIndex() << ", " << -force_B_dir << endl;
            //cout << "Force on " << obB->getUserIndex() << ", " << force_B_dir << endl;           

            if (int_to_ent_map.count(obB->getUserIndex()) > 0) {
                int_to_ent_map[obB->getUserIndex()]->motion()->set_external_force(-normal_B);
            }
            if (int_to_ent_map.count(obA->getUserIndex()) > 0) {
                int_to_ent_map[obA->getUserIndex()]->motion()->set_external_force(normal_B);
            }
        }
    }
    
    return true;
}

bool BulletCollision::collision_exists(std::list<sc::EntityPtr> &ents,
                                       Eigen::Vector3d &p)
{
    return false;
}
