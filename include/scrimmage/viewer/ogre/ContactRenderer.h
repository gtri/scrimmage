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
 * @author Ethan M Boos <ethan.boos@gtri.gatech.edu>
 * @date 9 April 2026
 * @version 0.1.0
 *
 */

#ifndef INCLUDE_SCRIMMAGE_VIEWER_OGRE_CONTACTRENDERER_H_
#define INCLUDE_SCRIMMAGE_VIEWER_OGRE_CONTACTRENDERER_H_

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreManualObject.h>
#include <list>
#include <map>
#include <memory>
#include <string>
#include "scrimmage/proto/Contact.pb.h"
#include "scrimmage/proto/Frame.pb.h"
#include "scrimmage/proto/Visual.pb.h"
#include "scrimmage/viewer/ogre/MaterialPool.h"

namespace scrimmage {
namespace viewer {

/*
 *
 */
struct RenderedContact {
    Ogre::SceneNode* sceneNode = nullptr;
    Ogre::Entity* entity = nullptr;
    Ogre::ManualObject* manualObject = nullptr;  // For basic shapes
    Ogre::SceneNode* labelNode = nullptr;
    std::list<Ogre::SceneNode*> trail;
    scrimmage_proto::Color color;
    std::string model_name;
    int id = 0;
    int team_id = 0;
    bool exists = true;
    
    // For smooth interpolation
    Ogre::Vector3 targetPosition = Ogre::Vector3::ZERO;
    Ogre::Quaternion targetOrientation = Ogre::Quaternion::IDENTITY;
    bool hasTarget = false;
};

/*
 *
 *
 * Handles entity creation, updates, trails, and labels.
 */
class ContactRenderer {
 public:
    ContactRenderer(Ogre::SceneManager* sceneMgr, MaterialPool* materialPool);
    ~ContactRenderer();

    /*
     *
     */
    void init();

    /*
     *
     */
    void updateContacts(const scrimmage_proto::Frame& frame);

    /*
     *
     */
    void updateContactVisual(const scrimmage_proto::ContactVisual& cv);

    /*
     *
     */
    RenderedContact* getFollowedContact();

    /*
     *
     */
    void setFollowId(int id);

    /*
     *
     */
    void nextFollow();

    /*
     *
     */
    void prevFollow();

    /*
     *
     */
    void toggleTrails();

    /*
     *
     */
    void setScale(double scale);

    /*
     *
     */
    std::vector<int> getContactIds() const;

    /*
     *
     */
    void clear();

    /*
     *
     * @param dt Delta time since last frame.
     */
    void interpolateContacts(float dt);

    /*
     *
     */
    Ogre::SceneNode* getContactsNode() const { return contacts_node_; }

 private:
    void createContact(int id, const scrimmage_proto::Contact& contact);
    void updateContact(RenderedContact& rc, const scrimmage_proto::Contact& contact);
    void updateTrail(RenderedContact& rc, const Ogre::Vector3& position);
    void removeContact(int id);

    // Create basic shape representations
    Ogre::ManualObject* createSphereShape(const std::string& name, float radius,
                                          const Ogre::ColourValue& color);
    Ogre::ManualObject* createPyramidShape(const std::string& name, float size,
                                           const Ogre::ColourValue& color);

    Ogre::SceneManager* scene_mgr_;
    MaterialPool* material_pool_;
    Ogre::SceneNode* contacts_node_;

    std::map<int, RenderedContact> contacts_;
    int follow_id_ = 0;
    double scale_ = 1.0;
    bool show_trails_ = true;
    size_t max_trail_length_ = 100;
    size_t trail_counter_ = 0;  // Global counter for unique trail names
};

}  // namespace viewer
}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_VIEWER_OGRE_CONTACTRENDERER_H_
