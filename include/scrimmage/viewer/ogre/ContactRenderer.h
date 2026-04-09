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
 * @brief Contact (entity) renderer for Ogre3D.
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

/**
 * @brief Tracks a rendered contact (entity).
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
};

/**
 * @brief Renders simulation entities using Ogre3D.
 *
 * Handles entity creation, updates, trails, and labels.
 */
class ContactRenderer {
 public:
    ContactRenderer(Ogre::SceneManager* sceneMgr, MaterialPool* materialPool);
    ~ContactRenderer();

    /**
     * @brief Initialize the contact renderer.
     */
    void init();

    /**
     * @brief Update contacts from a frame.
     */
    void updateContacts(const scrimmage_proto::Frame& frame);

    /**
     * @brief Update a contact's visual properties.
     */
    void updateContactVisual(const scrimmage_proto::ContactVisual& cv);

    /**
     * @brief Get the contact being followed (for camera).
     */
    RenderedContact* getFollowedContact();

    /**
     * @brief Set the contact to follow.
     */
    void setFollowId(int id);

    /**
     * @brief Cycle to next contact for following.
     */
    void nextFollow();

    /**
     * @brief Cycle to previous contact for following.
     */
    void prevFollow();

    /**
     * @brief Toggle trail visibility.
     */
    void toggleTrails();

    /**
     * @brief Set scale for all contacts.
     */
    void setScale(double scale);

    /**
     * @brief Get all contact IDs.
     */
    std::vector<int> getContactIds() const;

    /**
     * @brief Clear all contacts.
     */
    void clear();

    /**
     * @brief Get the contacts parent node.
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
};

}  // namespace viewer
}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_VIEWER_OGRE_CONTACTRENDERER_H_
