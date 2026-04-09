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
 * @brief Contact renderer implementation.
 */

#include "scrimmage/viewer/ogre/ContactRenderer.h"
#include "scrimmage/viewer/ogre/CoordinateConverter.h"
#include <cmath>

namespace scrimmage {
namespace viewer {

ContactRenderer::ContactRenderer(Ogre::SceneManager* sceneMgr, MaterialPool* materialPool)
    : scene_mgr_(sceneMgr), material_pool_(materialPool), contacts_node_(nullptr) {
}

ContactRenderer::~ContactRenderer() {
    clear();
}

void ContactRenderer::init() {
    contacts_node_ = scene_mgr_->getRootSceneNode()->createChildSceneNode("ContactsNode");
}

void ContactRenderer::updateContacts(const scrimmage_proto::Frame& frame) {
    // Mark all existing contacts as potentially removed
    for (auto& [id, contact] : contacts_) {
        contact.exists = false;
    }

    // Update or create contacts from frame
    for (const auto& contact : frame.contact()) {
        int id = contact.id().id();
        auto it = contacts_.find(id);

        if (it == contacts_.end()) {
            // Create new contact
            createContact(id, contact);
        } else {
            // Update existing contact
            it->second.exists = true;
            updateContact(it->second, contact);
        }
    }

    // Remove contacts that no longer exist
    std::vector<int> to_remove;
    for (const auto& [id, contact] : contacts_) {
        if (!contact.exists) {
            to_remove.push_back(id);
        }
    }
    for (int id : to_remove) {
        removeContact(id);
    }
}

void ContactRenderer::createContact(int id, const scrimmage_proto::Contact& contact) {
    RenderedContact rc;
    rc.id = id;
    rc.team_id = contact.id().team_id();

    // Default colors based on team (Contact doesn't have color, use ContactVisual for that)
    rc.color.set_r((rc.team_id == 1) ? 0 : 255);
    rc.color.set_g((rc.team_id == 2) ? 255 : 0);
    rc.color.set_b((rc.team_id == 1) ? 255 : 0);

    Ogre::ColourValue ogreColor = MaterialPool::toOgreColor(rc.color);

    // Create scene node for this contact
    std::string nodeName = "Contact_" + std::to_string(id);
    rc.sceneNode = contacts_node_->createChildSceneNode(nodeName);

    // Create visual representation based on contact type
    // For now, use simple shapes - models will be loaded later
    std::string shapeName = "ContactShape_" + std::to_string(id);
    
    // Check if we should use a sphere or pyramid based on contact type
    if (contact.type() == scrimmage_proto::AIRCRAFT ||
        contact.type() == scrimmage_proto::QUADROTOR) {
        rc.manualObject = createPyramidShape(shapeName, 5.0f * static_cast<float>(scale_), ogreColor);
    } else {
        rc.manualObject = createSphereShape(shapeName, 3.0f * static_cast<float>(scale_), ogreColor);
    }

    rc.sceneNode->attachObject(rc.manualObject);

    // Set initial position and orientation
    rc.sceneNode->setPosition(CoordinateConverter::toOgre(
        contact.state().position().x(),
        contact.state().position().y(),
        contact.state().position().z()));

    if (contact.state().has_orientation()) {
        const auto& q = contact.state().orientation();
        rc.sceneNode->setOrientation(CoordinateConverter::toOgre(
            scrimmage::Quaternion(q.w(), q.x(), q.y(), q.z())));
    }

    contacts_[id] = rc;

    // Set initial follow target if needed
    if (contacts_.size() == 1 || follow_id_ == 0) {
        follow_id_ = id;
    }
}

void ContactRenderer::updateContact(RenderedContact& rc, const scrimmage_proto::Contact& contact) {
    // Update position
    Ogre::Vector3 pos = CoordinateConverter::toOgre(
        contact.state().position().x(),
        contact.state().position().y(),
        contact.state().position().z());
    rc.sceneNode->setPosition(pos);

    // Update orientation
    if (contact.state().has_orientation()) {
        const auto& q = contact.state().orientation();
        rc.sceneNode->setOrientation(CoordinateConverter::toOgre(
            scrimmage::Quaternion(q.w(), q.x(), q.y(), q.z())));
    }

    // Update trail
    if (show_trails_) {
        updateTrail(rc, pos);
    }
}

void ContactRenderer::updateTrail(RenderedContact& rc, const Ogre::Vector3& position) {
    // Create trail point with globally unique name
    std::string trailName = "Trail_" + std::to_string(rc.id) + "_" + 
                            std::to_string(trail_counter_++);
    
    Ogre::ManualObject* trailPoint = scene_mgr_->createManualObject(trailName);
    Ogre::ColourValue color = MaterialPool::toOgreColor(rc.color);
    color.a = 0.5f;  // Semi-transparent trail
    
    std::string matName = material_pool_->getMaterial(
        static_cast<int>(color.r * 255),
        static_cast<int>(color.g * 255),
        static_cast<int>(color.b * 255),
        color.a);

    // Create small sphere for trail point
    trailPoint->begin(matName, Ogre::RenderOperation::OT_POINT_LIST);
    trailPoint->position(0, 0, 0);
    trailPoint->colour(color);
    trailPoint->end();

    Ogre::SceneNode* trailNode = contacts_node_->createChildSceneNode(trailName + "_node");
    trailNode->setPosition(position);
    trailNode->attachObject(trailPoint);

    rc.trail.push_back(trailNode);

    // Remove old trail points if over limit
    while (rc.trail.size() > max_trail_length_) {
        Ogre::SceneNode* oldNode = rc.trail.front();
        rc.trail.pop_front();
        
        if (oldNode->numAttachedObjects() > 0) {
            Ogre::MovableObject* obj = oldNode->getAttachedObject(0);
            oldNode->detachObject(obj);
            scene_mgr_->destroyManualObject(static_cast<Ogre::ManualObject*>(obj));
        }
        scene_mgr_->destroySceneNode(oldNode);
    }
}

void ContactRenderer::removeContact(int id) {
    auto it = contacts_.find(id);
    if (it == contacts_.end()) return;

    RenderedContact& rc = it->second;

    // Remove trail
    for (auto* node : rc.trail) {
        if (node->numAttachedObjects() > 0) {
            Ogre::MovableObject* obj = node->getAttachedObject(0);
            node->detachObject(obj);
            scene_mgr_->destroyManualObject(static_cast<Ogre::ManualObject*>(obj));
        }
        scene_mgr_->destroySceneNode(node);
    }

    // Remove entity/manual object
    if (rc.manualObject) {
        rc.sceneNode->detachObject(rc.manualObject);
        scene_mgr_->destroyManualObject(rc.manualObject);
    }
    if (rc.entity) {
        rc.sceneNode->detachObject(rc.entity);
        scene_mgr_->destroyEntity(rc.entity);
    }

    // Remove scene node
    scene_mgr_->destroySceneNode(rc.sceneNode);

    contacts_.erase(it);
}

void ContactRenderer::updateContactVisual(const scrimmage_proto::ContactVisual& cv) {
    int id = cv.id();
    auto it = contacts_.find(id);
    if (it == contacts_.end()) return;

    RenderedContact& rc = it->second;

    // Update color
    if (cv.has_color()) {
        rc.color = cv.color();
        // Recreate manual object with new color would be needed here
        // For now, we'll just update the stored color
    }

    // Update model would be handled here
    // cv.model() contains the model path
}

Ogre::ManualObject* ContactRenderer::createSphereShape(const std::string& name, float radius,
                                                        const Ogre::ColourValue& color) {
    std::string matName = material_pool_->getMaterial(color);
    Ogre::ManualObject* obj = scene_mgr_->createManualObject(name);
    
    const int rings = 8;
    const int segments = 8;

    obj->begin(matName, Ogre::RenderOperation::OT_TRIANGLE_LIST);

    for (int ring = 0; ring <= rings; ++ring) {
        float phi = M_PI * static_cast<float>(ring) / rings;
        for (int seg = 0; seg <= segments; ++seg) {
            float theta = 2.0f * M_PI * static_cast<float>(seg) / segments;

            float x = radius * std::sin(phi) * std::cos(theta);
            float y = radius * std::cos(phi);
            float z = radius * std::sin(phi) * std::sin(theta);

            obj->position(x, y, z);
            obj->normal(std::sin(phi) * std::cos(theta),
                        std::cos(phi),
                        std::sin(phi) * std::sin(theta));
            obj->colour(color);
        }
    }

    for (int ring = 0; ring < rings; ++ring) {
        for (int seg = 0; seg < segments; ++seg) {
            int curr = ring * (segments + 1) + seg;
            int next = curr + segments + 1;

            obj->index(curr);
            obj->index(next);
            obj->index(curr + 1);

            obj->index(curr + 1);
            obj->index(next);
            obj->index(next + 1);
        }
    }

    obj->end();
    return obj;
}

Ogre::ManualObject* ContactRenderer::createPyramidShape(const std::string& name, float size,
                                                         const Ogre::ColourValue& color) {
    std::string matName = material_pool_->getMaterial(color);
    Ogre::ManualObject* obj = scene_mgr_->createManualObject(name);

    float h = size;
    float b = size * 0.5f;

    obj->begin(matName, Ogre::RenderOperation::OT_TRIANGLE_LIST);

    // Apex (nose of aircraft, pointing forward = +Z in Ogre = -Y in ENU)
    Ogre::Vector3 apex(0, 0, h);
    
    // Base vertices
    Ogre::Vector3 bl(-b, -b, 0);
    Ogre::Vector3 br(b, -b, 0);
    Ogre::Vector3 tr(b, b, 0);
    Ogre::Vector3 tl(-b, b, 0);

    // Front face (apex, br, bl)
    Ogre::Vector3 n1 = (br - apex).crossProduct(bl - apex).normalisedCopy();
    obj->position(apex); obj->normal(n1); obj->colour(color);
    obj->position(br);   obj->normal(n1); obj->colour(color);
    obj->position(bl);   obj->normal(n1); obj->colour(color);

    // Right face (apex, tr, br)
    Ogre::Vector3 n2 = (tr - apex).crossProduct(br - apex).normalisedCopy();
    obj->position(apex); obj->normal(n2); obj->colour(color);
    obj->position(tr);   obj->normal(n2); obj->colour(color);
    obj->position(br);   obj->normal(n2); obj->colour(color);

    // Back face (apex, tl, tr)
    Ogre::Vector3 n3 = (tl - apex).crossProduct(tr - apex).normalisedCopy();
    obj->position(apex); obj->normal(n3); obj->colour(color);
    obj->position(tl);   obj->normal(n3); obj->colour(color);
    obj->position(tr);   obj->normal(n3); obj->colour(color);

    // Left face (apex, bl, tl)
    Ogre::Vector3 n4 = (bl - apex).crossProduct(tl - apex).normalisedCopy();
    obj->position(apex); obj->normal(n4); obj->colour(color);
    obj->position(bl);   obj->normal(n4); obj->colour(color);
    obj->position(tl);   obj->normal(n4); obj->colour(color);

    // Base (two triangles)
    Ogre::Vector3 nb(0, 0, -1);
    obj->position(bl); obj->normal(nb); obj->colour(color);
    obj->position(br); obj->normal(nb); obj->colour(color);
    obj->position(tr); obj->normal(nb); obj->colour(color);
    obj->position(bl); obj->normal(nb); obj->colour(color);
    obj->position(tr); obj->normal(nb); obj->colour(color);
    obj->position(tl); obj->normal(nb); obj->colour(color);

    // Add indices
    for (unsigned int i = 0; i < 18; ++i) {
        obj->index(i);
    }

    obj->end();
    return obj;
}

RenderedContact* ContactRenderer::getFollowedContact() {
    auto it = contacts_.find(follow_id_);
    if (it == contacts_.end()) return nullptr;
    return &it->second;
}

void ContactRenderer::setFollowId(int id) {
    follow_id_ = id;
}

void ContactRenderer::nextFollow() {
    if (contacts_.empty()) return;

    auto it = contacts_.find(follow_id_);
    if (it == contacts_.end()) {
        follow_id_ = contacts_.begin()->first;
    } else {
        ++it;
        if (it == contacts_.end()) {
            follow_id_ = contacts_.begin()->first;
        } else {
            follow_id_ = it->first;
        }
    }
}

void ContactRenderer::prevFollow() {
    if (contacts_.empty()) return;

    auto it = contacts_.find(follow_id_);
    if (it == contacts_.end() || it == contacts_.begin()) {
        follow_id_ = contacts_.rbegin()->first;
    } else {
        --it;
        follow_id_ = it->first;
    }
}

void ContactRenderer::toggleTrails() {
    show_trails_ = !show_trails_;
    
    // If disabling trails, clear existing trails
    if (!show_trails_) {
        for (auto& [id, rc] : contacts_) {
            for (auto* node : rc.trail) {
                if (node->numAttachedObjects() > 0) {
                    Ogre::MovableObject* obj = node->getAttachedObject(0);
                    node->detachObject(obj);
                    scene_mgr_->destroyManualObject(static_cast<Ogre::ManualObject*>(obj));
                }
                scene_mgr_->destroySceneNode(node);
            }
            rc.trail.clear();
        }
    }
}

void ContactRenderer::setScale(double scale) {
    scale_ = scale;
    for (auto& [id, rc] : contacts_) {
        rc.sceneNode->setScale(CoordinateConverter::toOgreScale(scale));
    }
}

std::vector<int> ContactRenderer::getContactIds() const {
    std::vector<int> ids;
    ids.reserve(contacts_.size());
    for (const auto& [id, _] : contacts_) {
        ids.push_back(id);
    }
    return ids;
}

void ContactRenderer::clear() {
    std::vector<int> ids;
    for (const auto& [id, _] : contacts_) {
        ids.push_back(id);
    }
    for (int id : ids) {
        removeContact(id);
    }
    contacts_.clear();
}

}  // namespace viewer
}  // namespace scrimmage
