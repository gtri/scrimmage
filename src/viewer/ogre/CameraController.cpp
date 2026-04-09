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
 * @brief Camera controller implementation.
 */

#include "scrimmage/viewer/ogre/CameraController.h"
#include "scrimmage/viewer/ogre/CoordinateConverter.h"
#include <algorithm>

namespace scrimmage {
namespace viewer {

CameraController::CameraController(Ogre::Camera* camera, Ogre::SceneNode* camNode,
                                   ContactRenderer* contactRenderer)
    : camera_(camera), cam_node_(camNode), contact_renderer_(contactRenderer) {
}

CameraController::~CameraController() {
}

void CameraController::init() {
    // Set default camera position
    resetCamera();
}

void CameraController::setResetParams(const CameraResetParams& params) {
    reset_params_ = params;
}

void CameraController::resetCamera() {
    cam_node_->setPosition(CoordinateConverter::toOgre(
        reset_params_.pos_x, reset_params_.pos_y, reset_params_.pos_z));
    
    Ogre::Vector3 focal = CoordinateConverter::toOgre(
        reset_params_.focal_x, reset_params_.focal_y, reset_params_.focal_z);
    cam_node_->lookAt(focal, Ogre::Node::TS_WORLD);
    
    yaw_ = 0.0f;
    pitch_ = 0.0f;
}

void CameraController::setViewMode(ViewMode mode) {
    view_mode_ = mode;
}

void CameraController::nextMode() {
    switch (view_mode_) {
        case ViewMode::FOLLOW:
            view_mode_ = ViewMode::FREE;
            break;
        case ViewMode::FREE:
            view_mode_ = ViewMode::OFFSET;
            break;
        case ViewMode::OFFSET:
            view_mode_ = ViewMode::FPV;
            break;
        case ViewMode::FPV:
            view_mode_ = ViewMode::FOLLOW;
            break;
    }
}

void CameraController::incFollowOffset() {
    follow_offset_ *= 1.2;
}

void CameraController::decFollowOffset() {
    follow_offset_ /= 1.2;
    if (follow_offset_ < 10.0) follow_offset_ = 10.0;
}

void CameraController::trackCameraPos() {
    position_history_.push_back(cam_node_->getPosition());
    orientation_history_.push_back(cam_node_->getOrientation());
    
    if (position_history_.size() > MAX_HISTORY) {
        position_history_.erase(position_history_.begin());
        orientation_history_.erase(orientation_history_.begin());
    }
}

void CameraController::undoCamera() {
    if (position_history_.empty()) return;
    
    cam_node_->setPosition(position_history_.back());
    cam_node_->setOrientation(orientation_history_.back());
    
    position_history_.pop_back();
    orientation_history_.pop_back();
}

void CameraController::mouseMoved(float relX, float relY) {
    if (view_mode_ != ViewMode::FREE) return;
    
    yaw_ -= relX * rotate_speed_;
    pitch_ -= relY * rotate_speed_;
    
    // Clamp pitch to avoid gimbal lock
    pitch_ = std::clamp(pitch_, -89.0f, 89.0f);
    
    Ogre::Quaternion yawRot(Ogre::Degree(yaw_), Ogre::Vector3::UNIT_Y);
    Ogre::Quaternion pitchRot(Ogre::Degree(pitch_), Ogre::Vector3::UNIT_X);
    
    cam_node_->setOrientation(yawRot * pitchRot);
}

void CameraController::mouseWheel(float delta) {
    if (view_mode_ == ViewMode::FREE) {
        move_speed_ += delta * 10.0f;
        move_speed_ = std::max(10.0f, move_speed_);
    } else {
        if (delta > 0) {
            decFollowOffset();
        } else {
            incFollowOffset();
        }
    }
}

void CameraController::injectKeyDown(int key) {
    // WASD + QE movement
    switch (key) {
        case 'w': case 'W': move_forward_ = true; break;
        case 's': case 'S': move_back_ = true; break;
        case 'a': case 'A': move_left_ = true; break;
        case 'd': case 'D': move_right_ = true; break;
        case 'q': case 'Q': move_up_ = true; break;
        case 'e': case 'E': move_down_ = true; break;
    }
}

void CameraController::injectKeyUp(int key) {
    switch (key) {
        case 'w': case 'W': move_forward_ = false; break;
        case 's': case 'S': move_back_ = false; break;
        case 'a': case 'A': move_left_ = false; break;
        case 'd': case 'D': move_right_ = false; break;
        case 'q': case 'Q': move_up_ = false; break;
        case 'e': case 'E': move_down_ = false; break;
    }
}

bool CameraController::frameRenderingQueued(const Ogre::FrameEvent& evt) {
    float dt = evt.timeSinceLastFrame;
    
    switch (view_mode_) {
        case ViewMode::FOLLOW:
            updateFollowCamera(dt);
            break;
        case ViewMode::FREE:
            updateFreeCamera(dt);
            break;
        case ViewMode::OFFSET:
            updateOffsetCamera(dt);
            break;
        case ViewMode::FPV:
            updateFPVCamera(dt);
            break;
    }
    
    return true;
}

void CameraController::updateFollowCamera(float dt) {
    RenderedContact* target = contact_renderer_->getFollowedContact();
    if (!target || !target->sceneNode) return;
    
    Ogre::Vector3 targetPos = target->sceneNode->getPosition();
    
    // Calculate desired camera position (behind and above target)
    Ogre::Vector3 offset(0, static_cast<Ogre::Real>(follow_height_),
                         static_cast<Ogre::Real>(-follow_offset_));
    
    // Get target's forward direction (in Ogre space)
    Ogre::Quaternion targetOrient = target->sceneNode->getOrientation();
    
    Ogre::Vector3 desiredPos = targetPos + targetOrient * offset;
    
    // Smooth interpolation
    Ogre::Vector3 currentPos = cam_node_->getPosition();
    Ogre::Vector3 newPos = currentPos + (desiredPos - currentPos) * 
                           static_cast<Ogre::Real>(follow_smooth_ * dt);
    
    cam_node_->setPosition(newPos);
    cam_node_->lookAt(targetPos, Ogre::Node::TS_WORLD);
}

void CameraController::updateFreeCamera(float dt) {
    Ogre::Vector3 move(0, 0, 0);
    
    if (move_forward_) move.z -= 1;
    if (move_back_) move.z += 1;
    if (move_left_) move.x -= 1;
    if (move_right_) move.x += 1;
    if (move_up_) move.y += 1;
    if (move_down_) move.y -= 1;
    
    if (move.length() > 0) {
        move.normalise();
        move *= move_speed_ * dt;
        
        // Transform movement to camera's local space
        cam_node_->translate(move, Ogre::Node::TS_LOCAL);
    }
}

void CameraController::updateOffsetCamera(float dt) {
    RenderedContact* target = contact_renderer_->getFollowedContact();
    if (!target || !target->sceneNode) return;
    
    Ogre::Vector3 targetPos = target->sceneNode->getPosition();
    
    // Fixed offset from target (world-aligned)
    Ogre::Vector3 offset(0, static_cast<Ogre::Real>(follow_height_),
                         static_cast<Ogre::Real>(-follow_offset_));
    
    cam_node_->setPosition(targetPos + offset);
    cam_node_->lookAt(targetPos, Ogre::Node::TS_WORLD);
}

void CameraController::updateFPVCamera(float dt) {
    RenderedContact* target = contact_renderer_->getFollowedContact();
    if (!target || !target->sceneNode) return;
    
    // Place camera at entity position
    Ogre::Vector3 targetPos = target->sceneNode->getPosition();
    Ogre::Quaternion targetOrient = target->sceneNode->getOrientation();
    
    // Slight offset forward and up for FPV
    Ogre::Vector3 fpvOffset = targetOrient * Ogre::Vector3(0, 2, 5);
    
    cam_node_->setPosition(targetPos + fpvOffset);
    cam_node_->setOrientation(targetOrient);
}

}  // namespace viewer
}  // namespace scrimmage
