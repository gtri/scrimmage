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
#include <cmath>

namespace scrimmage {
namespace viewer {

CameraController::CameraController(Ogre::Camera* camera, Ogre::SceneNode* camNode,
                                   ContactRenderer* contactRenderer)
    : camera_(camera), cam_node_(camNode), contact_renderer_(contactRenderer) {
}

CameraController::~CameraController() {
}

void CameraController::init() {
    // Set fixed yaw axis to prevent camera flipping
    // Ogre's Y is up (which corresponds to SCRIMMAGE's Z/Up)
    cam_node_->setFixedYawAxis(true, Ogre::Vector3::UNIT_Y);
    
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
    // Reset user offsets and look target when mode changes
    user_yaw_ = 0.0f;
    user_pitch_ = 0.0f;
    look_target_initialized_ = false;
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
    // Reset user offsets and look target when cycling modes
    user_yaw_ = 0.0f;
    user_pitch_ = 0.0f;
    look_target_initialized_ = false;
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

void CameraController::mousePressed(int button) {
    // Button: 1 = left, 2 = middle, 3 = right
    switch (button) {
        case 1:
            left_button_down_ = true;
            if (view_mode_ == ViewMode::FREE) {
                // Set up orbit pivot - point in front of camera
                orbit_pivot_ = cam_node_->getPosition() + 
                              cam_node_->getOrientation() * Ogre::Vector3(0, 0, -100);
                orbit_distance_ = (cam_node_->getPosition() - orbit_pivot_).length();
            }
            break;
        case 2:
            middle_button_down_ = true;
            break;
        case 3:
            right_button_down_ = true;
            break;
    }
}

void CameraController::mouseReleased(int button) {
    switch (button) {
        case 1: left_button_down_ = false; break;
        case 2: middle_button_down_ = false; break;
        case 3: right_button_down_ = false; break;
    }
}

void CameraController::mouseMoved(float relX, float relY) {
    // In FOLLOW/OFFSET/FPV modes, adjust user view offsets
    if (view_mode_ != ViewMode::FREE) {
        if (left_button_down_ || right_button_down_) {
            // Orbit/look: adjust yaw and pitch
            user_yaw_ -= relX * rotate_speed_;
            user_pitch_ -= relY * rotate_speed_;
            // Clamp pitch
            user_pitch_ = std::clamp(user_pitch_, -60.0f, 60.0f);
        }
        if (middle_button_down_) {
            // Pan: adjust follow offset (height and distance)
            follow_height_ += relY * 0.5;
            follow_offset_ += relX * 0.5;
            follow_height_ = std::max(5.0, follow_height_);
            follow_offset_ = std::max(10.0, follow_offset_);
        }
        return;
    }
    
    // FREE mode: full camera control
    // Left button: Orbit around pivot
    if (left_button_down_) {
        // Orbit the camera around the pivot point
        Ogre::Vector3 camPos = cam_node_->getPosition();
        Ogre::Vector3 offset = camPos - orbit_pivot_;
        
        // Apply yaw (horizontal rotation around Y axis)
        Ogre::Quaternion yawRot(Ogre::Degree(-relX * rotate_speed_), Ogre::Vector3::UNIT_Y);
        offset = yawRot * offset;
        
        // Apply pitch (vertical rotation) but keep camera above ground
        Ogre::Vector3 right = cam_node_->getOrientation() * Ogre::Vector3::UNIT_X;
        Ogre::Quaternion pitchRot(Ogre::Degree(-relY * rotate_speed_), right);
        Ogre::Vector3 newOffset = pitchRot * offset;
        
        // Only apply pitch if it doesn't flip camera
        if (newOffset.y > 5.0f || relY < 0) {
            offset = newOffset;
        }
        
        cam_node_->setPosition(orbit_pivot_ + offset);
        cam_node_->lookAt(orbit_pivot_, Ogre::Node::TS_WORLD);
        return;
    }
    
    // Middle button: Pan (translate camera)
    if (middle_button_down_) {
        Ogre::Vector3 right = cam_node_->getOrientation() * Ogre::Vector3::UNIT_X;
        Ogre::Vector3 up = cam_node_->getOrientation() * Ogre::Vector3::UNIT_Y;
        
        float panSpeed = move_speed_ * 0.01f;
        cam_node_->translate(-right * relX * panSpeed + up * relY * panSpeed, Ogre::Node::TS_WORLD);
        
        // Also move orbit pivot so subsequent orbits work correctly
        orbit_pivot_ += (-right * relX * panSpeed + up * relY * panSpeed);
        return;
    }
    
    // Right button: Free look (rotate in place)
    if (right_button_down_) {
        yaw_ -= relX * rotate_speed_;
        pitch_ -= relY * rotate_speed_;
        
        // Clamp pitch to avoid gimbal lock
        pitch_ = std::clamp(pitch_, -89.0f, 89.0f);
        
        Ogre::Quaternion yawRot(Ogre::Degree(yaw_), Ogre::Vector3::UNIT_Y);
        Ogre::Quaternion pitchRot(Ogre::Degree(pitch_), Ogre::Vector3::UNIT_X);
        
        cam_node_->setOrientation(yawRot * pitchRot);
        return;
    }
}

void CameraController::mouseWheel(float delta) {
    // Zoom: move camera closer/farther from orbit pivot
    Ogre::Vector3 direction = cam_node_->getOrientation() * Ogre::Vector3::NEGATIVE_UNIT_Z;
    
    // Zoom amount scales with distance
    float zoomAmount = delta * move_speed_ * 0.5f;
    
    if (view_mode_ == ViewMode::FREE) {
        // In free mode, just move forward/back
        cam_node_->translate(direction * zoomAmount, Ogre::Node::TS_WORLD);
    } else {
        // In follow modes, adjust the offset distance
        if (delta > 0) {
            decFollowOffset();
        } else {
            incFollowOffset();
        }
        // Also update orbit distance
        orbit_distance_ = std::max(10.0, orbit_distance_ - delta * 10.0);
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
    
    // Entity position is already interpolated smoothly by ContactRenderer
    Ogre::Vector3 targetPos = target->sceneNode->getPosition();
    
    // Calculate desired camera position (behind and above target)
    // Use world-space offset: behind (-Z in Ogre) and above (+Y)
    Ogre::Vector3 offset(0, static_cast<Ogre::Real>(follow_height_),
                         static_cast<Ogre::Real>(follow_offset_));
    
    // Extract only yaw from target orientation to avoid camera flipping
    // when target pitches or rolls
    Ogre::Quaternion targetOrient = target->sceneNode->getOrientation();
    Ogre::Radian yaw = targetOrient.getYaw();
    Ogre::Quaternion yawOnly(yaw, Ogre::Vector3::UNIT_Y);
    
    // Apply user's orbit adjustment on top of entity heading
    Ogre::Quaternion userYaw(Ogre::Degree(user_yaw_), Ogre::Vector3::UNIT_Y);
    Ogre::Quaternion userPitch(Ogre::Degree(user_pitch_), Ogre::Vector3::UNIT_X);
    
    // Apply yaw-only rotation + user adjustments to offset
    Ogre::Vector3 desiredPos = targetPos + yawOnly * userYaw * userPitch * offset;
    
    // Smooth camera position
    float smoothFactor = 1.0f - std::exp(-static_cast<float>(follow_smooth_) * dt);
    smoothFactor = std::clamp(smoothFactor, 0.0f, 1.0f);
    
    Ogre::Vector3 currentPos = cam_node_->getPosition();
    Ogre::Vector3 newPos = currentPos + (desiredPos - currentPos) * smoothFactor;
    
    cam_node_->setPosition(newPos);
    // Look directly at interpolated entity position (already smooth)
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
    
    // Entity position is already interpolated smoothly
    Ogre::Vector3 targetPos = target->sceneNode->getPosition();
    
    // World-aligned fixed offset from target (doesn't follow target heading)
    Ogre::Vector3 offset(0, static_cast<Ogre::Real>(follow_height_),
                         static_cast<Ogre::Real>(follow_offset_));
    
    // Apply user's orbit adjustment
    Ogre::Quaternion userYaw(Ogre::Degree(user_yaw_), Ogre::Vector3::UNIT_Y);
    Ogre::Quaternion userPitch(Ogre::Degree(user_pitch_), Ogre::Vector3::UNIT_X);
    
    Ogre::Vector3 desiredPos = targetPos + userYaw * userPitch * offset;
    
    // Smooth camera position
    float smoothFactor = 1.0f - std::exp(-static_cast<float>(follow_smooth_) * dt);
    smoothFactor = std::clamp(smoothFactor, 0.0f, 1.0f);
    
    Ogre::Vector3 currentPos = cam_node_->getPosition();
    Ogre::Vector3 newPos = currentPos + (desiredPos - currentPos) * smoothFactor;
    
    cam_node_->setPosition(newPos);
    cam_node_->lookAt(targetPos, Ogre::Node::TS_WORLD);
}

void CameraController::updateFPVCamera(float dt) {
    RenderedContact* target = contact_renderer_->getFollowedContact();
    if (!target || !target->sceneNode) return;
    
    // Entity position/orientation are already interpolated smoothly
    Ogre::Vector3 targetPos = target->sceneNode->getPosition();
    Ogre::Quaternion targetOrient = target->sceneNode->getOrientation();
    
    // After coordinate conversion, entity's forward direction is UNIT_X
    Ogre::Vector3 entityForward = targetOrient * Ogre::Vector3::UNIT_X;
    Ogre::Vector3 entityUp = Ogre::Vector3::UNIT_Y;  // World up
    
    // FPV: slightly behind and above the entity, looking forward
    Ogre::Vector3 fpvOffset = -entityForward * 15 + entityUp * 5;
    Ogre::Vector3 desiredPos = targetPos + fpvOffset;
    
    // Smooth camera position
    float smoothFactor = 1.0f - std::exp(-static_cast<float>(follow_smooth_) * dt);
    smoothFactor = std::clamp(smoothFactor, 0.0f, 1.0f);
    
    Ogre::Vector3 currentPos = cam_node_->getPosition();
    Ogre::Vector3 newPos = currentPos + (desiredPos - currentPos) * smoothFactor;
    
    cam_node_->setPosition(newPos);
    
    // Look in the direction the entity is heading
    Ogre::Vector3 lookTarget = targetPos + entityForward * 100;
    
    // User yaw/pitch adjusts look direction
    Ogre::Quaternion userYaw(Ogre::Degree(user_yaw_), Ogre::Vector3::UNIT_Y);
    Ogre::Quaternion userPitch(Ogre::Degree(user_pitch_), Ogre::Vector3::UNIT_X);
    Ogre::Vector3 lookDir = lookTarget - cam_node_->getPosition();
    lookDir = userYaw * userPitch * lookDir;
    
    cam_node_->lookAt(cam_node_->getPosition() + lookDir, Ogre::Node::TS_WORLD);
}

}  // namespace viewer
}  // namespace scrimmage
