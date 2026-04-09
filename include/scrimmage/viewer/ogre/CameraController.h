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
 * @brief Camera controller for Ogre3D viewer.
 */

#ifndef INCLUDE_SCRIMMAGE_VIEWER_OGRE_CAMERACONTROLLER_H_
#define INCLUDE_SCRIMMAGE_VIEWER_OGRE_CAMERACONTROLLER_H_

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreFrameListener.h>
#include <OGRE/Bites/OgreCameraMan.h>
#include "scrimmage/viewer/ogre/ContactRenderer.h"

namespace scrimmage {
namespace viewer {

/**
 * @brief Camera view modes.
 */
enum class ViewMode {
    FOLLOW = 0,  // Track entity position
    FREE,        // User-controlled camera
    OFFSET,      // Fixed offset from target
    FPV          // First-person view
};

/**
 * @brief Camera reset parameters.
 */
struct CameraResetParams {
    double pos_x = 0;
    double pos_y = 0;
    double pos_z = 200;
    double focal_x = 0;
    double focal_y = 0;
    double focal_z = 0;
};

/**
 * @brief Camera controller for SCRIMMAGE viewer.
 *
 * Provides multiple view modes: Follow, Free, Offset, FPV.
 * Uses Ogre3D FrameListener for per-frame updates.
 */
class CameraController : public Ogre::FrameListener {
 public:
    CameraController(Ogre::Camera* camera, Ogre::SceneNode* camNode,
                     ContactRenderer* contactRenderer);
    ~CameraController();

    /**
     * @brief Initialize the camera controller.
     */
    void init();

    /**
     * @brief Set camera reset parameters.
     */
    void setResetParams(const CameraResetParams& params);

    /**
     * @brief Reset camera to initial position.
     */
    void resetCamera();

    /**
     * @brief Get current view mode.
     */
    ViewMode getViewMode() const { return view_mode_; }

    /**
     * @brief Set view mode.
     */
    void setViewMode(ViewMode mode);

    /**
     * @brief Cycle to next view mode.
     */
    void nextMode();

    /**
     * @brief Get follow offset distance.
     */
    double getFollowOffset() const { return follow_offset_; }

    /**
     * @brief Increase follow offset.
     */
    void incFollowOffset();

    /**
     * @brief Decrease follow offset.
     */
    void decFollowOffset();

    /**
     * @brief Store current camera position for undo.
     */
    void trackCameraPos();

    /**
     * @brief Undo last camera movement.
     */
    void undoCamera();

    /**
     * @brief Process mouse movement (for free camera).
     */
    void mouseMoved(float relX, float relY);

    /**
     * @brief Process mouse wheel (zoom).
     */
    void mouseWheel(float delta);

    /**
     * @brief Process keyboard movement input.
     */
    void injectKeyDown(int key);
    void injectKeyUp(int key);

    /**
     * @brief FrameListener callback - called each frame.
     */
    bool frameRenderingQueued(const Ogre::FrameEvent& evt) override;

 private:
    void updateFollowCamera(float dt);
    void updateFreeCamera(float dt);
    void updateOffsetCamera(float dt);
    void updateFPVCamera(float dt);

    Ogre::Camera* camera_;
    Ogre::SceneNode* cam_node_;
    ContactRenderer* contact_renderer_;

    ViewMode view_mode_ = ViewMode::FOLLOW;
    CameraResetParams reset_params_;

    // Follow mode parameters
    double follow_offset_ = 100.0;
    double follow_height_ = 50.0;
    double follow_smooth_ = 5.0;

    // Free camera state
    float move_speed_ = 100.0f;
    float rotate_speed_ = 0.3f;
    float yaw_ = 0.0f;
    float pitch_ = 0.0f;
    bool move_forward_ = false;
    bool move_back_ = false;
    bool move_left_ = false;
    bool move_right_ = false;
    bool move_up_ = false;
    bool move_down_ = false;

    // Camera history for undo
    std::vector<Ogre::Vector3> position_history_;
    std::vector<Ogre::Quaternion> orientation_history_;
    static const size_t MAX_HISTORY = 10;
};

}  // namespace viewer
}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_VIEWER_OGRE_CAMERACONTROLLER_H_
