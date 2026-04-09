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

#ifndef INCLUDE_SCRIMMAGE_VIEWER_OGRE_CAMERACONTROLLER_H_
#define INCLUDE_SCRIMMAGE_VIEWER_OGRE_CAMERACONTROLLER_H_

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreFrameListener.h>
#include <OGRE/Bites/OgreCameraMan.h>
#include "scrimmage/viewer/ogre/ContactRenderer.h"

namespace scrimmage {
namespace viewer {

/*
 *
 */
enum class ViewMode {
    FOLLOW = 0,  // Track entity position
    FREE,        // User-controlled camera
    OFFSET,      // Fixed offset from target
    FPV          // First-person view
};

/*
 *
 */
struct CameraResetParams {
    double pos_x = 0;
    double pos_y = 0;
    double pos_z = 200;
    double focal_x = 0;
    double focal_y = 0;
    double focal_z = 0;
};

/*
 *
 *
 * Provides multiple view modes: Follow, Free, Offset, FPV.
 * Uses Ogre3D FrameListener for per-frame updates.
 */
class CameraController : public Ogre::FrameListener {
 public:
    CameraController(Ogre::Camera* camera, Ogre::SceneNode* camNode,
                     ContactRenderer* contactRenderer);
    ~CameraController();

    /*
     *
     */
    void init();

    /*
     *
     */
    void setResetParams(const CameraResetParams& params);

    /*
     *
     */
    void resetCamera();

    /*
     *
     */
    ViewMode getViewMode() const { return view_mode_; }

    /*
     *
     */
    void setViewMode(ViewMode mode);

    /*
     *
     */
    void nextMode();

    /*
     *
     */
    double getFollowOffset() const { return follow_offset_; }

    /*
     *
     */
    void incFollowOffset();

    /*
     *
     */
    void decFollowOffset();

    /*
     *
     */
    void trackCameraPos();

    /*
     *
     */
    void undoCamera();

    /*
     *
     * Behavior depends on which mouse button is pressed:
     * - Left button: Orbit around target/pivot
     * - Middle button: Pan camera
     * - Right button: Free look (rotate in place)
     */
    void mouseMoved(float relX, float relY);

    /*
     *
     */
    void mousePressed(int button);

    /*
     *
     */
    void mouseReleased(int button);

    /*
     *
     */
    void mouseWheel(float delta);

    /*
     *
     */
    bool isDragging() const { return left_button_down_ || middle_button_down_ || right_button_down_; }

    /*
     *
     */
    void injectKeyDown(int key);
    void injectKeyUp(int key);

    /*
     *
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
    double follow_smooth_ = 8.0;  // Match entity interpolation rate

    // User-adjustable view offsets (preserved within each mode)
    float user_yaw_ = 0.0f;    // Horizontal orbit angle around target
    float user_pitch_ = 0.0f;  // Vertical angle adjustment

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

    // Mouse button state
    bool left_button_down_ = false;
    bool middle_button_down_ = false;
    bool right_button_down_ = false;

    // Orbit pivot point
    Ogre::Vector3 orbit_pivot_ = Ogre::Vector3::ZERO;
    double orbit_distance_ = 200.0;

    // Smoothed look target to reduce jitter
    Ogre::Vector3 smoothed_look_target_ = Ogre::Vector3::ZERO;
    bool look_target_initialized_ = false;
};

}  // namespace viewer
}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_VIEWER_OGRE_CAMERACONTROLLER_H_
