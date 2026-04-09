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
 * @brief Ogre3D-based viewer for SCRIMMAGE.
 */

#ifndef INCLUDE_SCRIMMAGE_VIEWER_OGRE_OGREVIEWER_H_
#define INCLUDE_SCRIMMAGE_VIEWER_OGRE_OGREVIEWER_H_

#include <OGRE/Ogre.h>
#include <OGRE/Bites/OgreApplicationContext.h>
#include <OGRE/Bites/OgreInput.h>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "scrimmage/viewer/ogre/CameraController.h"
#include "scrimmage/viewer/ogre/ContactRenderer.h"
#include "scrimmage/viewer/ogre/MaterialPool.h"
#include "scrimmage/viewer/ogre/RenderCommand.h"
#include "scrimmage/viewer/ogre/ShapeRenderer.h"

namespace scrimmage {

class MissionParse;
class Interface;
using InterfacePtr = std::shared_ptr<Interface>;

namespace viewer {

/**
 * @brief Ogre3D-based viewer for SCRIMMAGE simulation.
 *
 * This class replaces the VTK-based Viewer. It uses OgreBites::ApplicationContext
 * for window management and input handling.
 */
class OgreViewer : public OgreBites::ApplicationContext,
                   public OgreBites::InputListener {
 public:
    OgreViewer();
    ~OgreViewer();

    /**
     * @brief Set the incoming interface (GUI -> Simulation).
     */
    void set_incoming_interface(InterfacePtr& incoming_interface);

    /**
     * @brief Set the outgoing interface (Simulation -> GUI).
     */
    void set_outgoing_interface(InterfacePtr& outgoing_interface);

    /**
     * @brief Enable network mode.
     */
    void set_enable_network(bool enable);

    /**
     * @brief Initialize the viewer.
     */
    bool init(const std::shared_ptr<MissionParse>& mp,
              const std::map<std::string, std::string>& camera_params);

    /**
     * @brief Run the viewer (blocking).
     */
    bool run();

    /**
     * @brief Request shutdown.
     */
    void shutdown();

    // OgreBites::ApplicationContext overrides
    void setup() override;

    // OgreBites::InputListener overrides
    bool keyPressed(const OgreBites::KeyboardEvent& evt) override;
    bool keyReleased(const OgreBites::KeyboardEvent& evt) override;
    bool mouseMoved(const OgreBites::MouseMotionEvent& evt) override;
    bool mousePressed(const OgreBites::MouseButtonEvent& evt) override;
    bool mouseReleased(const OgreBites::MouseButtonEvent& evt) override;
    bool mouseWheelRolled(const OgreBites::MouseWheelEvent& evt) override;

 protected:
    /**
     * @brief Process pending interface updates.
     */
    void processInterfaceUpdates();

    /**
     * @brief Update the display (called each frame).
     */
    void update(double dt);

    /**
     * @brief Create the scene (lights, grid, etc.).
     */
    void createScene();

    /**
     * @brief Create the grid.
     */
    void createGrid();

    /**
     * @brief Create origin axes.
     */
    void createOriginAxes();

    /**
     * @brief Send GUI message (pause, step, etc.).
     */
    void sendGuiMsg(const std::string& type, int value = 0);

 private:
    Ogre::SceneManager* scene_mgr_ = nullptr;
    Ogre::Camera* camera_ = nullptr;
    Ogre::SceneNode* cam_node_ = nullptr;
    Ogre::RenderWindow* render_window_ = nullptr;

    std::unique_ptr<MaterialPool> material_pool_;
    std::unique_ptr<ContactRenderer> contact_renderer_;
    std::unique_ptr<ShapeRenderer> shape_renderer_;
    std::unique_ptr<CameraController> camera_controller_;

    InterfacePtr incoming_interface_;
    InterfacePtr outgoing_interface_;

    bool enable_network_ = false;
    std::thread network_thread_;

    std::map<std::string, std::string> camera_params_;
    std::string log_dir_;
    double dt_ = 0.1;
    double frame_time_ = 0.0;
    double scale_ = 1.0;
    int warp_ = 1;
    bool paused_ = false;
    bool shutting_down_ = false;
    bool show_grid_ = true;
    bool show_origin_ = true;
    bool full_screen_ = false;

    // Frame listener class to process updates
    class UpdateListener : public Ogre::FrameListener {
     public:
        explicit UpdateListener(OgreViewer* viewer) : viewer_(viewer) {}
        bool frameStarted(const Ogre::FrameEvent& evt) override;
        bool frameRenderingQueued(const Ogre::FrameEvent& evt) override;
     private:
        OgreViewer* viewer_;
    };

    std::unique_ptr<UpdateListener> update_listener_;
};

}  // namespace viewer
}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_VIEWER_OGRE_OGREVIEWER_H_
