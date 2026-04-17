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
 */

#ifndef SRC_VIEWER_OGRE_NEXT_OGRENEXTVIEWER_H_
#define SRC_VIEWER_OGRE_NEXT_OGRENEXTVIEWER_H_

#include <map>
#include <string>
#include <thread>

#include "scrimmage/fwd_decl.h"

// Forward declarations for OGRE-Next types
namespace Ogre {
class Root;
class Window;
class SceneManager;
class Camera;
class CompositorWorkspace;
}  // namespace Ogre

namespace scrimmage {

// OGRE-Next viewer implementation mirroring VtkViewer functionality.
//
// Threading Model (CRITICAL)
//
// Scrimmage calls init() on the main thread, then spawns a viewer thread
// to call run(). This is problematic for OpenGL because GL contexts are
// thread-bound - a context created on one thread cannot be used for
// rendering from a different thread.
//
// Solution (same as VTK):
// - init(): Called on main thread. Only stores configuration and validates
//           environment (DISPLAY). Does NOT create any OGRE objects.
// - run():  Called on viewer thread. Creates all OGRE objects (including
//           the GL context via createRenderWindow) and runs the render loop.
//           This ensures the GL context is created and used on the same thread.
//
// If init() created OGRE objects, run() would see a black window because
// the GL context would be bound to the wrong thread.
//
// Initialization Order (in initOgre, called from run)
//
// Follows OGRE-Next 2.3 best practices:
//  1. Create Root with plugins.cfg
//  2. Setup RenderSystem (GL3Plus for Linux)
//  3. Initialize Root (don't create window yet)
//  4. Create RenderWindow with gamma correction (creates GL context)
//  5. Create SceneManager (single-threaded for now)
//  6. Register HLMS (MUST be before initialiseAllResourceGroups)
//  7. Initialize resource groups
//  8. Create Camera
//  9. Setup Compositor workspace (required for ALL rendering in OGRE-Next)
// 10. Setup scene content (lights, camera position)
//
class OgreNextViewer {
 public:
    OgreNextViewer();
    ~OgreNextViewer();

    void set_incoming_interface(InterfacePtr& incoming_interface);
    void set_outgoing_interface(InterfacePtr& outgoing_interface);
    void set_enable_network(bool enable);

    // Store configuration only. Does NOT create OGRE objects.
    bool init(const MissionParsePtr& mp,
              const std::map<std::string, std::string>& camera_params);
    
    // Create OGRE objects and run render loop. Must be called from the
    // viewer thread to ensure GL context is created on the same thread.
    bool run();

 protected:
    // Resolve the OGRE data directory path.
    std::string resolveResourcePath();

    // Register HlmsUnlit and HlmsPbs from the Hlms/ directory.
    bool registerHlms();

    // Setup the Compositor workspace (required for any rendering in OGRE-Next)
    void setupCompositor();

    // Create initial scene content (lights, etc.)
    void setupScene();
    
    // Perform actual OGRE initialization. Called from run() on render thread.
    bool initOgre();

    // OGRE core objects (created in initOgre, destroyed in destructor)
    Ogre::Root* mRoot = nullptr;
    Ogre::Window* mWindow = nullptr;
    Ogre::SceneManager* mSceneManager = nullptr;
    Ogre::Camera* mCamera = nullptr;
    Ogre::CompositorWorkspace* mWorkspace = nullptr;

    // Scrimmage integration
    InterfacePtr incoming_interface_;
    InterfacePtr outgoing_interface_;
    bool enable_network_ = false;
    std::thread network_thread_;

    // Configuration (stored in init(), used in initOgre())
    std::string mResourcePath;
    std::map<std::string, std::string> camera_params_;
    std::string log_dir_;
    double dt_ = 0.1;
    int window_width_ = 1280;
    int window_height_ = 720;

    // Network params (mirror VtkViewer)
    std::string local_ip_ = "localhost";
    int local_port_ = 50051;
    std::string remote_ip_ = "localhost";
    int remote_port_ = 50052;

    double init_scale_ = 1.0;
    bool full_screen_ = false;
    bool mQuit = false;
    bool mInitialized = false;  // Track if initOgre() succeeded
};

}  // namespace scrimmage

#endif  // SRC_VIEWER_OGRE_NEXT_OGRENEXTVIEWER_H_
