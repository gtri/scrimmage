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
 *   the terms of the GNU Lesser General Public as published by the
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

#include "OgreNextViewer.h"

#include <cstdlib>
#include <filesystem>
#include <iostream>

// OGRE-Next 2.3 headers
// NOTE: OGRE headers include X11/Xlib.h internally on Linux, which defines
// macros (Success, Status, etc.) that break Eigen and other libraries.
#include <OgreRoot.h>
#include <OgreRenderSystem.h>
#include <OgreWindow.h>
#include <OgreSceneManager.h>
#include <OgreCamera.h>
#include <OgreConfigFile.h>
#include <OgreArchiveManager.h>
#include <OgreHlmsManager.h>
#include <Hlms/Pbs/OgreHlmsPbs.h>
#include <Hlms/Unlit/OgreHlmsUnlit.h>
#include <OgreResourceGroupManager.h>
#include <OgreWindowEventUtilities.h>
#include <OgreLight.h>
#include <OgreMath.h>
#include <OgreTextureGpuManager.h>
#include <Compositor/OgreCompositorManager2.h>
#include <Compositor/OgreCompositorWorkspace.h>

// X11 for input handling - included after OGRE (which already includes Xlib)
#include <X11/Xlib.h>
#include <X11/keysym.h>

// Undefine X11 macros that conflict with Eigen and other libraries.
// X11/X.h defines these as macros but Eigen uses them as enum values.
#ifdef Success
#undef Success
#endif
#ifdef Status
#undef Status  
#endif
#ifdef None
#undef None
#endif
#ifdef Bool
#undef Bool
#endif
#ifdef True
#undef True
#endif
#ifdef False
#undef False
#endif

// Now safe to include scrimmage headers that use Eigen
#include "scrimmage/parse/MissionParse.h"
#include "scrimmage/parse/ParseUtils.h"
#include "scrimmage/network/Interface.h"

namespace scrimmage {

OgreNextViewer::OgreNextViewer() : enable_network_(false) {}

OgreNextViewer::~OgreNextViewer() {
    // OGRE-Next cleanup is complex and can cause crashes due to:
    // - HLMS objects being deleted by HlmsManager
    // - GL context issues when destroying from wrong thread
    // - Order-dependent destruction of internal OGRE objects
    //
    // For now, we let the OS clean up on process exit which is safe.
    // The process is ending anyway, so no memory is actually leaked.
    // This mirrors how many graphics applications handle cleanup.
    //
    // TODO: Implement proper shutdown sequence if needed for cases where
    // the viewer is destroyed but the process continues.
    
    // Clear our borrowed pointers (we don't own these)
    x11_display_ = nullptr;
    x11_window_ = 0;
    mWorkspace = nullptr;
    mSceneManager = nullptr;
    mWindow = nullptr;
    mCamera = nullptr;
    mRoot = nullptr;  // Don't delete - let OS cleanup
}

void OgreNextViewer::set_incoming_interface(InterfacePtr& incoming_interface) {
    incoming_interface_ = incoming_interface;
}

void OgreNextViewer::set_outgoing_interface(InterfacePtr& outgoing_interface) {
    outgoing_interface_ = outgoing_interface;
}

void OgreNextViewer::set_enable_network(bool enable) {
    enable_network_ = enable;
}

std::string OgreNextViewer::resolveResourcePath() {
    // Priority 1: Environment variable
    const char* envPath = std::getenv("SCRIMMAGE_OGRE_MEDIA_DIR");
    if (envPath && envPath[0] != '\0') {
        std::string path(envPath);
        if (path.back() != '/') path += '/';
        return path;
    }

    // Priority 2: Check for data/ogre_next/ relative to current dir (development)
    // User may run from source root or build directory
    if (std::filesystem::exists("./data/ogre_next/plugins.cfg")) {
        return "./data/ogre_next/";
    }
    if (std::filesystem::exists("../data/ogre_next/plugins.cfg")) {
        return "../data/ogre_next/";
    }
    
    // Priority 3: Check SCRIMMAGE_DATA_PATH
    const char* dataPath = std::getenv("SCRIMMAGE_DATA_PATH");
    if (dataPath && dataPath[0] != '\0') {
        std::string path = std::string(dataPath) + "/ogre_next/";
        if (std::filesystem::exists(path + "plugins.cfg")) {
            return path;
        }
    }

    // Priority 4: Relative to executable (installed layout)
    return "../share/scrimmage/ogre_next/";
}

bool OgreNextViewer::registerHlms() {
    Ogre::ConfigFile cfg;
    try {
        cfg.load(mResourcePath + "resources2.cfg");
    } catch (Ogre::Exception& e) {
        std::cerr << "Error: Could not load resources2.cfg from " << mResourcePath
                  << "\n  " << e.getDescription()
                  << "\n  Set SCRIMMAGE_OGRE_MEDIA_DIR or ensure data/ogre_next/ exists."
                  << std::endl;
        return false;
    }

    Ogre::String rootHlmsFolder = mResourcePath +
        cfg.getSetting("DoNotUseAsResource", "Hlms", "");

    if (rootHlmsFolder.empty()) {
        rootHlmsFolder = mResourcePath;
    } else if (rootHlmsFolder.back() != '/') {
        rootHlmsFolder += "/";
    }

    Ogre::ArchiveManager& archiveManager = Ogre::ArchiveManager::getSingleton();
    const Ogre::String archiveType = "FileSystem";

    // Register HlmsUnlit
    Ogre::String mainFolderPath;
    Ogre::StringVector libraryFoldersPaths;

    Ogre::HlmsUnlit::getDefaultPaths(mainFolderPath, libraryFoldersPaths);

    Ogre::Archive* archiveUnlit = nullptr;
    try {
        archiveUnlit = archiveManager.load(
            rootHlmsFolder + mainFolderPath, archiveType, true);
    } catch (Ogre::Exception& e) {
        std::cerr << "Error: Could not load HlmsUnlit from " << rootHlmsFolder + mainFolderPath
                  << "\n  " << e.getDescription()
                  << "\n  Ensure Hlms/Unlit/ exists in your OGRE media directory."
                  << std::endl;
        return false;
    }

    Ogre::ArchiveVec archiveUnlitLibraries;
    for (const Ogre::String& libraryPath : libraryFoldersPaths) {
        try {
            archiveUnlitLibraries.push_back(
                archiveManager.load(rootHlmsFolder + libraryPath, archiveType, true));
        } catch (Ogre::Exception& e) {
            std::cerr << "Warning: Could not load HLMS library " << rootHlmsFolder + libraryPath
                      << "\n  " << e.getDescription() << std::endl;
        }
    }

    Ogre::HlmsUnlit* hlmsUnlit = OGRE_NEW Ogre::HlmsUnlit(
        archiveUnlit, &archiveUnlitLibraries);
    mRoot->getHlmsManager()->registerHlms(hlmsUnlit);

    // Register HlmsPbs (same pattern)
    Ogre::HlmsPbs::getDefaultPaths(mainFolderPath, libraryFoldersPaths);

    Ogre::Archive* archivePbs = nullptr;
    try {
        archivePbs = archiveManager.load(
            rootHlmsFolder + mainFolderPath, archiveType, true);
    } catch (Ogre::Exception& e) {
        std::cerr << "Error: Could not load HlmsPbs from " << rootHlmsFolder + mainFolderPath
                  << "\n  " << e.getDescription()
                  << "\n  Ensure Hlms/Pbs/ exists in your OGRE media directory."
                  << std::endl;
        return false;
    }

    Ogre::ArchiveVec archivePbsLibraries;
    for (const Ogre::String& libraryPath : libraryFoldersPaths) {
        try {
            archivePbsLibraries.push_back(
                archiveManager.load(rootHlmsFolder + libraryPath, archiveType, true));
        } catch (Ogre::Exception& e) {
            std::cerr << "Warning: Could not load HLMS library " << rootHlmsFolder + libraryPath
                      << "\n  " << e.getDescription() << std::endl;
        }
    }

    Ogre::HlmsPbs* hlmsPbs = OGRE_NEW Ogre::HlmsPbs(
        archivePbs, &archivePbsLibraries);
    mRoot->getHlmsManager()->registerHlms(hlmsPbs);

    return true;
}

void OgreNextViewer::setupCompositor() {
    if (!mWindow) {
        std::cerr << "OGRE Error: Window is NULL in setupCompositor!" << std::endl;
        return;
    }

    Ogre::CompositorManager2* compositorMgr = mRoot->getCompositorManager2();
    const Ogre::String workspaceName("ScrimmageWorkspace");

    if (!compositorMgr->hasWorkspaceDefinition(workspaceName)) {
        // Use BRIGHT RED background - same as working standalone test
        Ogre::ColourValue backgroundColor(1.0f, 0.0f, 0.0f, 1.0f);  // RED
        
        compositorMgr->createBasicWorkspaceDef(
            workspaceName,
            backgroundColor,
            Ogre::IdString()  // No shadow node
        );
        
        std::cout << "OGRE: Created workspace definition with RED background" << std::endl;
    }

    // Get the window's render target texture
    Ogre::TextureGpu* renderTarget = mWindow->getTexture();
    if (!renderTarget) {
        std::cerr << "OGRE Error: Window texture is NULL!" << std::endl;
        return;
    }

    std::cout << "OGRE: Render target: " << renderTarget->getNameStr() 
              << " (" << renderTarget->getWidth() << "x" << renderTarget->getHeight() << ")" << std::endl;

    mWorkspace = compositorMgr->addWorkspace(
        mSceneManager,
        renderTarget,
        mCamera,
        workspaceName,
        true  // enabled
    );

    if (mWorkspace) {
        std::cout << "OGRE: Compositor workspace created successfully." << std::endl;
        std::cout << "OGRE: Workspace valid: " << mWorkspace->isValid() << std::endl;
    } else {
        std::cerr << "OGRE Error: Failed to create compositor workspace!" << std::endl;
    }
}

void OgreNextViewer::setupScene() {
    // Set ambient light so the scene isn't completely dark
    mSceneManager->setAmbientLight(
        Ogre::ColourValue(0.3f, 0.3f, 0.3f),  // Upper hemisphere
        Ogre::ColourValue(0.1f, 0.1f, 0.1f),  // Lower hemisphere
        Ogre::Vector3::UNIT_Y);

    // Create a directional light for basic scene illumination
    Ogre::Light* light = mSceneManager->createLight();
    Ogre::SceneNode* lightNode = mSceneManager->getRootSceneNode(Ogre::SCENE_DYNAMIC)
        ->createChildSceneNode(Ogre::SCENE_DYNAMIC);
    lightNode->attachObject(light);

    light->setType(Ogre::Light::LT_DIRECTIONAL);
    light->setPowerScale(Ogre::Math::PI);  // Required for LDR pipeline
    light->setDiffuseColour(1.0f, 1.0f, 0.9f);  // Slightly warm white
    lightNode->setDirection(Ogre::Vector3(-1, -1, -1).normalisedCopy());

    // Position camera - check if already attached to a node
    Ogre::SceneNode* cameraNode = mCamera->getParentSceneNode();
    if (!cameraNode) {
        // Camera not yet attached, create a new node
        cameraNode = mSceneManager->getRootSceneNode(Ogre::SCENE_DYNAMIC)
            ->createChildSceneNode(Ogre::SCENE_DYNAMIC);
        cameraNode->attachObject(mCamera);
    }
    cameraNode->setPosition(40, 40, 100);
    mCamera->lookAt(Ogre::Vector3(0, 0, 0));
}

bool OgreNextViewer::init(const MissionParsePtr& mp,
                          const std::map<std::string, std::string>& camera_params) {
    // =========================================================================
    // init() - Called on MAIN THREAD
    //
    // IMPORTANT: Do NOT create any OGRE objects here!
    //
    // Scrimmage's architecture:
    //   main thread:   viewer->init()  ← we are here
    //   viewer thread: viewer->run()   ← OGRE init happens there
    //
    // OpenGL contexts are thread-bound. If we create the GL context here
    // (via Root/Window), run() on the viewer thread would render to a
    // context owned by a different thread → BLACK WINDOW.
    //
    // Solution: Only validate config here, defer ALL OGRE init to run().
    // This matches VTK's pattern (vtkRenderWindow::Render() in run()).
    // =========================================================================

    // Check DISPLAY FIRST (before any OGRE calls)
    const char* display = std::getenv("DISPLAY");
    if (!display || display[0] == '\0') {
        std::cerr << "Error: No DISPLAY environment variable set.\n"
                  << "Run with enable_gui:=false or set DISPLAY for X11 forwarding."
                  << std::endl;
        return false;
    }

    // Store config for later use in initOgre()
    camera_params_ = camera_params;
    if (mp) {
        log_dir_ = mp->log_dir();
        dt_ = mp->dt();
        init_scale_ = get<double>("scale", mp->params(), 1.0);
        full_screen_ = mp->full_screen();
        window_width_ = mp->window_width();
        window_height_ = mp->window_height();
    }

    // Resolve and validate resource path
    mResourcePath = resolveResourcePath();
    std::cout << "OGRE-Next resource path: " << mResourcePath << std::endl;

    // Check that plugins.cfg exists (early validation)
    std::string pluginsCfg = mResourcePath + "plugins.cfg";
    if (!std::filesystem::exists(pluginsCfg)) {
        std::cerr << "Error: plugins.cfg not found at: " << pluginsCfg << "\n"
                  << "Set SCRIMMAGE_OGRE_MEDIA_DIR or ensure data/ogre_next/ exists."
                  << std::endl;
        return false;
    }

    std::cout << "OGRE-Next configuration validated. OGRE init deferred to render thread." << std::endl;
    return true;
}

bool OgreNextViewer::initOgre() {
    // =========================================================================
    // initOgre() - Called on VIEWER THREAD (from run())
    //
    // All OGRE objects are created here. The GL context is created inside
    // createRenderWindow() and bound to THIS thread. All rendering in run()
    // happens on this same thread, so the context is valid.
    //
    // Initialization order (per OGRE-Next 2.3 best practices):
    //   1. Create Root with plugins.cfg
    //   2. Select and configure RenderSystem (GL3Plus)
    //   3. Initialize Root (no window yet)
    //   4. Create RenderWindow (creates GL context on THIS thread)
    //   5. Create SceneManager
    //   6. Register HLMS (before initialiseAllResourceGroups)
    //   7. Initialize resource groups
    //   8. Create Camera
    //   9. Setup Compositor workspace (required for all rendering)
    //  10. Setup scene (lights, camera position)
    // =========================================================================

    std::cout << "OGRE: Initializing on render thread..." << std::endl;

    // Create Root with plugins.cfg
    const Ogre::String pluginsCfg = mResourcePath + "plugins.cfg";
    const Ogre::String ogreCfg = "";
    const Ogre::String ogreLog = log_dir_.empty() ? "Ogre.log" : (log_dir_ + "/Ogre.log");

    try {
        mRoot = OGRE_NEW Ogre::Root(pluginsCfg, ogreCfg, ogreLog, "Scrimmage");
    } catch (Ogre::Exception& e) {
        std::cerr << "Error: Failed to create OGRE Root.\n"
                  << "  " << e.getDescription() << "\n"
                  << "  Check that plugins.cfg exists at: " << pluginsCfg << std::endl;
        return false;
    }

    // Setup RenderSystem (GL3Plus for Linux)
    Ogre::RenderSystem* rs = mRoot->getRenderSystemByName("OpenGL 3+ Rendering Subsystem");
    if (!rs) {
        std::cerr << "Error: RenderSystem_GL3Plus not found.\n"
                  << "  Check plugins.cfg points to the correct plugin directory."
                  << std::endl;
        return false;
    }
    mRoot->setRenderSystem(rs);

    // Initialize Root (don't create window yet)
    mRoot->initialise(false);

    // Create RenderWindow
    Ogre::NameValuePairList params;
    params["gamma"] = "false";
    params["vsync"] = "true";

    try {
        mWindow = mRoot->createRenderWindow(
            "Scrimmage - OGRE-Next",
            static_cast<Ogre::uint32>(window_width_),
            static_cast<Ogre::uint32>(window_height_),
            full_screen_,
            &params);
    } catch (Ogre::Exception& e) {
        std::cerr << "Error: Failed to create render window.\n"
                  << "  " << e.getDescription() << std::endl;
        return false;
    }

    // Create SceneManager
    const size_t numThreads = 1;
    std::cout << "OGRE: Creating SceneManager..." << std::endl;
    mSceneManager = mRoot->createSceneManager(Ogre::ST_GENERIC, numThreads,
                                               "ScrimmageSceneManager");
    std::cout << "OGRE: SceneManager created: " << (mSceneManager ? "OK" : "NULL") << std::endl;

    // Initialize all resource groups
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups(true);

    // Create Camera
    mCamera = mSceneManager->createCamera("MainCamera");
    mCamera->setNearClipDistance(0.1f);
    mCamera->setFarClipDistance(10000.0f);
    mCamera->setAutoAspectRatio(true);

    // Setup Compositor workspace (required for rendering)
    setupCompositor();

    // Create scene content (lights, camera position)
    setupScene();

    std::cout << "OGRE-Next initialized successfully on render thread." << std::endl;
    mInitialized = true;
    return true;
}

bool OgreNextViewer::run() {
    // =========================================================================
    // run() - Called on VIEWER THREAD
    //
    // This is the entry point for the viewer thread. We initialize OGRE here
    // (not in init()) to ensure the GL context is created on this thread.
    //
    // Flow:
    //   1. initOgre() - creates Root, Window, SceneManager, Compositor, etc.
    //   2. Render loop - calls renderOneFrame() repeatedly
    //   3. Clean exit when window closes or mQuit is set
    //
    // This matches VTK's pattern where vtkRenderWindow::Render() and
    // vtkRenderWindowInteractor::Initialize() are called in run(), not init().
    // =========================================================================

    // Initialize OGRE on this thread
    if (!initOgre()) {
        std::cerr << "Error: Failed to initialize OGRE on render thread." << std::endl;
        return false;
    }
    
    // Initialize X11 input handling (get display/window from OGRE)
    if (!initX11Input()) {
        std::cerr << "Error: Failed to initialize X11 input handling." << std::endl;
        return false;
    }

    std::cout << "OGRE: Starting render loop..." << std::endl;
    std::cout << "OGRE: Window visible: " << (mWindow->isVisible() ? "yes" : "no") << std::endl;
    std::cout << "OGRE: Workspace enabled: " << (mWorkspace && mWorkspace->getEnabled() ? "yes" : "no") << std::endl;
    std::cout << "OGRE: Press 'b' to start simulation, 'q' or ESC to quit" << std::endl;

    // Render loop with X11 event handling
    int frameCount = 0;
    while (!mWindow->isClosed() && !mQuit) {
        // Note: We intentionally do NOT check for shutting_down messages here.
        // The simulation will continue running until completion. When it's done,
        // SimControl::shutdown() will wait for us via viewer_thread->join().
        // We exit when the user closes the window or presses quit.
        
        // Process X11 events (keyboard, window close)
        if (!processX11Events()) {
            std::cout << "OGRE: processX11Events returned false, exiting loop" << std::endl;
            break;  // Quit requested
        }
        
        Ogre::WindowEventUtilities::messagePump();

        // Render frame
        bool success = mRoot->renderOneFrame();
        
        if (frameCount < 5) {
            std::cout << "OGRE: Frame " << frameCount << " rendered: " << (success ? "yes" : "no") << std::endl;
            frameCount++;
        }
    }
    
    // Debug: Why did we exit?
    std::cout << "OGRE: Loop exit - window closed: " << mWindow->isClosed() 
              << ", mQuit: " << mQuit << std::endl;

    std::cout << "OGRE-Next render loop ended." << std::endl;
    
    // Signal the simulation that we're shutting down (mirror VTK behavior)
    // This tells SimControl to stop the simulation loop
    if (outgoing_interface_) {
        gui_msg_.set_shutting_down(true);
        outgoing_interface_->send_gui_msg(gui_msg_);
        std::cout << "OGRE: Sent shutting_down message to simulation" << std::endl;
    }
    
    // Destroy the window to close it properly
    if (mWindow) {
        mWindow->setHidden(true);
        mWindow->destroy();
        mWindow = nullptr;
        std::cout << "OGRE: Window destroyed" << std::endl;
    }
    
    return true;
}

// ============================================================================
// X11 Input Handling
// ============================================================================

bool OgreNextViewer::initX11Input() {
    // Get X11 display and window handles from OGRE
    // OGRE stores these as custom attributes on the window
    
    mWindow->getCustomAttribute("DISPLAY", &x11_display_);
    mWindow->getCustomAttribute("WINDOW", &x11_window_);
    
    if (!x11_display_) {
        std::cerr << "X11 Error: Could not get DISPLAY from OGRE window" << std::endl;
        return false;
    }
    
    if (!x11_window_) {
        std::cerr << "X11 Error: Could not get WINDOW from OGRE window" << std::endl;
        return false;
    }
    
    Display* display = static_cast<Display*>(x11_display_);
    ::Window window = static_cast< ::Window>(x11_window_);
    
    // Select the events we want to receive
    XSelectInput(display, window, 
                 KeyPressMask | KeyReleaseMask | 
                 StructureNotifyMask |  // For window close (DestroyNotify)
                 FocusChangeMask);
    
    std::cout << "X11: Input handling initialized for window " << x11_window_ << std::endl;
    return true;
}

bool OgreNextViewer::processX11Events() {
    if (!x11_display_) {
        return true;  // No X11, just continue
    }
    
    Display* display = static_cast<Display*>(x11_display_);
    
    while (XPending(display) > 0) {
        XEvent event;
        XNextEvent(display, &event);
        
        switch (event.type) {
            case KeyPress: {
                KeySym keysym = XLookupKeysym(&event.xkey, 0);
                
                // Handle Escape key
                if (keysym == XK_Escape) {
                    mQuit = true;
                    return false;
                }
                
                // Convert keysym to string for handleKeyPress
                std::string key;
                
                switch (keysym) {
                    case XK_Left:       key = "Left"; break;
                    case XK_Right:      key = "Right"; break;
                    case XK_Up:         key = "Up"; break;
                    case XK_Down:       key = "Down"; break;
                    case XK_space:      key = "space"; break;
                    case XK_bracketleft:  key = "bracketleft"; break;
                    case XK_bracketright: key = "bracketright"; break;
                    case XK_equal:      key = "equal"; break;
                    case XK_plus:       key = "plus"; break;
                    case XK_minus:      key = "minus"; break;
                    default: {
                        // For regular characters, convert keysym to character
                        // Keysyms for ASCII characters match their ASCII values
                        if (keysym >= XK_a && keysym <= XK_z) {
                            key = std::string(1, static_cast<char>(keysym - XK_a + 'a'));
                        } else if (keysym >= XK_A && keysym <= XK_Z) {
                            key = std::string(1, static_cast<char>(keysym - XK_A + 'a')); // lowercase
                        } else if (keysym >= XK_0 && keysym <= XK_9) {
                            key = std::string(1, static_cast<char>(keysym - XK_0 + '0'));
                        }
                        break;
                    }
                }
                
                if (!key.empty()) {
                    handleKeyPress(key);
                }
                break;
            }
            
            case DestroyNotify:
                // Window was closed
                mQuit = true;
                return false;
                
            case ClientMessage:
                // Check for window manager close (WM_DELETE_WINDOW)
                // This is typically how the X button works
                mQuit = true;
                return false;
                
            default:
                break;
        }
    }
    return true;
}

void OgreNextViewer::handleKeyPress(const std::string& key) {
    // Mirror VtkCameraInterface::OnKeyPress() behavior
    // See src/viewer/vtk/VtkCameraInterface.cpp
    
    if (key == "q") {
        mQuit = true;
    } else if (key == "b") {
        togglePause();
    } else if (key == "space") {
        singleStep();
    } else if (key == "bracketleft") {
        decWarp();
    } else if (key == "bracketright") {
        incWarp();
    } else if (key == "Left" || key == "left") {
        // TODO: dec_follow() when OgreNextUpdater is implemented
        std::cout << "OGRE: Left arrow (dec_follow) - not yet implemented" << std::endl;
    } else if (key == "Right" || key == "right") {
        // TODO: inc_follow() when OgreNextUpdater is implemented
        std::cout << "OGRE: Right arrow (inc_follow) - not yet implemented" << std::endl;
    } else if (key == "a") {
        // TODO: next_mode() when OgreNextUpdater is implemented
        std::cout << "OGRE: 'a' (next camera mode) - not yet implemented" << std::endl;
    } else if (key == "r") {
        // TODO: reset_view() when OgreNextUpdater is implemented
        std::cout << "OGRE: 'r' (reset view) - not yet implemented" << std::endl;
    } else if (key == "h") {
        // TODO: toggle_helpmenu() when OgreNextUpdater is implemented
        std::cout << "OGRE: 'h' (help menu) - not yet implemented" << std::endl;
    } else if (key == "t") {
        // TODO: toggle_trails() when OgreNextUpdater is implemented
        std::cout << "OGRE: 't' (toggle trails) - not yet implemented" << std::endl;
    } else if (key == "plus" || key == "equal") {
        // TODO: inc_scale() when OgreNextUpdater is implemented
        std::cout << "OGRE: '+' (inc scale) - not yet implemented" << std::endl;
    } else if (key == "minus") {
        // TODO: dec_scale() when OgreNextUpdater is implemented
        std::cout << "OGRE: '-' (dec scale) - not yet implemented" << std::endl;
    } else if (key == "0") {
        // TODO: reset_scale() when OgreNextUpdater is implemented
        std::cout << "OGRE: '0' (reset scale) - not yet implemented" << std::endl;
    } else {
        // Uncomment for debugging unknown keys
        // std::cout << "OGRE: Unhandled key: " << key << std::endl;
    }
}

// ============================================================================
// GUI Message Helpers (mirror VtkUpdater pattern)
// ============================================================================

void OgreNextViewer::togglePause() {
    if (!outgoing_interface_) {
        std::cout << "OGRE: Toggle pause (no outgoing interface)" << std::endl;
        return;
    }
    
    gui_msg_.set_toggle_pause(true);
    outgoing_interface_->send_gui_msg(gui_msg_);
    gui_msg_.set_toggle_pause(false);
    std::cout << "OGRE: Toggle pause sent" << std::endl;
}

void OgreNextViewer::singleStep() {
    if (!outgoing_interface_) {
        std::cout << "OGRE: Single step (no outgoing interface)" << std::endl;
        return;
    }
    
    gui_msg_.set_single_step(true);
    outgoing_interface_->send_gui_msg(gui_msg_);
    gui_msg_.set_single_step(false);
    std::cout << "OGRE: Single step sent" << std::endl;
}

void OgreNextViewer::incWarp() {
    if (!outgoing_interface_) {
        std::cout << "OGRE: Inc warp (no outgoing interface)" << std::endl;
        return;
    }
    
    gui_msg_.set_inc_warp(true);
    outgoing_interface_->send_gui_msg(gui_msg_);
    gui_msg_.set_inc_warp(false);
    std::cout << "OGRE: Inc warp sent" << std::endl;
}

void OgreNextViewer::decWarp() {
    if (!outgoing_interface_) {
        std::cout << "OGRE: Dec warp (no outgoing interface)" << std::endl;
        return;
    }
    
    gui_msg_.set_dec_warp(true);
    outgoing_interface_->send_gui_msg(gui_msg_);
    gui_msg_.set_dec_warp(false);
    std::cout << "OGRE: Dec warp sent" << std::endl;
}

}  // namespace scrimmage
