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

#include "scrimmage/parse/MissionParse.h"
#include "scrimmage/parse/ParseUtils.h"

namespace scrimmage {

OgreNextViewer::OgreNextViewer() : enable_network_(false) {}

OgreNextViewer::~OgreNextViewer() {
    // Only clean up if we actually initialized OGRE
    if (mRoot) {
        if (mSceneManager) {
            mRoot->destroySceneManager(mSceneManager);
            mSceneManager = nullptr;
        }
        OGRE_DELETE mRoot;
        mRoot = nullptr;
    }
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
    // This handles the case where we're running from the build directory
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

    std::cout << "OGRE: Starting render loop..." << std::endl;
    std::cout << "OGRE: Window visible: " << (mWindow->isVisible() ? "yes" : "no") << std::endl;
    std::cout << "OGRE: Workspace enabled: " << (mWorkspace && mWorkspace->getEnabled() ? "yes" : "no") << std::endl;

    // Simple render loop - will be enhanced with OgreNextUpdater in Phase 3
    int frameCount = 0;
    while (!mWindow->isClosed() && !mQuit) {
        Ogre::WindowEventUtilities::messagePump();

        // Render frame
        bool success = mRoot->renderOneFrame();
        
        if (frameCount < 5) {
            std::cout << "OGRE: Frame " << frameCount << " rendered: " << (success ? "yes" : "no") << std::endl;
            frameCount++;
        }
    }

    std::cout << "OGRE-Next render loop ended." << std::endl;
    return true;
}

}  // namespace scrimmage
