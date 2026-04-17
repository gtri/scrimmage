# OGRE-Next Implementation Plan for Scrimmage

## Current Status (April 2026)

**Phase 2 COMPLETE** - OGRE-Next viewer renders successfully with bright red background.

### What Works
- Window opens with correct dimensions
- Compositor workspace renders background color
- Render loop runs smoothly (frames render successfully)
- Clean shutdown without crashes
- Threading model correctly defers all OGRE init to viewer thread

### Files Added/Modified This Session
```
NEW:  src/viewer/ogre_next/OgreNextViewer.h      (148 lines) - Main viewer class
NEW:  src/viewer/ogre_next/OgreNextViewer.cpp    (471 lines) - Full OGRE init & render loop
MOD:  src/viewer/ogre_next/OgreNextViewerBackend.cpp - Delegates to OgreNextViewer
MOD:  src/viewer/ogre_next/OgreNextViewerBackend.h   - Added OgreNextViewer member
MOD:  development-docs/OGRE-Next-Guide.md        - Added GL context threading section
```

### Key Issues Solved
1. **ABI Compatibility** - Ubuntu OGRE-Next package requires `_DEBUG=1` flag
2. **Black Window Bug** - GL context created on wrong thread; fixed by deferring all OGRE init to run()
3. **Threading Model** - Matches VTK pattern: init() stores config, run() creates OGRE objects

### Next Steps (Phase 3)
- Add keyboard input. Scrimmage starts simulations paused, needs a b key press to begin.
- Implement OgreNextUpdater to receive entity data from SimControl
- Add entity rendering (Items, meshes)
- Implement camera controls (CameraInterface)
- Add grid and origin axes

---

## Overview

This document outlines the plan to add an OGRE-Next 2.3 viewer backend to scrimmage, mirroring the existing VTK viewer architecture while following the OGRE-Next best practices documented in `OGRE-Next-Guide.md`.

**Target runtime:** Linux desktop with OpenGL 3.3+ via GLX (same constraints as the guide).

**Threading model:** Scrimmage runs the viewer on a separate thread:
- `init()` is called on the **main thread**
- `run()` is called on the **viewer thread**

This is critical for OGRE-Next because OpenGL contexts are thread-bound. The GL context must be created and used on the same thread. The solution (matching VTK's pattern) is:
- `init()`: Only validate configuration and store parameters (no OGRE objects)
- `run()`: Create all OGRE objects and enter render loop (GL context created here)

---

## CRITICAL: Threading Model - OpenGL Context Affinity

### The Problem

Scrimmage's architecture calls `viewer->init()` on the main thread, then spawns a viewer thread to call `viewer->run()`. This creates a threading issue for OpenGL:

```
Main Thread:          init() → would create GL context (thread A)
                              ↓
Viewer Thread:        run()  → tries to render (thread B)
                              ↓
Result:               BLACK WINDOW (GL context isn't current on render thread)
```

OpenGL contexts are **thread-bound**. A context created on one thread cannot be used for rendering from a different thread without explicit context switching.

### The Solution (VTK Pattern)

VTK solves this by deferring all GPU initialization to `run()`:

**VTK's init():**
- Creates VTK wrapper objects (`vtkRenderer`, `vtkRenderWindow`)
- Does NOT call `Render()` or `Initialize()`
- No actual GL context is created

**VTK's run():**
- Calls `renderWindow_->Render()` → creates GL context
- Calls `renderWindowInteractor_->Initialize()`
- Enters render loop

The OGRE-Next viewer follows this exact pattern:

**OgreNextViewer::init() - Main Thread:**
```cpp
// Only validate environment and store configuration.
// Do NOT create any OGRE objects here.
bool init(const MissionParsePtr& mp, ...) {
    // Check DISPLAY
    // Store config (camera_params_, log_dir_, window_width_, etc.)
    // Validate plugins.cfg exists
    return true;  // Deferred init
}
```

**OgreNextViewer::run() - Viewer Thread:**
```cpp
bool run() {
    // Initialize OGRE on THIS thread (creates GL context)
    if (!initOgre()) return false;
    
    // Enter render loop (uses GL context on same thread)
    while (!mWindow->isClosed() && !mQuit) {
        mRoot->renderOneFrame();
    }
    return true;
}
```

### Why This Works

1. GL context is created in `initOgre()` which is called from `run()`
2. `run()` executes on the viewer thread
3. All rendering happens on the viewer thread
4. Context and rendering are on the same thread ✓

### Alternative Approaches (Not Used)

1. **Context switching**: Use `GL3PlusContext::setCurrent()` to make context current on render thread
   - Requires accessing OGRE internals
   - Platform-specific

2. **Run viewer on main thread**: Don't create viewer_thread
   - Requires significant architecture changes to scrimmage
   - May break other assumptions

The deferred initialization pattern is the cleanest solution and matches VTK's proven approach.

---

## CRITICAL: Ubuntu Package ABI Workaround

The Ubuntu `libogre-next-dev` package (2.3.3+dfsg-0ubuntu2) has an ABI compatibility issue that causes memory corruption and crashes when client code is compiled without `_DEBUG` defined. Symptoms include crashes in `Window::getTexture()` after calling `createBasicWorkspaceDef()`, Valgrind reporting "Invalid write of size 8", and GDB showing corrupted vtable pointers containing ASCII text. The fix is to add `add_compile_definitions(_DEBUG=1 OGRE_IGNORE_DEBUG_FLAG_CONTRADICTION=1)` to CMakeLists.txt. This makes OGRE headers use debug-safe code paths that happen to be ABI-compatible with the Ubuntu package. If you see random crashes after compositor setup, check that this flag is present.

---

## Quick Start: First Render Test

The runtime data files (`data/ogre_next/`) are committed to the repository, so no download is needed.

```bash
# 1. Build with OGRE-Next enabled
cd build
cmake -DSCRIMMAGE_BUILD_OGRE_NEXT_VIEWER=ON ..
make -j$(nproc)

# 2. Source environment and test (requires DISPLAY)
source ~/.scrimmage/setup.bash
./bin/scrimmage --ogre time_warp:=1 ../missions/straight.xml
```

**Expected result:** A window opens with a **bright red background**. The simulation will run (press a key to start if paused). Entities won't appear yet (OgreNextUpdater not fully implemented), but if the window opens with the red background and the render loop runs without crashing, the OGRE initialization is working correctly.

**Key success indicators in the console output:**
```
OGRE-Next configuration validated. OGRE init deferred to render thread.
OGRE: Initializing on render thread...
OGRE: Created workspace definition with RED background
OGRE-Next initialized successfully on render thread.
OGRE: Starting render loop...
OGRE: Frame 0 rendered: yes
```

---

## Usage

The OGRE-Next viewer is selected at runtime via the `--ogre` command-line flag:

```bash
# Build scrimmage
cd build
make -j$(nproc)

# Source environment
cd ..
source ~/.scrimmage/setup.bash

# Run with default VTK viewer
scrimmage missions/straight.xml

# Run with OGRE-Next viewer
scrimmage --ogre missions/straight.xml
```

The flag is handled in `share/scrimmage/main.cpp` which selects `ViewerBackendType::OGRE_NEXT` when `--ogre` is passed, otherwise defaults to `ViewerBackendType::VTK`.

---

## Requirements Checklist

- [x] Create `OgreNextViewerBackend` implementing the `ViewerBackend` interface
- [x] Create `OgreNextViewer` mirroring `VtkViewer` functionality  
- [ ] Create `OgreNextUpdater` for periodic scene updates from interface messages
- [ ] Create `OgreNextCameraInterface` for user input handling
- [ ] Create `OgreNextGrid` and `OgreNextOriginAxes` scene primitives
- [x] CMake integration with `SCRIMMAGE_BUILD_OGRE_NEXT_VIEWER` option
- [x] HLMS registration and resource loading following OGRE-Next guide
- [x] Compositor workspace setup (required for OGRE-Next rendering)
- [ ] **Runtime data files** (`plugins.cfg`, `resources2.cfg`, `Hlms/` shaders)
- [ ] Container/devcontainer X11 passthrough support
- [ ] Documentation and smoke tests

---

## Architecture Mapping: VTK → OGRE-Next

| VTK Component | OGRE-Next Equivalent | Purpose |
|---------------|---------------------|---------|
| `VtkViewerBackend` | `OgreNextViewerBackend` | Implements `ViewerBackend`, delegates to viewer |
| `VtkViewer` | `OgreNextViewer` | Owns render system, window, scene, camera |
| `VtkUpdater` | `OgreNextUpdater` | Periodic tick: updates scene from interface messages |
| `VtkCameraInterface` | `OgreNextCameraInterface` | Mouse/keyboard input handling |
| `VtkGrid` | `OgreNextGrid` | Ground plane/grid visualization |
| `VtkOriginAxes` | `OgreNextOriginAxes` | World origin axes visualization |
| `vtkRenderer` | `Ogre::SceneManager` | Scene graph management |
| `vtkRenderWindow` | `Ogre::RenderWindow` | OS window and render target |
| `vtkRenderWindowInteractor` | `Ogre::WindowEventUtilities` + FrameListener | Event loop and input |
| `vtkCamera` | `Ogre::Camera` | View/projection |

---

## Files to Create

```
src/viewer/ogre_next/
├── OgreNextViewerBackend.h      # EXISTS - needs viewer_ member added
├── OgreNextViewerBackend.cpp    # EXISTS - needs delegation to OgreNextViewer
├── OgreNextViewer.h             # MISSING - REQUIRED for build
├── OgreNextViewer.cpp           # MISSING - REQUIRED for build
├── OgreNextCameraInterface.h    # Phase 3
├── OgreNextCameraInterface.cpp  # Phase 3
├── OgreNextUpdater.h            # Phase 3
├── OgreNextUpdater.cpp          # Phase 3
├── OgreNextGrid.h               # Phase 3 (optional)
├── OgreNextGrid.cpp             # Phase 3 (optional)
├── OgreNextOriginAxes.h         # Phase 3 (optional)
└── OgreNextOriginAxes.cpp       # Phase 3 (optional)

data/ogre_next/
├── plugins.cfg                  # MISSING - REQUIRED for runtime
├── resources2.cfg               # MISSING - REQUIRED for runtime
└── Hlms/                        # MISSING - REQUIRED (download from OGRE-Next repo)
    ├── Common/
    ├── Pbs/
    └── Unlit/

# NOTE: No compositor script needed - using createBasicWorkspaceDef() instead
```

---

## Implementation Phases

### Phase 0: Preparation & Decisions

**Decision required:** How to obtain OGRE-Next dependencies.

| Option | Pros | Cons |
|--------|------|------|
| EmptyProject-style (copy OGRE source into `Dependencies/Ogre`) | Full control, matches guide exactly | Larger repo, manual updates |
| System-installed package (`find_package`) | Smaller repo, easier updates | Requires `OGRE_MEDIA_DIR` env var for HLMS |
| Hybrid: find_package + runtime `OGRE_MEDIA_DIR` | Flexible | More configuration complexity |

**Recommendation:** Use `find_package(OGRE)` with a CMake configure-time copy of HLMS data, or require `OGRE_MEDIA_DIR` at runtime.

**Decision made:** Use system-installed OGRE-Next (`libogre-next-dev`) with HLMS data bundled in the scrimmage repo under `data/ogre_next/Hlms/`.

**CRITICAL PRE-REQUISITE:** Before any implementation, verify OGRE-Next is properly installed and accessible:

```bash
# 1. Check package is installed
dpkg -l | grep ogre-next

# 2. Check pkg-config works
pkg-config --modversion OGRE-Next    # Should show 2.3.x
pkg-config --cflags OGRE-Next        # Should show include path
pkg-config --libs OGRE-Next          # Should show library flags

# 3. Check for HLMS libraries (REQUIRED)
ls /usr/lib/x86_64-linux-gnu/libOgreHlms*   # Should show HlmsPbs and HlmsUnlit

# 4. Check plugin directory
ls /usr/lib/x86_64-linux-gnu/OGRE-Next/     # Should show RenderSystem_GL3Plus.so
```

If any of these fail, install OGRE-Next first before proceeding.

---

### Phase 0.5: Runtime Data Files (REQUIRED BEFORE TESTING)

**Critical:** The system-installed `libogre-next-dev` package does NOT include the HLMS shader template files. These must be obtained from the OGRE-Next source repository and bundled with scrimmage.

#### Step 1: Obtain HLMS Shader Data

Download from OGRE-Next 2.3 release:

```bash
# Create data directory
mkdir -p data/ogre_next/Hlms

# Download HLMS data from OGRE-Next 2.3.3 release
cd data/ogre_next
curl -L https://github.com/OGRECave/ogre-next/archive/refs/tags/v2.3.3.tar.gz | \
    tar -xz --strip-components=3 ogre-next-2.3.3/Samples/Media/Hlms

# Verify structure
ls Hlms/
# Expected: Common/  Pbs/  Unlit/
```

**Alternative:** Clone the repo and copy:

```bash
git clone --depth 1 --branch v2.3.3 https://github.com/OGRECave/ogre-next.git /tmp/ogre-next
cp -r /tmp/ogre-next/Samples/Media/Hlms data/ogre_next/
rm -rf /tmp/ogre-next
```

#### Step 2: Create `plugins.cfg`

Create `data/ogre_next/plugins.cfg`:

```ini
# OGRE-Next plugins configuration for Scrimmage
# Linux GL3+ only - system-installed OGRE-Next 2.3

PluginFolder=/usr/lib/x86_64-linux-gnu/OGRE-Next/

# Required: OpenGL 3.3+ render system
Plugin=RenderSystem_GL3Plus

# Optional: Particle effects
# Plugin=Plugin_ParticleFX
```

#### Step 3: Create `resources2.cfg`

Create `data/ogre_next/resources2.cfg`:

```ini
# OGRE-Next resource configuration for Scrimmage
# 
# IMPORTANT: The [Hlms] section's DoNotUseAsResource entry specifies
# the root folder containing Common/, Pbs/, Unlit/ subdirectories.
# This is parsed specially by registerHlms() - do NOT add Hlms paths
# to regular resource groups.

[Hlms]
DoNotUseAsResource=Hlms/

[General]
# Add custom materials, meshes, textures here as needed
# FileSystem=Materials
# FileSystem=Meshes
# FileSystem=Textures
```

#### Step 4: Update CMakeLists.txt to Install Data

Add to `src/viewer/CMakeLists.txt` (in the OGRE-Next section):

```cmake
if (SCRIMMAGE_HAVE_OGRE_NEXT_VIEWER)
  # ... existing library setup ...

  # Install OGRE-Next runtime data
  install(DIRECTORY ${CMAKE_SOURCE_DIR}/data/ogre_next/
    DESTINATION share/scrimmage/ogre_next
    FILES_MATCHING
    PATTERN "*.cfg"
    PATTERN "Hlms/*"
    PATTERN "*.compositor"
  )
endif()
```

#### Step 5: Verify Directory Structure

After completing steps 1-4, verify:

```
data/ogre_next/
├── plugins.cfg              # Points to /usr/lib/x86_64-linux-gnu/OGRE-Next/
├── resources2.cfg           # Contains [Hlms] DoNotUseAsResource=Hlms/
└── Hlms/
    ├── Common/
    │   ├── GLSL/
    │   │   └── *.glsl       # Shader pieces
    │   └── Any/
    │       └── *.any        # Cross-platform shader pieces
    ├── Pbs/
    │   ├── GLSL/
    │   └── Any/
    └── Unlit/
        ├── GLSL/
        └── Any/
```

#### Resource Path Resolution

The `OgreNextViewer::resolveResourcePath()` function resolves the data directory:

1. **Environment variable** (highest priority): `SCRIMMAGE_OGRE_MEDIA_DIR`
2. **Relative to executable** (default): `../share/scrimmage/ogre_next/`

For development, either:
- Set `SCRIMMAGE_OGRE_MEDIA_DIR=/path/to/scrimmage/data/ogre_next/`
- Or create a symlink: `ln -s ../../../data/ogre_next build/share/scrimmage/ogre_next`

---

### Phase 1: Skeleton & API Parity ✅ COMPLETE

**Goal:** Create stub files that compile and integrate with scrimmage build.

**Status:** COMPLETE. `OgreNextViewer.h/cpp` created and integrated with `OgreNextViewerBackend`.

#### OgreNextViewerBackend API (mirrors VtkViewerBackend)

```cpp
class OgreNextViewerBackend : public ViewerBackend {
 public:
    void set_incoming_interface(InterfacePtr& incoming_interface) override;
    void set_outgoing_interface(InterfacePtr& outgoing_interface) override;
    void set_enable_network(bool enable) override;

    bool init(const MissionParsePtr& mp,
              const std::map<std::string, std::string>& camera_params) override;
    bool run() override;

 protected:
    OgreNextViewer viewer_;  // Must create OgreNextViewer class
    InterfacePtr incoming_interface_;
    InterfacePtr outgoing_interface_;
    bool enable_network_ = false;
};
```

#### OgreNextViewer API (mirrors VtkViewer)

```cpp
class OgreNextViewer {
 public:
    OgreNextViewer();
    ~OgreNextViewer();

    void set_incoming_interface(InterfacePtr& incoming_interface);
    void set_outgoing_interface(InterfacePtr& outgoing_interface);
    void set_enable_network(bool enable);

    bool init(const MissionParsePtr& mp,
              const std::map<std::string, std::string>& camera_params);
    bool run();

 protected:
    // Resource path resolution (see implementation below)
    Ogre::String resolveResourcePath();

    // HLMS registration (MUST be called before ResourceGroupManager::initialiseAllResourceGroups)
    void registerHlms();

    // Compositor setup (required for any rendering in OGRE-Next)
    void setupCompositor();

    // OGRE core objects (use OGRE_NEW/OGRE_DELETE, not raw new/delete)
    Ogre::Root* mRoot = nullptr;
    Ogre::RenderWindow* mWindow = nullptr;
    Ogre::SceneManager* mSceneManager = nullptr;
    Ogre::Camera* mCamera = nullptr;
    Ogre::CompositorWorkspace* mWorkspace = nullptr;

    // Scrimmage integration
    InterfacePtr incoming_interface_;
    InterfacePtr outgoing_interface_;
    bool enable_network_ = false;
    std::thread network_thread_;

    // Configuration
    Ogre::String mResourcePath;  // Set by resolveResourcePath()
    std::map<std::string, std::string> camera_params_;
    std::string log_dir_;
    double dt_ = 0.1;

    // Network params (mirror VtkViewer)
    std::string local_ip_ = "localhost";
    int local_port_ = 50051;
    std::string remote_ip_ = "localhost";
    int remote_port_ = 50052;

    double init_scale_ = 1.0;
    bool full_screen_ = false;
};
```

#### Resource Path Resolution Implementation

```cpp
Ogre::String OgreNextViewer::resolveResourcePath() {
    // Priority 1: Environment variable
    const char* envPath = std::getenv("SCRIMMAGE_OGRE_MEDIA_DIR");
    if (envPath && envPath[0] != '\0') {
        Ogre::String path(envPath);
        if (path.back() != '/') path += '/';
        return path;
    }

    // Priority 2: Relative to executable (installed layout)
    // Executable is in build/bin/, data is in build/share/scrimmage/ogre_next/
    return "../share/scrimmage/ogre_next/";
}
```

---

### Phase 2: OGRE Bootstrap, Resources, HLMS, Compositor ✅ COMPLETE

**Goal:** Initialize OGRE-Next following the guide's best practices.

**Status:** COMPLETE. Window opens, compositor renders red background, render loop works.

**Key Implementation Detail:** All OGRE initialization happens in `initOgre()` which is called from `run()` on the viewer thread, NOT from `init()` on the main thread. This ensures the GL context is created on the same thread that renders.

#### Initialization Order (Critical)

The initialization sequence MUST follow this exact order per the OGRE-Next guide:

```cpp
bool OgreNextViewer::init(const MissionParsePtr& mp,
                          const std::map<std::string, std::string>& camera_params) {
    // STEP 1: Check DISPLAY FIRST (before any OGRE calls)
    const char* display = std::getenv("DISPLAY");
    if (!display || display[0] == '\0') {
        std::cerr << "Error: No DISPLAY environment variable set. "
                  << "Run with enable_gui:=false or set DISPLAY for X11 forwarding."
                  << std::endl;
        return false;
    }

    // STEP 2: Resolve resource path
    mResourcePath = resolveResourcePath();

    // STEP 3: Create Root with plugins.cfg, ogre.cfg, Ogre.log, AND app name (2.3 API)
    const Ogre::String pluginsCfg = mResourcePath + "plugins.cfg";
    const Ogre::String ogreCfg = "";  // Empty = don't save config dialog results
    const Ogre::String ogreLog = log_dir_.empty() ? "Ogre.log" : (log_dir_ + "/Ogre.log");
    mRoot = OGRE_NEW Ogre::Root(pluginsCfg, ogreCfg, ogreLog, "Scrimmage");

    // STEP 4: Setup RenderSystem (GL3Plus for Linux)
    Ogre::RenderSystem* rs = mRoot->getRenderSystemByName("OpenGL 3+ Rendering Subsystem");
    if (!rs) {
        std::cerr << "Error: RenderSystem_GL3Plus not found. Check plugins.cfg." << std::endl;
        return false;
    }
    mRoot->setRenderSystem(rs);

    // STEP 5: Initialize Root (creates render window)
    mRoot->initialise(false);  // false = we'll create window manually for gamma control

    // STEP 6: Create RenderWindow WITH gamma correction (critical for PBS)
    Ogre::NameValuePairList params;
    params["gamma"] = "true";
    params["vsync"] = "false";
    const Ogre::uint32 width = 1280;
    const Ogre::uint32 height = 720;
    mWindow = mRoot->createRenderWindow("Scrimmage - OGRE-Next", width, height,
                                        full_screen_, &params);

    // STEP 7: Create SceneManager (single-threaded for now)
    // TODO: Enable multi-threading when scrimmage's threading model is fixed
    const size_t numThreads = 1;
    mSceneManager = mRoot->createSceneManager(Ogre::ST_GENERIC, numThreads,
                                               "ScrimmageSceneManager");

    // STEP 8: Register HLMS ***BEFORE*** ResourceGroupManager::initialiseAllResourceGroups
    registerHlms();

    // STEP 9: Setup non-HLMS resource locations (if any)
    // The [General] section of resources2.cfg would be loaded here

    // STEP 10: Initialize all resource groups
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups(true);

    // STEP 11: Create Camera
    mCamera = mSceneManager->createCamera("MainCamera");
    mCamera->setNearClipDistance(0.1f);
    mCamera->setFarClipDistance(10000.0f);
    mCamera->setAutoAspectRatio(true);

    // STEP 12: Setup Compositor workspace (required for rendering)
    setupCompositor();

    // STEP 13: Create scene content (lights with proper LDR power scale)
    Ogre::Light* light = mSceneManager->createLight();
    Ogre::SceneNode* lightNode = mSceneManager->getRootSceneNode()
        ->createChildSceneNode();
    lightNode->attachObject(light);
    light->setType(Ogre::Light::LT_DIRECTIONAL);
    light->setPowerScale(Ogre::Math::PI);  // Required for LDR pipeline
    lightNode->setDirection(Ogre::Vector3(-1, -1, -1).normalisedCopy());

    return true;
}
```

#### Threading Configuration

OGRE-Next's SceneManager supports multi-threaded culling and transform updates. The design should be multi-thread ready but default to single-threaded until scrimmage's threading model is fixed:

```cpp
// Multi-thread ready design, but defaulting to single thread for now
// TODO: Enable when scrimmage threading is fixed
const size_t numThreads = 1;  // Single-threaded for now

// Future multi-threaded version:
// const unsigned int hwThreads = std::thread::hardware_concurrency();
// const size_t numThreads = hwThreads > 1u ? static_cast<size_t>(hwThreads - 1u) : 1u;

mSceneManager = mRoot->createSceneManager(Ogre::ST_GENERIC, numThreads, "MainSceneManager");
```

#### Root Constructor (2.3 API)

The 2.3 API requires an app name as the fourth parameter:

```cpp
// CORRECT for OGRE-Next 2.3:
mRoot = OGRE_NEW Ogre::Root("plugins.cfg", "ogre.cfg", "Ogre.log", "Scrimmage");

// NOT: mRoot = OGRE_NEW Ogre::Root("plugins.cfg", "ogre.cfg", "Ogre.log");
```

#### DISPLAY Check (Must Be First)

Check DISPLAY before creating Root, as Root may attempt X11 operations:

```cpp
bool OgreNextViewer::init(...) {
    // Check DISPLAY FIRST - before any OGRE calls
    const char* display = std::getenv("DISPLAY");
    if (!display || display[0] == '\0') {
        std::cerr << "Error: No DISPLAY environment variable set. "
                  << "Run with enable_gui:=false or set DISPLAY for X11 forwarding."
                  << std::endl;
        return false;
    }

    // Now safe to create Root
    mRoot = OGRE_NEW Ogre::Root("plugins.cfg", "ogre.cfg", "Ogre.log", "Scrimmage");
    // ...
}
```

#### HLMS Registration Pattern (from guide)

```cpp
void OgreNextViewer::registerHlms() {
    Ogre::ConfigFile cfg;
    cfg.load(mResourcePath + "resources2.cfg");

    Ogre::String rootHlmsFolder = mResourcePath +
        cfg.getSetting("DoNotUseAsResource", "Hlms", "");

    if (rootHlmsFolder.empty())
        rootHlmsFolder = "./";
    else if (*(rootHlmsFolder.end() - 1) != '/')
        rootHlmsFolder += "/";

    Ogre::ArchiveManager& archiveManager = Ogre::ArchiveManager::getSingleton();
    const Ogre::String archiveType = "FileSystem";

    // Register HlmsUnlit
    Ogre::String mainFolderPath;
    Ogre::StringVector libraryFoldersPaths;

    Ogre::HlmsUnlit::getDefaultPaths(mainFolderPath, libraryFoldersPaths);
    Ogre::Archive* archiveUnlit = archiveManager.load(
        rootHlmsFolder + mainFolderPath, archiveType, true);
    Ogre::ArchiveVec archiveUnlitLibraries;
    for (const Ogre::String& libraryPath : libraryFoldersPaths) {
        archiveUnlitLibraries.push_back(
            archiveManager.load(rootHlmsFolder + libraryPath, archiveType, true));
    }
    Ogre::HlmsUnlit* hlmsUnlit = OGRE_NEW Ogre::HlmsUnlit(
        archiveUnlit, &archiveUnlitLibraries);
    mRoot->getHlmsManager()->registerHlms(hlmsUnlit);

    // Register HlmsPbs (same pattern)
    Ogre::HlmsPbs::getDefaultPaths(mainFolderPath, libraryFoldersPaths);
    Ogre::Archive* archivePbs = archiveManager.load(
        rootHlmsFolder + mainFolderPath, archiveType, true);
    Ogre::ArchiveVec archivePbsLibraries;
    for (const Ogre::String& libraryPath : libraryFoldersPaths) {
        archivePbsLibraries.push_back(
            archiveManager.load(rootHlmsFolder + libraryPath, archiveType, true));
    }
    Ogre::HlmsPbs* hlmsPbs = OGRE_NEW Ogre::HlmsPbs(
        archivePbs, &archivePbsLibraries);
    mRoot->getHlmsManager()->registerHlms(hlmsPbs);
}
```

#### Compositor Setup (Required)

```cpp
void OgreNextViewer::setupCompositor() {
    Ogre::CompositorManager2* compositorMgr = mRoot->getCompositorManager2();
    const Ogre::String workspaceName("ScrimmageWorkspace");

    if (!compositorMgr->hasWorkspaceDefinition(workspaceName)) {
        compositorMgr->createBasicWorkspaceDef(
            workspaceName,
            Ogre::ColourValue(0.1f, 0.1f, 0.1f)  // Dark background
        );
    }

    mWorkspace = compositorMgr->addWorkspace(
        mSceneManager,
        mWindow,
        mCamera,
        workspaceName,
        true  // enabled
    );
}
```

#### Gamma Correction (Critical for PBS)

```cpp
Ogre::NameValuePairList params;
params["gamma"] = "true";
mWindow = mRoot->createRenderWindow(title, width, height, fullscreen, &params);
```

#### Light Power for LDR Pipeline

When creating lights, set the power scale for correct LDR rendering:

```cpp
Ogre::Light* light = mSceneManager->createLight();
Ogre::SceneNode* lightNode = mSceneManager->getRootSceneNode()->createChildSceneNode();
lightNode->attachObject(light);
light->setType(Ogre::Light::LT_DIRECTIONAL);
light->setPowerScale(Ogre::Math::PI);  // Required for LDR pipeline
lightNode->setDirection(Ogre::Vector3(-1, -1, -1).normalisedCopy());
```

---

### Phase 3: Scene Objects, Updater, Camera Input 🔄 IN PROGRESS

**Goal:** Implement scene updates and user interaction.

**Status:** NOT STARTED. This is the next phase to implement.

#### OgreNextUpdater Design

The updater can use `Ogre::FrameListener` for simplicity, but for deterministic simulation sync, consider a manual loop pattern:

**Option A: FrameListener (simpler)**

```cpp
class OgreNextUpdater : public Ogre::FrameListener {
 public:
    void set_incoming_interface(InterfacePtr incoming);
    void set_outgoing_interface(InterfacePtr outgoing);
    void set_scene_manager(Ogre::SceneManager* sceneMgr);
    void set_camera(Ogre::Camera* camera);

    // FrameListener interface
    bool frameStarted(const Ogre::FrameEvent& evt) override;

    // View modes (mirror VTK)
    enum class ViewMode { FOLLOW, FREE, OFFSET };
    void set_view_mode(ViewMode mode);
    void set_follow_id(int id);

 protected:
    void processIncomingMessages();
    void updateSceneObjects();
    void updateCamera(float dt);

    InterfacePtr incoming_interface_;
    InterfacePtr outgoing_interface_;
    Ogre::SceneManager* mSceneManager = nullptr;
    Ogre::Camera* mCamera = nullptr;

    // Scene object tracking
    std::map<int, Ogre::Item*> entity_items_;
    std::map<int, Ogre::SceneNode*> entity_nodes_;

    ViewMode view_mode_ = ViewMode::FOLLOW;
    int follow_id_ = 0;
};
```

**Option B: Deterministic Loop (recommended for simulation)**

For tighter simulation synchronization, use a manual loop instead of FrameListener:

```cpp
void OgreNextViewer::run() {
    const float FIXED_TIMESTEP = 1.0f / 60.0f;
    float accumulator = 0.0f;
    Ogre::Timer timer;

    while (!mWindow->isClosed() && !mQuit) {
        Ogre::WindowEventUtilities::messagePump();

        float frameTime = timer.getMicroseconds() / 1000000.0f;
        timer.reset();
        accumulator += frameTime;

        // Fixed timestep logic updates
        while (accumulator >= FIXED_TIMESTEP) {
            processIncomingMessages();
            updateSceneObjects();
            updateCamera(FIXED_TIMESTEP);
            accumulator -= FIXED_TIMESTEP;
        }

        mRoot->renderOneFrame();
    }
}
```

#### Thread Safety Rules

1. **Network thread** only writes to thread-safe queues
2. **Main thread** (in `frameStarted`) reads from queues and updates OGRE scene
3. Never call OGRE scene modification from network thread

```cpp
// Thread-safe message queue pattern
std::mutex message_mutex_;
std::queue<Message> pending_messages_;

// In network thread:
{
    std::lock_guard<std::mutex> lock(message_mutex_);
    pending_messages_.push(msg);
}

// In frameStarted (main thread):
{
    std::lock_guard<std::mutex> lock(message_mutex_);
    while (!pending_messages_.empty()) {
        processMessage(pending_messages_.front());
        pending_messages_.pop();
    }
}
```

#### Camera Input

```cpp
class OgreNextCameraInterface {
 public:
    void injectMouseMove(int relX, int relY);
    void injectMouseButton(int button, bool pressed);
    void injectKeyPress(int key, bool pressed);

    void update(float dt);

 protected:
    Ogre::Camera* mCamera = nullptr;
    OgreNextUpdater* mUpdater = nullptr;

    // Input state
    bool mouse_left_down_ = false;
    bool mouse_right_down_ = false;
    float yaw_ = 0.0f;
    float pitch_ = 0.0f;
};
```

---

### Phase 4: Integration, QA, and Smoke Tests

#### CMake Integration

**CRITICAL:** The existing `src/viewer/CMakeLists.txt` uses `${OGRE_LIBRARIES}` from pkg-config which may not include the HLMS libraries. You MUST verify and fix this before building.

**Step 1: Verify pkg-config output**
```bash
pkg-config --libs OGRE-Next
# Look for: -lOgreHlmsPbs -lOgreHlmsUnlit
# If missing, you need to add them explicitly
```

**Step 2: Update CMakeLists.txt if needed**

The current `src/viewer/CMakeLists.txt` (lines ~97-127) needs HLMS libraries added:

```cmake
# In src/viewer/CMakeLists.txt, OGRE-Next section:
target_link_libraries(scrimmage-viewer-ogre-next
    scrimmage-core
    scrimmage-protos
    scrimmage-boost
    ${OGRE_LIBRARIES}
    OgreHlmsPbs       # REQUIRED - add if not in ${OGRE_LIBRARIES}
    OgreHlmsUnlit     # REQUIRED - add if not in ${OGRE_LIBRARIES}
)
```

**Step 3: Verify include paths**

The OGRE-Next 2.3 headers may be in `/usr/include/OGRE-Next/` rather than `/usr/include/OGRE/`. Verify:
```bash
pkg-config --cflags OGRE-Next
# Should show include path like -I/usr/include/OGRE-Next
```

If using the system package, ensure CMakeLists.txt adds:
```cmake
if (OGRE_INCLUDE_DIRS)
  target_include_directories(scrimmage-viewer-ogre-next PRIVATE ${OGRE_INCLUDE_DIRS})
endif()
```

#### Ideal CMake Setup (if creating separate CMakeLists.txt)

```cmake
# src/viewer/ogre_next/CMakeLists.txt (alternative standalone approach)
add_library(scrimmage_viewer_ogre
    OgreNextViewerBackend.cpp
    OgreNextViewer.cpp
    OgreNextUpdater.cpp
    OgreNextCameraInterface.cpp
    OgreNextGrid.cpp
    OgreNextOriginAxes.cpp
)

target_link_libraries(scrimmage_viewer_ogre
    PUBLIC
        scrimmage_viewer_interface
        OgreMain
        OgreHlmsPbs
        OgreHlmsUnlit
)

target_include_directories(scrimmage_viewer_ogre
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)
```

#### Runtime Data Setup

Configure-time copy or runtime resolution:

```cmake
# Copy HLMS data during configure
if(OGRE_MEDIA_DIR)
    file(COPY ${OGRE_MEDIA_DIR}/Hlms/Common
              ${OGRE_MEDIA_DIR}/Hlms/Pbs
              ${OGRE_MEDIA_DIR}/Hlms/Unlit
         DESTINATION ${CMAKE_BINARY_DIR}/bin/Data/Hlms)
endif()

# Generate resources2.cfg
configure_file(
    ${CMAKE_SOURCE_DIR}/CMake/Templates/OgreResources2.cfg.in
    ${CMAKE_BINARY_DIR}/bin/Data/resources2.cfg
    @ONLY
)

# Generate plugins.cfg
configure_file(
    ${CMAKE_SOURCE_DIR}/CMake/Templates/OgrePlugins.cfg.in
    ${CMAKE_BINARY_DIR}/bin/plugins.cfg
    @ONLY
)
```

#### plugins.cfg Content (Linux GL3+ Only)

Since this targets Linux desktop with GL3+/GLX only, keep the plugin config minimal:

```ini
# CMake/Templates/OgrePlugins.cfg.in
# Linux GL3+ only - no multi-backend fallback needed
PluginFolder=@OGRE_PLUGIN_DIR@
Plugin=RenderSystem_GL3Plus
```

Or if using system-installed OGRE:

```ini
# plugins.cfg for system-installed OGRE-Next
PluginFolder=/usr/lib/x86_64-linux-gnu/OGRE-Next/
Plugin=RenderSystem_GL3Plus
```

#### resources2.cfg Template

```ini
# CMake/Templates/OgreResources2.cfg.in
[Hlms]
DoNotUseAsResource=@CMAKE_BINARY_DIR@/bin/Data/Hlms/

[General]
FileSystem=@CMAKE_BINARY_DIR@/bin/Data/Materials
FileSystem=@CMAKE_BINARY_DIR@/bin/Data/Meshes
FileSystem=@CMAKE_BINARY_DIR@/bin/Data/Textures
```

#### ViewerFactory Improvements

Use modern C++ patterns in `ViewerFactory.cpp`:

```cpp
#include <memory>
#include <iostream>

std::unique_ptr<ViewerBackend> create_viewer_backend(ViewerBackendType backend) {
    switch (backend) {
        case ViewerBackendType::VTK:
#if SCRIMMAGE_HAVE_VTK_VIEWER
            return std::make_unique<VtkViewerBackend>();
#else
            std::cerr << "VTK viewer requested but not compiled in." << std::endl;
            return nullptr;
#endif
        case ViewerBackendType::OGRE_NEXT:
#if SCRIMMAGE_HAVE_OGRE_NEXT_VIEWER
            return std::make_unique<OgreNextViewerBackend>();
#else
            std::cerr << "OGRE-Next viewer requested but not compiled in." << std::endl;
            return nullptr;
#endif
    }
    return nullptr;
}
```

#### Smoke Test Commands

```bash
# 1. Ensure runtime data exists (see Phase 0.5)
ls data/ogre_next/Hlms/Common/  # Should show GLSL/, Any/, etc.
ls data/ogre_next/plugins.cfg   # Should exist
ls data/ogre_next/resources2.cfg # Should exist

# 2. Configure with OGRE enabled
cd build
cmake -DSCRIMMAGE_BUILD_OGRE_NEXT_VIEWER=ON -DCMAKE_BUILD_TYPE=Debug ..

# 3. Build
make -j$(nproc)

# 4. Set resource path (development mode)
export SCRIMMAGE_OGRE_MEDIA_DIR=/root/scrimmage/data/ogre_next/

# 5. Ensure DISPLAY is set (for containers)
echo $DISPLAY  # Should show :0 or similar

# 6. Run
./bin/scrimmage --ogre missions/straight.xml
```

**Troubleshooting:**

| Symptom | Cause | Fix |
|---------|-------|-----|
| "No DISPLAY environment variable" | X11 not available | Set `DISPLAY=:0` or run with X11 forwarding |
| "RenderSystem_GL3Plus not found" | plugins.cfg wrong path | Verify `PluginFolder=/usr/lib/x86_64-linux-gnu/OGRE-Next/` |
| "HLMS data not found" | Missing shader files | Run HLMS download from Phase 0.5 |
| Segfault on startup | OGRE init order wrong | Check DISPLAY before Root creation |
| Black window (no rendering) | Compositor not setup | Verify `setupCompositor()` succeeded |

#### Success Criteria

**Phase 1-2 (current):**
- [x] CMake configure succeeds with `SCRIMMAGE_BUILD_OGRE_NEXT_VIEWER=ON`
- [ ] Build completes without errors (blocked: OgreNextViewer.h/cpp missing)
- [ ] Window opens with dark background
- [ ] Clean shutdown without crashes

**Phase 3 (after OgreNextUpdater):**
- [ ] Grid and axes visible
- [ ] Entities appear and move based on simulation
- [ ] Camera controls work (follow, free, offset modes)

---

### Phase 5: Polishing & Extras

- [ ] PBS materials for entities (proper lighting)
- [ ] Unlit materials for debug/wireframe
- [ ] Shadow mapping (optional)
- [ ] README with devcontainer/X11 setup instructions
- [ ] Unit tests for resource path resolution
- [ ] Performance profiling

---

## Error Handling

| Error | Detection | User Message |
|-------|-----------|--------------|
| No DISPLAY | `getenv("DISPLAY")` null/empty | "Error: GUI enabled but no DISPLAY environment variable set. Run with enable_gui:=false or set DISPLAY for X11 forwarding." |
| Missing HLMS | Archive load fails | "Error: HLMS data not found at {path}. Set SCRIMMAGE_OGRE_MEDIA_DIR or ensure Hlms/{Common,Pbs,Unlit} exist." |
| No RenderSystem | Plugin load fails | "Error: RenderSystem_GL3Plus not found. Check plugins.cfg path." |
| Missing plugins.cfg | File not found | "Error: plugins.cfg not found at {path}. Ensure data/ogre_next/plugins.cfg exists." |
| Thread misuse | Debug assertion | Ensure all OGRE calls from main thread |

---

## Container/Devcontainer Support

For X11 passthrough in containers:

```json
// devcontainer.json
"runArgs": [
    "-e", "DISPLAY=${localEnv:DISPLAY}",
    "-v", "/tmp/.X11-unix:/tmp/.X11-unix",
    "--device=/dev/dri"
]
```

Host-side setup:

```bash
xhost +si:localuser:$(id -un)
```

---

## References

- `development-docs/OGRE-Next-Guide.md` — Source of truth for OGRE-Next 2.3 usage
- `src/viewer/vtk/` — Reference implementation for VTK viewer
- `include/scrimmage/viewer/ViewerBackend.h` — Interface to implement
- [OGRE-Next 2.3 API Docs](https://ogrecave.github.io/ogre-next/api/2.3/)
- [OGRE-Next GitHub](https://github.com/OGRECave/ogre-next)

---

## Current Implementation Status

### Completed Files

| File | Status | Notes |
|------|--------|-------|
| `src/viewer/ogre_next/OgreNextViewerBackend.h` | ✅ Complete | Includes `OgreNextViewer viewer_` member |
| `src/viewer/ogre_next/OgreNextViewerBackend.cpp` | ✅ Complete | Delegates to `viewer_` (matches VTK pattern) |
| `src/viewer/ogre_next/OgreNextViewer.h` | ✅ Complete | Full class declaration with OGRE 2.3 types |
| `src/viewer/ogre_next/OgreNextViewer.cpp` | ✅ Complete | Full initialization following best practices |
| `src/viewer/CMakeLists.txt` | ✅ Complete | OGRE-Next section with HLMS lib links |
| `src/viewer/ViewerFactory.cpp` | ✅ Complete | Backend selection logic |
| `include/scrimmage/viewer/ViewerBackend.h` | ✅ Complete | Enum includes OGRE_NEXT |
| `share/scrimmage/main.cpp` | ✅ Complete | `--ogre` flag handling |
| `CMakeLists.txt` (root) | ✅ Complete | OGRE-Next find with HLMS include paths |
| `data/ogre_next/plugins.cfg` | ✅ Complete | Points to system OGRE plugins |
| `data/ogre_next/resources2.cfg` | ✅ Complete | HLMS DoNotUseAsResource path |
| `data/ogre_next/Hlms/` | ✅ Complete | Downloaded from OGRE-Next 2.3.3 |

### Missing Files (Required for Build)

| File | Status | Blocks |
|------|--------|--------|
| ~~`src/viewer/ogre_next/OgreNextViewer.h`~~ | ✅ Created | N/A |
| ~~`src/viewer/ogre_next/OgreNextViewer.cpp`~~ | ✅ Created | N/A |

**BUILD STATUS:** ✅ Build succeeds with `SCRIMMAGE_BUILD_OGRE_NEXT_VIEWER=ON`

### Missing Files (Required for Runtime)

| File | Status | Priority |
|------|--------|----------|
| ~~`data/ogre_next/plugins.cfg`~~ | ✅ Created | N/A |
| ~~`data/ogre_next/resources2.cfg`~~ | ✅ Created | N/A |
| ~~`data/ogre_next/Hlms/`~~ | ✅ Downloaded | N/A |

### Missing Files (Phase 3)

| File | Status | Priority |
|------|--------|----------|
| `src/viewer/ogre_next/OgreNextUpdater.h/cpp` | ❌ Missing | Medium - For entity visualization |
| `src/viewer/ogre_next/OgreNextCameraInterface.h/cpp` | ❌ Missing | Medium - For user input |
| `src/viewer/ogre_next/OgreNextGrid.h/cpp` | ❌ Missing | Low - Visual aid |
| `src/viewer/ogre_next/OgreNextOriginAxes.h/cpp` | ❌ Missing | Low - Visual aid |

### Next Steps

**BEFORE ANY CODING - Verify Environment:**

0. **Verify OGRE-Next installation** - Run the pkg-config commands from Phase 0
1. **Verify HLMS libraries exist** - `ls /usr/lib/x86_64-linux-gnu/libOgreHlms*`
2. **Update CMakeLists.txt** - Add explicit HLMS library links and include dirs (see Phase 4)

**Before you can build with `SCRIMMAGE_BUILD_OGRE_NEXT_VIEWER=ON`:**

1. **Create `OgreNextViewer.h`** - Minimal header with class declaration
2. **Create `OgreNextViewer.cpp`** - Minimal implementation (can return false initially)
3. **Update `OgreNextViewerBackend.h`** - Add `OgreNextViewer viewer_` member
4. **Update `OgreNextViewerBackend.cpp`** - Delegate to `viewer_` (match VTK pattern)

**After build succeeds, before runtime testing:**

5. **Create `data/ogre_next/` directory**
6. **Create `plugins.cfg`** - Per Phase 0.5
7. **Create `resources2.cfg`** - Per Phase 0.5  
8. **Download HLMS shaders** - Per Phase 0.5 (curl command provided)

**First runtime test:**

9. **Set `SCRIMMAGE_OGRE_MEDIA_DIR`** environment variable
10. **Run `scrimmage --ogre missions/straight.xml`** - Should open window with dark background

---

## Version History

| Date | Author | Changes |
|------|--------|---------|
| 2026-04-17 | AI Assistant | Initial plan created |
| 2026-04-17 | AI Assistant | Added: Root constructor app name (2.3 API), threading design note, light power for LDR, deterministic loop option, DISPLAY check timing, plugins.cfg content, resources2.cfg template, ViewerFactory improvements |
| 2026-04-17 | AI Assistant | Added: Phase 0.5 (runtime data files), Quick Start section, implementation status tracking, troubleshooting table, updated checklist with completion status |
| 2026-04-17 | AI Assistant | Review: Fixed file list (removed compositor script), added full init sequence code, clarified CMake HLMS linkage, updated status table, added concrete next steps checklist |
| 2026-04-17 | AI Assistant | Review pass: Added pkg-config verification steps, emphasized HLMS library linkage requirement, added pre-implementation environment checks, clarified immediate blockers |
| 2026-04-17 | AI Assistant | Implementation: Created OgreNextViewer.h/cpp, updated OgreNextViewerBackend to delegate pattern, created plugins.cfg/resources2.cfg, downloaded HLMS data, fixed CMake OGRE-Next include paths (added Hlms/Common), fixed OGRE 2.3 API (Window vs RenderWindow). Build succeeds. |
| 2026-04-17 | AI Assistant | **PHASE 2 COMPLETE**: Fixed black window bug by deferring ALL OGRE init to run() (GL context thread affinity). Added Threading Model section documenting the VTK pattern. Window now renders bright red background. Added detailed comments to OgreNextViewer.cpp explaining threading model. |

---

## Detailed Changelog - Phase 2 Completion

### Problem: Black Window Despite Successful Initialization

**Symptoms:**
- OGRE reported successful initialization
- Compositor workspace created successfully
- renderOneFrame() returned true
- BUT window showed solid black instead of red background

**Root Cause:**
Scrimmage's architecture calls `init()` on the main thread and `run()` on a separate viewer thread. OpenGL contexts are thread-bound - a context created on one thread cannot be used for rendering from another thread.

The original code created the GL context (via `createRenderWindow()`) in `init()` on the main thread, but then `run()` tried to render on the viewer thread where the context wasn't current.

**Solution:**
Follow VTK's pattern - defer ALL OGRE object creation to `run()`:

```cpp
// init() - Main thread: ONLY store config
bool OgreNextViewer::init(...) {
    // Validate DISPLAY
    // Store camera_params_, log_dir_, window_width_, etc.
    // Do NOT create any OGRE objects
    return true;
}

// run() - Viewer thread: Create OGRE objects and render
bool OgreNextViewer::run() {
    initOgre();  // Creates Root, Window, SceneManager, Compositor
    while (!quit) {
        mRoot->renderOneFrame();  // Renders on same thread as context
    }
}
```

### Files Changed

**New Files:**
- `src/viewer/ogre_next/OgreNextViewer.h` (148 lines)
- `src/viewer/ogre_next/OgreNextViewer.cpp` (471 lines)

**Modified Files:**
- `src/viewer/ogre_next/OgreNextViewerBackend.h` - Added `OgreNextViewer viewer_` member
- `src/viewer/ogre_next/OgreNextViewerBackend.cpp` - Delegates to viewer_ instead of stub implementation
- `development-docs/OGRE-Next-Guide.md` - Added "OpenGL Context Thread Affinity" section

### CMake Notes

The `_DEBUG=1` flag is set via `add_compile_definitions()` in the root CMakeLists.txt. For clangd to see this flag, you must run cmake with `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`.
