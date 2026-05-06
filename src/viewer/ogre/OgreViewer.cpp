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

#include "scrimmage/viewer/ogre/OgreViewer.h"
#include "scrimmage/viewer/ogre/CoordinateConverter.h"
#include "scrimmage/network/Interface.h"
#include "scrimmage/parse/MissionParse.h"
#include "scrimmage/proto/GUIControl.pb.h"

#include <OGRE/OgreLight.h>
#include <OGRE/OgreLogManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreBillboardSet.h>
#include <OGRE/OgreBillboard.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgrePass.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/OgreMath.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>

namespace scrimmage {
namespace viewer {

// UpdateListener implementation
bool OgreViewer::UpdateListener::frameStarted(const Ogre::FrameEvent& evt) {
    viewer_->processInterfaceUpdates();
    return !viewer_->shutting_down_;
}

bool OgreViewer::UpdateListener::frameRenderingQueued(const Ogre::FrameEvent& evt) {
    viewer_->update(evt.timeSinceLastFrame);
    return !viewer_->shutting_down_;
}

OgreViewer::OgreViewer()
    : OgreBites::ApplicationContext("SCRIMMAGE") {
    // Suppress verbose Ogre logging - only show warnings and errors
    Ogre::LogManager* logMgr = new Ogre::LogManager();
    Ogre::Log* log = logMgr->createLog("Ogre.log", true, false, true);  // default, no console, suppressed
    log->setLogDetail(Ogre::LL_LOW);  // Only log low-priority (important) messages
}

OgreViewer::~OgreViewer() {
    if (network_thread_.joinable()) {
        network_thread_.join();
    }
}

bool OgreViewer::oneTimeConfig() {
    // Skip the config dialog - automatically select OpenGL render system
    Ogre::Root* root = getRoot();
    const auto& renderers = root->getAvailableRenderers();
    
    Ogre::RenderSystem* selected = nullptr;
    
    // Try to find OpenGL 3+ render system first, then fall back to any OpenGL
    for (auto* rs : renderers) {
        std::string name = rs->getName();
        if (name.find("OpenGL 3+") != std::string::npos) {
            selected = rs;
            break;
        }
    }
    
    if (!selected) {
        for (auto* rs : renderers) {
            std::string name = rs->getName();
            if (name.find("OpenGL") != std::string::npos) {
                selected = rs;
                break;
            }
        }
    }
    
    // Fall back to first available renderer
    if (!selected && !renderers.empty()) {
        selected = renderers.front();
    }
    
    if (!selected) {
        std::cerr << "No render system available" << std::endl;
        return false;
    }
    
    std::cout << "Using render system: " << selected->getName() << std::endl;
    root->setRenderSystem(selected);
    
    // Configure basic render system options
    selected->setConfigOption("Full Screen", "No");
    selected->setConfigOption("Video Mode", "1280 x 800 @ 32-bit colour");
    
    return true;
}

void OgreViewer::set_incoming_interface(InterfacePtr& incoming_interface) {
    incoming_interface_ = incoming_interface;
}

void OgreViewer::set_outgoing_interface(InterfacePtr& outgoing_interface) {
    outgoing_interface_ = outgoing_interface;
}

void OgreViewer::set_enable_network(bool enable) {
    enable_network_ = enable;
}

bool OgreViewer::init(const std::shared_ptr<MissionParse>& mp,
                      const std::map<std::string, std::string>& camera_params) {
    camera_params_ = camera_params;
    log_dir_ = mp->log_dir();
    dt_ = mp->dt();
    
    // Parse camera parameters (store for later use in run())
    auto it = camera_params.find("pos_x");
    init_pos_x_ = (it != camera_params.end()) ? std::stod(it->second) : 0.0;
    
    it = camera_params.find("pos_y");
    init_pos_y_ = (it != camera_params.end()) ? std::stod(it->second) : 0.0;
    
    it = camera_params.find("pos_z");
    init_pos_z_ = (it != camera_params.end()) ? std::stod(it->second) : 200.0;
    
    it = camera_params.find("focal_x");
    init_focal_x_ = (it != camera_params.end()) ? std::stod(it->second) : 0.0;
    
    it = camera_params.find("focal_y");
    init_focal_y_ = (it != camera_params.end()) ? std::stod(it->second) : 0.0;
    
    it = camera_params.find("focal_z");
    init_focal_z_ = (it != camera_params.end()) ? std::stod(it->second) : 0.0;

    // Note: Ogre initialization is deferred to run() to ensure GL context is
    // created and used from the same thread.
    return true;
}

void OgreViewer::setup() {
    // Call base setup
    OgreBites::ApplicationContext::setup();
    
    // Register as input listener
    addInputListener(this);
    
    // Get scene manager
    scene_mgr_ = getRoot()->createSceneManager();
    
    // Register for resource loading
    Ogre::RTShader::ShaderGenerator* shadergen = 
        Ogre::RTShader::ShaderGenerator::getSingletonPtr();
    if (shadergen) {
        shadergen->addSceneManager(scene_mgr_);
    }
    
    // Create camera
    camera_ = scene_mgr_->createCamera("MainCamera");
    camera_->setNearClipDistance(0.5);
    camera_->setFarClipDistance(10000);
    camera_->setAutoAspectRatio(true);
    
    // Create camera node
    cam_node_ = scene_mgr_->getRootSceneNode()->createChildSceneNode("CameraNode");
    cam_node_->attachObject(camera_);
    cam_node_->setPosition(0, 200, 100);
    cam_node_->lookAt(Ogre::Vector3(0, 0, 0), Ogre::Node::TS_WORLD);
    
    // Create viewport — background matches the skydome horizon so the seam
    // disappears if the dome is ever culled.
    render_window_ = getRenderWindow();
    Ogre::Viewport* vp = render_window_->addViewport(camera_);
    vp->setBackgroundColour(Ogre::ColourValue(0.78f, 0.84f, 0.90f));
    
    // Initialize material pool
    material_pool_ = std::make_unique<MaterialPool>();
    material_pool_->init();
    
    // Initialize renderers
    contact_renderer_ = std::make_unique<ContactRenderer>(scene_mgr_, material_pool_.get());
    contact_renderer_->init();
    
    shape_renderer_ = std::make_unique<ShapeRenderer>(scene_mgr_, material_pool_.get());
    shape_renderer_->init();
    
    // Initialize camera controller
    camera_controller_ = std::make_unique<CameraController>(
        camera_, cam_node_, contact_renderer_.get());
    camera_controller_->init();
    
    // Create scene elements
    createScene();
    
    // Create update listener
    update_listener_ = std::make_unique<UpdateListener>(this);
    getRoot()->addFrameListener(update_listener_.get());
    getRoot()->addFrameListener(camera_controller_.get());
}

void OgreViewer::createScene() {
    // Calm-midday ambient — slightly cool sky bounce, lifted enough that the
    // unlit side of geometry doesn't go dark (no high-contrast shadows).
    scene_mgr_->setAmbientLight(Ogre::ColourValue(0.46f, 0.49f, 0.55f));

    // Sun nearer overhead with a touch of southern bias — early-afternoon
    // light, neutral white (no sunset warmth).
    Ogre::Light* sun = scene_mgr_->createLight("Sun");
    sun->setType(Ogre::Light::LT_DIRECTIONAL);
    sun->setDiffuseColour(Ogre::ColourValue(0.96f, 0.96f, 0.94f));
    sun->setSpecularColour(Ogre::ColourValue(0.40f, 0.40f, 0.38f));

    Ogre::SceneNode* sunNode = scene_mgr_->getRootSceneNode()->createChildSceneNode("SunNode");
    sunNode->attachObject(sun);
    sunNode->setDirection(Ogre::Vector3(-0.30f, -1.0f, -0.20f).normalisedCopy());

    // Stylized sky (gradient dome + soft cloud puffs)
    createSky();

    // Create grid and origin axes
    if (show_grid_) {
        createGrid();
    }
    if (show_origin_) {
        createOriginAxes();
    }
}

void OgreViewer::createGrid() {
    const float size = 1000.0f;
    const float step = 50.0f;
    const int lines = static_cast<int>(size / step) * 2 + 1;
    
    Ogre::ManualObject* grid = scene_mgr_->createManualObject("Grid");
    std::string matName = material_pool_->getMaterial(128, 128, 128, 0.5f);
    
    grid->begin(matName, Ogre::RenderOperation::OT_LINE_LIST);
    
    // Lines along X axis
    for (int i = 0; i < lines; ++i) {
        float z = -size + i * step;
        grid->position(-size, 0, z);
        grid->colour(0.5f, 0.5f, 0.5f);
        grid->position(size, 0, z);
        grid->colour(0.5f, 0.5f, 0.5f);
    }
    
    // Lines along Z axis
    for (int i = 0; i < lines; ++i) {
        float x = -size + i * step;
        grid->position(x, 0, -size);
        grid->colour(0.5f, 0.5f, 0.5f);
        grid->position(x, 0, size);
        grid->colour(0.5f, 0.5f, 0.5f);
    }
    
    grid->end();
    
    Ogre::SceneNode* gridNode = scene_mgr_->getRootSceneNode()->createChildSceneNode("GridNode");
    gridNode->attachObject(grid);
}

void OgreViewer::createOriginAxes() {
    const float length = 50.0f;
    
    Ogre::ManualObject* axes = scene_mgr_->createManualObject("OriginAxes");
    
    // X axis (red) - in Ogre this is East
    std::string redMat = material_pool_->getMaterial(255, 0, 0, 1.0f);
    axes->begin(redMat, Ogre::RenderOperation::OT_LINE_LIST);
    axes->position(0, 0, 0);
    axes->colour(1, 0, 0);
    axes->position(length, 0, 0);
    axes->colour(1, 0, 0);
    axes->end();
    
    // Y axis (green) - in Ogre this is Up (ENU Z)
    std::string greenMat = material_pool_->getMaterial(0, 255, 0, 1.0f);
    axes->begin(greenMat, Ogre::RenderOperation::OT_LINE_LIST);
    axes->position(0, 0, 0);
    axes->colour(0, 1, 0);
    axes->position(0, length, 0);
    axes->colour(0, 1, 0);
    axes->end();
    
    // Z axis (blue) - in Ogre this is -North (ENU -Y)
    std::string blueMat = material_pool_->getMaterial(0, 0, 255, 1.0f);
    axes->begin(blueMat, Ogre::RenderOperation::OT_LINE_LIST);
    axes->position(0, 0, 0);
    axes->colour(0, 0, 1);
    axes->position(0, 0, length);
    axes->colour(0, 0, 1);
    axes->end();
    
    Ogre::SceneNode* axesNode = scene_mgr_->getRootSceneNode()->createChildSceneNode("AxesNode");
    axesNode->attachObject(axes);
}

void OgreViewer::createSky() {
    using Ogre::ColourValue;
    const std::string group =
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;

    // ---------- Skydome material (unlit, vertex-colored gradient) ----------
    Ogre::MaterialPtr skyMat =
        Ogre::MaterialManager::getSingleton().create("SCRIMMAGE/SkyDome", group);
    Ogre::Pass* skyPass = skyMat->getTechnique(0)->getPass(0);
    skyPass->setLightingEnabled(false);
    skyPass->setDepthWriteEnabled(false);
    skyPass->setDepthCheckEnabled(false);
    skyPass->setCullingMode(Ogre::CULL_NONE);
    skyPass->setVertexColourTracking(Ogre::TVC_DIFFUSE);
    skyMat->load();

    // ---------- Skydome geometry: inverted hemisphere with gradient ----------
    // Realistic atmosphere — four-stop blend with most of the variation
    // packed into the lowest 25% of the dome (where atmospheric scattering
    // and haze do most of their work in real skies):
    //   ti=0.00  zenith     medium-desaturated blue
    //   ti=0.45  upper sky  paler blue
    //   ti=0.80  low sky    pale bluish-gray
    //   ti=1.00  horizon    muted warm-gray haze band
    const ColourValue zenith    (0.26f, 0.48f, 0.78f);
    const ColourValue upper     (0.42f, 0.62f, 0.86f);
    const ColourValue lower     (0.66f, 0.78f, 0.90f);
    const ColourValue horizonHz (0.78f, 0.84f, 0.90f);
    const float p0 = 0.45f;
    const float p1 = 0.80f;
    const float radius = 5000.0f;
    const int rings = 48;        // dense enough to avoid banding in the haze band
    const int segments = 48;
    // Extend slightly past 90° so the dome closes below the horizon line.
    const float thetaMax = Ogre::Math::PI * 0.55f;

    Ogre::ManualObject* dome = scene_mgr_->createManualObject("SkyDome");
    dome->setRenderQueueGroup(Ogre::RENDER_QUEUE_SKIES_EARLY);
    dome->setCastShadows(false);
    dome->begin("SCRIMMAGE/SkyDome", Ogre::RenderOperation::OT_TRIANGLE_LIST);

    for (int i = 0; i <= rings; ++i) {
        float ti = static_cast<float>(i) / static_cast<float>(rings);
        float theta = ti * thetaMax;
        float y = std::cos(theta);
        float rxz = std::sin(theta);
        // Three-segment blend across four color stops.
        ColourValue c;
        if (ti < p0) {
            float t = ti / p0;
            c = zenith * (1.0f - t) + upper * t;
        } else if (ti < p1) {
            float t = (ti - p0) / (p1 - p0);
            c = upper * (1.0f - t) + lower * t;
        } else {
            float t = (ti - p1) / (1.0f - p1);
            c = lower * (1.0f - t) + horizonHz * t;
        }
        for (int j = 0; j <= segments; ++j) {
            float pj = static_cast<float>(j) / static_cast<float>(segments);
            float phi = pj * 2.0f * Ogre::Math::PI;
            float x = rxz * std::cos(phi);
            float z = rxz * std::sin(phi);
            dome->position(x * radius, y * radius, z * radius);
            dome->colour(c);
        }
    }
    for (int i = 0; i < rings; ++i) {
        for (int j = 0; j < segments; ++j) {
            int row0 = i * (segments + 1);
            int row1 = (i + 1) * (segments + 1);
            int a = row0 + j;
            int b = row0 + j + 1;
            int cc = row1 + j;
            int d = row1 + j + 1;
            // Reversed winding: we view the inside of the sphere.
            dome->triangle(a, cc, b);
            dome->triangle(b, cc, d);
        }
    }
    dome->end();

    sky_node_ = scene_mgr_->getRootSceneNode()->createChildSceneNode("SkyNode");
    sky_node_->attachObject(dome);

    // ---------- Procedural cloud-puff texture (multi-lobe, crisp edge) ----------
    // The previous version was a single wide Gaussian, which has no real edge
    // and reads as a hazy blur. This builds a multi-lobed silhouette with a
    // tight smoothstep edge so each puff has a clean, defined outline while
    // still feeling soft at the very rim.
    const std::string cloudTexName = "SCRIMMAGE/CloudPuff";
    Ogre::TexturePtr cloudTex = Ogre::TextureManager::getSingleton().createManual(
        cloudTexName, group, Ogre::TEX_TYPE_2D,
        256, 256, 0, Ogre::PF_BYTE_BGRA, Ogre::TU_DEFAULT);

    struct Lobe { float cx, cy, r; };
    // Wide stratocumulus-style silhouette: many overlapping lobes so the
    // outline reads as one cohesive cloud rather than a ring of distinct
    // bumps. Coords are in [-1, 1]; lobes stay inside the texture footprint.
    const Lobe lobes[] = {
        // Body — tightly overlapping, defines the bulk
        {-0.45f, -0.05f, 0.40f},
        {-0.18f,  0.00f, 0.46f},
        { 0.10f, -0.02f, 0.46f},
        { 0.40f, -0.06f, 0.40f},
        // Top edge — small irregular bumps that merge with the body
        {-0.30f,  0.20f, 0.28f},
        {-0.05f,  0.26f, 0.30f},
        { 0.22f,  0.22f, 0.26f},
        // Base — wider lobes pushed downward for a flat-ish bottom
        {-0.42f, -0.30f, 0.36f},
        {-0.10f, -0.34f, 0.40f},
        { 0.22f, -0.34f, 0.38f},
        { 0.50f, -0.28f, 0.32f},
    };
    const int numLobes = static_cast<int>(sizeof(lobes) / sizeof(lobes[0]));

    Ogre::HardwarePixelBufferSharedPtr buf = cloudTex->getBuffer();
    buf->lock(Ogre::HardwareBuffer::HBL_DISCARD);
    const Ogre::PixelBox& pb = buf->getCurrentLock();
    auto* dst = static_cast<std::uint8_t*>(pb.data);
    const int W = static_cast<int>(pb.getWidth());
    const int H = static_cast<int>(pb.getHeight());
    const std::size_t rowBytes = pb.rowPitch * 4;  // rowPitch is in pixels
    // Feathered edge band: solid inside [0, edge0], wider smoothstep falloff
    // to 0 at d = 1.0 — gives the soft cumulus rim instead of a crisp outline.
    const float edge0 = 0.55f;
    // Per-pixel shading: light bluish-gray near the base, white near the top,
    // for subtle volumetric reading.
    const float shadowR = 0.80f, shadowG = 0.83f, shadowB = 0.88f;
    const float litR    = 1.00f, litG    = 1.00f, litB    = 1.00f;
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            float nx = (x - (W - 1) * 0.5f) / ((W - 1) * 0.5f);
            float ny = (y - (H - 1) * 0.5f) / ((H - 1) * 0.5f);
            float a = 0.0f;
            for (int k = 0; k < numLobes; ++k) {
                float ddx = (nx - lobes[k].cx) / lobes[k].r;
                float ddy = (ny - lobes[k].cy) / lobes[k].r;
                float d = std::sqrt(ddx * ddx + ddy * ddy);
                float la;
                if (d <= edge0) {
                    la = 1.0f;
                } else if (d < 1.0f) {
                    float t = (d - edge0) / (1.0f - edge0);
                    la = 1.0f - t * t * (3.0f - 2.0f * t);  // smoothstep
                } else {
                    la = 0.0f;
                }
                if (la > a) a = la;
            }
            // Vertical brightness ramp: ny=+1 (top) lit, ny=-1 (bottom) shaded.
            float ny01 = std::clamp((ny + 1.0f) * 0.5f, 0.0f, 1.0f);
            float shade = std::pow(ny01, 1.15f);  // ease-in: more shade at base
            float r = shadowR + (litR - shadowR) * shade;
            float g = shadowG + (litG - shadowG) * shade;
            float b = shadowB + (litB - shadowB) * shade;
            // Premultiplied alpha: storing RGB*A means transparent texels carry
            // zero color, so hardware mipmap averaging can't bleed white into
            // the corners of the billboard quad.
            float aClamped = std::clamp(a, 0.0f, 1.0f);
            std::uint8_t a8 = static_cast<std::uint8_t>(std::round(aClamped * 255.0f));
            std::uint8_t* p = dst + y * rowBytes + x * 4;
            p[0] = static_cast<std::uint8_t>(std::round(b * aClamped * 255.0f));
            p[1] = static_cast<std::uint8_t>(std::round(g * aClamped * 255.0f));
            p[2] = static_cast<std::uint8_t>(std::round(r * aClamped * 255.0f));
            p[3] = a8;
        }
    }
    buf->unlock();

    // ---------- Cloud material ----------
    Ogre::MaterialPtr cloudMat =
        Ogre::MaterialManager::getSingleton().create("SCRIMMAGE/Clouds", group);
    Ogre::Pass* cp = cloudMat->getTechnique(0)->getPass(0);
    cp->setLightingEnabled(false);
    cp->setDepthWriteEnabled(false);
    // Premultiplied alpha blend (texture stores RGB*A): src is added directly,
    // dest is attenuated by 1-A. This is what kills the white-rectangle halo.
    cp->setSceneBlending(Ogre::SBF_ONE, Ogre::SBF_ONE_MINUS_SOURCE_ALPHA);
    cp->setCullingMode(Ogre::CULL_NONE);
    cp->setVertexColourTracking(Ogre::TVC_DIFFUSE);
    // Only discard true zero-alpha texels so the feathered rim stays soft.
    cp->setAlphaRejectSettings(Ogre::CMPF_GREATER_EQUAL, 4);
    Ogre::TextureUnitState* tu = cp->createTextureUnitState(cloudTexName);
    tu->setTextureFiltering(Ogre::TFO_ANISOTROPIC);
    tu->setTextureAnisotropy(8);
    tu->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
    // Slight negative mip bias keeps distant puff edges crisp.
    tu->setTextureMipmapBias(-0.5f);
    cloudMat->load();

    // ---------- Cloud billboards ----------
    Ogre::BillboardSet* bbs = scene_mgr_->createBillboardSet("CloudPuffs");
    bbs->setMaterialName("SCRIMMAGE/Clouds");
    bbs->setBillboardType(Ogre::BBT_POINT);
    bbs->setBillboardRotationType(Ogre::BBR_TEXCOORD);
    bbs->setRenderQueueGroup(Ogre::RENDER_QUEUE_SKIES_LATE);
    bbs->setSortingEnabled(true);
    bbs->setCastShadows(false);

    // Asymmetric polar distribution: most clouds in a "front" 180° sector
    // with a triangular bias toward its center (upper-right / upper-center),
    // a small fraction scattered through the back sector ("left" stays sparse).
    // Radius is biased outward so the horizon ring carries most of the mass.
    // Per-billboard tint blends toward the horizon haze with distance, giving
    // realistic atmospheric perspective. ~30% of billboards become wispy
    // (lower opacity, flatter aspect) to read as semi-transparent layers.
    std::mt19937 rng(20260506u);
    std::uniform_real_distribution<float> u01(0.0f, 1.0f);
    std::uniform_real_distribution<float> uy(750.0f, 1700.0f);
    std::uniform_real_distribution<float> uvar(0.80f, 1.35f);
    std::uniform_real_distribution<float> uarMain(0.55f, 0.70f);
    std::uniform_real_distribution<float> uarWisp(0.32f, 0.45f);
    std::uniform_real_distribution<float> uaMain(0.78f, 0.95f);
    std::uniform_real_distribution<float> uaWisp(0.28f, 0.50f);
    std::uniform_real_distribution<float> urot(0.0f, 2.0f * Ogre::Math::PI);

    const int kClouds = 80;
    const float minRadius = 600.0f;
    const float maxRadius = 8500.0f;
    const float frontWeight = 0.88f;             // share of clouds in front sector
    const float frontStart = 0.0f;               // [0, π] — front 180°
    const float frontSpan = Ogre::Math::PI;
    const Ogre::ColourValue hazeTint(0.78f, 0.84f, 0.92f);

    for (int i = 0; i < kClouds; ++i) {
        // ----- Angular placement -----
        float angle;
        if (u01(rng) < frontWeight) {
            // Triangular bias toward the center of the front sector
            // (sum of two uniforms ⇒ symmetric peaked pdf).
            float t = (u01(rng) + u01(rng)) * 0.5f;
            angle = frontStart + t * frontSpan;
        } else {
            // Sparse back sector: plain uniform so left side stays open.
            angle = Ogre::Math::PI + u01(rng) * Ogre::Math::PI;
        }

        // ----- Radial placement biased outward -----
        float r01 = std::pow(u01(rng), 0.45f);
        float dist = minRadius + r01 * (maxRadius - minRadius);
        float x = dist * std::cos(angle);
        float z = dist * std::sin(angle);
        float y = uy(rng);

        // ----- Size scales with distance (constant-ish angular size) -----
        bool isWisp = (u01(rng) < 0.30f);
        float w = (260.0f + dist * 0.16f) * uvar(rng);
        if (isWisp) w *= 1.25f;  // wisps spread a touch wider
        float ar = isWisp ? uarWisp(rng) : uarMain(rng);
        float h = w * ar;

        // ----- Atmospheric perspective: tint toward horizon haze with distance -----
        float distFactor = (dist - minRadius) / (maxRadius - minRadius);
        float hazeBlend = std::clamp(distFactor, 0.0f, 1.0f) * 0.55f;
        float r = (1.0f - hazeBlend) + hazeTint.r * hazeBlend;
        float g = (1.0f - hazeBlend) + hazeTint.g * hazeBlend;
        float bcol = (1.0f - hazeBlend) + hazeTint.b * hazeBlend;
        float alpha = isWisp ? uaWisp(rng) : uaMain(rng);

        Ogre::Billboard* b = bbs->createBillboard(Ogre::Vector3(x, y, z));
        b->setDimensions(w, h);
        // Vertex tint must also be premultiplied so it composes with the
        // premultiplied texture and stays in premultiplied space.
        b->setColour(Ogre::ColourValue(r * alpha, g * alpha, bcol * alpha, alpha));
        b->setRotation(Ogre::Radian(urot(rng)));
    }

    cloud_node_ =
        scene_mgr_->getRootSceneNode()->createChildSceneNode("CloudNode");
    cloud_node_->attachObject(bbs);
}

bool OgreViewer::run() {
    // Initialize Ogre in the viewer thread (GL context must be created in same thread that uses it)
    if (!initialized_) {
        initApp();
        initialized_ = true;
        
        // Set camera reset parameters (now that camera_controller_ is created)
        if (camera_controller_) {
            CameraResetParams params;
            params.pos_x = init_pos_x_;
            params.pos_y = init_pos_y_;
            params.pos_z = init_pos_z_;
            params.focal_x = init_focal_x_;
            params.focal_y = init_focal_y_;
            params.focal_z = init_focal_z_;
            camera_controller_->setResetParams(params);
            camera_controller_->resetCamera();
        }
    }
    
    if (!getRoot()) {
        return false;
    }
    
    // Start main loop
    getRoot()->startRendering();
    
    return true;
}

void OgreViewer::shutdown() {
    shutting_down_ = true;
}

void OgreViewer::processInterfaceUpdates() {
    if (!incoming_interface_) return;
    
    // Check for shutdown signal from SimControl via SimInfo
    {
        std::lock_guard<std::mutex> lock(incoming_interface_->sim_info_mutex);
        auto& info_list = incoming_interface_->sim_info();
        for (const auto& info : info_list) {
            if (info.shutting_down()) {
                shutting_down_ = true;
                info_list.clear();
                return;
            }
        }
        info_list.clear();
    }
    
    // Process frames
    {
        std::lock_guard<std::mutex> lock(incoming_interface_->frames_mutex);
        auto& frames = incoming_interface_->frames();
        while (!frames.empty()) {
            auto frame = frames.front();
            frames.pop_front();
            
            if (frame) {
                frame_time_ = frame->time();
                contact_renderer_->updateContacts(*frame);
            }
        }
    }
    
    // Process shapes
    {
        std::lock_guard<std::mutex> lock(incoming_interface_->shapes_mutex);
        auto& shapes_list = incoming_interface_->shapes();
        while (!shapes_list.empty()) {
            auto shapes = shapes_list.front();
            shapes_list.pop_front();
            shape_renderer_->processShapes(shapes);
        }
    }
    
    // Process contact visuals
    {
        std::lock_guard<std::mutex> lock(incoming_interface_->contact_visual_mutex);
        auto& cv_list = incoming_interface_->contact_visual();
        while (!cv_list.empty()) {
            auto cv = cv_list.front();
            cv_list.pop_front();
            if (cv) {
                contact_renderer_->updateContactVisual(*cv);
            }
        }
    }
}

void OgreViewer::update(double dt) {
    // Interpolate entity positions for smooth rendering
    contact_renderer_->interpolateContacts(static_cast<float>(dt));

    // Update shape renderer (handles TTL)
    shape_renderer_->update(dt);

    // Keep the sky centered on the camera so the horizon never recedes,
    // and slide the cloud layer with the camera in X/Z while leaving its
    // altitude fixed in world space.
    if (camera_) {
        const Ogre::Vector3 camPos = camera_->getDerivedPosition();
        if (sky_node_) {
            sky_node_->setPosition(camPos);
        }
        if (cloud_node_) {
            cloud_node_->setPosition(camPos.x, 0.0f, camPos.z);
        }
    }
}

void OgreViewer::sendGuiMsg(const std::string& type, int value) {
    if (!outgoing_interface_) return;
    
    scrimmage_proto::GUIMsg msg;
    
    if (type == "pause") {
        msg.set_toggle_pause(true);
        paused_ = !paused_;
    } else if (type == "single_step") {
        msg.set_single_step(true);
    } else if (type == "inc_warp") {
        msg.set_inc_warp(1);
    } else if (type == "dec_warp") {
        msg.set_dec_warp(1);
    } else if (type == "shutdown") {
        msg.set_shutting_down(true);
    }
    
    outgoing_interface_->push_gui_msg(msg);
}

bool OgreViewer::keyPressed(const OgreBites::KeyboardEvent& evt) {
    switch (evt.keysym.sym) {
        case OgreBites::SDLK_ESCAPE:
            sendGuiMsg("shutdown");
            shutdown();
            break;
            
        case OgreBites::SDLK_SPACE:
            sendGuiMsg("pause");
            break;
            
        case 'n':
            sendGuiMsg("single_step");
            break;
            
        case 'r':
            camera_controller_->resetCamera();
            break;
            
        case 'a':
            camera_controller_->nextMode();
            break;
            
        case OgreBites::SDLK_RIGHT:
            contact_renderer_->nextFollow();
            break;
            
        case OgreBites::SDLK_LEFT:
            contact_renderer_->prevFollow();
            break;
            
        case 't':
            contact_renderer_->toggleTrails();
            break;
            
        case '+':
        case '=':
            warp_ = std::min(warp_ + 1, 100);
            sendGuiMsg("inc_warp");
            break;
            
        case '-':
            warp_ = std::max(warp_ - 1, 1);
            sendGuiMsg("dec_warp");
            break;
            
        case '.':
            scale_ *= 1.2;
            contact_renderer_->setScale(scale_);
            break;
            
        case ',':
            scale_ /= 1.2;
            if (scale_ < 0.1) scale_ = 0.1;
            contact_renderer_->setScale(scale_);
            break;
            
        case OgreBites::SDLK_UP:
            camera_controller_->decFollowOffset();
            break;
            
        case OgreBites::SDLK_DOWN:
            camera_controller_->incFollowOffset();
            break;
            
        case 'u':
            camera_controller_->undoCamera();
            break;
            
        case 'c':
            camera_controller_->trackCameraPos();
            break;
            
        case 'g':
            // Toggle grid visibility
            {
                Ogre::SceneNode* gridNode = scene_mgr_->getSceneNode("GridNode");
                if (gridNode) {
                    gridNode->flipVisibility();
                }
            }
            break;
            
        default:
            camera_controller_->injectKeyDown(evt.keysym.sym);
            break;
    }
    
    return true;
}

bool OgreViewer::keyReleased(const OgreBites::KeyboardEvent& evt) {
    camera_controller_->injectKeyUp(evt.keysym.sym);
    return true;
}

bool OgreViewer::mouseMoved(const OgreBites::MouseMotionEvent& evt) {
    if (evt.type == OgreBites::MOUSEMOTION) {
        camera_controller_->mouseMoved(static_cast<float>(evt.xrel),
                                       static_cast<float>(evt.yrel));
    }
    return true;
}

bool OgreViewer::mousePressed(const OgreBites::MouseButtonEvent& evt) {
    camera_controller_->mousePressed(evt.button);
    return true;
}

bool OgreViewer::mouseReleased(const OgreBites::MouseButtonEvent& evt) {
    camera_controller_->mouseReleased(evt.button);
    return true;
}

bool OgreViewer::mouseWheelRolled(const OgreBites::MouseWheelEvent& evt) {
    camera_controller_->mouseWheel(static_cast<float>(evt.y));
    return true;
}

}  // namespace viewer
}  // namespace scrimmage
