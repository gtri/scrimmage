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
 * @brief Ogre3D viewer implementation.
 */

#include "scrimmage/viewer/ogre/OgreViewer.h"
#include "scrimmage/viewer/ogre/CoordinateConverter.h"
#include "scrimmage/network/Interface.h"
#include "scrimmage/parse/MissionParse.h"
#include "scrimmage/proto/GUIControl.pb.h"

#include <OGRE/OgreLight.h>
#include <OGRE/OgreManualObject.h>
#include <iostream>

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
}

OgreViewer::~OgreViewer() {
    if (network_thread_.joinable()) {
        network_thread_.join();
    }
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
    
    // Create viewport
    render_window_ = getRenderWindow();
    Ogre::Viewport* vp = render_window_->addViewport(camera_);
    vp->setBackgroundColour(Ogre::ColourValue(0.2f, 0.3f, 0.4f));
    
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
    // Ambient light
    scene_mgr_->setAmbientLight(Ogre::ColourValue(0.3f, 0.3f, 0.3f));
    
    // Directional light (sun)
    Ogre::Light* sun = scene_mgr_->createLight("Sun");
    sun->setType(Ogre::Light::LT_DIRECTIONAL);
    sun->setDiffuseColour(Ogre::ColourValue(1.0f, 1.0f, 0.9f));
    sun->setSpecularColour(Ogre::ColourValue(0.4f, 0.4f, 0.4f));
    
    Ogre::SceneNode* sunNode = scene_mgr_->getRootSceneNode()->createChildSceneNode("SunNode");
    sunNode->attachObject(sun);
    sunNode->setDirection(Ogre::Vector3(-1, -1, -0.5).normalisedCopy());
    
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
    // Update shape renderer (handles TTL)
    shape_renderer_->update(dt);
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
