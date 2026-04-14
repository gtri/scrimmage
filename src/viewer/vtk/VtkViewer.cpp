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

#include "VtkViewer.h"

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <vtkCamera.h>

#include "scrimmage/log/Logger.h"
#include "scrimmage/network/Interface.h"
#include "scrimmage/parse/MissionParse.h"
#include "scrimmage/parse/ParseUtils.h"
#include "VtkCameraInterface.h"
#include "VtkUpdater.h"

namespace scrimmage {

VtkViewer::VtkViewer() : enable_network_(false) {}

void VtkViewer::set_incoming_interface(InterfacePtr& incoming_interface) {
    incoming_interface_ = incoming_interface;
}

void VtkViewer::set_outgoing_interface(InterfacePtr& outgoing_interface) {
    outgoing_interface_ = outgoing_interface;
}

void VtkViewer::set_enable_network(bool enable) {
    enable_network_ = enable;
}

bool VtkViewer::init(
    const MissionParsePtr& mp,
    const std::map<std::string, std::string>& camera_params) {
    const char* display = std::getenv("DISPLAY");
    if (display == nullptr || display[0] == '\0') {
        std::cerr << "Error: GUI enabled but no DISPLAY environment variable set.\n"
                  << "Run with enable_gui:=false or set DISPLAY for X11 forwarding."
                  << std::endl;
        return false;
    }

    renderer_ = vtkSmartPointer<vtkRenderer>::New();
    renderWindow_ = vtkSmartPointer<vtkRenderWindow>::New();

    renderWindow_->AddRenderer(renderer_);

    renderWindowInteractor_ = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor_->SetRenderWindow(renderWindow_);

    renderer_->SetBackground(0, 0, 0);

    vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();
    camera->SetViewUp(0, 0, 1);
    camera->SetPosition(40, 40, 1000);
    camera->SetFocalPoint(0, 0, 0);

    cam_int_ = vtkSmartPointer<VtkCameraInterface>::New();
    renderWindowInteractor_->SetInteractorStyle(cam_int_);
    cam_int_->SetCurrentRenderer(renderer_);

    camera_params_ = camera_params;
    renderer_->SetActiveCamera(camera);

    renderWindow_->SetWindowName("SCRIMMAGE");
    renderWindow_->SetFullScreen(false);

    renderWindow_->SetSize(mp->window_width(), mp->window_height());
    init_scale_ = get<double>("scale", mp->params(), 1.0);

    log_dir_ = mp->log_dir();
    dt_ = mp->dt();

    local_ip_ = get<std::string>("local_ip", camera_params_, local_ip_);
    local_port_ = get<int>("local_port", camera_params_, local_port_);
    remote_ip_ = get<std::string>("remote_ip", camera_params_, remote_ip_);
    remote_port_ = get<int>("remote_port", camera_params_, remote_port_);

    full_screen_ = mp->full_screen();

    return true;
}

bool VtkViewer::run() {
    double update_rate = 50;

    if (enable_network_) {
        outgoing_interface_->init_network(Interface::client, remote_ip_, remote_port_);
        network_thread_ = std::thread(
            &Interface::init_network,
            &(*incoming_interface_),
            Interface::server,
            local_ip_,
            local_port_);
        network_thread_.detach();
    } else {
        incoming_interface_->set_mode(Interface::shared);
        outgoing_interface_->set_mode(Interface::shared);
    }

    renderWindow_->Render();
    renderWindowInteractor_->Initialize();

    if (full_screen_) {
        renderWindow_->SetSize(renderWindow_->GetScreenSize());
    }

    vtkSmartPointer<scrimmage::Updater> updater = vtkSmartPointer<scrimmage::Updater>::New();
    renderWindowInteractor_->AddObserver(vtkCommand::TimerEvent, updater);
    updater->set_renderer(renderer_);
    updater->set_rwi(renderWindowInteractor_);
    updater->set_incoming_interface(incoming_interface_);
    updater->set_outgoing_interface(outgoing_interface_);
    updater->set_max_update_rate(update_rate);
    updater->set_init_scale(init_scale_);
    updater->reset_scale();

    std::string camera_pos_str = get<std::string>("pos", camera_params_, "0, 1, 200");

    std::vector<double> camera_pos;
    if (!str2container(camera_pos_str, ",", camera_pos, 3)) {
        LOG_ERROR("camera_position should have 3 comma separated entries");
        return false;
    }

    std::string camera_focal_pos_str = get<std::string>("focal_point", camera_params_, "0, 0, 0");

    std::vector<double> camera_focal_pos;
    if (!str2container(camera_focal_pos_str, ",", camera_focal_pos, 3)) {
        LOG_ERROR("camera_focal_point should have 3 comma separated entries");
        return false;
    }

    updater->set_camera_reset_params(
        camera_pos[0],
        camera_pos[1],
        camera_pos[2],
        camera_focal_pos[0],
        camera_focal_pos[1],
        camera_focal_pos[2]);
    updater->set_show_fps(get("show_fps", camera_params_, false));

    updater->set_follow_id(get("follow_id", camera_params_, 1) - 1);

    std::string view_mode =
        boost::to_upper_copy(get<std::string>("mode", camera_params_, "follow"));

    if (view_mode == "FOLLOW") {
        updater->set_view_mode(Updater::ViewMode::FOLLOW);
    } else if (view_mode == "FREE") {
        updater->set_view_mode(Updater::ViewMode::FREE);
        updater->set_reset_camera();
    } else if (view_mode == "OFFSET") {
        updater->set_view_mode(Updater::ViewMode::OFFSET);
    } else {
        std::stringstream ss;
        ss << "Unrecognized attribute \"" << view_mode << "\" for camera_view_mode";
        LOG_WARN(ss.str());
        updater->set_view_mode(Updater::ViewMode::FOLLOW);
    }

    updater->init(log_dir_, dt_);

    cam_int_->set_updater(updater);

    renderWindowInteractor_->CreateRepeatingTimer(1.0 / update_rate * 1e3);
    renderWindowInteractor_->Start();
    updater->shutting_down();

    return true;
}

}  // namespace scrimmage