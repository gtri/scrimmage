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
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <vtkCamera.h>

#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/network/Interface.h>
#include <scrimmage/viewer/Viewer.h>
#include <scrimmage/viewer/Updater.h>
#include <scrimmage/viewer/CameraInterface.h>

#include <boost/algorithm/string.hpp>

namespace scrimmage {

Viewer::Viewer() : enable_network_(false) { }

void Viewer::set_incoming_interface(InterfacePtr &incoming_interface) {
    incoming_interface_ = incoming_interface;
}

void Viewer::set_outgoing_interface(InterfacePtr &outgoing_interface) {
    outgoing_interface_ = outgoing_interface;
}

void Viewer::set_enable_network(bool enable) {
    enable_network_ = enable;
}

bool Viewer::init(const std::shared_ptr<MissionParse>& mp,
                  const std::map<std::string, std::string>& camera_params) {
    renderer_ = vtkSmartPointer<vtkRenderer>::New();
    renderWindow_ = vtkSmartPointer<vtkRenderWindow>::New();

    renderWindow_->AddRenderer(renderer_);

    renderWindowInteractor_ = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor_->SetRenderWindow(renderWindow_);

    renderer_->SetBackground(0, 0, 0);

    // Setup camera
    vtkSmartPointer<vtkCamera> camera =
        vtkSmartPointer<vtkCamera>::New();
    camera->SetViewUp(0, 0, 1);
    camera->SetPosition(40, 40, 1000);
    camera->SetFocalPoint(0, 0, 0);

    // Setup camera interface
    cam_int_ = vtkSmartPointer<CameraInterface>::New();
    renderWindowInteractor_->SetInteractorStyle(cam_int_);
    cam_int_->SetCurrentRenderer(renderer_);

    camera_params_ = camera_params;
    renderer_->SetActiveCamera(camera);

    // Render and interact
    renderWindow_->SetWindowName("SCRIMMAGE");
    if (mp->full_screen()) {
        renderWindow_->SetFullScreen(true);
        renderWindow_->FullScreenOn();
    } else {
        renderWindow_->SetFullScreen(false);
        renderWindow_->SetSize(mp->window_width(), mp->window_height());
    }

    log_dir_ = mp->log_dir();
    dt_ = mp->dt();

    // Get network parameters
    local_ip_ = get<std::string>("local_ip", camera_params_, local_ip_);
    local_port_ = get<int>("local_port", camera_params_, local_port_);
    remote_ip_ = get<std::string>("remote_ip", camera_params_, remote_ip_);
    remote_port_ = get<int>("remote_port", camera_params_, remote_port_);

    return true;
}

bool Viewer::run() {
    double update_rate = 50; // Hz

    if (enable_network_) {
        outgoing_interface_->init_network(Interface::client, remote_ip_, remote_port_);
        network_thread_ = std::thread(&Interface::init_network, &(*incoming_interface_),
                                      Interface::server, local_ip_, local_port_);
        network_thread_.detach();
    } else {
        incoming_interface_->set_mode(Interface::shared);
        outgoing_interface_->set_mode(Interface::shared);
    }

    // Render and interact
    renderWindow_->Render();

    // Initialize must be called prior to creating timer events.
    renderWindowInteractor_->Initialize();

    // Sign up to receive TimerEvent
    vtkSmartPointer<scrimmage::Updater> updater =
        vtkSmartPointer<scrimmage::Updater>::New();
    renderWindowInteractor_->AddObserver(vtkCommand::TimerEvent, updater);
    updater->set_renderer(renderer_);
    updater->set_rwi(renderWindowInteractor_);
    updater->set_incoming_interface(incoming_interface_);
    updater->set_outgoing_interface(outgoing_interface_);
    updater->set_max_update_rate(update_rate);

    std::string camera_pos_str =
        get<std::string>("pos", camera_params_, "0, 1, 200");

    std::vector<double> camera_pos;
    if (!str2container(camera_pos_str, ",", camera_pos, 3)) {
        std::cout << "camera_position should have 3 comma separated entries" << std::endl;
        return false;
    }

    std::string camera_focal_pos_str =
        get<std::string>("focal_point", camera_params_, "0, 0, 0");

    std::vector<double> camera_focal_pos;
    if (!str2container(camera_focal_pos_str, ",", camera_focal_pos, 3)) {
        std::cout << "camera_focal_point should have 3 comma separated entries" << std::endl;
        return false;
    }

    updater->set_camera_reset_params(camera_pos[0], camera_pos[1], camera_pos[2],
        camera_focal_pos[0], camera_focal_pos[1], camera_focal_pos[2]);
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
        std::cout << "Unrecognized attribute \"" << view_mode
            << "\" for camera_view_mode" << std::endl;
        updater->set_view_mode(Updater::ViewMode::FOLLOW);
    }

    updater->init(log_dir_, dt_);

    cam_int_->set_updater(updater);

    renderWindowInteractor_->CreateRepeatingTimer(1.0 / update_rate * 1e3); // ms

    // Start the interaction and timer
    renderWindowInteractor_->Start();

    updater->shutting_down();

    return true;
}

bool Viewer::stop() {
    renderWindowInteractor_->TerminateApp();
    return true;
}
} // namespace scrimmage
