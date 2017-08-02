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

#include <iostream>

#include <vtkCamera.h>

#include <scrimmage/viewer/Viewer.h>
#include <scrimmage/viewer/Updater.h>
#include <scrimmage/viewer/CameraInterface.h>

using std::cout;
using std::endl;

namespace scrimmage {

Viewer::Viewer() : enable_network_(false) { }

void Viewer::set_incoming_interface(InterfacePtr &incoming_interface)
{ incoming_interface_ = incoming_interface; }

void Viewer::set_outgoing_interface(InterfacePtr &outgoing_interface)
{ outgoing_interface_ = outgoing_interface; }

void Viewer::set_enable_network(bool enable)
{ enable_network_ = enable; }

bool Viewer::init()
{
    renderer_ = vtkSmartPointer<vtkRenderer>::New();
    renderWindow_ = vtkSmartPointer<vtkRenderWindow>::New();

    renderWindow_->AddRenderer(renderer_);

    renderWindowInteractor_ = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor_->SetRenderWindow(renderWindow_);

    renderer_->SetBackground(0,0,0);

    // Setup camera
    vtkSmartPointer<vtkCamera> camera =
        vtkSmartPointer<vtkCamera>::New();
    camera->SetViewUp(0,0,1);
    camera->SetPosition(40, 40, 1000);
    camera->SetFocalPoint(0, 0, 0);

    // Setup camera interface
    cam_int_ = vtkSmartPointer<CameraInterface>::New();
    renderWindowInteractor_->SetInteractorStyle(cam_int_);
    cam_int_->SetCurrentRenderer(renderer_);
    //actor_interface_->cam_int_ = style;

    renderer_->SetActiveCamera(camera);

    // Render and interact
    renderWindow_->SetWindowName("SCRIMMAGE");
    renderWindow_->SetSize(800,600);

    return true;
}

bool Viewer::run()
{
    double update_rate = 50; // Hz

    if (enable_network_) {
        outgoing_interface_->init_network(Interface::client,
                                          "localhost", 50052);

        network_thread_ = std::thread(&Interface::init_network, &(*incoming_interface_),
                                      Interface::server, "localhost", 50051);
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
    updater->init();

    cam_int_->set_updater(updater);

    renderWindowInteractor_->CreateRepeatingTimer(1.0 / update_rate * 1e3); // ms

    // Start the interaction and timer
    renderWindowInteractor_->Start();

    updater->shutting_down();

    return true;
}

bool Viewer::stop()
{
    renderWindowInteractor_->TerminateApp();
    return true;
}
}
