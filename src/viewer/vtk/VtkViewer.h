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

#ifndef SRC_VIEWER_VTK_VTKVIEWER_H_
#define SRC_VIEWER_VTK_VTKVIEWER_H_

#include <map>
#include <string>
#include <thread>

#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

#include "scrimmage/fwd_decl.h"

namespace scrimmage {

class VtkCameraInterface;

class VtkViewer {
 public:
    VtkViewer();

    void set_incoming_interface(InterfacePtr& incoming_interface);
    void set_outgoing_interface(InterfacePtr& outgoing_interface);
    void set_enable_network(bool enable);

    bool init(
        const MissionParsePtr& mp,
        const std::map<std::string, std::string>& camera_params);
    bool run();
    void stop();

 protected:
    vtkSmartPointer<vtkRenderer> renderer_;
    vtkSmartPointer<vtkRenderWindow> renderWindow_;
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor_;

    vtkSmartPointer<VtkCameraInterface> cam_int_;
    InterfacePtr incoming_interface_;
    InterfacePtr outgoing_interface_;

    bool enable_network_;
    std::thread network_thread_;

    std::map<std::string, std::string> camera_params_;
    std::string log_dir_;
    double dt_ = 0.1;

    std::string local_ip_ = "localhost";
    int local_port_ = 50051;
    std::string remote_ip_ = "localhost";
    int remote_port_ = 50052;

    double init_scale_ = 1.0;
    bool full_screen_;
};

}  // namespace scrimmage

#endif  // SRC_VIEWER_VTK_VTKVIEWER_H_