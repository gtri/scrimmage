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

#ifndef SRC_VIEWER_VTK_VTKCAMERAINTERFACE_H_
#define SRC_VIEWER_VTK_VTKCAMERAINTERFACE_H_

#include <string>

#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkSmartPointer.h>

#include "VtkUpdater.h"

namespace scrimmage {

class VtkCameraInterface : public vtkInteractorStyleTrackballCamera {
 public:
    VtkCameraInterface();

    static VtkCameraInterface* New();

    void OnKeyPress() override;
    void OnLeftButtonDown() override;
    void OnLeftButtonUp() override;
    void Rotate() override;

    void OnMiddleButtonUp() override;
    void Pan() override;
    void Dolly() override;

    void set_updater(vtkSmartPointer<Updater>& updater) { updater_ = updater; }

 protected:
    vtkSmartPointer<Updater> updater_;
    bool enable_object_draw_ = false;
    std::string last_key_;
};

}  // namespace scrimmage

#endif  // SRC_VIEWER_VTK_VTKCAMERAINTERFACE_H_