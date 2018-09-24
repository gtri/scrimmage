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

#ifndef INCLUDE_SCRIMMAGE_VIEWER_CAMERAINTERFACE_H_
#define INCLUDE_SCRIMMAGE_VIEWER_CAMERAINTERFACE_H_

#include <vtkCommand.h>
#include <vtkSmartPointer.h>
#include <vtkProperty.h>
#include <vtkVersion.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>

#include <scrimmage/viewer/Updater.h>

#include <string>

namespace scrimmage {
// Define interaction style
class CameraInterface : public vtkInteractorStyleTrackballCamera {
 public:
    CameraInterface();

    static CameraInterface* New();
    // vtkTypeMacro(CameraInterface, vtkInteractorStyleTrackballCamera);

    virtual void OnKeyPress();
    virtual void OnLeftButtonDown();
    virtual void OnLeftButtonUp();
    virtual void Rotate();

    virtual void Pan();
    virtual void Dolly();

    void set_updater(vtkSmartPointer<Updater> & updater) { updater_ = updater; }

 protected:
    vtkSmartPointer<Updater> updater_;
    bool enable_object_draw_ = false;
    std::string last_key_;
};
// vtkStandardNewMacro(CameraInterface);
} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_VIEWER_CAMERAINTERFACE_H_
