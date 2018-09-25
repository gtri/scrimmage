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
#include <scrimmage/viewer/CameraInterface.h>

#include <vtkWorldPointPicker.h>
#include <vtkRendererCollection.h>

namespace scrimmage {

CameraInterface::CameraInterface() {}

CameraInterface *CameraInterface::New() {
    CameraInterface *cb = new CameraInterface;
    return cb;
}

void CameraInterface::OnKeyPress() {
    // Get the keypress
    vtkRenderWindowInteractor *rwi = this->Interactor;
    std::string key = rwi->GetKeySym();

    if (last_key_ == "semicolon") {
      updater_->process_custom_key(key);
      last_key_ = "";
    } else if (key == "Right") {
        updater_->inc_follow();
    } else if (key == "Left") {
        updater_->dec_follow();
    } else if (key == "a") {
        updater_->next_mode();
    } else if (key == "c") {
        updater_->request_cached();
    } else if (key == "t") {
        updater_->toggle_trails();
    } else if (key == "bracketleft") {
        updater_->dec_warp();
    } else if (key == "bracketright") {
        updater_->inc_warp();
    } else if (key == "b") {
        updater_->toggle_pause();
    } else if (key == "space") {
        updater_->single_step();
    } else if (key == "plus" || key == "equal") {
        updater_->inc_scale();
    } else if (key == "minus") {
        updater_->dec_scale();
    } else if (key == "0") {
        updater_->reset_scale();
    } else if (key == "r") {
        updater_->reset_view();
    } else if (key == "z") {
        updater_->inc_follow_offset();
    } else if (key == "Z") {
        updater_->dec_follow_offset();
    } else if (key == "A") {
        updater_->set_reset_camera();
    } else if (key == "n") {
        updater_->dec_label_scale();
    } else if (key == "N") {
        updater_->inc_label_scale();
    } else if (key == "o") {
        enable_object_draw_ = not enable_object_draw_;
    } else if (key == "h") {
        updater_->toggle_helpmenu();
    } else if (key == "semicolon") {
      last_key_ = key;
    } else {
        // cout << "key: " << key << endl;
    }

    // Forward events
    vtkInteractorStyleTrackballCamera::OnKeyPress();
}

void CameraInterface::Rotate() {
    updater_->update();
    vtkInteractorStyleTrackballCamera::Rotate();
}

void CameraInterface::OnLeftButtonDown() {
    if (enable_object_draw_) {
        this->Interactor->GetPicker()->Pick(this->Interactor->GetEventPosition()[0],
                                            this->Interactor->GetEventPosition()[1],
                                            0,  // always zero.
                                            this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
        double picked[3];
        this->Interactor->GetPicker()->GetPickPosition(picked);
        updater_->world_point_clicked(picked[0], picked[1], picked[2]);
    }
    // Forward events
    vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
}

void CameraInterface::OnLeftButtonUp() {
    vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
}

void CameraInterface::Pan() {
    updater_->update();
    vtkInteractorStyleTrackballCamera::Pan();
}

void CameraInterface::Dolly() {
    updater_->update();
    vtkInteractorStyleTrackballCamera::Dolly();
}
} // namespace scrimmage
