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

#include <scrimmage/viewer/Grid.h>

namespace scrimmage {

void Grid::create(int size, double spacing, vtkSmartPointer<vtkRenderer> &renderer)
{
    renderer_ = renderer;

    // Create a plane
    double plane_size = size;
    vtkSmartPointer<vtkPlaneSource> planeSource =
            vtkSmartPointer<vtkPlaneSource>::New();
    planeSource->SetCenter(0,0,0);
    planeSource->SetNormal(plane_size/2.0, plane_size/2.0, 1.0);
    planeSource->SetPoint1(plane_size,0,0);
    planeSource->SetPoint2(0,plane_size,0);

    // Create the grid:
    int x_low = -plane_size/2.0;
    int x_high = plane_size/2.0;
    int x_step = spacing;
    int y_low = -plane_size/2.0;
    int y_high = plane_size/2.0;
    int y_step = spacing;
    double z_pos = 0;

    for (int y = y_low; y < y_high; y += y_step) {
        vtkSmartPointer<vtkLineSource> lineSource =
                vtkSmartPointer<vtkLineSource>::New();

        lineSource->SetPoint1(x_low, y, z_pos);
        lineSource->SetPoint2(x_high, y, z_pos);
        lineSource->Update();

        // Visualize
        vtkSmartPointer<vtkPolyDataMapper> mapper =
                vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(lineSource->GetOutputPort());

        vtkSmartPointer<vtkActor> actor =
                vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetLineWidth(1);

        // Set the color
        actor->GetProperty()->SetColor(1,1,1);
        actor->GetProperty()->SetOpacity(1);

        actors_.push_back(actor);
        renderer->AddActor(actor);
    }

    for (int x = x_low; x < x_high; x += x_step) {
        vtkSmartPointer<vtkLineSource> lineSource =
                vtkSmartPointer<vtkLineSource>::New();

        lineSource->SetPoint1(x, y_low, z_pos);
        lineSource->SetPoint2(x, y_high, z_pos);
        lineSource->Update();

        // Visualize
        vtkSmartPointer<vtkPolyDataMapper> mapper =
                vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(lineSource->GetOutputPort());

        vtkSmartPointer<vtkActor> actor =
                vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetLineWidth(1);

        // Set the color
        actor->GetProperty()->SetColor(1,1,1);
        actor->GetProperty()->SetOpacity(1);

        actors_.push_back(actor);
        renderer->AddActor(actor);
    }
}

void Grid::remove()
{
    for (vtkSmartPointer<vtkActor> actor : actors_) {
        renderer_->RemoveActor(actor);
    }
    actors_.clear();
}

}
