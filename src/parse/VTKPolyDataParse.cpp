/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2024 by the Georgia Tech Research Institute (GTRI)
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
 * @author Wesley Ford <wesley.ford@gatech.edu>
 * @date 9 Feb 2024
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/parse/VTKPolyDataParse.h>

#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkSmartPointer.h>

#include <array>
#include <optional>
#include <vector>
#include <memory>

namespace scrimmage {

    std::unique_ptr<std::array<std::vector<double>, 3>> VTKPolyDataParse::Parse(const std::string& filename) {
      auto elevation_map = std::make_unique<std::array<std::vector<double>, 3>>();

      vtkSmartPointer<vtkPolyDataReader> elevation_reader =
        vtkSmartPointer<vtkPolyDataReader>::New();

      elevation_reader->SetFileName(filename.c_str());
      bool validFile = elevation_reader->IsFilePolyData() != 0;
      if (!validFile) { 
        std::cout << "Invalid VTK File: \'" << filename <<
          "\'. Elevation information is unavailable\n";
        return nullptr;
      }

      elevation_reader->Update();
      vtkSmartPointer<vtkPolyData> polydata;
      polydata = elevation_reader->GetOutput();

      std::size_t num_pts = polydata->GetNumberOfPoints();
      for(int i = 0; i < 3; i++) {
        elevation_map->at(i).reserve(num_pts);
      }

      for(size_t n = 0; n < num_pts; n++){
        // Copies the point information from polydata into the raw array
        // of the elevation map
        std::array<double, 3> tmp_point;
        polydata->GetPoint(static_cast<vtkIdType>(n), tmp_point.data());
        //Sort for binary lookup? 
        elevation_map->at(0).push_back(tmp_point[0]);
        elevation_map->at(1).push_back(tmp_point[1]);
        elevation_map->at(2).push_back(tmp_point[2]);
      }
      return elevation_map;
    }
} // namespace scrimmage
