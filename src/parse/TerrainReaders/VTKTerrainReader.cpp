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

#include <scrimmage/parse/TerrainReaders/VTKTerrainReader.h>

#include <vtkPolyDataReader.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include <array>
#include <vector>
#include <memory>

namespace scrimmage {

    VTKTerrainReader::VTKTerrainReader(std::string filename):
      filename_(filename) {}

    std::unique_ptr<terrain::ElevationGrid> VTKTerrainReader::Parse() const {
      std::unique_ptr<terrain::ElevationGrid> elevation_grid = nullptr;

      vtkSmartPointer<vtkPolyDataReader> elevation_reader =
        vtkSmartPointer<vtkPolyDataReader>::New();

      elevation_reader->SetFileName(filename_.c_str());
      bool validFile = elevation_reader->IsFilePolyData() != 0;
      if (!validFile) { 
        std::cout << "Invalid VTK File: \'" << filename_ <<
          "\'. Elevation information is unavailable\n";
        return nullptr;
      }

      elevation_reader->Update();
      vtkSmartPointer<vtkPolyData> polydata;
      polydata = elevation_reader->GetOutput();

      std::vector<double> x, y, z;
      std::size_t num_pts = polydata->GetNumberOfPoints();
      x.reserve(num_pts);
      y.reserve(num_pts);
      z.reserve(num_pts);

      for(size_t n = 0; n < num_pts; n++){
        // Copies the point information from polydata into the raw array
        // of the elevation map
        std::array<double, 3> tmp_point;
        polydata->GetPoint(static_cast<vtkIdType>(n), tmp_point.data());
        //Sort for binary lookup? 
        x.push_back(tmp_point[0]);
        y.push_back(tmp_point[1]);
        z.push_back(tmp_point[2]);
      }
      elevation_grid = std::make_unique<terrain::ElevationGrid>(
          std::move(x), std::move(y), std::move(z));
      return elevation_grid;
    }
} // namespace scrimmage
