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
#include <scrimmage/proto/Visual.pb.h>

#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkSmartPointer.h>

#include <array>
#include <filesystem>
#include <memory>
#include <vector>

namespace scrimmage {

namespace fs = std::filesystem;

VTKTerrainReader::VTKTerrainReader(fs::path file,
                                   const scrimmage_proto::UTMTerrain::TerrainExtent& terrain_extent)
    : file_(file),
      x_bounds{terrain_extent.x_min(), terrain_extent.x_max()},
      y_bounds{terrain_extent.y_min(), terrain_extent.y_max()} {}

std::unique_ptr<terrain::ElevationGrid> VTKTerrainReader::Parse() const {
  std::unique_ptr<terrain::ElevationGrid> elevation_grid = nullptr;

  vtkSmartPointer<vtkPolyDataReader> elevation_reader = vtkSmartPointer<vtkPolyDataReader>::New();

  elevation_reader->SetFileName(file_.c_str());
  bool validFile = elevation_reader->IsFilePolyData() != 0;
  if (!validFile) {
    std::cout << "Invalid VTK File: \'" << file_ << "\'. Elevation information is unavailable\n";
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

  for (size_t n = 0; n < num_pts; n++) {
    // Copies the point information from polydata into the raw array
    // of the elevation map
    std::array<double, 3> tmp_point;
    polydata->GetPoint(static_cast<vtkIdType>(n), tmp_point.data());

    bool within_x_bounds = x_bounds.first <= tmp_point[0] && tmp_point[0] <= x_bounds.second;
    bool within_y_bounds = y_bounds.first <= tmp_point[1] && tmp_point[1] <= y_bounds.second;

    if (within_x_bounds && within_y_bounds) {
      x.push_back(tmp_point[0]);
      y.push_back(tmp_point[1]);
      z.push_back(tmp_point[2]);
    }
  }
  x.shrink_to_fit();
  y.shrink_to_fit();
  z.shrink_to_fit();

  elevation_grid =
      std::make_unique<terrain::ElevationGrid>(std::move(x), std::move(y), std::move(z));
  return elevation_grid;
}
}  // namespace scrimmage
