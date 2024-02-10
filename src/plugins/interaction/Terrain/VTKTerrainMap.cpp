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
 * @date 31 Jan 2024
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/common/Utilities.h>
#include <scrimmage/plugins/interaction/Terrain/TerrainMap.h>
#include <scrimmage/plugins/interaction/Terrain/VTKTerrainMap.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/pubsub/Publisher.h>
#include <GeographicLib/GeoCoords.hpp>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <gdal/gdal.h>
#include <gdal/gdal_priv.h>

#include <algorithm>
#include <array>
#include <optional>
#include <vector>

namespace scrimmage {
  namespace interaction {

    VTKTerrainMap::VTKTerrainMap() {}

    bool VTKTerrainMap::init(const std::string filename, const int utm_zone, const bool northern_hemisphere) {
      constexpr int max_utm_zone = 60;
      constexpr int min_utm_zone = 1;
      utm_zone_ = utm_zone;
      utm_northern_hemisphere_ = northern_hemisphere;

      if (utm_zone_ < min_utm_zone || utm_zone_ > max_utm_zone) {
        std::cout << "The utm zone \'" << utm_zone << "\' is invalid."
          "Valid Zones must be between " << min_utm_zone
          << " and " << max_utm_zone << "\n";
        return false;
      }

      std::size_t extension_ind = filename.find_last_of("."); 
      if(extension_ind == std::string::npos) {
        std::cout << "No file extension of terrain file found. Unable"
          "to determine terrain file type\n";
        return false;
      }
      return InitFromVTK(filename);
    }

    /*
     * The vtk renderer requires terrain data in the form of 
     * polydata to correctly map a terrain image to a 3D rendering 
     * of the terrain. Copy the underlying data model to this format.
     * Skeptical of having this here
     */
    vtkSmartPointer<vtkPolyData> TerrainMap::ToPolyData() {
      vtkSmartPointer<vtkPolyData> polydata = vtkPolyData::New();
      vtkSmartPointer<vtkPoints> points = vtkPoints::New();
      std::size_t num_pts = number_points();
      points->SetNumberOfPoints(num_pts); 
      for (int i = 0; i < num_pts; i++) {
        points->SetPoint(i,
          elevation_map_[0][i],
          elevation_map_[1][i],
          elevation_map_[2][i]);
      }
      polydata->SetPoints(points);
      return polydata;
    }

    bool VTKTerrainMap::InitFromVTK(const std::string filename) {
      // Use vtkPolyReader
      // Read the terrain polydata
      vtkSmartPointer<vtkPolyDataReader> elevation_reader =
        vtkSmartPointer<vtkPolyDataReader>::New();

      elevation_reader->SetFileName(filename.c_str());
      bool validFile = elevation_reader->IsFilePolyData() != 0;
      if (!validFile) { 
        std::cout << "Invalid VTK File: \'" << filename <<
          "\'. Elevation information is unavailable\n";
        return false; 
      }

      elevation_reader->Update();
      vtkSmartPointer<vtkPolyData> polydata;
      polydata = elevation_reader->GetOutput();

      std::size_t num_pts = polydata->GetNumberOfPoints();
      for(int i = 0; i < 3; i++) {
        elevation_map_[i].reserve(num_pts);
      }

      for(size_t n = 0; n < num_pts; n++){
        // Copies the point information from polydata into the raw array
        // of the elevation map
        std::array<double, 3> tmp_point;
        polydata->GetPoint(static_cast<vtkIdType>(n), tmp_point.data());
        //Sort for binary lookup? 
        elevation_map_[0].push_back(tmp_point[0]);
        elevation_map_[1].push_back(tmp_point[1]);
        elevation_map_[2].push_back(tmp_point[2]);
      }
      auto& y_vec = elevation_map_[1];
      stride_ = std::upper_bound(y_vec.begin(), y_vec.end(), y_vec[0]) - y_vec.begin();
      return true;
    }

      std::optional<double> VTKTerrainMap::QueryUTM(
          const double easting, const double northing) const  {
        // Default Coordinates are already in easting/northing
        return QueryTerrain(easting, northing);
      }

      std::optional<double> VTKTerrainMap::QueryLongLat(
          const double longitude, const double latitude) const {
        GeographicLib::GeoCoords GC = GeographicLib::GeoCoords(
            latitude, longitude,
            utm_zone_);
          return QueryUTM(GC.Easting(), GC.Northing());
      }

  } // namespace interaction
} // namespace scrimmage
