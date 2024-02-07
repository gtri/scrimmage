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

#include <scrimmage/plugins/interaction/Terrain/TerrainMap.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/pubsub/Publisher.h>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <gdal/gdal.h>
#include <gdal/gdal_priv.h>

#include <algorithm>
#include <array>
#include <memory>
#include <optional>
#include <vector>

namespace scrimmage {
  namespace interaction {

    TerrainMap::TerrainMap():
      utm_zone_(0), utm_northern_hemisphere_(true) {};


    bool TerrainMap::init(const std::string filename, const int utm_zone, const bool northern_hemisphere) {
      constexpr int max_utm_zone = 60;
      constexpr int min_utm_zone = 1;
      utm_zone_ = utm_zone;
      utm_northern_hemisphere_ = northern_hemisphere;

      if (utm_zone_ < min_utm_zone || utm_zone_ > max_utm_zone) {
        std::cout << "The utm zone \'" << utm_zone << "\' is invalid".
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
      std::string extension = filename.substr(extension_ind + 1);
      if(extension == "vtk") {
        return initFromVTK(filename);
      }
      if(extension.find("dt") && extension.size() == 3) {
        return initFromDTED(filename);
      }
      return false;
    }

    bool TerrainMap::initFromVTK(const std::string filename) {
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
      return true;
    }

    bool TerrainMap::initFromDTED(const std::string filename) {
      int xBlockSize, yBlockSize, xActualBlockSize, yActualBlockSize;
      GDALAllRegister();

      // Read the entire DTED file as a single block. 
      // https://gdal.org/drivers/raster/dted.html#config-GDAL_DTED_SINGLE_BLOCK
      CPLSetConfigOption("GDAL_DTED_SINGLE_BLOCK", "TRUE");

      GDALDatasetUniquePtr dataset(GDALDataset::FromHandle(
            GDALOpen(filename.c_str(), GA_ReadOnly)));

      if(dataset == nullptr) { 
        std::cout << "Unable to read DTED File: \'" << filename <<
          "\'. Elevation information is unavailable\n";
        return false; 
      }

      // Assume that there is only a single band in the dataset. Fetch first
      // band
      std::unique_ptr<GDALRasterBand> band(dataset->GetRasterBand(1));
      if (band == nullptr) { 
        std::cout << "\'" << filename << " \' conatins no rasterband information."
          " Elevation information is unavailable\n";
        return false;
      }

      band->GetBlockSize(&xBlockSize, &yBlockSize);
      //
      // DTED data is represented as raw 16 bit signed integers.
      std::unique_ptr<int16_t[]> dted_data(new int16_t[xBlockSize * yBlockSize]); 
      CPLErr err = band->ReadBlock(0, 0, dted_data.get());
      if (err != CE_None) {
        std::cout << "Error parsing DTED File \'" << filename << "\'\n"; 
        return false;
      }

      // The size of the block being read can be smaller than the "ideal" 
      // block size.
      band->GetActualBlockSize(0, 0, &xActualBlockSize, &yActualBlockSize);
      const int num_pts = xActualBlockSize * yActualBlockSize;
      for(int i = 0; i < 3; i++) {
        elevation_map_[i].reserve(num_pts);
      }

      // Linear transformation of pixel/line coordinates (i.e. col-row index)
      // to lon/lat coordinates specified in the dted file.
      // https://gdal.org/api/gdaldataset_cpp.html#_CPPv4N11GDALDataset15GetGeoTransformEPd
      std::unique_ptr<double[]> geoTransform(new double[6]);
      dataset->GetGeoTransform(geoTransform.get());

      // Setup Projection Transformation from WGS 84 (used by DTED)
      // to UTM projection defined by initalization parameters.
      std::unique_ptr<const OGRSpatialReference> wgs(dataset->GetSpatialRef());
      std::unique_ptr<OGRSpatialReference> utm(wgs->Clone());
      utm->SetProjCS("UTM");
      utm->SetUTM(utm_zone_, utm_northern_hemisphere_);

      std::unique_ptr<OGRCoordinateTransformation> wgs_to_utm(
          OGRCreateCoordinateTransformation(wgs.get(), utm.get()));

      // Read data into our elevation_map. Coordinates still need
      // to be transfomred to utm, but this can be done in place.
      for(int i = 0; i < num_pts; i++) {
        int row = i / yActualBlockSize;
        int col = i % yActualBlockSize;
        double lon = geoTransform[0] + col*geoTransform[1] + row*geoTransform[2];
        double lat = geoTransform[3] + col*geoTransform[4] + row*geoTransform[5];
        elevation_map_[0].push_back(lon);
        elevation_map_[1].push_back(lat);
        elevation_map_[2].push_back(dted_data[i]);
      }

      wgs_to_utm->Transform(num_pts,
          elevation_map_[0].data(), 
          elevation_map_[1].data(),
          elevation_map_[2].data());


      return true;
    }

    /*
     * Queries terrain map by choosing the closest x and y positions  
     * to the ones provided to determine the terrain elevation.
     * The elevation map is layed out as three vectors corresponding 
     * to the x (index 0), y (index 1), and z (index 2) directions.
     *
     * Searching these values can be thought of as searching a 2D matrix formed
     * by the x and y values with y-values as rows, and x-values as columns. 
     * It is assumed that all y-values (rows) are increasing, and all
     * x-values (colums) are increasing within each row. (This is for efficent
     * search using std::upper_bound/lower_bound).
     */
    std::optional<double> TerrainMap::queryTerrain(
        const double xpos, const double ypos) const {
      // Do some checks here that we are within the 
      // boundary of terrain I suppose
      int y_idx, x_idx, stride;

      const std::vector<double>& x_vec = elevation_map_[0];
      const std::vector<double>& y_vec = elevation_map_[1];
      const std::vector<double>& z_vec = elevation_map_[2];

      // Find the width" of the terrain. In otherwords, how many 
      // x-values corresond to a single y-value
      stride = std::upper_bound(y_vec.begin(), y_vec.end(), y_vec[0]) - y_vec.begin();
      y_idx = std::lower_bound(y_vec.begin(), y_vec.end(), ypos) - y_vec.begin(); 

      auto x_search_start = x_vec.begin() + y_idx;
      x_idx = std::lower_bound(x_search_start, x_search_start + stride, xpos) - x_vec.begin();

      // We may want to interpolate here at some point, as we are 
      // always selecting the lower bound of indicies. But for now 
      // just reutrn the z-value

      return  z_vec[x_idx];
    }
  } // namespace interaction
} // namespace scrimmage
