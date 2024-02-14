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

#include <scrimmage/parse/DTEDParse.h>

#include <gdal/gdal.h>
#include <gdal/gdal_priv.h>

#include <array>
#include <iostream>
#include <memory>
#include <vector>

namespace scrimmage {

  std::unique_ptr<std::array<std::vector<double>, 3>> DTEDParse::Parse(const std::string& filename) {
    int xBlockSize, yBlockSize, xActualBlockSize, yActualBlockSize;
    auto elevation_map = std::make_unique<std::array<std::vector<double>, 3>>();

    GDALAllRegister();

    // Read the entire DTED file as a single block. 
    // https://gdal.org/drivers/raster/dted.html#config-GDAL_DTED_SINGLE_BLOCK
    CPLSetConfigOption("GDAL_DTED_SINGLE_BLOCK", "TRUE");

    GDALDatasetUniquePtr dataset(GDALDataset::FromHandle(
          GDALOpen(filename.c_str(), GA_ReadOnly)));

    if(dataset == nullptr) { 
      std::cout << "Unable to read DTED File: \'" << filename <<
        "\'. Elevation information is unavailable\n";
      return nullptr;
    }

    // Assume that there is only a single band in the dataset. Fetch first
    // band
    std::unique_ptr<GDALRasterBand> band(dataset->GetRasterBand(1));
    if (band == nullptr) { 
      std::cout << "\'" << filename << " \' conatins no rasterband information."
        " Elevation information is unavailable\n";
      return nullptr; 
    }

    band->GetBlockSize(&xBlockSize, &yBlockSize);
    //
    // DTED data is represented as raw 16 bit signed integers.
    std::unique_ptr<int16_t[]> dted_data(new int16_t[xBlockSize * yBlockSize]); 
    CPLErr err = band->ReadBlock(0, 0, dted_data.get());
    if (err != CE_None) {
      std::cout << "Error parsing DTED File \'" << filename << "\'\n"; 
      return nullptr; 
    }

    // The size of the block being read can be smaller than the "ideal" 
    // block size.
    band->GetActualBlockSize(0, 0, &xActualBlockSize, &yActualBlockSize);
    const int num_pts = xActualBlockSize * yActualBlockSize;
    for(int i = 0; i < 3; i++) {
      elevation_map->at(i).reserve(num_pts);
    }

    // Linear transformation of pixel/line coordinates (i.e. col-row index)
    // to lon/lat coordinates specified in the dted file.
    // https://gdal.org/api/gdaldataset_cpp.html#_CPPv4N11GDALDataset15GetGeoTransformEPd
    std::unique_ptr<double[]> geoTransform(new double[6]);
    dataset->GetGeoTransform(geoTransform.get());



    // DTED raster is starting from top-left and grows down and to the 
    // right. This means that the y-axis is flipped from the one 
    // that the current format (and query function) supports. 

    for (int row = yActualBlockSize - 1; row >= 0; row--){
      for(int col = 0; col < xActualBlockSize; col++) {
        double lon = geoTransform[0] + col*geoTransform[1] + row*geoTransform[2];
        double lat = geoTransform[3] + col*geoTransform[4] + row*geoTransform[5];
        elevation_map->at(0).push_back(lon);
        elevation_map->at(1).push_back(lat);
        elevation_map->at(2).push_back(dted_data[row*yActualBlockSize + col]);
      }
    }
    dataset.release();

    return elevation_map;
  }


  std::unique_ptr<std::array<std::vector<double>, 3>> DTEDParse::ParseAsUTM(
      const std::string& filename,
      const int utm_zone,
      const bool northern_hemisphere) {
    // Setup Projection Transformation from WGS 84 (used by DTED)
    // to UTM projection defined by initalization parameters.
    GDALDatasetUniquePtr dataset(GDALDataset::FromHandle(
          GDALOpen(filename.c_str(), GA_ReadOnly)));

    if(dataset == nullptr) { 
      std::cout << "Unable to read DTED File: \'" << filename <<
        "\'. Elevation information is unavailable\n";
      return nullptr;
    }

    std::unique_ptr<const OGRSpatialReference> wgs(dataset->GetSpatialRef());
    std::unique_ptr<OGRSpatialReference> utm(wgs->Clone());
    utm->SetProjCS("UTM");
    utm->SetUTM(utm_zone, northern_hemisphere);

    dataset.release();

    std::unique_ptr<OGRCoordinateTransformation> wgs_to_utm(
        OGRCreateCoordinateTransformation(wgs.get(), utm.get()));

    auto elevation_map = DTEDParse::Parse(filename);

    if (elevation_map) {
      std::size_t num_pts = elevation_map->at(0).size();
      wgs_to_utm->Transform(num_pts,
          elevation_map->at(0).data(),
          elevation_map->at(1).data(),
          elevation_map->at(2).data());
    }
    return elevation_map;
  }
} // namespace scrimmage
