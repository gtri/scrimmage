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
#include <optional>
#include <vector>

namespace scrimmage {

    DTEDParse::DTEDParse(int utm_zone, bool northern_hemisphere):
      utm_zone_(utm_zone),
      utm_northern_hemisphere_(northern_hemisphere) {};

    std::optional<std::array<std::vector<double>, 3>> DTEDParse::Parse(const std::string& filename) {
      std::array<std::vector<double>, 3> elevation_map;
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
        return std::nullopt; 
      }

      // Assume that there is only a single band in the dataset. Fetch first
      // band
      std::unique_ptr<GDALRasterBand> band(dataset->GetRasterBand(1));
      if (band == nullptr) { 
        std::cout << "\'" << filename << " \' conatins no rasterband information."
          " Elevation information is unavailable\n";
        return std::nullopt; 
      }

      band->GetBlockSize(&xBlockSize, &yBlockSize);
      //
      // DTED data is represented as raw 16 bit signed integers.
      std::unique_ptr<int16_t[]> dted_data(new int16_t[xBlockSize * yBlockSize]); 
      CPLErr err = band->ReadBlock(0, 0, dted_data.get());
      if (err != CE_None) {
        std::cout << "Error parsing DTED File \'" << filename << "\'\n"; 
        return std::nullopt; 
      }

      // The size of the block being read can be smaller than the "ideal" 
      // block size.
      band->GetActualBlockSize(0, 0, &xActualBlockSize, &yActualBlockSize);
      const int num_pts = xActualBlockSize * yActualBlockSize;
      for(int i = 0; i < 3; i++) {
        elevation_map[i].reserve(num_pts);
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
        elevation_map[0].push_back(lon);
        elevation_map[1].push_back(lat);
        elevation_map[2].push_back(dted_data[i]);
      }

      wgs_to_utm->Transform(num_pts,
          elevation_map[0].data(), 
          elevation_map[1].data(),
          elevation_map[2].data());

      // Seems to be an effective way to not seg fault
      dataset.release();

      return std::make_optional(elevation_map);
    }
} // namespace scrimmage
