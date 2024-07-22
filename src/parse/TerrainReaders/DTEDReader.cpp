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

#include <scrimmage/parse/TerrainReaders/DTEDReader.h>
#include <scrimmage/proto/Visual.pb.h>

#include <gdal.h>
#include <gdal_priv.h>

#include <array>
#include <cstring>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

namespace scrimmage {

namespace fs = std::filesystem;

DTEDReader::DTEDReader(fs::path file,
                       const scrimmage_proto::UTMTerrain::TerrainExtent& terrain_extent)
    : file_{file},
      x_bounds{terrain_extent.x_min(), terrain_extent.x_max()},
      y_bounds{terrain_extent.y_min(), terrain_extent.y_max()} {
  GDALAllRegister();
}

std::unique_ptr<GDALDataset, GDALDatasetUniquePtrReleaser> DTEDReader::OpenDTED() const {
  // Read the entire DTED file as a single block.
  // https://gdal.org/drivers/raster/dted.html#config-GDAL_DTED_SINGLE_BLOCK
  CPLSetConfigOption("GDAL_DTED_SINGLE_BLOCK", "TRUE");
  std::unique_ptr<GDALDataset, GDALDatasetUniquePtrReleaser> dataset(
      GDALDataset::FromHandle(GDALOpenShared(file_.c_str(), GA_ReadOnly)),
      GDALDatasetUniquePtrReleaser{});
  return dataset;
}

std::unique_ptr<std::array<std::vector<double>, 3>> DTEDReader::ReadDTED(
    GDALDataset& dataset) const {
  int xBlockSize, yBlockSize, xActualBlockSize, yActualBlockSize;
  auto arr = std::make_unique<std::array<std::vector<double>, 3>>();
  std::vector<double>& x(arr->at(0));
  std::vector<double>& y(arr->at(1));
  std::vector<double>& z(arr->at(2));
  // Assume that there is only a single band in the dataset. Fetch first
  // band
  GDALDataset::Bands bands = dataset.GetBands();
  if (bands.size() == 0) {
    return nullptr;
  }
  auto band = bands[0];

  band->GetBlockSize(&xBlockSize, &yBlockSize);
  //
  // DTED data is represented as raw 16 bit signed integers.
  std::unique_ptr<int16_t[]> dted_data(new int16_t[xBlockSize * yBlockSize]);
  CPLErr err = band->ReadBlock(0, 0, dted_data.get());
  if (err != CE_None) {
    return nullptr;
  }

  band->GetActualBlockSize(0, 0, &xActualBlockSize, &yActualBlockSize);
  const int num_pts = xActualBlockSize * yActualBlockSize;
  x.reserve(num_pts);
  y.reserve(num_pts);
  z.reserve(num_pts);
  //
  // The size of the block being read can be smaller than the "ideal"
  // block size.

  // Linear transformation of pixel/line coordinates (i.e. col-row index)
  // to lon/lat coordinates specified in the dted file.
  // https://gdal.org/api/gdaldataset_cpp.html#_CPPv4N11GDALDataset15GetGeoTransformEPd
  double geoTransform[6];
  dataset.GetGeoTransform(geoTransform);

  // The DTED dataset indexs the raster starting from the top-left
  // corner (like images). Scrimmage stars in the bottom right.
  // This means that the y-axis is flipped from the one
  // that the current format (and query function) supports. Read from the
  // bottom of the raser.
  for (int row = yActualBlockSize - 1; row >= 0; row--) {
    for (int col = 0; col < xActualBlockSize; col++) {
      double lon = geoTransform[0] + col * geoTransform[1] + row * geoTransform[2];
      double lat = geoTransform[3] + col * geoTransform[4] + row * geoTransform[5];

      bool within_x_bounds = x_bounds.first <= lon && lon <= x_bounds.second;
      bool within_y_bounds = y_bounds.first <= lat && lat <= y_bounds.second;

      if (within_x_bounds && within_y_bounds) {
        x.push_back(lon);
        y.push_back(lat);
        z.push_back(dted_data[row * yActualBlockSize + col]);
      }
    }
  }
  x.shrink_to_fit();
  y.shrink_to_fit();
  z.shrink_to_fit();
  return arr;
}

std::unique_ptr<terrain::ElevationGrid> DTEDReader::Parse() const {
  auto dataset = OpenDTED();
  std::unique_ptr<terrain::ElevationGrid> elevation_grid = nullptr;

  if (dataset) {
    auto elevation_data = ReadDTED(*dataset);
    if (elevation_data) {
      elevation_grid = std::make_unique<terrain::ElevationGrid>(std::move(elevation_data->at(0)),
                                                                std::move(elevation_data->at(1)),
                                                                std::move(elevation_data->at(2)));
    }
  }
  if (!elevation_grid) {
    std::cout << "Unable to Read DTED file \'" << file_ << "\'\n";
  }
  return elevation_grid;
}

/*
 * Utility for Updater to help read DTED files into vtkPolydata (all vtk
 * dependencies are in the Updater.cpp). DOES not return an elevation_grid
 * object, as the translation to UTM violates the assumptions
 * elevation_grid makes about the nature of its data (mainly the uniformly
 * spaced grid aspect).
 */

std::unique_ptr<std::array<std::vector<double>, 3>> DTEDReader::ParseAsUTM(
    const int utm_zone, const bool northern_hemisphere) const {
  // Setup Projection Transformation from WGS 84 (used by DTED)
  // to UTM projection defined by initalization parameters.
  std::unique_ptr<std::array<std::vector<double>, 3>> elevation_data = nullptr;
  auto dataset = OpenDTED();
  if (dataset) {
    const OGRSpatialReference* wgs = dataset->GetSpatialRef();
    elevation_data = ReadDTED(*dataset);
    if (elevation_data && wgs != nullptr) {
      std::unique_ptr<OGRSpatialReference> utm(wgs->Clone());
      utm->SetProjCS("UTM");
      utm->SetUTM(utm_zone, northern_hemisphere);
      std::unique_ptr<OGRCoordinateTransformation> wgs_to_utm(
          OGRCreateCoordinateTransformation(wgs, utm.get()));

      std::size_t num_pts = elevation_data->at(0).size();
      // Provide raw ptrs to bluk transform coordinates to UTM
      wgs_to_utm->Transform(num_pts,
                            elevation_data->at(0).data(),
                            elevation_data->at(1).data(),
                            elevation_data->at(2).data());
    }
  }
  if (!elevation_data) {
    std::cout << "Unable to open or read DTED file \'" << file_ << "\'\n";
  }
  return elevation_data;
}
}  // namespace scrimmage
