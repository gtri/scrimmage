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
 * @author Wesley Ford <wesley.ford@gatech.edu>
 * @date 30 June 2024
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/common/terrain/TerrainFactory.h>
#include <scrimmage/common/terrain/UTMTerrainMap.h>
#include <scrimmage/common/terrain/WGSTerrainMap.h>
#include <scrimmage/proto/Visual.pb.h>

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>

#include <GeographicLib/GeoCoords.hpp>

namespace fs = std::filesystem;
using scrimmage_proto::UTMTerrain;

class TerrainTest : public ::testing::Test {
 protected:
  void get_terrain_info(fs::path& target_file,
                        std::string& filename,
                        std::shared_ptr<UTMTerrain> utm_terrain) {
    auto build_full_path = [](fs::path root, fs::path stem) -> fs::path {
      fs::path full = root / stem;
      return full;
    };

    char* scrimmage_data_path = std::getenv("SCRIMMAGE_DATA_PATH");
    ASSERT_NE(scrimmage_data_path, nullptr);
    char* data_path = std::strtok(scrimmage_data_path, ":");
    while (data_path != nullptr) {
      fs::path tmp_path = build_full_path(data_path, target_file);
      if (fs::exists(tmp_path) && !fs::exists(target_file)) {
        target_file = tmp_path;
        break;
      }
      data_path = std::strtok(nullptr, ":");
    }
    utm_terrain->set_poly_data_file(target_file.string());
    utm_terrain->set_zone(10);
    utm_terrain->set_hemisphere("north");
  }

  static std::vector<double> get_spaced_values(double min, double max, double spacing) {
    assert((min < max && spacing > 0) || (max < min && spacing < 0));
    std::vector<double> values;

    for (double value = min; value <= max; value += spacing) {
      values.push_back(value);
    }
    return values;
  }

  void SetUp() override {
    vtk_terrain_proto = std::make_shared<UTMTerrain>();
    dted_terrain_proto = std::make_shared<UTMTerrain>();

    auto vtk_extent = vtk_terrain_proto->mutable_terrain_extent();
    auto dted_extent = dted_terrain_proto->mutable_terrain_extent();

    get_terrain_info(vtk_file, vtk_filename, vtk_terrain_proto);
    get_terrain_info(dted_file, dted_filename, dted_terrain_proto);

    dted_extent->set_x_min(-122.0);
    dted_extent->set_x_max(-121.7);
    dted_extent->set_y_min(37.7);
    dted_extent->set_y_max(38.0);

    // We want to use the same extent in utm coordinates
    GeographicLib::GeoCoords utm_min =
        GeographicLib::GeoCoords(dted_extent->y_min(), dted_extent->x_min(), 10);
    GeographicLib::GeoCoords utm_max =
        GeographicLib::GeoCoords(dted_extent->y_max(), dted_extent->x_max(), 10);

    vtk_extent->set_x_min(utm_min.Easting());
    vtk_extent->set_x_max(utm_max.Easting());

    vtk_extent->set_y_min(utm_min.Northing());
    vtk_extent->set_y_max(utm_max.Northing());

    ASSERT_TRUE(fs::exists(vtk_file));
    ASSERT_TRUE(fs::exists(dted_file));

    lon_values = get_spaced_values(dted_extent->x_min(), dted_extent->x_max(), lon_spacing);
    lat_values = get_spaced_values(dted_extent->y_min(), dted_extent->y_max(), lat_spacing);
  }
  // void TearDown() override {

  //}

  fs::path vtk_file = "gui/terrain/mt_diablo/mt_diablo.vtk";
  fs::path dted_file = "gui/terrain/mt_diablo_dted/mt_diablo.dt2";

  std::string vtk_filename;
  std::string dted_filename;

  std::shared_ptr<UTMTerrain> vtk_terrain_proto, dted_terrain_proto;
  static constexpr double lon_spacing = 1.0 / 3600.0;  // dt2 lon spacing is 1 arcsecond
  static constexpr double lat_spacing =
      2.0 / 3600.0;  // dt2 lat spacing between 50-70 is 2 arcseconds

  // Used to query both formats
  std::vector<double> lon_values;
  std::vector<double> lat_values;
};

using scrimmage::terrain::TerrainFactory;
using scrimmage::terrain::TerrainMapPtr;

TEST_F(TerrainTest, TerrainQuery) {
  TerrainMapPtr vtk_terrain = TerrainFactory::MakeTerrain(vtk_terrain_proto);
  TerrainMapPtr dted_terrain = TerrainFactory::MakeTerrain(dted_terrain_proto);

  ASSERT_NE(vtk_terrain, nullptr);
  ASSERT_NE(dted_terrain, nullptr);

  for (double latitude : lat_values) {
    for (double longitude : lon_values) {
      GeographicLib::GeoCoords gc = GeographicLib::GeoCoords(latitude, longitude, 10);
      double easting = gc.Easting();
      double northing = gc.Northing();

      double elevation_dted_wgs = dted_terrain->QueryLongLat(longitude, latitude, false);
      double elevation_dted_utm = dted_terrain->QueryUTM(easting, northing, false);

      double elevation_vtk_wgs = vtk_terrain->QueryLongLat(longitude, latitude, false);
      double elevation_vtk_utm = vtk_terrain->QueryUTM(easting, northing, false);

      ASSERT_DOUBLE_EQ(elevation_dted_wgs, elevation_dted_utm);
      ASSERT_DOUBLE_EQ(elevation_vtk_wgs, elevation_vtk_utm);
    }
  }
}
