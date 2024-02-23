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

#ifndef INCLUDE_SCRIMMAGE_TERRAIN_TERRAINREADER_DTEDREADER_H_
#define INCLUDE_SCRIMMAGE_TERRAIN_TERRAINREADER_DTEDREADER_H_

#include <scrimmage/parse/TerrainReaders/TerrainReader.h>
#include <scrimmage/common/terrain/ElevationGrid.h>

#include <gdal_priv.h>

#include <string>
#include <memory>

namespace scrimmage {
  class DTEDReader: public TerrainReader {
    public:
      DTEDReader(std::string filename);

      std::unique_ptr<terrain::ElevationGrid> Parse() const; 

      std::unique_ptr<std::array<std::vector<double>, 3>> ParseAsUTM(
          const int utm_zone, const bool northern_hemisphere) const;

      std::string get_filename() const {
        return filename_;
      }

      void set_filename(std::string filename) {
        filename_ = filename;
      }

    protected:
      std::string filename_;
      char c_filename_[1024];

      std::unique_ptr<GDALDataset, GDALDatasetUniquePtrReleaser> OpenDTED() const;
      //void CloseDTED(std::unique_ptr<GDALDataset> dataset) const;

      std::unique_ptr<std::array<std::vector<double>, 3>>
        ReadDTED(GDALDataset& dataset) const;
  };

} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_TERRAIN_TERRAINREADER_DTEDREADER_H_
