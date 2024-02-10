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

#ifndef INCLUDE_SCRIMMAGE_PARSE_DTEDPARSE_H_
#define INCLUDE_SCRIMMAGE_PARSE_DTEDPARSE_H_

#include <scrimmage/parse/TerrainParse.h>

#include <array>
#include <string>
#include <vector>

namespace scrimmage {
    class DTEDParse: public TerrainParse {
      public:
        DTEDParse(int utm_zone, bool northern_hemisphere); 

        std::optional<std::array<std::vector<double>, 3>> Parse(const std::string& filename);
      protected:
        int utm_zone_;
        bool utm_northern_hemisphere_;
    };

} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PARSE_DTEDPARSE_H_
