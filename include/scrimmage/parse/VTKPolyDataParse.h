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

#ifndef INCLUDE_SCRIMMAGE_PARSE_VTKPOLYDATAPARSE_H_
#define INCLUDE_SCRIMMAGE_PARSE_VTKPOLYDATAPARSE_H_

#include <array>
#include <string>
#include <vector>
#include <memory>

namespace scrimmage {
    class VTKPolyDataParse {
      public:
        VTKPolyDataParse() = delete;; 
        static std::unique_ptr<std::array<std::vector<double>, 3>> Parse(const std::string& filename);
    };

} // namespace scrimmage
#endif //INCLUDE_SCRIMMAGE_PARSE_VTKPOLYDATAPARSE_H_
