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
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_SCRIMMAGEOPENAIAUTONOMY_OPENAIUTILS_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_SCRIMMAGEOPENAIAUTONOMY_OPENAIUTILS_H_

#include <pybind11/pybind11.h>

#include <scrimmage/common/Visibility.h>

#include <utility>
#include <string>
#include <vector>

namespace scrimmage {

namespace autonomy {

pybind11::object DLL_PUBLIC get_gym_space(const std::string &type);

void DLL_PUBLIC to_continuous(
        std::vector<std::pair<double, double>> &p,
        pybind11::list &minima,
        pybind11::list &maxima);

void DLL_PUBLIC to_discrete(std::vector<double> &p, pybind11::list &maxima);

pybind11::object DLL_PUBLIC create_space(
        pybind11::list discrete_maxima,
        pybind11::list continuous_minima,
        pybind11::list continuous_maxima);
} // namespace autonomy
} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_SCRIMMAGEOPENAIAUTONOMY_OPENAIUTILS_H_
