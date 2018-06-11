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

#include <scrimmage/pubsub/MessageBase.h>
#include <cmath>

#if ENABLE_PYTHON_BINDINGS == 1
// namespace py = pybind11;
#endif

namespace scrimmage {

#if ENABLE_PYTHON_BINDINGS == 1
// void MessageBase::serialize_to_python(std::string module_name, std::string object_name) {
//     if (serialized_data == "") {
//         return;
//     }
//     py::module pb_module = pybind11::module::import(module_name.c_str());
//     py::object pb_object_class = pb_module.attr(object_name.c_str());
//     py_data = pb_object_class();
//     py::object parse_func = py_data.attr("ParseFromString");
//     parse_func(pybind11::bytes(serialized_data));
// }
#endif


} // namespace scrimmage
