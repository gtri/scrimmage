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


#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <scrimmage/math/Quaternion.h>
#include <scrimmage/math/State.h>
#include <py_utils.h>

namespace py = pybind11;
namespace sc = scrimmage;

void add_math(pybind11::module &m) {
    py::class_<sc::Quaternion>(m, "Quaternion")
        .def(py::init<double, double, double, double>(),
             py::arg("w"), py::arg("x"), py::arg("y"), py::arg("z"))
        .def(py::init<double, double, double>(),
             py::arg("roll"), py::arg("pitch"), py::arg("yaw"))
        .def(py::init<Eigen::Vector3d, double>(),
             py::arg("vector"), py::arg("scalar"))
        .def_property_readonly("w", (double &(sc::Quaternion::*)()) &sc::Quaternion::w)
        .def_property_readonly("x", (double &(sc::Quaternion::*)()) &sc::Quaternion::x)
        .def_property_readonly("y", (double &(sc::Quaternion::*)()) &sc::Quaternion::y)
        .def_property_readonly("z", (double &(sc::Quaternion::*)()) &sc::Quaternion::z)
        .def("set", (void (sc::Quaternion::*)(double, double, double)) &sc::Quaternion::set)
        .def("set", (void (sc::Quaternion::*)
                    (const Eigen::Vector3d &vector, double angle))
                    &sc::Quaternion::set)
        .def("roll", &sc::Quaternion::roll)
        .def("yaw", &sc::Quaternion::yaw)
        .def("pitch", &sc::Quaternion::pitch)
        .def("yaw", &sc::Quaternion::yaw)
        .def("rotation_angle", &sc::Quaternion::rotation_angle)
        .def("rotate", &sc::Quaternion::rotate)
        .def("rotate_reverse", &sc::Quaternion::rotate_reverse);

    py::class_<sc::State, std::shared_ptr<sc::State>>(m, "State")
        .def(py::init<>())
        .def(py::init<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d,
             sc::Quaternion>(), py::arg("position"), py::arg("velocity"),
             py::arg("angular_velocity"), py::arg("orientation"))
        .def("pos_mutable", py::overload_cast<>(&sc::State::pos))
        .def("vel_mutable", py::overload_cast<>(&sc::State::vel))
        .def("ang_vel_mutable", py::overload_cast<>(&sc::State::ang_vel))
        .def("quat_mutable", py::overload_cast<>(&sc::State::quat))
        .def("pos_const", py::overload_cast<>(&sc::State::pos, py::const_))
        .def("vel_const", py::overload_cast<>(&sc::State::vel, py::const_))
        .def("ang_vel_const", py::overload_cast<>(&sc::State::ang_vel, py::const_))
        .def("quat_const", py::overload_cast<>(&sc::State::quat, py::const_))
        .def("in_field_of_view", &sc::State::InFieldOfView)
        .def("rel_pos_local_frame", &sc::State::rel_pos_local_frame)
        .def("orient_global_frame", &sc::State::orient_global_frame);
}
