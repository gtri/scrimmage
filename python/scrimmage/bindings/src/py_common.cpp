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
#include <py_utils.h>
#include <scrimmage/common/ID.h>
#include <scrimmage/entity/Contact.h>
#include <scrimmage/math/State.h>

namespace py = pybind11;
namespace sc = scrimmage;

void add_common(pybind11::module &m) {
    py::class_<sc::ID>(m, "ID")
        .def(py::init<int, int, int>())
        .def_property("id", &sc::ID::id, &sc::ID::set_id)
        .def_property("sub_swarm_id", &sc::ID::sub_swarm_id, &sc::ID::set_sub_swarm_id)
        .def_property("team_id", &sc::ID::team_id, &sc::ID::set_team_id);

    py::class_<sc::Contact, std::shared_ptr<sc::Contact>> contact(m, "Contact");

    contact.def(py::init<>())
           .def_property("id", &sc::Contact::id, &sc::Contact::set_id)
           .def_property("state", &sc::Contact::state, &sc::Contact::set_state)
           .def_property("type", &sc::Contact::type, &sc::Contact::set_type);

    py::enum_<sc::Contact::Type>(contact, "ContactType")
        .value("AIRCRAFT", sc::Contact::Type::AIRCRAFT)
        .value("QUADROTOR", sc::Contact::Type::QUADROTOR)
        .value("SPHERE", sc::Contact::Type::SPHERE)
        .value("MESH", sc::Contact::Type::MESH)
        .value("UNKNOWN", sc::Contact::Type::UNKNOWN)
        .export_values();

}

