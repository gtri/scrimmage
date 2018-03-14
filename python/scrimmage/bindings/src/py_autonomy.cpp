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

#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/plugin_manager/Plugin.h>
#include <scrimmage/pubsub/Network.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>

namespace py = pybind11;
namespace sc = scrimmage;

void add_autonomy(pybind11::module &m) {
    py::class_<sc::Plugin, std::shared_ptr<sc::Plugin>> plugin(m, "Plugin");
    plugin.def(py::init<>())
        .def("name", &sc::Plugin::name)
        .def("type", &sc::Plugin::type);
        // TODO .def_property("network", &sc::Plugin::network, &sc::Plugin::set_network)
        // TODO .def_property("pubs", &sc::Plugin::pubs, &sc::Plugin::set_pubs)
        // TODO .def_property("subs", &sc::Plugin::subs, &sc::Plugin::set_subs)
        // TODO .def("create_publisher", &sc::Plugin::create_publisher)
        // TODO .def("create_subscriber", &sc::Plugin::create_subscriber)
        // TODO .def("stop_publishing", &sc::Plugin::stop_publishing)
        // TODO .def("stop_subscribing", &sc::Plugin::stop_subscribing);

    py::class_<sc::Autonomy, std::shared_ptr<sc::Autonomy>>(m, "Autonomy", plugin)
        .def(py::init<>())
        .def_property("desired_state", &sc::Autonomy::desired_state, &sc::Autonomy::set_desired_state)
        .def("init", (void (sc::Autonomy::*)(std::map<std::string, std::string>&)) &sc::Autonomy::init)
        .def("name", &sc::Autonomy::name)
        .def("type", &sc::Autonomy::type)
        .def("step_autonomy", &sc::Autonomy::step_autonomy)
        .def("get_contacts", &sc::Autonomy::get_contacts_raw)
        .def("shapes", &sc::Autonomy::shapes); // TODO, anyone using shapes in Python yet?
}
