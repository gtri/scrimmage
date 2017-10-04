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
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/pubsub/MessageBase.h>
#include <scrimmage/pubsub/Network.h>
#include <py_utils.h>

namespace py = pybind11;
namespace sc = scrimmage;

void add_network(pybind11::module &m) {
    py::class_<sc::MessageBase, std::shared_ptr<sc::MessageBase>>(m, "Message")
        .def(py::init<>())
        .def(py::init<int>())
        .def(py::init<int, std::string>())
        .def(py::init<int, std::string, py::object>())
        .def_readwrite("sender", &sc::MessageBase::sender)
        .def_readwrite("serialized_data", &sc::MessageBase::serialized_data)
        .def_readwrite("data", &sc::MessageBase::py_data);

    py::class_<sc::NetworkDevice, std::shared_ptr<sc::NetworkDevice>>(m, "NetworkDevice")
        .def(py::init<>())
        .def_property("topic", &sc::NetworkDevice::get_topic, &sc::NetworkDevice::set_topic);

    py::class_<sc::Publisher, std::shared_ptr<sc::Publisher>, sc::NetworkDevice>(m, "Publisher")
        .def(py::init<>())
        .def("publish", &sc::Publisher::publish);

    py::class_<sc::Subscriber, std::shared_ptr<sc::Subscriber>, sc::NetworkDevice>(m, "Subscriber")
        .def(py::init<>());

    py::class_<sc::Network, std::shared_ptr<sc::Network>>(m, "Network")
        .def(py::init<>())
        .def("add_publisher", &sc::Network::add_publisher)
        .def("add_subscriber", &sc::Network::add_subscriber)
        .def("rm_publisher", &sc::Network::rm_publisher)
        .def("rm_publisher", &sc::Network::rm_publisher);
}
