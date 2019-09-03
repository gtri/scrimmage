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
#include <scrimmage/entity/EntityPlugin.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/SubscriberBase.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/pubsub/MessageBase.h>
#include <scrimmage/pubsub/PubSub.h>
#include <py_utils.h>

namespace py = pybind11;
namespace sc = scrimmage;

void add_pubsub(pybind11::module &m) {
    py::class_<sc::MessageBase, std::shared_ptr<sc::MessageBase>>(m, "Message")
        // .def(py::init<>())
        // .def(py::init<std::string>())
        // .def(py::init<std::string, py::object>())
        .def_readwrite("time", &sc::MessageBase::time)
        .def_readwrite("serialized_data", &sc::MessageBase::serialized_data);
        // .def_readwrite("data", &sc::MessageBase::py_data);

    py::class_<sc::NetworkDevice, std::shared_ptr<sc::NetworkDevice>>(m, "NetworkDevice")
        .def(py::init<>())
        .def_property("topic", &sc::NetworkDevice::get_topic, &sc::NetworkDevice::set_topic);

    py::class_<sc::Publisher, std::shared_ptr<sc::Publisher>, sc::NetworkDevice>(m, "Publisher")
         .def(py::init<>())
         .def("publish", &sc::Publisher::publish<scrimmage::MessageBase>);

    // TODO : Need to update for new callback interface
    // py::class_<sc::Subscriber<sc::MessagePtr>, std::shared_ptr<sc::Subscriber>, sc::SubscriberBase, sc::NetworkDevice>(m, "Subscriber")
    //     .def(py::init<>());

    py::class_<sc::PubSub, std::shared_ptr<sc::PubSub>>(m, "PubSub")
        .def(py::init<>())
        .def("add_network_name", &sc::PubSub::add_network_name)
        .def("advertise", &sc::PubSub::advertise);
    //     .def("add_publisher", &sc::Network::add_publisher)
    //     .def("add_subscriber", &sc::Network::add_subscriber)
    //     .def("rm_publisher", &sc::Network::rm_publisher)
    //     .def("rm_publisher", &sc::Network::rm_publisher);
}
