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

#include <scrimmage/plugins/autonomy/PyAutonomy/PyAutonomy.h>

#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/math/Quaternion.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/proto/Shape.pb.h>

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/eigen.h>
#include <cstddef>
#include <stdexcept>
#include <iostream>

using std::cout;
using std::endl;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::PyAutonomy,
                PyAutonomy_plugin)

namespace py = pybind11;
namespace sc = scrimmage;
namespace sp = scrimmage_proto;

namespace py = pybind11;

namespace scrimmage {
namespace autonomy {

PyAutonomy::PyAutonomy() {
    need_reset_ = true;
}

void PyAutonomy::init(std::map<std::string, std::string> &params) {
    py_obj_ = get_py_obj(params);
    py_obj_.attr("id") = py::cast(parent_->id());
    init_py_obj(params);
}

py::object PyAutonomy::get_py_obj(std::map<std::string, std::string> &params) {
    py::module module = py::module::import(params["module"].c_str());
    py::object py_obj_class = module.attr(params["class"].c_str());
    py_obj_ = py_obj_class();
    return py_obj_;
}

void PyAutonomy::init_py_obj(std::map<std::string, std::string> &params) {
    py::dict py_params;
    for (auto &kv : params) {
        if (kv.first != "module" && kv.first != "class" && kv.first != "library") {
            py_params[kv.first.c_str()] = py::str(kv.second);
        }
    }
    // py_obj_.attr("subs") = py::dict();
    // py_obj_.attr("pubs") = py::dict();
    py_obj_.attr("shapes") = py::list();

    py::object init = py_obj_.attr("init");
    init(py_params);
}

void PyAutonomy::cache_python_vars() {
    if (py_state_class_.ptr() == nullptr) {
        py::module py_scrimmage = py::module::import("scrimmage.bindings");
        py_state_class_ = py_scrimmage.attr("State");
        py_quat_class_ = py_scrimmage.attr("Quaternion");
        py_contact_class_ = py_scrimmage.attr("Contact");
        py_msg_class_ = py_scrimmage.attr("Message");

        py_contact_types_[sc::Contact::Type::AIRCRAFT] = py_contact_class_.attr("AIRCRAFT");
        py_contact_types_[sc::Contact::Type::QUADROTOR] = py_contact_class_.attr("QUADROTOR");
        py_contact_types_[sc::Contact::Type::SPHERE] = py_contact_class_.attr("SPHERE");
        py_contact_types_[sc::Contact::Type::MESH] = py_contact_class_.attr("MESH");
        py_contact_types_[sc::Contact::Type::UNKNOWN] = py_contact_class_.attr("UNKNOWN");

        py::module py_numpy = py::module::import("numpy");

        py_id_class_ = py_scrimmage.attr("ID");
    }
}

py::object PyAutonomy::state2py(sc::StatePtr &state) {
    cache_python_vars();

    sc::Quaternion quat = state->quat();
    Eigen::Vector3d pos = state->pos();
    Eigen::Vector3d vel = state->vel();
    Eigen::Vector3d ang_vel = state->ang_vel();

    py::object py_quat = py_quat_class_(quat.w(), quat.x(), quat.y(), quat.z());
    py::object py_pos = py::cast(pos);
    py::object py_vel = py::cast(vel);
    py::object py_ang_vel = py::cast(ang_vel);

    py::object py_state = py_state_class_(py_pos, py_vel, py_ang_vel, py_quat);
    return py_state;
}

py::object PyAutonomy::contact2py(scrimmage::Contact contact) {
    cache_python_vars();

    py::object py_contact = py_contact_class_();
    scrimmage::ID id = contact.id();
    py_contact.attr("state") = state2py(contact.state());
    py_contact.attr("id") = py_id_class_(id.id(), id.sub_swarm_id(), id.team_id());
    py_contact.attr("type") = py_contact_types_[contact.type()];

    return py_contact;
}

std::shared_ptr<scrimmage_proto::Shape> PyAutonomy::py2shape(const pybind11::handle& py_handle) {
    // Convert python shape to c++ shape.
    py::object shape_obj = py_handle.cast<py::object>();
    py::function serialize_func = shape_obj.attr("SerializeToString").cast<py::function>();
    std::shared_ptr<scrimmage_proto::Shape> cpp_shape(new scrimmage_proto::Shape());
    cpp_shape->ParseFromString(serialize_func().cast<std::string>());

    return cpp_shape;
}

// void PyAutonomy::sub_msgs_to_py_subs() {
//     // copy all sub msgs to py_subs
//     py::dict py_pubs = py_obj_.attr("pubs").cast<py::dict>();
//     py::dict py_subs = py_obj_.attr("subs").cast<py::dict>();
//
//     for (auto &kv : subs_) {
//         const char* topic = kv.first.c_str();
//         sc::SubscriberPtr &sub  = kv.second;
//         py::object py_sub = py_subs[topic];
//
//         py::list py_msg_list;
//         for (auto msg : sub->msgs<sc::MessageBase>()) {
//             py_msg_list.append(py::cast(*msg));
//         }
//         py_subs[topic].attr("msg_list") = py_msg_list;
//     }
//
//     if (step_autonomy_called_) {
//         for (auto kv : py_pubs) {
//             kv.second.attr("msg_list") = py::list();
//         }
//     }
// }

// void PyAutonomy::py_pub_msgs_to_pubs() {
//     py::dict py_pubs = py_obj_.attr("pubs").cast<py::dict>();
//     for (auto kv : py_pubs) {
//         std::string topic = kv.first.cast<std::string>();
//         sc::Publisher py_pub = kv.second.cast<sc::Publisher>();
//         for (auto msg : py_pub.msgs<sc::MessageBase>()) {
//             pubs_[topic]->add_msg(msg);
//         }
//         py_pub.clear_msg_list();
//     }
// }

// void PyAutonomy::sync_topics() {
//     py::dict py_pubs = py_obj_.attr("pubs").cast<py::dict>();
//     py::dict py_subs = py_obj_.attr("subs").cast<py::dict>();
//
//     // add any topics not in subs that are in py_subs
//     for (auto kv : py_subs) {
//         std::string topic = kv.first.cast<std::string>();
//         if (subs_.count(topic) == 0) {
//             create_subscriber(topic);
//         }
//         kv.second.attr("msg_list") = py::list();
//     }
//
//     // remove any topics in subs not in py_subs
//     for (auto &kv : subs_) {
//         std::string topic = kv.second->get_topic();
//         if (!py_subs.contains(topic.c_str())) {
//             subs_.erase(topic);
//         }
//     }
//
//     // add any topics not in pubs that are in py_pubs
//     for (auto kv : py_pubs) {
//         std::string topic = kv.first.cast<std::string>();
//         if (pubs_.count(topic) == 0) {
//             create_publisher(topic);
//         }
//     }
//
//     // remove any topics in pubs not in py_pubs
//     for (auto &kv : pubs_) {
//         std::string topic = kv.second->get_topic();
//         if (!py_pubs.contains(topic.c_str())) {
//             pubs_.erase(topic);
//         }
//     }
// }

bool PyAutonomy::step_autonomy(double t, double dt) {
    cache_python_vars();

    py_state_ = state2py(state_);

    py_obj_.attr("contacts") = py_contacts_;
    py_obj_.attr("state") = py_state_;

    // sub_msgs_to_py_subs();
    py::object step_autonomy = py_obj_.attr("step_autonomy");
    bool out = step_autonomy(py::float_(t), py::float_(dt)).cast<bool>();
    // sync_topics();
    // py_pub_msgs_to_pubs();

    sc::State state = py_obj_.attr("desired_state").cast<sc::State>();
    *desired_state_ = state;

    py::list py_shapes = py_obj_.attr("shapes").cast<py::list>();
    std::list< std::shared_ptr<scrimmage_proto::Shape> > cpp_shapes;
    std::transform(py_shapes.begin(), py_shapes.end(), std::back_inserter(cpp_shapes), py2shape);
    // shapes_ = cpp_shapes; // SHAPES TODO

    step_autonomy_called_ = true;

    return out;
}
} // namespace autonomy
} // namespace scrimmage
