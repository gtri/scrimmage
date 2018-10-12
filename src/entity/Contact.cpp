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

#include <scrimmage/entity/Contact.h>
#include <scrimmage/math/State.h>
#include <scrimmage/common/ID.h>

#include <memory>
#include <iostream>

namespace scrimmage {

Contact::Contact() : state_(std::make_shared<State>()) {}

Contact::Contact(ID &id, double radius, StatePtr &state, Type type,
                 scrimmage_proto::ContactVisualPtr cv,
                 const std::unordered_map<std::string, MessageBasePtr> &properties) :
    id_(id), state_(state), type_(type), contact_visual_(cv),
    active_(true), radius_(radius), properties_(properties) {}

void Contact::set_id(const ID &id) { id_ = id; }

ID &Contact::id() { return id_; }

void Contact::set_state(StatePtr &state) { state_ = state; }

StatePtr &Contact::state() { return state_; }

std::shared_ptr<const State> Contact::state_const() const { return state_; }

void Contact::set_type(Contact::Type type) { type_ = type; }

Contact::Type Contact::type() { return type_; }

scrimmage_proto::ContactVisualPtr &Contact::contact_visual()
{ return contact_visual_; }

void Contact::set_active(bool active) { active_ = active; }

void Contact::set_radius(double radius) { radius_ = radius; }

bool Contact::active() { return active_; }

std::ostream& operator<<(std::ostream& os, const Contact& c) {
    os << c.id_ << ": " << *c.state_;
    return os;
}
} // namespace scrimmage
