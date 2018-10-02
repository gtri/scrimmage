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

#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/entity/Contact.h>
#include <scrimmage/math/State.h>

#include <memory>
#include <unordered_map>

#include <GeographicLib/LocalCartesian.hpp>

namespace scrimmage {

Autonomy::Autonomy() : state_(std::make_shared<State>()),
    desired_state_(std::make_shared<State>()), need_reset_(false), is_controlling_(false) {}

void Autonomy::set_contacts(ContactMapPtr &contacts) {
    contacts_ = contacts;
}

void Autonomy::set_contacts_from_plugin(AutonomyPtr &ptr) {
    contacts_ = ptr->contacts_;
}

RTreePtr &Autonomy::rtree() {return rtree_;}

void Autonomy::set_rtree(RTreePtr &rtree) {rtree_ = rtree;}

StatePtr &Autonomy::state() {return state_;}

void Autonomy::set_state(StatePtr &state) {state_ = state;}

void Autonomy::set_projection(std::shared_ptr<GeographicLib::LocalCartesian> &proj) {
    proj_ = proj;
}

std::string &Autonomy::logging_msg() {return logging_msg_;}

bool Autonomy::get_is_controlling() {return is_controlling_;}

void Autonomy::set_is_controlling(bool is_controlling) {is_controlling_ = is_controlling;}

std::string Autonomy::type() { return std::string("Autonomy"); }

bool Autonomy::step_autonomy(double /*t*/, double /*dt*/) { return false; }
bool Autonomy::posthumous(double /*t*/) { return true; }
void Autonomy::init() {}
void Autonomy::init(std::map<std::string, std::string> &/*params*/) {}
bool Autonomy::need_reset() {return need_reset_;}

StatePtr &Autonomy::desired_state() {return desired_state_;}

void Autonomy::set_desired_state(StatePtr desired_state) {desired_state_ = desired_state;}

ContactMapPtr &Autonomy::get_contacts() {return contacts_;}

ContactMap &Autonomy::get_contacts_raw() {return *contacts_;}

void Autonomy::close(double /*t*/) {
    proj_ = nullptr;
    state_ = nullptr;
    desired_state_ = nullptr;
    contacts_ = nullptr;
    rtree_ = nullptr;
}
} // namespace scrimmage
