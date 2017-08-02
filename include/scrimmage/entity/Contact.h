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

#ifndef CONTACT_H_
#define CONTACT_H_
#include <scrimmage/fwd_decl.h>
#include <scrimmage/common/ID.h>
#include <memory>
#include <unordered_map>
#include <list>

#include <scrimmage/proto/Visual.pb.h>

namespace scrimmage {

using ContactVisualPtr = std::shared_ptr<scrimmage_proto::ContactVisual>;

class Contact {
 public:

    enum class Type {AIRCRAFT = 0, QUADROTOR, SPHERE, MESH, UNKNOWN};

    Contact();

    Contact(ID &id, double radius, StatePtr &state, Type type,
            ContactVisualPtr cv,
            std::unordered_map<std::string, std::list<SensablePtr>> sp);
    
    void set_id(const ID &id);
    ID &id();

    void set_state(StatePtr &state);
    StatePtr &state();

    void set_type(Type type);
    Type type();

    ContactVisualPtr & contact_visual();

    std::unordered_map<std::string, std::list<SensablePtr>> &sensables();

    void set_active(bool active);
    bool active();
    double radius() { return radius_; }

 protected:
    ID id_;
    StatePtr state_;
    Type type_;
    ContactVisualPtr contact_visual_;
    std::unordered_map<std::string, std::list<SensablePtr>> sensables_;
    bool active_;
    double radius_;
};

using ContactMap = std::unordered_map<int, Contact>;
using ContactMapPtr = std::shared_ptr<ContactMap>;
}
#endif
