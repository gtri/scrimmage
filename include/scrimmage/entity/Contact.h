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

#ifndef INCLUDE_SCRIMMAGE_ENTITY_CONTACT_H_
#define INCLUDE_SCRIMMAGE_ENTITY_CONTACT_H_

#include <scrimmage/common/ID.h>
#include <scrimmage/pubsub/Message.h>

#include <memory>
#include <unordered_map>
#include <string>
#include <iosfwd>

namespace scrimmage_proto {
class ContactVisual;
using ContactVisualPtr = std::shared_ptr<scrimmage_proto::ContactVisual>;
}

namespace scrimmage {

class State;
using StatePtr = std::shared_ptr<State>;

class Contact {
 public:
    enum class Type {AIRCRAFT = 0, QUADROTOR, SPHERE, MESH, UNKNOWN};

    Contact();

    Contact(ID &id, double radius, StatePtr &state, Type type,
        scrimmage_proto::ContactVisualPtr cv,
        const std::unordered_map<std::string, MessageBasePtr> &properties);

    template <class T>
    std::unordered_map<std::string, MessagePtr<T>> get_properties(std::string name = "") {
        std::unordered_map<std::string, MessagePtr<T>> out;
        for (auto &kv : properties_) {
            if (name == "" || kv.first.find(name) != std::string::npos) {
                auto property_cast =
                    std::dynamic_pointer_cast<Message<T>>(kv.second);
                if (property_cast) {
                    out[kv.first] = property_cast;
                }
            }
        }
        return out;
    }

    std::unordered_map<std::string, MessageBasePtr> &properties() {return properties_;}
    void set_id(const ID &id);
    ID &id();

    void set_state(StatePtr &state);
    StatePtr &state();
    std::shared_ptr<const State> state_const() const;

    void set_type(Type type);
    Type type();

    scrimmage_proto::ContactVisualPtr & contact_visual();

    void set_active(bool active);
    bool active();

    void set_radius(double radius);
    double radius() { return radius_; }

    friend std::ostream& operator<<(std::ostream& os, const Contact& c);

 protected:
    ID id_;
    StatePtr state_;
    Type type_ = Type::AIRCRAFT;
    scrimmage_proto::ContactVisualPtr contact_visual_;
    bool active_ = true;
    double radius_ = 0;
    std::unordered_map<std::string, MessageBasePtr> properties_;
};

using ContactMap = std::unordered_map<int, Contact>;
using ContactMapPtr = std::shared_ptr<ContactMap>;
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_ENTITY_CONTACT_H_
