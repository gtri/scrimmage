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

#include <scrimmage/common/ID.h>

#include <iostream>

namespace scrimmage {

ID::ID() : id_(0), sub_swarm_id_(0), team_id_(0) {}

ID::ID(int id, int sub_swarm_id, int team_id) :
    id_(id), sub_swarm_id_(sub_swarm_id), team_id_(team_id) {}

void ID::set_id(int id) { id_ = id; }

void ID::set_sub_swarm_id(int sub_swarm_id) {sub_swarm_id_ = sub_swarm_id; }

void ID::set_team_id(int team_id) { team_id_ = team_id; }

int ID::id() const { return id_; }

int ID::sub_swarm_id() const { return sub_swarm_id_; }

int ID::team_id() const { return team_id_; }

bool ID::operator==(const ID &other) const {
    return id_ == other.id_ &&
        sub_swarm_id_ == other.sub_swarm_id_ &&
        team_id_ == other.team_id_;
}

bool ID::operator<(const ID &other) const {
    if (id_ < other.id_) {
        return true;
    } else if (sub_swarm_id_ < other.sub_swarm_id_) {
        return true;
    } else if (team_id_ < other.team_id_) {
        return true;
    } else {
        return false;
    }
}

std::ostream &operator<<(std::ostream &os, const ID &id) {
    os << id.id_ << ", " << id.sub_swarm_id_ << ", " << id.team_id_;
    return os;
}

} // namespace scrimmage

