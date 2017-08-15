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

#ifndef INCLUDE_SCRIMMAGE_HASH_H_
#define INCLUDE_SCRIMMAGE_HASH_H_
#include <functional>

// custom specialization of std::hash can be injected in namespace std
// see http://en.cppreference.com/w/cpp/utility/hash
namespace std {
template<> struct hash<scrimmage::ID> {
    size_t operator()(scrimmage::ID const& id) const {
        // assume up to 16 teams and 64 subswarms
        // for a 32-bit int this leaves 16 bits (up to 65000 ids)
        return id.team_id() &
            (id.sub_swarm_id() << 4) &
            (id.id() << 12);
    }
};
} // namespace std

#endif // INCLUDE_SCRIMMAGE_HASH_H_
