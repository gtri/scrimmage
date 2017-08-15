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

#ifndef INCLUDE_SCRIMMAGE_COMMON_ID_H_
#define INCLUDE_SCRIMMAGE_COMMON_ID_H_

namespace scrimmage {
class ID {
 public:
    ID();

    ID(int id, int sub_swarm_id, int team_id);

    void set_id(int id);
    void set_sub_swarm_id(int sub_swarm_id);
    void set_team_id(int team_id);

    int id() const;
    int sub_swarm_id() const;
    int team_id() const;

    bool operator==(const ID &other) const;
    bool operator<(const ID &other) const;

 protected:
    int id_;
    int sub_swarm_id_;
    int team_id_;
};
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_COMMON_ID_H_
