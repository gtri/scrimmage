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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_NETWORK_SPHERENETWORK_SPHERENETWORK_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_NETWORK_SPHERENETWORK_SPHERENETWORK_H_

#include <scrimmage/pubsub/Network.h>

#include <map>
#include <string>

namespace scrimmage {
namespace network {

class SphereNetwork : public scrimmage::Network {
 public:
    bool init(std::map<std::string, std::string> &mission_params,
                      std::map<std::string, std::string> &plugin_params) override;
 protected:
    bool is_reachable(const scrimmage::EntityPluginPtr &pub_plugin,
                              const scrimmage::EntityPluginPtr &sub_plugin) override;

    bool is_successful_transmission(const scrimmage::EntityPluginPtr &pub_plugin,
                                            const scrimmage::EntityPluginPtr &sub_plugin) override;
    double range_;
    double prob_transmit_;

    // Attributes of a boundary plane that blocks communication. Example use
    // would be putting a boundary at a water/air intersection such that
    // communications wouldn't go between water and the surface.
    double comms_boundary_altitude_ = 0;
    // The buffer zone around the boundary plane that doesn't directly block
    // comms
    double comms_boundary_epsilon_ = 0;
    bool filter_comms_plane_ = false;
    bool within_planar_boundary(double z1, double z2);
};
} // namespace network
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_NETWORK_SPHERENETWORK_SPHERENETWORK_H_
