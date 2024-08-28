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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_NETWORK_GPUSPHERENETWORK_GPUSPHERENETWORKUTILS_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_NETWORK_GPUSPHERENETWORK_GPUSPHERENETWORKUTILS_H_

#include <scrimmage/gpu/GPUMapBuffer.h>
#include <scrimmage/pubsub/Network.h>

#include <map>
#include <set>

namespace scrimmage {
namespace network {

class GPUSphereNetworkUtils {
 public:
    GPUSphereNetworkUtils(double range, const GPUPluginBuildParams& network_kernel_params);
    std::set<std::pair<int, int>> proximity_pairs(std::map<int, StatePtr> states);

 protected:
    double range_;

    GPUControllerPtr gpu_;
    cl::CommandQueue queue_;
    cl::Kernel kernel_;
    std::size_t prefered_workgroup_size_multiple_;
    std::size_t max_workgroup_size_;
};
}  // namespace network
}  // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_PLUGINS_NETWORK_GPUSPHERENETWORK_GPUSPHERENETWORKUTILS_H_
