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

#include <(>>>PROJECT_NAME<<<)/plugins/network/(>>>PLUGIN_NAME<<<)/(>>>PLUGIN_NAME<<<).h>

#include <scrimmage/common/ID.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/common/RTree.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/network/SphereNetwork/SphereNetwork.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/pubsub/Message.h>

#include <memory>
#include <limits>
#include <iostream>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Network,
                scrimmage::network::(>>>PLUGIN_NAME<<<),
                (>>>PLUGIN_NAME<<<)_plugin)

namespace scrimmage {
namespace network {

(>>>PLUGIN_NAME<<<)::(>>>PLUGIN_NAME<<<)() {
}

bool (>>>PLUGIN_NAME<<<)::init(std::map<std::string, std::string> &mission_params,
                               std::map<std::string, std::string> &plugin_params) {
    return true;
}

bool (>>>PLUGIN_NAME<<<)::is_reachable(const scrimmage::PluginPtr &pub_plugin,
                                       const scrimmage::PluginPtr &sub_plugin) {
    return true;
}

bool (>>>PLUGIN_NAME<<<)::is_successful_transmission(const scrimmage::PluginPtr &pub_plugin,
                                                     const scrimmage::PluginPtr &sub_plugin) {
    return true;
}


} // namespace network
} // namespace scrimmage
