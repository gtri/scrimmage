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
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>

#include <scrimmage/plugins/autonomy/RLConsensus/RLConsensus.h>

#include <iostream>

#include <boost/range/algorithm/count_if.hpp>
#include <boost/range/adaptor/transformed.hpp>

namespace sp = scrimmage_proto;
namespace br = boost::range;
namespace ba = boost::adaptors;

REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::RLConsensus, RLConsensus_plugin)

namespace scrimmage {
namespace autonomy {

void RLConsensus::set_environment() {
    RLSimple::set_environment();
    reward_range = std::make_pair(0, 1 / parent_->mp()->tend());
}

std::pair<bool, double> RLConsensus::calc_reward() {
    const bool done = false;
    const double x = state_->pos()(0);
    auto dist = [&](auto &kv) {return std::abs(kv.second.state()->pos()(0) - x);};
    auto close = [&](double d) {return std::abs(d) < radius_;};

    const int num_veh = contacts_->size();
    const int num_close = br::count_if(*contacts_ | ba::transformed(dist), close) - 1;

    const double reward = num_close / (num_veh * parent_->mp()->tend());
    return {done, reward};
}

} // namespace autonomy
} // namespace scrimmage
