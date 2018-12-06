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
 * @author Jeremy Feltracco <jeremy.feltracco@gtri.gatech.edu>
 * @date 06 December 2018
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_RANDOMATTRIT_RANDOMATTRIT_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_RANDOMATTRIT_RANDOMATTRIT_H_

#include <scrimmage/simcontrol/EntityInteraction.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/pubsub/PubSub.h>
#include <scrimmage/pubsub/Publisher.h>

#include <map>
#include <list>
#include <string>
#include <vector>

namespace scrimmage {
namespace interaction {

class RandomAttrit : public scrimmage::EntityInteraction {
 public:
    RandomAttrit();
    bool init(std::map<std::string, std::string> &mission_params,
              std::map<std::string, std::string> &plugin_params) override;
    bool step_entity_interaction(std::list<scrimmage::EntityPtr> &ents,
                                 double t, double dt) override;
 protected:
    bool started_ = false;
    int team_id_ = -1;
    std::string decay_method_ = "";
    double start_wait_time_s_ = 0.0;
    double duration_s_ = 10.0;
    std::vector<int> stay_alive_ids_;
    int total_num_entities_ = 0;
    double start_num_ent_ = 0.0;
    double num_ent_ = 0.0;
    Random random_;

    PublisherPtr attrit_pub_;
 private:
};
} // namespace interaction
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_RANDOMATTRIT_RANDOMATTRIT_H_
