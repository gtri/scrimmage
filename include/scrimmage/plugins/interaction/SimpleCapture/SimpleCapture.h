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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_SIMPLECAPTURE_SIMPLECAPTURE_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_SIMPLECAPTURE_SIMPLECAPTURE_H_

#include <scrimmage/simcontrol/EntityInteraction.h>

#include <map>
#include <unordered_set>
#include <string>
#include <list>

namespace scrimmage {
namespace interaction {
class SimpleCapture : public scrimmage::EntityInteraction {
 public:
    SimpleCapture();
    bool init(std::map<std::string, std::string> &mission_params,
              std::map<std::string, std::string> &plugin_params) override;
    bool step_entity_interaction(std::list<scrimmage::EntityPtr> &ents,
                                 double t, double dt) override;
 protected:
    double capture_range_;
    bool enable_team_captures_;
    bool enable_non_team_captures_;

    scrimmage::PublisherPtr team_capture_pub_;
    scrimmage::PublisherPtr non_team_capture_pub_;
    std::unordered_set<int> already_captured_;
};
}  // namespace interaction
}  // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_SIMPLECAPTURE_SIMPLECAPTURE_H_
