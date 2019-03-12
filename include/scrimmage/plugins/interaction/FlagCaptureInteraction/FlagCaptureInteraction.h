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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_FLAGCAPTUREINTERACTION_FLAGCAPTUREINTERACTION_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_FLAGCAPTUREINTERACTION_FLAGCAPTUREINTERACTION_H_

#include <scrimmage/simcontrol/EntityInteraction.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/common/ID.h>

#include <scrimmage/plugins/interaction/Boundary/BoundaryBase.h>
#include <scrimmage/proto/Shape.pb.h>

#include <map>
#include <list>
#include <string>
#include <memory>

namespace sc = scrimmage;
namespace sci = scrimmage::interaction;

namespace scrimmage {
namespace interaction {

class FlagCaptureInteraction : public scrimmage::EntityInteraction {
 public:
    FlagCaptureInteraction();
    bool init(std::map<std::string, std::string> &mission_params,
              std::map<std::string, std::string> &plugin_params) override;
    bool step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                 double t, double dt) override;

 protected:
    int flag_boundary_id_ = 0;
    int capture_boundary_id_ = 0;

    std::shared_ptr<sci::BoundaryBase> flag_boundary_;
    sp::Shape flag_boundary_shape_;

    std::shared_ptr<sci::BoundaryBase> capture_boundary_;
    sp::Shape capture_boundary_shape_;

    bool flag_taken_ = false;
    bool flag_captured_ = false;
    scrimmage::ID entity_with_flag_;

    scrimmage::PublisherPtr flag_taken_pub_;
    scrimmage::PublisherPtr flag_captured_pub_;

 private:
};
} // namespace interaction
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_FLAGCAPTUREINTERACTION_FLAGCAPTUREINTERACTION_H_
