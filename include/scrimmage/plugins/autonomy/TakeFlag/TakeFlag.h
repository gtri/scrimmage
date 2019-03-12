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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_TAKEFLAG_TAKEFLAG_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_TAKEFLAG_TAKEFLAG_H_
#include <scrimmage/autonomy/Autonomy.h>

#include <scrimmage/plugins/interaction/Boundary/BoundaryBase.h>
#include <scrimmage/proto/Shape.pb.h>

#include <string>
#include <map>
#include <utility>
#include <memory>

namespace scrimmage {

namespace interaction {
class BoundaryBase;
}

namespace autonomy {
class TakeFlag : public scrimmage::Autonomy {
 public:
    TakeFlag();
    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;

 protected:
    int flag_boundary_id_ = -1;
    int capture_boundary_id_ = -1;

    scrimmage::PublisherPtr pub_wp_list_;

    std::map<int, std::pair<scrimmage_proto::Shape,
        std::shared_ptr<interaction::BoundaryBase>>> boundaries_;

    bool has_flag_ = false;

    void publish_waypoint(const Eigen::Vector3d &point);
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_TAKEFLAG_TAKEFLAG_H_
