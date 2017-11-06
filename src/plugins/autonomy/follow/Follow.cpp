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

#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/autonomy/follow/Follow.h>

#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::Follow, Follow_plugin)

namespace scrimmage {
namespace autonomy {

Follow::Follow() {}

void Follow::init(std::map<std::string, std::string> &params) {
     desired_state_->vel() = 21 * Eigen::Vector3d::UnitX();
     desired_state_->quat().set(0, 0, state_->quat().yaw());
     desired_state_->pos() = state_->pos()(2) * Eigen::Vector3d::UnitZ();
}

bool Follow::step_autonomy(double t, double dt) {
     // // Find the closest contact
     // const GeographicLib::Geodesic& geod = Geodesic::WGS84();
     // double min_dist = 1e10;
     // bool contact_found = false;
     // std::list<scrimmage::Contact>::iterator min_champ;
     // for (std::list<scrimmage::Contact>::iterator it = contacts_.begin();
     //      it != contacts_.end(); it++) {
     //
     //      if (it->id().id() != parent_->id().id()) {
     //           //cout << "Own Lat: " << state_.pose().latitude() << endl;
     //           //cout << "Own Long: " << state_.pose().longitude() << endl;
     //           //cout << "Cnt Lat: " << it->state().pose().latitude()<< endl;
     //           //cout << "Cnt Long: " << it->state().pose().longitude()<< endl;
     //
     //           double dist;
     //           geod.Inverse(state_.pose().latitude(), state_.pose().longitude(),
     //                        it->state().pose().latitude(), it->state().pose().longitude(),
     //                        dist);
     //
     //           if (dist < min_dist) {
     //                min_dist = dist;
     //                min_champ = it;
     //                contact_found = true;
     //           }
     //      }
     // }
     //
     // // Head towards the closet contact if it exists
     // if (contact_found) {
     //      Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());
     //      // Alternatively: const Geocentric& earth = Geocentric::WGS84();
     //
     //      LocalCartesian proj(state_.pose().latitude(),
     //                          state_.pose().longitude(), 0, earth);
     //      // Sample forward calculation
     //      double x, y, z;
     //      double h = 0;
     //      proj.Forward(min_champ->state().pose().latitude(), min_champ->state().pose().longitude(), h, x, y, z);
     //      double heading = atan2(x,y) * 180.0 / scrimmage::PI;
     //      desired_state_.pose().set_heading(heading);
     // }

     return true;
}
} // namespace autonomy
} // namespace scrimmage
