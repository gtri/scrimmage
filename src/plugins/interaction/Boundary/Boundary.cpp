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

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>

#include <scrimmage/plugins/interaction/Boundary/BoundaryBase.h>
#include <scrimmage/plugins/interaction/Boundary/Boundary.h>
#include <scrimmage/plugins/interaction/Boundary/Cuboid.h>

#include <memory>
#include <limits>
#include <iostream>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::EntityInteraction,
                scrimmage::interaction::Boundary,
                Boundary_plugin)

namespace scrimmage {
namespace interaction {

Boundary::Boundary() {
}

bool Boundary::init(std::map<std::string, std::string> &mission_params,
                    std::map<std::string, std::string> &plugin_params) {
    std::string boundary_type = sc::get<std::string>("boundary_type",
                                                     plugin_params,
                                                     "cuboid");

    std::vector<std::vector<std::string>> vecs;
    std::string cuboid_boundary = sc::get<std::string>("cuboid_boundary",
                                                       plugin_params, "");

    if (boundary_type == "cuboid") {
        if (!sc::get_vec_of_vecs(cuboid_boundary, vecs)) {
            cout << "Failed to parse cuboid_boundary." << endl;
            return false;
        }

        // Convert string representation of points into Eigen::Vector3d
        std::vector<Eigen::Vector3d> points;
        for (std::vector<std::string> vec : vecs) {
            if (vec.size() != 3) {
                cout << "Invalid vector size in: " << cuboid_boundary << endl;
                return false;
            }
            Eigen::Vector3d p(std::stod(vec[0]), std::stod(vec[1]),
                              std::stod(vec[2]));
            points.push_back(p);
        }

        boundary_info_.type = BoundaryInfo::Type::Cuboid;
        boundary_info_.points = points;

        std::shared_ptr<Cuboid> c = std::make_shared<Cuboid>();
        c->set_points(points);
        boundary_ = c;
    } else {
        cout << "Invalid boundary_type: " << boundary_type << endl;
        return false;
    }

    if (sc::get<bool>("show_boundary", plugin_params, false)) {
        double opacity = sc::get<double>("boundary_opacity", plugin_params,
                                         1.0);
        std::vector<int> color;
        if (!sc::get_vec("boundary_color", plugin_params, " ", color, 3)) {
            cout << "Failed to parse boundary_color" << endl;
            color.clear();
            color = {255, 0, 0};
        }
        boundary_->set_visual(color[0], color[1], color[2], opacity);
        shapes_.insert(shapes_.end(), boundary_->shapes().begin(),
                       boundary_->shapes().end());
    }

    std::string network_name = sc::get("network_name", plugin_params, "GlobalNetwork");
    pub_boundary_ = advertise(network_name, "Boundary");

    return true;
}


bool Boundary::step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                                  double t, double dt) {
    if (!boundary_published_) {
        boundary_published_ = true;
        auto msg = std::make_shared<sc::Message<BoundaryInfo>>();
        msg->data = boundary_info_;
        pub_boundary_->publish(msg);
    }
    return true;
}
} // namespace interaction
} // namespace scrimmage
