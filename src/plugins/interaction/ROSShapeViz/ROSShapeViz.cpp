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

#include <scrimmage/plugins/interaction/ROSShapeViz/ROSShapeViz.h>

#include <scrimmage/common/Utilities.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/math/State.h>
#include <scrimmage/common/Shape.h>

#include <memory>
#include <limits>
#include <iostream>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::EntityInteraction,
                scrimmage::interaction::ROSShapeViz,
                ROSShapeViz_plugin)

namespace scrimmage {
namespace interaction {

ROSShapeViz::ROSShapeViz() {
}

bool ROSShapeViz::init(std::map<std::string, std::string> &mission_params,
                       std::map<std::string, std::string> &plugin_params) {

    // initialize ros
    if (!ros::isInitialized()) {
        int argc = 0;
        ros::init(argc, NULL, "scrimmage", ros::init_options::NoSigintHandler);
    }

    nh_ = std::make_shared<ros::NodeHandle>();
    sub_shapes_ = nh_->subscribe("shapes", 1000, &ROSShapeViz::shapes_cb, this);
    return true;
}

void ROSShapeViz::shapes_cb(const visualization_msgs::Marker::ConstPtr& msg) {
    if (msg->type == visualization_msgs::Marker::POINTS &&
        msg->action == 0 /* ADD an object */) {

        std::list<Eigen::Vector3d> points;
        for (auto& p : msg->points) {
            points.push_back(Eigen::Vector3d(p.x, p.y, p.z));
        }

        std::list<Eigen::Vector3d> point_colors;
        auto shape = shape::make_pointcloud(points, point_colors, 5,
                                            Eigen::Vector3d(255, 0, 0));

        // Set the ID based on the message namespace and ID
        std::string hash_str = msg->ns + std::to_string(msg->id);
        std::size_t hash_id = std::hash<std::string>{}(hash_str);
        shape->set_hash(hash_id);
        shape->set_hash_set(true);

        draw_shape(shape);
    } else if (msg->type == visualization_msgs::Marker::SPHERE &&
               msg->action == 0 /* ADD an object */) {

        Eigen::Vector3d point(msg->pose.position.x,
                              msg->pose.position.y,
                              msg->pose.position.z);

        auto shape = shape::make_sphere(point,
                                        msg->scale.x,
                                        Eigen::Vector3d(0, 0, 0));
        // Set the ID based on the message namespace and ID
        std::string hash_str = msg->ns + std::to_string(msg->id);
        std::size_t hash_id = std::hash<std::string>{}(hash_str);
        shape->set_hash(hash_id);
        shape->set_hash_set(true);

        draw_shape(shape);
    }
}

bool ROSShapeViz::step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                          double t, double dt) {
    ros::spinOnce();
    return true;
}
} // namespace interaction
} // namespace scrimmage
