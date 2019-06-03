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
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugins/autonomy/AvoidWalls/AvoidWalls.h>
#include <scrimmage/plugins/sensor/RayTrace/RayTrace.h>
#include <scrimmage/pubsub/Subscriber.h>

REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::AvoidWalls, AvoidWalls_plugin)

namespace scrimmage {
namespace autonomy {

void AvoidWalls::init(std::map<std::string, std::string> &params) {
    double initial_speed = get<double>("initial_speed", params, 21);
    avoid_distance_ = get<double>("avoid_distance", params, 20);

    auto pc_cb = [&] (scrimmage::MessagePtr<sensor::RayTrace::PointCloud> msg) {
        point_cloud_ = msg->data;
    };
    subscribe<sensor::RayTrace::PointCloud>("LocalNetwork", "RayTrace/pointcloud", pc_cb);

    heading_idx_ = vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::Out);
    speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::Out);

    vars_.output(heading_idx_, state_->quat().yaw());
    vars_.output(speed_idx_, initial_speed);
}

bool AvoidWalls::step_autonomy(double t, double dt) {
    // Find closest point and move away from it
    bool all_close_points = true;
    std::list<Eigen::Vector3d> points;
    for (sensor::RayTrace::PCPoint &p : point_cloud_.points) {
        if (p.point.norm() < avoid_distance_) {
            points.push_back(p.point);
        } else {
            all_close_points = false;
        }
    }

    if (!points.empty()) {
        std::vector<Eigen::Vector3d> O_vecs;
        for (Eigen::Vector3d &p : points) {
            // Transform the point to global coordinate space
            double dist = p.norm();
            p = state_->quat().rotate(p);
            p += state_->pos();

            // Get vector pointing towards point
            Eigen::Vector3d diff = p - state_->pos();
            double O_mag = avoid_distance_ - dist;

            Eigen::Vector3d O_dir = - O_mag * diff.normalized();
            O_vecs.push_back(O_dir);
        }

        // Normalize each repulsion vector and sum
        Eigen::Vector3d O_vec(0, 0, 0);
        for (auto it = O_vecs.begin(); it != O_vecs.end(); it++) {
            if (it->hasNaN()) {
                continue; // ignore misbehaved vectors
            }
            O_vec += *it;
        }

        Eigen::Vector3d dir;
        // Just turn left if all points are too close (reduces chattering)
        if (all_close_points) {
            dir = state_->quat().rotate(Eigen::Vector3d(0, -1, 0));
        } else {
            dir = O_vec.normalized();
        }
        dir *= 10;

        const double heading = atan2(dir(1), dir(0));

        vars_.output(heading_idx_, heading);
        vars_.output(speed_idx_, 10);
    } else {
        vars_.output(heading_idx_, state_->quat().yaw());
    }

    return true;
}
} // namespace autonomy
} // namespace scrimmage
