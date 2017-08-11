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

#include <iostream>
#include <limits>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/proto/ProtoConversions.h>

#include <scrimmage/plugins/autonomy/AvoidWalls/AvoidWalls.h>
#include <scrimmage/plugins/sensor/RayTrace/RayTrace.h>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy, AvoidWalls, AvoidWalls_plugin)

AvoidWalls::AvoidWalls()
{
}

void AvoidWalls::init(std::map<std::string,std::string> &params)
{
    double initial_speed = sc::get<double>("initial_speed", params, 21);
    avoid_distance_ = sc::get<double>("avoid_distance", params, 20);

    desired_state_->vel() = Eigen::Vector3d::UnitX()*initial_speed;
    desired_state_->quat().set(0,0,state_->quat().yaw());
    desired_state_->pos() = Eigen::Vector3d::UnitZ()*state_->pos()(2);

    pcl_sub_ = create_subscriber(std::to_string(parent_->id().id()) + "/0/pointcloud");
}

bool AvoidWalls::step_autonomy(double t, double dt)
{
    std::list<Eigen::Vector3d> points;

    for (auto msg : pcl_sub_->msgs<sc::Message<RayTrace::PointCloud>>()) {
        // Find closest point and move away from it        
        for (RayTrace::PCPoint &p : msg->data.points) {
            if (p.point.norm() < avoid_distance_) {
                points.push_back(p.point);
            }
        }        
    }

    if (points.size() > 0) {
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
        Eigen::Vector3d O_vec(0,0,0);
        for (auto it = O_vecs.begin(); it != O_vecs.end(); it++) {
            if (it->hasNaN()) {
                continue; // ignore misbehaved vectors
            }
            O_vec += *it;
        }

        Eigen::Vector3d dir = O_vec.normalized() * 10;

        std::shared_ptr<sp::Shape> vec(new sp::Shape);
        vec->set_type(sp::Shape::Line);
        sc::set(vec->mutable_color(), 255, 255, 255);
        vec->set_opacity(1.0);
        sc::add_point(vec, state_->pos());
        sc::add_point(vec, state_->pos() + dir);
        shapes_.push_back(vec);

        double heading = atan2(dir(1), dir(0));
        
        
        desired_state_->quat().set(0,0,heading);        
        desired_state_->vel() = Eigen::Vector3d::UnitX() * 10;
                
    } else {
        desired_state_->quat().set(0,0,state_->quat().yaw());
    }

    return true;
}
