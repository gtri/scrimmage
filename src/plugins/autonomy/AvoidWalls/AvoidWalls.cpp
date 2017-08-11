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
    bool min_point_found = false;
    Eigen::Vector3d min_point(0,0,0);

    cout << "-----------" << endl;
    for (auto msg : pcl_sub_->msgs<sc::Message<RayTrace::PointCloud>>()) {
        // Find closest point and move away from it        
        double min_dist = std::numeric_limits<double>::infinity();        
        for (RayTrace::PCPoint &p : msg->data.points) {
            double dist = p.point.norm();            
            if (dist < avoid_distance_ && dist < min_dist) {                
                min_dist = dist;
                min_point = p.point;
                min_point_found = true;
            }
        }        
    }

    if (min_point_found) {
        // Transform the min_point to global coordinate space
        min_point = state_->quat().rotate(min_point);
        min_point += state_->pos();        
        
        // Go in opposite direction
        Eigen::Vector3d diff = state_->pos() - min_point;
        desired_state_->vel() = diff.normalized()  * 10;

        cout << "vel: " << desired_state_->vel() << endl;
        
        //double heading = atan2(dir(1) - state_->pos()(1),
        //                       dir(0) - state_->pos()(0));       

        // Set the heading
            //desired_state_->quat().set(0, 0, heading); // roll, pitch, heading
    }        

    return true;
}
