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
#include <scrimmage/common/RTree.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>

#include <scrimmage/plugins/autonomy/Boids/Boids.h>

REGISTER_PLUGIN(scrimmage::Autonomy, Boids, Boids_plugin)

Boids::Boids()
{     
}

void Boids::init(std::map<std::string,std::string> &params)
{    
    desired_state_->vel() << 30, 0, 0;
    desired_state_->quat().set(0, 0, state_->quat().yaw());
    desired_state_->pos() << 0, 0, state_->pos()(2);    

    w_align_ = scrimmage::get("align_weight", params, 0.01);
    w_avoid_ = scrimmage::get("avoid_weight", params, 0.95);
    w_centroid_ = scrimmage::get("centroid_weight", params, 0.05);
}

bool Boids::step_autonomy(double t, double dt)
{     
    // Find n nearest neighbor of ownship
    unsigned int num_neighbors = 10;
    std::vector<scrimmage::ID> rtree_neighbors;     
    rtree_->nearest_n_neighbors(state_->pos_const(), rtree_neighbors, num_neighbors);
     
    {
        // Ensure that the neighbors are "in front" (abs(bearing) < 100)
        // Remove contacts that are behind
        double bearing = scrimmage::Angles::deg2rad(100.0*2.0);
        std::vector<scrimmage::ID>::iterator it = rtree_neighbors.begin();
        while (it != rtree_neighbors.end()) {
            if (state_->InFieldOfView(*(*contacts_)[it->id()].state(),bearing,bearing)) {
                // The neighbor is "in front"                    
                it++;
            } else {
                // The neighbor is "behind." Remove it.
                it = rtree_neighbors.erase(it);
            }
        }
    }      
     
    // Steer to avoid local neighbors
    // Align with neighbors     
    // Cohesion: move towards average position of neighbors
    // (i.e., Find centroid of neighbors)
    Eigen::Vector3d avoid = Eigen::Vector3d::Zero();
    Eigen::Vector3d align = Eigen::Vector3d::Zero();
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (std::vector<scrimmage::ID>::iterator it = rtree_neighbors.begin();
         it != rtree_neighbors.end(); it++) {          

        if (it->id() == parent_->id().id()) {
            continue;
        }
         
        centroid = centroid + (*contacts_)[it->id()].state()->pos();
     
        align = align + (*contacts_)[it->id()].state()->vel();
          
        double dist = (state_->pos() - (*contacts_)[it->id()].state()->pos()).norm();
        avoid = avoid + (state_->pos() - (*contacts_)[it->id()].state()->pos()) / pow(dist,2);        
    }
    centroid = centroid / (double)rtree_neighbors.size();
    avoid = state_->pos() + avoid;
    avoid(2) = state_->pos()(2);
     
    if (rtree_neighbors.size() > 0) {      
          
        //cout << "Centroid: " << centroid << endl;
        //cout << "Align: " << align << endl;
        //cout << "Avoid: " << avoid << endl;          
          
        Eigen::Vector3d target = avoid * w_avoid_ + centroid * w_centroid_ + align * w_align_;
                            
        double heading = atan2(target(1) - state_->pos()(1), 
                               target(0) - state_->pos()(0));
                    
        desired_state_->quat().set(0, 0, heading);
        desired_state_->pos() << 0, 0, target(2);
    }
     
    return true;
}
