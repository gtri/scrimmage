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

#include <scrimmage/common/RTree.h>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

namespace scrimmage {

void RTree::init(int size) {
    if (size > 0) {
        rtree_ = std::make_shared<rtree_t>(bgi::dynamic_rstar(size));
        size_ = size;
    }
}

void RTree::clear() {
    rtree_->clear();
    rtree_team_.clear();
}

void RTree::add(Eigen::Vector3d &pos, const ID &id) {
    point p(pos(0), pos(1), pos(2));
    std::pair<point, ID> pair(p, id);
    rtree_->insert(pair);

    int team_id = id.team_id();
    auto it = rtree_team_.find(team_id);
    if (it == rtree_team_.end()) {
        rtreePtr rtree = std::make_shared<rtree_t>(bgi::dynamic_rstar(size_));
        it = rtree_team_.insert(std::make_pair(team_id, rtree)).first;
    }
    it->second->insert(pair);
}

void results_to_neighbors(std::list<point_id_t> &results,
                          std::vector<ID> &neighbors,
                          int self_id) {
    neighbors.clear();
    neighbors.reserve(results.size());
    if (self_id >= 0) {
        for (point_id_t &pt_id : results) {
            if (pt_id.second.id() != self_id) {
                neighbors.push_back(pt_id.second);
            }
        }
    } else {
        for (point_id_t &pt_id : results) {
            neighbors.push_back(pt_id.second);
        }
    }
}

void RTree::nearest_n_neighbors(const Eigen::Vector3d &pos,
                                std::vector<ID> &neighbors, unsigned int n,
                                int self_id, int team_id) const {
    std::list<point_id_t> results;
    point sought(pos(0), pos(1), pos(2));

    if (self_id != -1) {
        // assume that if an id is given then it will be located at pos so
        // would always be in the neighborhood
        n += 1;
    }

    if (team_id == -1) {
        rtree_->query(bgi::nearest(sought, n), std::front_inserter(results));
    } else {
        auto it = rtree_team_.find(team_id);
        if (it == rtree_team_.end()) {
            neighbors.clear();
            return;
        } else {
            it->second->query(bgi::nearest(sought, n), std::front_inserter(results));
        }
    }
    results_to_neighbors(results, neighbors, self_id);
}

void RTree::neighbors_in_range(const Eigen::Vector3d &pos,
                               std::vector<ID> &neighbors,
                               double dist,
                               int self_id, int team_id) const {
    // see here: http://stackoverflow.com/a/22910447
    std::list<point_id_t> results;
    double x = pos(0);
    double y = pos(1);
    double z = pos(2);
    point sought(x, y, z);

    bg::model::box<point> box(
        point(x - dist, y - dist, z - dist), point(x + dist, y + dist, z + dist)
    );

    auto dist_func = [&](point_id_t const& v) {return bg::distance(v.first, sought) < dist;};

    if (team_id == -1) {
        rtree_->query(
            bgi::within(box) && bgi::satisfies(dist_func),
            std::front_inserter(results)
        );
    } else {
        auto it = rtree_team_.find(team_id);
        if (it == rtree_team_.end()) {
            neighbors.clear();
            return;
        } else {
            it->second->query(
                bgi::within(box) && bgi::satisfies(dist_func),
                std::front_inserter(results)
            );
        }
    }

    results_to_neighbors(results, neighbors, self_id);
}

} // namespace scrimmage
